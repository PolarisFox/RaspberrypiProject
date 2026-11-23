// ============================================================================
// db_writer.cpp - SQLite 비동기 저장기 구현
// ============================================================================
#include "db_writer.hpp"

#include <chrono>
#include <cstdio>
#include <sqlite3.h>
#include <sys/timex.h>
#include <type_traits>
#include <unistd.h>

namespace daq {

namespace {

const char* kSchema = R"SQL(
CREATE TABLE IF NOT EXISTS runs (
    id            INTEGER PRIMARY KEY,
    started_ts_ms INTEGER NOT NULL,
    ended_ts_ms   INTEGER,
    hostname      TEXT,
    can_iface     TEXT,
    period_ms     INTEGER,
    adc_channel   INTEGER,
    adc_vref      REAL,
    note          TEXT
);

CREATE TABLE IF NOT EXISTS samples (
    id          INTEGER PRIMARY KEY,
    run_id      INTEGER NOT NULL REFERENCES runs(id),
    ts_ms       INTEGER NOT NULL,   -- UTC epoch milliseconds
    mono_ms     INTEGER NOT NULL,   -- ms since program start (monotonic)
    adc_raw     INTEGER,            -- ADS1263 raw signed code (re-scale later if vref changes)
    adc_v       REAL,
    enc_counts  INTEGER,
    enc_dist    INTEGER,
    inc_x_deg   REAL,
    inc_y_deg   REAL,
    enc_age_ms  INTEGER,            -- age of encoder data at ts_ms
    inc_age_ms  INTEGER,            -- age of inclinometer data at ts_ms
    flags       INTEGER NOT NULL DEFAULT 0
);

CREATE INDEX IF NOT EXISTS idx_samples_ts  ON samples(ts_ms);
CREATE INDEX IF NOT EXISTS idx_samples_run ON samples(run_id, ts_ms);

-- 사람이 읽기 쉬운 현지 시각 뷰
CREATE VIEW IF NOT EXISTS samples_local AS
SELECT id, run_id,
       strftime('%Y-%m-%d %H:%M:%f', ts_ms / 1000.0, 'unixepoch', 'localtime') AS local_time,
       ts_ms, adc_v, enc_dist, inc_x_deg, inc_y_deg, enc_age_ms, inc_age_ms, flags
FROM samples;
)SQL";

const char* kInsert =
    "INSERT INTO samples (run_id, ts_ms, mono_ms, adc_raw, adc_v, enc_counts, enc_dist, "
    "inc_x_deg, inc_y_deg, enc_age_ms, inc_age_ms, flags) "
    "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12)";

template <typename T>
void bindOpt(sqlite3_stmt* st, int idx, const std::optional<T>& v) {
    if (!v) { sqlite3_bind_null(st, idx); return; }
    if constexpr (std::is_floating_point_v<T>) sqlite3_bind_double(st, idx, *v);
    else sqlite3_bind_int64(st, idx, (sqlite3_int64)*v);
}

int64_t nowEpochMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

} // namespace

bool isSystemClockSynced() {
    struct timex tx {};
    int state = adjtimex(&tx);
    return state != TIME_ERROR && !(tx.status & STA_UNSYNC);
}

bool DbWriter::exec(const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        std::fprintf(stderr, "[DB] %s\n", err ? err : "unknown error");
        sqlite3_free(err);
        return false;
    }
    return true;
}

bool DbWriter::open(const std::string& path, const RunInfo& info, int64_t start_ts_ms) {
    if (sqlite3_open(path.c_str(), &db_) != SQLITE_OK) {
        std::fprintf(stderr, "[DB] cannot open %s: %s\n", path.c_str(), sqlite3_errmsg(db_));
        sqlite3_close(db_);
        db_ = nullptr;
        return false;
    }
    sqlite3_busy_timeout(db_, 2000);
    // WAL: 쓰는 동안에도 다른 프로세스(분석 스크립트)가 읽을 수 있음
    // synchronous=NORMAL: WAL에서 안전하면서 SD카드 fsync 횟수를 줄임
    if (!exec("PRAGMA journal_mode=WAL;") || !exec("PRAGMA synchronous=NORMAL;") ||
        !exec("PRAGMA foreign_keys=ON;") || !exec(kSchema)) {
        close();
        return false;
    }

    char host[256] = {0};
    gethostname(host, sizeof(host) - 1);

    sqlite3_stmt* st = nullptr;
    const char* runSql = "INSERT INTO runs (started_ts_ms, hostname, can_iface, period_ms, adc_channel, adc_vref, note) "
                         "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)";
    if (sqlite3_prepare_v2(db_, runSql, -1, &st, nullptr) != SQLITE_OK) {
        std::fprintf(stderr, "[DB] %s\n", sqlite3_errmsg(db_));
        close();
        return false;
    }
    sqlite3_bind_int64(st, 1, start_ts_ms);
    sqlite3_bind_text(st, 2, host, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 3, info.iface.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(st, 4, info.period_ms);
    sqlite3_bind_int(st, 5, info.adc_channel);
    sqlite3_bind_double(st, 6, info.vref);
    sqlite3_bind_text(st, 7, info.note.c_str(), -1, SQLITE_TRANSIENT);
    int rc = sqlite3_step(st);
    sqlite3_finalize(st);
    if (rc != SQLITE_DONE) {
        std::fprintf(stderr, "[DB] cannot insert run: %s\n", sqlite3_errmsg(db_));
        close();
        return false;
    }
    run_id_ = sqlite3_last_insert_rowid(db_);

    if (sqlite3_prepare_v2(db_, kInsert, -1, &insert_, nullptr) != SQLITE_OK) {
        std::fprintf(stderr, "[DB] %s\n", sqlite3_errmsg(db_));
        close();
        return false;
    }

    stop_ = false;
    thread_ = std::thread(&DbWriter::threadMain, this);
    return true;
}

void DbWriter::push(const SampleRow& row) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (stop_ || queue_.size() >= kMaxQueue) {
            dropped_++;
            return;
        }
        queue_.push_back(row);
        if (queue_.size() < kBatchRows) return;
    }
    cv_.notify_one();
}

bool DbWriter::writeBatch(std::deque<SampleRow>& batch) {
    if (batch.empty()) return true;
    if (!exec("BEGIN;")) { failed_ += batch.size(); batch.clear(); return false; }

    size_t ok = 0;
    for (const auto& r : batch) {
        sqlite3_reset(insert_);
        sqlite3_clear_bindings(insert_);
        sqlite3_bind_int64(insert_, 1, run_id_);
        sqlite3_bind_int64(insert_, 2, r.ts_ms);
        sqlite3_bind_int64(insert_, 3, r.mono_ms);
        bindOpt(insert_, 4, r.adc_raw);
        bindOpt(insert_, 5, r.adc_v);
        bindOpt(insert_, 6, r.enc_counts);
        bindOpt(insert_, 7, r.enc_dist);
        bindOpt(insert_, 8, r.inc_x_deg);
        bindOpt(insert_, 9, r.inc_y_deg);
        bindOpt(insert_, 10, r.enc_age_ms);
        bindOpt(insert_, 11, r.inc_age_ms);
        sqlite3_bind_int64(insert_, 12, r.flags);
        if (sqlite3_step(insert_) == SQLITE_DONE) ok++;
        else std::fprintf(stderr, "[DB] insert failed: %s\n", sqlite3_errmsg(db_));
    }
    if (!exec("COMMIT;")) {
        exec("ROLLBACK;");
        failed_ += batch.size();
        batch.clear();
        return false;
    }
    written_ += ok;
    failed_ += batch.size() - ok;
    batch.clear();
    return true;
}

void DbWriter::threadMain() {
    std::deque<SampleRow> batch;
    for (;;) {
        bool stopping;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, std::chrono::milliseconds(kFlushMs),
                         [this] { return stop_ || queue_.size() >= kBatchRows; });
            batch.swap(queue_);
            stopping = stop_;
        }
        writeBatch(batch);
        if (stopping) {
            // stop_ 이후에는 push 가 거부되므로 큐는 비어 있음. 혹시 남았으면 마저 기록.
            std::lock_guard<std::mutex> lk(mtx_);
            batch.swap(queue_);
            writeBatch(batch);
            return;
        }
    }
}

void DbWriter::close() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();

    if (db_) {
        if (run_id_ >= 0) {
            sqlite3_stmt* st = nullptr;
            if (sqlite3_prepare_v2(db_, "UPDATE runs SET ended_ts_ms=?1 WHERE id=?2", -1, &st, nullptr) == SQLITE_OK) {
                sqlite3_bind_int64(st, 1, nowEpochMs());
                sqlite3_bind_int64(st, 2, run_id_);
                sqlite3_step(st);
            }
            sqlite3_finalize(st);
        }
        if (insert_) { sqlite3_finalize(insert_); insert_ = nullptr; }
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

} // namespace daq
