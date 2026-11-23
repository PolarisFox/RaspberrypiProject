// Host test for DbWriter: writes rows (incl. NULLs) and reads them back.
#include <cstdio>
#include <cstdlib>
#include <sqlite3.h>
#include <string>
#include <unistd.h>
#include "db_writer.hpp"

using namespace daq;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

static long long scalar(sqlite3* db, const char* sql) {
    sqlite3_stmt* st = nullptr;
    long long v = -1;
    if (sqlite3_prepare_v2(db, sql, -1, &st, nullptr) == SQLITE_OK && sqlite3_step(st) == SQLITE_ROW)
        v = sqlite3_column_int64(st, 0);
    sqlite3_finalize(st);
    return v;
}

int main() {
    char path[] = "/tmp/daq_test_XXXXXX";
    int fd = mkstemp(path);
    close(fd);
    unlink(path);

    const int N = 1234;
    {
        DbWriter w;
        RunInfo info{"vcan0", 100, 0, 5.0, "unit test"};
        CHECK(w.open(path, info, 1700000000000LL));
        for (int i = 0; i < N; i++) {
            SampleRow r;
            r.ts_ms = 1700000000000LL + i * 100;
            r.mono_ms = i * 100;
            if (i % 10 != 0) { r.adc_raw = 0x40000000; r.adc_v = 2.5; }
            else r.flags |= FLAG_ADC_ERROR;
            r.enc_counts = i;
            r.enc_dist = i * 2;
            r.enc_age_ms = 5;
            r.flags |= FLAG_INC_STALE;   // inclination never received -> NULL columns
            w.push(r);
        }
        w.close();
        CHECK(w.written() == (uint64_t)N);
        CHECK(w.dropped() == 0);
        CHECK(w.failed() == 0);
        w.close();  // idempotent
    }

    sqlite3* db = nullptr;
    CHECK(sqlite3_open(path, &db) == SQLITE_OK);
    CHECK(scalar(db, "SELECT COUNT(*) FROM samples") == N);
    CHECK(scalar(db, "SELECT COUNT(*) FROM samples WHERE adc_v IS NULL") == (N + 9) / 10);
    CHECK(scalar(db, "SELECT COUNT(*) FROM samples WHERE inc_x_deg IS NOT NULL") == 0);
    CHECK(scalar(db, "SELECT COUNT(*) FROM samples WHERE flags & 1") == (N + 9) / 10);
    CHECK(scalar(db, "SELECT MAX(ts_ms) - MIN(ts_ms) FROM samples") == (N - 1) * 100LL);
    CHECK(scalar(db, "SELECT COUNT(*) FROM runs WHERE ended_ts_ms IS NOT NULL") == 1);
    CHECK(scalar(db, "SELECT COUNT(*) FROM samples_local") == N);
    sqlite3_close(db);

    unlink(path);
    std::string wal = std::string(path) + "-wal", shm = std::string(path) + "-shm";
    unlink(wal.c_str());
    unlink(shm.c_str());

    if (g_fail == 0) std::printf("test_db_writer: all passed\n");
    return g_fail ? 1 : 0;
}
