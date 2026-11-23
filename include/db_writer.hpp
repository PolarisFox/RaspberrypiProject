// ============================================================================
// db_writer.hpp - SQLite 비동기 저장기
// - 측정 루프는 push()만 호출 (디스크 I/O로 100ms 주기가 흔들리지 않게)
// - 별도 스레드가 1초마다(또는 일정 개수마다) 한 트랜잭션으로 묶어서 INSERT
//   -> SD카드 쓰기 횟수 감소, 정전 시 최대 손실 약 1초
// ============================================================================
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct sqlite3;
struct sqlite3_stmt;

namespace daq {

// flags 비트
enum SampleFlags : uint32_t {
    FLAG_ADC_ERROR      = 1u << 0,  // ADC 타임아웃/CRC 오류 (adc_* 는 NULL)
    FLAG_ENC_STALE      = 1u << 1,  // 엔코더 데이터가 오래됨 또는 아직 미수신
    FLAG_INC_STALE      = 1u << 2,  // 경사계 데이터가 오래됨 또는 아직 미수신
    FLAG_CLOCK_UNSYNCED = 1u << 3,  // 시스템 시계가 NTP 동기화 안 됨 (ts_ms 신뢰도 낮음)
    FLAG_LOOP_OVERRUN   = 1u << 4,  // 직전 주기가 목표 주기를 넘김
    FLAG_ADC_DISABLED   = 1u << 5,  // --no-adc 로 실행
};

// 한 시점의 병합된 측정값 (한 행)
struct SampleRow {
    int64_t ts_ms = 0;     // UTC epoch ms (system_clock) - 다른 데이터와 맞출 기준 시각
    int64_t mono_ms = 0;   // 프로그램 시작 후 경과 ms (steady_clock) - 시계 보정에 영향 없음
    std::optional<int64_t> adc_raw;
    std::optional<double>  adc_v;
    std::optional<int32_t> enc_counts;
    std::optional<int32_t> enc_dist;
    std::optional<double>  inc_x_deg;
    std::optional<double>  inc_y_deg;
    std::optional<int64_t> enc_age_ms;  // 샘플 시각 기준 마지막 엔코더 프레임 이후 경과
    std::optional<int64_t> inc_age_ms;
    uint32_t flags = 0;
};

struct RunInfo {
    std::string iface;
    int period_ms = 100;
    int adc_channel = 0;
    double vref = 5.0;
    std::string note;
};

class DbWriter {
public:
    DbWriter() = default;
    ~DbWriter() { close(); }
    DbWriter(const DbWriter&) = delete;
    DbWriter& operator=(const DbWriter&) = delete;

    // DB 열기 + 스키마 생성 + runs 행 추가 + 쓰기 스레드 시작
    bool open(const std::string& path, const RunInfo& info, int64_t start_ts_ms);
    // 논블로킹. 큐가 가득 차면 버리고 dropped 카운트 증가
    void push(const SampleRow& row);
    // 남은 데이터 모두 기록 후 종료 (여러 번 호출해도 안전)
    void close();

    int64_t runId() const { return run_id_; }
    uint64_t written() const { return written_.load(); }
    uint64_t dropped() const { return dropped_.load(); }
    uint64_t failed()  const { return failed_.load(); }

    static constexpr size_t kMaxQueue = 20000;   // 100ms 주기면 약 33분 분량
    static constexpr size_t kBatchRows = 200;
    static constexpr int    kFlushMs  = 1000;

private:
    void threadMain();
    bool writeBatch(std::deque<SampleRow>& batch);
    bool exec(const char* sql);

    sqlite3* db_ = nullptr;
    sqlite3_stmt* insert_ = nullptr;
    int64_t run_id_ = -1;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<SampleRow> queue_;
    bool stop_ = false;
    std::thread thread_;

    std::atomic<uint64_t> written_{0};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<uint64_t> failed_{0};
};

// 시스템 시계가 NTP 등으로 동기화되었는지 (adjtimex). 라즈베리파이는 RTC가 없어
// 부팅 직후 네트워크가 없으면 시간이 틀릴 수 있음.
bool isSystemClockSynced();

} // namespace daq
