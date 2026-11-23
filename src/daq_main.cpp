// ============================================================================
// daq_main.cpp
// 엔코더/경사계(CAN) + ADS1263 ADC 를 주기적으로 샘플링해서
// 시각(UTC ms)과 병합한 한 행으로 SQLite에 저장합니다.
//
//   sudo ./build/daq -d /home/pi/daq.db -i can0 -p 100 --vref 5.0
//   sudo ./build/daq --no-adc         # ADC 없이 CAN + DB만 (vcan 테스트용)
// ============================================================================
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "app_signal.hpp"
#include "can_iface.hpp"
#include "db_writer.hpp"
#include "sensor_parser.hpp"

extern "C" {
#include "ADS1263.h"
}

using namespace daq;
using Clock = std::chrono::steady_clock;

// ----------------------------------------------------------------------------
// 설정
// ----------------------------------------------------------------------------
struct Config {
    std::string db_path = "daq.db";
    std::string iface = "can0";
    int period_ms = 100;
    int adc_channel = 0;
    double vref = 5.0;          // REFMUX=0x24 (AVDD/AVSS). 실측값 사용 권장
    bool use_adc = true;
    uint32_t sync_id = 0x80;    // CANopen SYNC. 센서 매뉴얼 확인
    int sync_period_ms = 100;   // 0 이면 SYNC 송신 안 함
    int stale_ms = 500;         // 이보다 오래된 CAN 데이터는 STALE 플래그
    bool quiet = false;
    std::string note;
};

static void usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  -d, --db PATH          SQLite file (default daq.db)\n"
        "  -i, --iface NAME       CAN interface (default can0)\n"
        "  -p, --period MS        sampling period (default 100)\n"
        "  -c, --channel N        ADC1 channel AIN0..AIN10 (default 0)\n"
        "  -r, --vref VOLTS       ADC reference voltage (default 5.0)\n"
        "      --no-adc           run without ADC (CAN + DB only)\n"
        "      --sync-id HEX      SYNC frame id (default 0x80)\n"
        "      --sync-ms MS       SYNC period, 0 disables (default 100)\n"
        "      --stale-ms MS      CAN data older than this is flagged (default 500)\n"
        "  -n, --note TEXT        note stored in runs table\n"
        "  -q, --quiet            do not print each sample\n", prog);
}

static bool parseArgs(int argc, char** argv, Config& c) {
    enum { OPT_NO_ADC = 1000, OPT_SYNC_ID, OPT_SYNC_MS, OPT_STALE };
    static const option opts[] = {
        {"db", required_argument, nullptr, 'd'},
        {"iface", required_argument, nullptr, 'i'},
        {"period", required_argument, nullptr, 'p'},
        {"channel", required_argument, nullptr, 'c'},
        {"vref", required_argument, nullptr, 'r'},
        {"note", required_argument, nullptr, 'n'},
        {"quiet", no_argument, nullptr, 'q'},
        {"no-adc", no_argument, nullptr, OPT_NO_ADC},
        {"sync-id", required_argument, nullptr, OPT_SYNC_ID},
        {"sync-ms", required_argument, nullptr, OPT_SYNC_MS},
        {"stale-ms", required_argument, nullptr, OPT_STALE},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}};
    int o;
    while ((o = getopt_long(argc, argv, "d:i:p:c:r:n:qh", opts, nullptr)) != -1) {
        switch (o) {
        case 'd': c.db_path = optarg; break;
        case 'i': c.iface = optarg; break;
        case 'p': c.period_ms = std::atoi(optarg); break;
        case 'c': c.adc_channel = std::atoi(optarg); break;
        case 'r': c.vref = std::atof(optarg); break;
        case 'n': c.note = optarg; break;
        case 'q': c.quiet = true; break;
        case OPT_NO_ADC: c.use_adc = false; break;
        case OPT_SYNC_ID: c.sync_id = (uint32_t)std::strtoul(optarg, nullptr, 0); break;
        case OPT_SYNC_MS: c.sync_period_ms = std::atoi(optarg); break;
        case OPT_STALE: c.stale_ms = std::atoi(optarg); break;
        case 'h': usage(argv[0]); std::exit(0);
        default: usage(argv[0]); return false;
        }
    }
    if (c.period_ms < 5 || c.adc_channel < 0 || c.adc_channel > 10 || c.vref <= 0.0 ||
        c.sync_period_ms < 0 || c.sync_id > CAN_SFF_MASK || c.stale_ms <= 0) {
        std::fprintf(stderr, "invalid option value\n");
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------------
// CAN 수신 상태 (수신 스레드가 쓰고 메인 루프가 읽음 -> mutex)
// ----------------------------------------------------------------------------
struct SensorState {
    std::mutex mtx;
    std::optional<int32_t> enc_counts, enc_dist;
    std::optional<int32_t> inc_x_cdeg, inc_y_cdeg;   // 필터 후, 0.01°
    Clock::time_point enc_rx{}, inc_rx{};
    InclinationFilter filt_x, filt_y;
    uint64_t frames = 0, bad_frames = 0;
};

static void handleFrame(SensorState& s, const can_frame& f) {
    auto id = standardId(f);
    if (!id) return;
    const auto now = Clock::now();

    if (*id == ID_ENCODER) {
        auto counts = decodeEncoderCounts(f);
        std::lock_guard<std::mutex> lk(s.mtx);
        s.frames++;
        if (!counts) { s.bad_frames++; return; }
        s.enc_counts = *counts;
        s.enc_dist = encoderToDistance(*counts);
        s.enc_rx = now;
    } else if (*id == ID_INCL_A || *id == ID_INCL_B) {
        auto inc = decodeInclination(f);
        std::lock_guard<std::mutex> lk(s.mtx);
        s.frames++;
        if (!inc) { s.bad_frames++; return; }
        s.inc_x_cdeg = s.filt_x.update(inc->x_cdeg);
        s.inc_y_cdeg = s.filt_y.update(inc->y_cdeg);
        s.inc_rx = now;
    }
}

static void canRxThread(CanSocket& can, SensorState& s) {
    can_frame f {};
    int consecutive_errors = 0;
    while (g_running) {
        switch (can.read(f)) {
        case CanSocket::ReadResult::Ok:
            consecutive_errors = 0;
            handleFrame(s, f);
            break;
        case CanSocket::ReadResult::Timeout:
            break;  // g_running 확인하러 돌아감
        case CanSocket::ReadResult::Error:
            if (++consecutive_errors == 1) perror("[CAN] read");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 버스오프 등에서 CPU 100% 방지
            break;
        }
    }
}

static void canSyncThread(CanSocket& can, uint32_t id, int period_ms) {
    auto next = Clock::now();
    uint64_t failures = 0;
    while (g_running) {
        if (!can.send(id, nullptr, 0) && failures++ == 0) perror("[CAN] sync send");
        next += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next);
    }
}

static int64_t epochMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

static const char* adcErrName(int rc) {
    switch (rc) {
    case ADS1263_ERR_TIMEOUT: return "DRDY timeout";
    case ADS1263_ERR_CRC: return "CRC error";
    case ADS1263_ERR_NOT_READY: return "no new data";
    case ADS1263_ERR_SPI: return "SPI error";
    default: return "error";
    }
}

// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
    Config cfg;
    if (!parseArgs(argc, argv, cfg)) return 2;
    installStopHandlers();

    // [1] ADC
    bool hw_init = false;
    if (cfg.use_adc) {
        if (DEV_Module_Init() != 0) {
            std::fprintf(stderr, "GPIO/SPI init failed. Run with sudo?\n");
            return 1;
        }
        hw_init = true;
        if (ADS1263_init_ADC1(ADS1263_400SPS) != 0) {
            std::fprintf(stderr, "ADS1263 not responding (chip ID mismatch). Check wiring/CS/SPI.\n");
            DEV_Module_Exit();
            return 1;
        }
        ADS1263_SetMode(0);  // single-ended (AINx vs AINCOM)
    }

    // [2] CAN
    CanSocket can;
    if (!can.open(cfg.iface)) {
        std::fprintf(stderr, "Setup CAN first, e.g.\n"
                             "  sudo ip link set %s type can bitrate 125000\n"
                             "  sudo ip link set up %s\n", cfg.iface.c_str(), cfg.iface.c_str());
        if (hw_init) DEV_Module_Exit();
        return 1;
    }

    // [3] DB
    const auto t0 = Clock::now();
    const int64_t start_ts = epochMs();
    DbWriter db;
    RunInfo info{cfg.iface, cfg.period_ms, cfg.adc_channel, cfg.vref, cfg.note};
    if (!db.open(cfg.db_path, info, start_ts)) {
        if (hw_init) DEV_Module_Exit();
        return 1;
    }
    std::printf("DB %s, run_id=%lld\n", cfg.db_path.c_str(), (long long)db.runId());

    bool clock_synced = isSystemClockSynced();
    if (!clock_synced)
        std::fprintf(stderr, "[WARN] system clock is not NTP-synchronized; ts_ms may be wrong "
                             "(rows are flagged, mono_ms is still valid)\n");

    // [4] 스레드
    SensorState state;
    std::thread rx(canRxThread, std::ref(can), std::ref(state));
    std::thread tx;
    if (cfg.sync_period_ms > 0) tx = std::thread(canSyncThread, std::ref(can), cfg.sync_id, cfg.sync_period_ms);

    std::printf("Sampling every %d ms. Ctrl+C to stop.\n", cfg.period_ms);

    // [5] 측정 루프: 절대 시각 기준 sleep_until -> 누적 드리프트 없음
    const auto period = std::chrono::milliseconds(cfg.period_ms);
    auto next = Clock::now();
    bool prev_overrun = false;
    uint64_t samples = 0, adc_errors = 0, overruns = 0;
    auto last_clock_check = Clock::now();

    while (g_running) {
        const auto t_sample = Clock::now();
        SampleRow row;
        row.ts_ms = epochMs();
        row.mono_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_sample - t0).count();

        // ADC
        if (cfg.use_adc) {
            UDOUBLE raw = 0;
            int rc = ADS1263_ReadADC1((UBYTE)cfg.adc_channel, &raw);
            if (rc == ADS1263_OK) {
                row.adc_raw = (int32_t)raw;
                row.adc_v = adcRawToVolts(raw, cfg.vref);
            } else {
                row.flags |= FLAG_ADC_ERROR;
                if (adc_errors++ < 10 || adc_errors % 100 == 0)
                    std::fprintf(stderr, "[ADC] %s (total %llu)\n", adcErrName(rc), (unsigned long long)adc_errors);
            }
        } else {
            row.flags |= FLAG_ADC_DISABLED;
        }

        // CAN 스냅샷 + 데이터 나이
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            auto age = [&](Clock::time_point tp) {
                return std::chrono::duration_cast<std::chrono::milliseconds>(t_sample - tp).count();
            };
            if (state.enc_dist) {
                row.enc_counts = state.enc_counts;
                row.enc_dist = state.enc_dist;
                row.enc_age_ms = age(state.enc_rx);
            }
            if (state.inc_x_cdeg) {
                row.inc_x_deg = *state.inc_x_cdeg / 100.0;
                row.inc_y_deg = *state.inc_y_cdeg / 100.0;
                row.inc_age_ms = age(state.inc_rx);
            }
        }
        if (!row.enc_age_ms || *row.enc_age_ms > cfg.stale_ms) row.flags |= FLAG_ENC_STALE;
        if (!row.inc_age_ms || *row.inc_age_ms > cfg.stale_ms) row.flags |= FLAG_INC_STALE;

        if (t_sample - last_clock_check > std::chrono::seconds(10)) {
            clock_synced = isSystemClockSynced();
            last_clock_check = t_sample;
        }
        if (!clock_synced) row.flags |= FLAG_CLOCK_UNSYNCED;
        if (prev_overrun) row.flags |= FLAG_LOOP_OVERRUN;

        db.push(row);
        samples++;

        if (!cfg.quiet) {
            std::printf("[T:%lld] ADC: %s | Dist: %s | IncX: %s | IncY: %s | flags=0x%02x\n",
                        (long long)row.ts_ms,
                        row.adc_v ? (std::to_string(*row.adc_v).substr(0, 8) + "V").c_str() : "--",
                        row.enc_dist ? std::to_string(*row.enc_dist).c_str() : "--",
                        row.inc_x_deg ? std::to_string(*row.inc_x_deg).substr(0, 7).c_str() : "--",
                        row.inc_y_deg ? std::to_string(*row.inc_y_deg).substr(0, 7).c_str() : "--",
                        row.flags);
        }

        // 다음 주기
        next += period;
        const auto now = Clock::now();
        prev_overrun = now > next;
        if (prev_overrun) {
            overruns++;
            next = now;  // 밀린 주기를 몰아서 찍지 않고 현재 시각부터 다시
        } else {
            std::this_thread::sleep_until(next);
        }
    }

    // [6] 정리 (Ctrl+C / SIGTERM 시 여기 도달)
    std::printf("\nStopping...\n");
    g_running = false;
    rx.join();
    if (tx.joinable()) tx.join();
    can.close();
    db.close();
    if (hw_init) DEV_Module_Exit();

    std::printf("samples=%llu written=%llu dropped=%llu db_failed=%llu adc_errors=%llu overruns=%llu "
                "can_frames=%llu bad_frames=%llu\n",
                (unsigned long long)samples, (unsigned long long)db.written(),
                (unsigned long long)db.dropped(), (unsigned long long)db.failed(),
                (unsigned long long)adc_errors, (unsigned long long)overruns,
                (unsigned long long)state.frames, (unsigned long long)state.bad_frames);
    return 0;
}
