// ============================================
// Step 1: CAN Module Communication Only
// Raspberry Pi 4 + MCP2515
//   ./build/can_module_test [iface] [seconds]   (default: can0 60)
// 수정 사항:
//  - 공유 데이터 mutex 보호, running 플래그 atomic
//  - 수신 루프의 sleep(10ms) 제거 (read가 블로킹 + 타임아웃)
//  - 거리 계산 int64 (오버플로 수정), DLC 검사
//  - 1ms sleep 카운터 대신 steady_clock 경과시간
//  - Ctrl+C 시 정상 종료
// ============================================
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "app_signal.hpp"
#include "can_iface.hpp"
#include "sensor_parser.hpp"

using namespace daq;
using Clock = std::chrono::steady_clock;

struct State {
    std::mutex mtx;
    std::optional<int32_t> enc_read, enc_dist, enc_base;
    std::optional<int32_t> inc_x, inc_y;
    InclinationFilter fx, fy;
    uint64_t frames = 0, bad = 0;
};

static void rxTask(CanSocket& can, State& s) {
    can_frame f {};
    while (g_running) {
        auto r = can.read(f);
        if (r == CanSocket::ReadResult::Error) {
            perror("[CAN] read");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (r != CanSocket::ReadResult::Ok) continue;
        auto id = standardId(f);
        if (!id) continue;

        std::lock_guard<std::mutex> lk(s.mtx);
        if (*id == ID_ENCODER) {
            s.frames++;
            auto c = decodeEncoderCounts(f);
            if (!c) { s.bad++; continue; }
            s.enc_read = *c;
            s.enc_dist = encoderToDistance(*c);
            if (!s.enc_base) s.enc_base = s.enc_dist;
            std::printf("Encoder - Read: %d, Dist: %d\n", *s.enc_read, *s.enc_dist);
        } else if (*id == ID_INCL_A || *id == ID_INCL_B) {
            s.frames++;
            auto inc = decodeInclination(f);
            if (!inc) { s.bad++; continue; }
            s.inc_x = s.fx.update(inc->x_cdeg);
            s.inc_y = s.fy.update(inc->y_cdeg);
            std::printf("Inclination - X: %d, Y: %d (0.01 deg)\n", *s.inc_x, *s.inc_y);
        }
    }
}

static void syncTask(CanSocket& can) {
    auto next = Clock::now();
    while (g_running) {
        can.send(0x80, nullptr, 0);   // SYNC (main DAQ와 동일하게 DLC 0)
        next += std::chrono::milliseconds(50);
        std::this_thread::sleep_until(next);
    }
}

int main(int argc, char** argv) {
    const std::string iface = argc > 1 ? argv[1] : "can0";
    const int seconds = argc > 2 ? std::atoi(argv[2]) : 60;

    std::printf("========================================\n"
                "Step 1: CAN Communication Test (%s, %d s)\n"
                "========================================\n", iface.c_str(), seconds);
    installStopHandlers();

    CanSocket can;
    if (!can.open(iface)) {
        std::fprintf(stderr, "Setup CAN first:\n  sudo ip link set %s type can bitrate 125000\n"
                             "  sudo ip link set up %s\n", iface.c_str(), iface.c_str());
        return 1;
    }

    State s;
    const auto t0 = Clock::now();
    std::thread rx(rxTask, std::ref(can), std::ref(s));
    std::thread tx(syncTask, std::ref(can));

    for (int i = 0; i < seconds && g_running; i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();
        std::lock_guard<std::mutex> lk(s.mtx);
        std::printf("\n--- Status (elapsed %lld ms) frames=%llu bad=%llu ---\n", (long long)ms,
                    (unsigned long long)s.frames, (unsigned long long)s.bad);
        std::printf("Encoder Distance: %s\n", s.enc_dist ? std::to_string(*s.enc_dist).c_str() : "--");
        std::printf("Inclination X: %s, Y: %s\n",
                    s.inc_x ? std::to_string(*s.inc_x).c_str() : "--",
                    s.inc_y ? std::to_string(*s.inc_y).c_str() : "--");
    }

    g_running = false;
    rx.join();
    tx.join();
    std::printf("Test completed.\n");
    return 0;
}
