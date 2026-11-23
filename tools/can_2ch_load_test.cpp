// ============================================
// 2-CH CAN 통신 + CPU 부하 테스트
//   ./build/can_2ch_load_test [seconds] [--verbose]   (default 60)
// 수정 사항:
//  - 수신 루프의 sleep(10ms) 제거 -> 측정 처리량이 인위적으로 제한되지 않음
//  - 프레임마다 콘솔 출력하던 것을 기본 OFF (출력 자체가 병목/측정 왜곡)
//  - 출력은 한 mutex로 직렬화 (cout/printf 혼용, hex 상태 누수 제거)
//  - CPU 부하 스레드를 코어 수만큼 생성 (기존 1개는 4코어에서 25% 부하)
//  - 송신은 절대시각 기준 주기, Ctrl+C 정상 종료
// ============================================
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include "app_signal.hpp"
#include "can_iface.hpp"

using namespace daq;
using Clock = std::chrono::steady_clock;

static std::mutex g_out;
static bool g_verbose = false;

struct Counters {
    std::atomic<uint64_t> rx{0}, tx{0}, tx_fail{0}, rx_err{0};
};

static void rxTask(CanSocket& can, int ch, Counters& c) {
    can_frame f {};
    while (g_running) {
        auto r = can.read(f);
        if (r == CanSocket::ReadResult::Ok) {
            c.rx++;
            if (g_verbose) {
                std::lock_guard<std::mutex> lk(g_out);
                std::printf("[CH%d] RX ID 0x%03X DLC %u:", ch, f.can_id & CAN_EFF_MASK, f.can_dlc);
                for (int i = 0; i < f.can_dlc && i < 8; i++) std::printf(" %02X", f.data[i]);
                std::printf("\n");
            }
        } else if (r == CanSocket::ReadResult::Error) {
            c.rx_err++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

static void txTask(CanSocket& can, int ch, Counters& c, int period_ms) {
    uint32_t counter = 0;
    const uint32_t id = 0x100 + ch;
    auto next = Clock::now();
    while (g_running) {
        uint8_t data[8] = {(uint8_t)(counter & 0xFF), (uint8_t)((counter >> 8) & 0xFF), 0, 0, 0, 0, 0, 0};
        if (can.send(id, data, 8)) c.tx++;
        else c.tx_fail++;
        counter++;
        next += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next);
    }
}

static void cpuLoadTask() {
    while (g_running) {
        volatile double res = 0.0;
        for (int i = 0; i < 100000; i++) res = res + std::sqrt(std::fabs(std::sin(i) * std::cos(i)));
    }
}

int main(int argc, char** argv) {
    int seconds = 60;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--verbose") == 0) g_verbose = true;
        else seconds = std::atoi(argv[i]);
    }
    installStopHandlers();

    CanSocket can0, can1;
    if (!can0.open("can0") || !can1.open("can1")) {
        std::fprintf(stderr, "CAN interface initialization failed.\n");
        return 1;
    }

    const unsigned cores = std::max(1u, std::thread::hardware_concurrency());
    std::printf("=== 2-CH CAN + CPU load (%u threads), %d s ===\n", cores, seconds);

    Counters c0, c1;
    std::vector<std::thread> th;
    th.emplace_back(rxTask, std::ref(can0), 0, std::ref(c0));
    th.emplace_back(rxTask, std::ref(can1), 1, std::ref(c1));
    th.emplace_back(txTask, std::ref(can0), 0, std::ref(c0), 333);
    th.emplace_back(txTask, std::ref(can1), 1, std::ref(c1), 333);
    for (unsigned i = 0; i < cores; i++) th.emplace_back(cpuLoadTask);

    uint64_t prev[4] = {0, 0, 0, 0};
    for (int s = 0; s < seconds && g_running; s++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint64_t cur[4] = {c0.rx, c1.rx, c0.tx, c1.tx};
        std::lock_guard<std::mutex> lk(g_out);
        std::printf("[%3ds] CAN0 rx %llu/s tx %llu/s | CAN1 rx %llu/s tx %llu/s | tx_fail %llu/%llu rx_err %llu/%llu\n",
                    s + 1,
                    (unsigned long long)(cur[0] - prev[0]), (unsigned long long)(cur[2] - prev[2]),
                    (unsigned long long)(cur[1] - prev[1]), (unsigned long long)(cur[3] - prev[3]),
                    (unsigned long long)c0.tx_fail.load(), (unsigned long long)c1.tx_fail.load(),
                    (unsigned long long)c0.rx_err.load(), (unsigned long long)c1.rx_err.load());
        std::memcpy(prev, cur, sizeof(prev));
    }

    g_running = false;
    for (auto& t : th) t.join();
    std::printf("Test finished. total rx %llu/%llu tx %llu/%llu\n",
                (unsigned long long)c0.rx.load(), (unsigned long long)c1.rx.load(),
                (unsigned long long)c0.tx.load(), (unsigned long long)c1.tx.load());
    return 0;
}
