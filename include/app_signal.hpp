// ============================================================================
// app_signal.hpp - Ctrl+C / SIGTERM(systemd stop) 처리
// ============================================================================
#pragma once

#include <atomic>
#include <csignal>

namespace daq {

inline std::atomic<bool> g_running{true};

inline void onStopSignal(int) { g_running.store(false); }

inline void installStopHandlers() {
    struct sigaction sa {};
    sa.sa_handler = onStopSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // SA_RESTART 없음: 블로킹 호출이 EINTR로 깨어나도록
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

} // namespace daq
