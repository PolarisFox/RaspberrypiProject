// ============================================================================
// can_iface.hpp - SocketCAN RAII 래퍼
// - 수신 타임아웃(SO_RCVTIMEO)으로 종료 신호에 반응 가능
// - 인터페이스 이름 길이 검사, 모든 시스템콜 결과 확인
// - read/write 는 서로 다른 스레드에서 동시에 호출해도 안전(커널 소켓)
// ============================================================================
#pragma once

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace daq {

class CanSocket {
public:
    enum class ReadResult { Ok, Timeout, Error };

    CanSocket() = default;
    ~CanSocket() { close(); }
    CanSocket(const CanSocket&) = delete;
    CanSocket& operator=(const CanSocket&) = delete;

    bool open(const std::string& ifname, int rx_timeout_ms = 200) {
        close();
        if (ifname.empty() || ifname.size() >= IFNAMSIZ) {
            std::fprintf(stderr, "[CAN] invalid interface name '%s'\n", ifname.c_str());
            return false;
        }
        fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) { perror("[CAN] socket"); return false; }

        struct ifreq ifr {};
        std::memcpy(ifr.ifr_name, ifname.c_str(), ifname.size() + 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            std::fprintf(stderr, "[CAN] interface %s not found: %s\n", ifname.c_str(), std::strerror(errno));
            close();
            return false;
        }

        struct timeval tv { rx_timeout_ms / 1000, (rx_timeout_ms % 1000) * 1000 };
        if (::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            perror("[CAN] SO_RCVTIMEO");
            close();
            return false;
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::fprintf(stderr, "[CAN] bind %s: %s\n", ifname.c_str(), std::strerror(errno));
            close();
            return false;
        }
        name_ = ifname;
        return true;
    }

    ReadResult read(can_frame& frame) {
        ssize_t n = ::read(fd_, &frame, sizeof(frame));
        if (n == (ssize_t)sizeof(frame)) return ReadResult::Ok;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return ReadResult::Timeout;
        return ReadResult::Error;
    }

    bool send(uint32_t id, const uint8_t* data, uint8_t len) {
        if (len > CAN_MAX_DLEN) return false;
        can_frame frame {};
        frame.can_id = id;
        frame.can_dlc = len;
        if (len) std::memcpy(frame.data, data, len);
        return ::write(fd_, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool isOpen() const { return fd_ >= 0; }
    const std::string& name() const { return name_; }

private:
    int fd_ = -1;
    std::string name_;
};

} // namespace daq
