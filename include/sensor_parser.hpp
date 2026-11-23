// ============================================================================
// sensor_parser.hpp
// CAN 센서 프레임 디코딩과 ADC 변환 (하드웨어 의존성 없는 순수 함수)
// - PC에서 단위 테스트 가능 (tests/test_sensor_parser.cpp)
// ============================================================================
#pragma once

#include <cstdint>
#include <optional>
#include <linux/can.h>

namespace daq {

// ---- CAN ID ----------------------------------------------------------------
constexpr uint32_t ID_ENCODER = 0x2A0;  // 엔코더 (RM9000)
constexpr uint32_t ID_INCL_A  = 0x28A;  // 경사계
constexpr uint32_t ID_INCL_B  = 0x28C;  // 경사계 (대체 ID)

// ---- 엔코더 -> 거리 환산 (원본 공식 그대로: dist = counts * 1682 / 9999) ----
constexpr int64_t ENC_DIST_NUM = 1682;
constexpr int64_t ENC_DIST_DEN = 9999;

// 표준(11bit) 데이터 프레임이면 ID, 아니면(확장/RTR/에러 프레임) nullopt
inline std::optional<uint32_t> standardId(const can_frame& f) {
    if (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) return std::nullopt;
    return f.can_id & CAN_SFF_MASK;
}

// 리틀엔디언 32bit (endianness/union 트릭에 의존하지 않음)
inline int32_t le32(const uint8_t* d) {
    uint32_t u = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                 ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    return (int32_t)u;
}

// 엔코더 카운트. 24bit 2의 보수(0x800000~0xFFFFFF -> 음수)와
// 이미 부호확장된 32bit 음수 모두 처리. DLC가 3 미만이면 무효.
inline std::optional<int32_t> decodeEncoderCounts(const can_frame& f) {
    if (f.can_dlc < 3) return std::nullopt;
    uint8_t b[4] = { f.data[0], f.data[1], f.data[2], f.can_dlc >= 4 ? f.data[3] : (uint8_t)0 };
    int32_t v = le32(b);
    if (v >= 0x00800000 && v <= 0x00FFFFFF) v -= 0x01000000;
    return v;
}

// int64로 계산해서 오버플로 방지 (int32로는 |counts| > 약 127만에서 넘침)
inline int32_t encoderToDistance(int32_t counts) {
    return (int32_t)(((int64_t)counts * ENC_DIST_NUM) / ENC_DIST_DEN);
}

// 경사계 각도: 0.01° 단위, [-18000, 18000]으로 정규화
inline int32_t normalizeCentiDeg(int32_t v) {
    v %= 36000;
    if (v > 18000)  v -= 36000;
    if (v < -18000) v += 36000;
    return v;
}

struct InclinationRaw { int32_t x_cdeg; int32_t y_cdeg; };

// DLC 8 미만이면 무효 (X: data[0..3], Y: data[4..7])
inline std::optional<InclinationRaw> decodeInclination(const can_frame& f) {
    if (f.can_dlc < 8) return std::nullopt;
    return InclinationRaw{ normalizeCentiDeg(le32(&f.data[0])),
                           normalizeCentiDeg(le32(&f.data[4])) };
}

// 원본과 동일한 1차 IIR 저역통과 필터
//   y[n] = (2x[n-2] + 2x[n-1] + 4x[n] + 8y[n-1]) / 16
// (주석에 "가중 이동평균"이라 되어 있었지만 이전 출력을 다시 쓰므로 IIR입니다.)
// 주의: ±180° 경계를 넘나드는 값에는 쓰지 마세요(평균이 엉뚱해짐).
class InclinationFilter {
public:
    int32_t update(int32_t x) {
        if (!init_) {
            buf_[0] = buf_[1] = buf_[2] = x;
            prev_ = x;
            init_ = true;
        } else {
            buf_[2] = buf_[1];
            buf_[1] = buf_[0];
            buf_[0] = x;
        }
        int64_t y = (2LL * buf_[2] + 2LL * buf_[1] + 4LL * buf_[0] + 8LL * prev_) / 16;
        prev_ = (int32_t)y;
        return prev_;
    }
    void reset() { init_ = false; }
private:
    bool init_ = false;
    int32_t buf_[3] = {0, 0, 0};
    int32_t prev_ = 0;
};

// ADS1263 ADC1 원시값(32bit 2의 보수) -> 전압
// 드라이버는 REFMUX=0x24 (AVDD/AVSS 기준, 약 5V)로 설정하므로 기본 vref는 5.0.
// 정확도가 필요하면 AVDD-AVSS를 멀티미터로 재서 --vref 로 넘기세요.
inline double adcRawToVolts(uint32_t raw, double vref) {
    return (double)(int32_t)raw / 2147483648.0 * vref;
}

} // namespace daq
