// Host unit tests for sensor_parser.hpp (no hardware needed): make test
#include <cmath>
#include <cstdio>
#include <cstring>
#include "sensor_parser.hpp"

using namespace daq;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

static can_frame mk(uint32_t id, std::initializer_list<uint8_t> bytes) {
    can_frame f{};
    f.can_id = id;
    f.can_dlc = (uint8_t)bytes.size();
    int i = 0;
    for (auto b : bytes) f.data[i++] = b;
    return f;
}

int main() {
    // --- encoder: 24-bit two's complement ---
    CHECK(*decodeEncoderCounts(mk(ID_ENCODER, {0x01, 0x00, 0x00, 0x00})) == 1);
    CHECK(*decodeEncoderCounts(mk(ID_ENCODER, {0xFF, 0xFF, 0xFF, 0x00})) == -1);
    CHECK(*decodeEncoderCounts(mk(ID_ENCODER, {0x00, 0x00, 0x80, 0x00})) == -8388608);
    CHECK(*decodeEncoderCounts(mk(ID_ENCODER, {0xFF, 0xFF, 0x7F, 0x00})) == 8388607);
    // already sign-extended 32-bit negative
    CHECK(*decodeEncoderCounts(mk(ID_ENCODER, {0xFE, 0xFF, 0xFF, 0xFF})) == -2);
    // 3-byte frame
    CHECK(*decodeEncoderCounts(mk(ID_ENCODER, {0xFF, 0xFF, 0xFF})) == -1);
    // too short
    CHECK(!decodeEncoderCounts(mk(ID_ENCODER, {0x01, 0x02})));

    // --- distance: no int32 overflow ---
    CHECK(encoderToDistance(9999) == 1682);
    CHECK(encoderToDistance(-9999) == -1682);
    CHECK(encoderToDistance(8388607) == (int32_t)(8388607LL * 1682 / 9999));   // 1410958
    CHECK(encoderToDistance(-8388608) == (int32_t)(-8388608LL * 1682 / 9999));
    CHECK(encoderToDistance(2000000) == 336433);  // old int32 code overflowed here

    // --- inclination ---
    // X = 1234 (12.34 deg), Y = -500 (-5.00 deg)
    auto f = mk(ID_INCL_A, {0xD2, 0x04, 0x00, 0x00, 0x0C, 0xFE, 0xFF, 0xFF});
    auto inc = decodeInclination(f);
    CHECK(inc && inc->x_cdeg == 1234 && inc->y_cdeg == -500);
    CHECK(!decodeInclination(mk(ID_INCL_A, {1, 2, 3, 4})));       // DLC < 8
    CHECK(normalizeCentiDeg(27000) == -9000);
    CHECK(normalizeCentiDeg(-27000) == 9000);
    CHECK(normalizeCentiDeg(18000) == 18000);
    CHECK(normalizeCentiDeg(36000 + 100) == 100);

    // --- filter: first sample passes through, then converges ---
    InclinationFilter flt;
    CHECK(flt.update(1000) == 1000);
    int32_t y = 0;
    for (int i = 0; i < 50; i++) y = flt.update(2000);
    CHECK(std::abs(y - 2000) <= 1);

    // --- frame type filtering ---
    can_frame ext = mk(ID_ENCODER | CAN_EFF_FLAG, {1, 0, 0, 0});
    CHECK(!standardId(ext));
    can_frame rtr = mk(ID_ENCODER | CAN_RTR_FLAG, {});
    CHECK(!standardId(rtr));
    CHECK(*standardId(mk(ID_INCL_B, {})) == ID_INCL_B);

    // --- ADC conversion (5 V reference) ---
    CHECK(adcRawToVolts(0, 5.0) == 0.0);
    CHECK(std::fabs(adcRawToVolts(0x40000000u, 5.0) - 2.5) < 1e-9);
    CHECK(std::fabs(adcRawToVolts(0xC0000000u, 5.0) + 2.5) < 1e-9);
    CHECK(std::fabs(adcRawToVolts(0x7FFFFFFFu, 5.0) - 5.0) < 1e-6);

    if (g_fail == 0) std::printf("test_sensor_parser: all passed\n");
    return g_fail ? 1 : 0;
}
