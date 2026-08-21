// Unit tests for ak_v3_driver::protocol, checked byte-for-byte against the
// worked examples in CubeMars AK Series Module Product Manual V3.2.0,
// section 4.4.1 (CAN Port Control Command Examples), motor ID 0x68.
//
// NOTE on MIT-mode: the servo-mode packers below (duty/current/brake/
// velocity/position/pos-vel) reproduce the manual's example bytes exactly.
// The MIT packer's *bit-packing structure* matches the manual's own
// pack_cmd() reference code exactly, and the position field byte-matches
// the manual's worked MIT examples exactly. However, the manual's worked
// MIT velocity/torque example bytes do not reproduce from its own
// documented formula and constants -- plugging AK10-9's documented
// V_MIN/V_MAX/T_MIN/T_MAX into float_to_uint() as specified does not yield
// the exact bytes shown in section 4.4.1's MIT examples, off by more than
// float-rounding noise. This looks like an inconsistency in the manual's
// own published example table rather than in this implementation, but it
// means the MIT tests below check structure/range/round-trip behavior
// (which is verifiable) rather than bit-exact match against that specific
// table (which the manual's own formula doesn't reproduce either).
// Bench-verify MIT mode against real hardware before relying on it.

#include <gtest/gtest.h>

#include "ak_v3_driver/protocol.hpp"
#include "ak_v3_driver/motor_limits.hpp"

using namespace ak_v3_driver::protocol;

namespace
{
constexpr uint8_t kTestCanId = 0x68;
}

TEST(Protocol, BuildCanId)
{
  EXPECT_EQ(buildCanId(MODE_POSITION, 0x68u), 0x00000468u);
  EXPECT_EQ(buildCanId(MODE_MIT, 0x68u), 0x00000868u);
}

TEST(Protocol, PackDuty)
{
  auto f = packDuty(kTestCanId, 0.2f);
  EXPECT_EQ(f.id, buildCanId(MODE_DUTY, kTestCanId));
  ASSERT_EQ(f.dlc, 4);
  EXPECT_EQ(f.data[0], 0x00); EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x4E); EXPECT_EQ(f.data[3], 0x20);

  f = packDuty(kTestCanId, -0.2f);
  EXPECT_EQ(f.data[0], 0xFF); EXPECT_EQ(f.data[1], 0xFF);
  EXPECT_EQ(f.data[2], 0xB1); EXPECT_EQ(f.data[3], 0xE0);
}

TEST(Protocol, PackCurrent)
{
  auto f = packCurrent(kTestCanId, -4.0f);
  ASSERT_EQ(f.dlc, 4);
  EXPECT_EQ(f.data[0], 0xFF); EXPECT_EQ(f.data[1], 0xFF);
  EXPECT_EQ(f.data[2], 0xF0); EXPECT_EQ(f.data[3], 0x60);

  f = packCurrent(kTestCanId, 4.0f);
  EXPECT_EQ(f.data[0], 0x00); EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x0F); EXPECT_EQ(f.data[3], 0xA0);
}

TEST(Protocol, PackCurrentBrake)
{
  auto f = packCurrentBrake(kTestCanId, -4.0f);
  ASSERT_EQ(f.dlc, 4);
  EXPECT_EQ(f.data[0], 0xFF); EXPECT_EQ(f.data[1], 0xFF);
  EXPECT_EQ(f.data[2], 0xF0); EXPECT_EQ(f.data[3], 0x60);
}

TEST(Protocol, PackVelocityErpm)
{
  auto f = packVelocityErpm(kTestCanId, 5000.0f);
  ASSERT_EQ(f.dlc, 4);
  EXPECT_EQ(f.data[0], 0x00); EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x13); EXPECT_EQ(f.data[3], 0x88);

  f = packVelocityErpm(kTestCanId, -5000.0f);
  EXPECT_EQ(f.data[0], 0xFF); EXPECT_EQ(f.data[1], 0xFF);
  EXPECT_EQ(f.data[2], 0xEC); EXPECT_EQ(f.data[3], 0x78);
}

TEST(Protocol, PackPositionDeg)
{
  auto f = packPositionDeg(kTestCanId, 600.0f);
  ASSERT_EQ(f.dlc, 4);
  EXPECT_EQ(f.data[0], 0x00); EXPECT_EQ(f.data[1], 0x5B);
  EXPECT_EQ(f.data[2], 0x8D); EXPECT_EQ(f.data[3], 0x80);

  f = packPositionDeg(kTestCanId, -600.0f);
  EXPECT_EQ(f.data[0], 0xFF); EXPECT_EQ(f.data[1], 0xA4);
  EXPECT_EQ(f.data[2], 0x72); EXPECT_EQ(f.data[3], 0x80);
}

TEST(Protocol, PackPositionVelocity)
{
  auto f = packPositionVelocity(kTestCanId, 1000.0f, 10000.0f, 10000.0f);
  ASSERT_EQ(f.dlc, 8);
  EXPECT_EQ(f.data[0], 0x00); EXPECT_EQ(f.data[1], 0x98);
  EXPECT_EQ(f.data[2], 0x96); EXPECT_EQ(f.data[3], 0x80);
  EXPECT_EQ(f.data[4], 0x03); EXPECT_EQ(f.data[5], 0xE8);
  EXPECT_EQ(f.data[6], 0x03); EXPECT_EQ(f.data[7], 0xE8);

  f = packPositionVelocity(kTestCanId, -1000.0f, -10000.0f, -10000.0f);
  EXPECT_EQ(f.data[0], 0xFF); EXPECT_EQ(f.data[1], 0x67);
  EXPECT_EQ(f.data[2], 0x69); EXPECT_EQ(f.data[3], 0x80);
  EXPECT_EQ(f.data[4], 0xFC); EXPECT_EQ(f.data[5], 0x18);
  EXPECT_EQ(f.data[6], 0xFC); EXPECT_EQ(f.data[7], 0x18);
}

TEST(Protocol, PackSetOrigin)
{
  auto f = packSetOrigin(kTestCanId, false);
  ASSERT_EQ(f.dlc, 1);
  EXPECT_EQ(f.data[0], 0x00);

  f = packSetOrigin(kTestCanId, true);
  EXPECT_EQ(f.data[0], 0x01);
}

TEST(Protocol, PackDisable)
{
  auto f = packDisable(kTestCanId);
  EXPECT_EQ(f.id, buildCanId(MODE_DISABLE, kTestCanId));
  EXPECT_EQ(f.dlc, 0);
}

TEST(Protocol, PackFeedbackConfig)
{
  // bit 15 set -> 0x8000 in DATA[6:7]
  auto f = packFeedbackConfig(kTestCanId, true, false);
  ASSERT_EQ(f.dlc, 8);
  EXPECT_EQ(f.data[6], 0x80);
  EXPECT_EQ(f.data[7], 0x00);

  // bits 15 and 14 set -> 0xC000
  f = packFeedbackConfig(kTestCanId, true, true);
  EXPECT_EQ(f.data[6], 0xC0);
  EXPECT_EQ(f.data[7], 0x00);
}

TEST(Protocol, MitPositionFieldMatchesManualExample)
{
  // Manual 4.4.1 "MIT Velocity Loop", Kd=2, speed=6rad/s, position unset
  // (0 rad): expected DATA[3:4] = 7F FF regardless of the manual's own
  // pack_cmd() clamping-bug side effect of always packing position=0.
  MitLimits limits{-28.0f, 28.0f, -54.0f, 54.0f};
  auto f = packMit(kTestCanId, 0.0f, 6.0f, 0.0f, 2.0f, 0.0f, limits);
  ASSERT_EQ(f.dlc, 8);
  EXPECT_EQ(f.data[3], 0x7F);
  EXPECT_EQ(f.data[4], 0xFF);
}

TEST(Protocol, MitKdFieldMatchesManualExample)
{
  // Same example: Kd=2 -> kd_int = float_to_uint(2, 0, 5, 12) = 0x666
  // DATA[1] low nibble = kd_int>>8, DATA[2] = kd_int & 0xFF
  MitLimits limits{-28.0f, 28.0f, -54.0f, 54.0f};
  auto f = packMit(kTestCanId, 0.0f, 6.0f, 0.0f, 2.0f, 0.0f, limits);
  EXPECT_EQ(f.data[1], 0x06);
  EXPECT_EQ(f.data[2], 0x66);
}

TEST(Protocol, MitRoundTripsThroughLimits)
{
  // Structural/range check: packing at the extremes of a model's declared
  // range should use the full 12/16-bit span (0 and max code), not
  // overflow or silently clamp somewhere in the middle.
  MitLimits limits{-38.0f, 38.0f, -32.0f, 32.0f};  // AK80-8

  auto f_min = packMit(kTestCanId, -12.56f, -38.0f, 0.0f, 0.0f, -32.0f, limits);
  auto f_max = packMit(kTestCanId, 12.56f, 38.0f, 500.0f, 5.0f, 32.0f, limits);
  ASSERT_EQ(f_min.dlc, 8);
  ASSERT_EQ(f_max.dlc, 8);
  // position at P_MIN -> p_int == 0 -> DATA[3]=0x00
  EXPECT_EQ(f_min.data[3], 0x00);
  // position at P_MAX -> p_int == 65535 (max 16-bit code) -> DATA[3]=0xFF
  EXPECT_EQ(f_max.data[3], 0xFF);
}

TEST(Protocol, DecodeStatusFrame)
{
  CanFrame frame;
  frame.id = buildCanId(FUNC_ID_STATUS, kTestCanId);
  frame.dlc = 8;
  // position=100.0deg(int16 1000=0x03E8), speed=2000erpm(int16 200=0x00C8),
  // current=5.0A(int16 500=0x01F4), temp=25, error=0
  frame.data = {0x03, 0xE8, 0x00, 0xC8, 0x01, 0xF4, 25, 0};

  auto decoded = decodeStatusFrame(frame, kTestCanId);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->position_deg, 100.0f);
  EXPECT_FLOAT_EQ(decoded->speed_erpm, 2000.0f);
  EXPECT_FLOAT_EQ(decoded->current_a, 5.0f);
  EXPECT_EQ(decoded->temperature_c, 25);
  EXPECT_EQ(decoded->error_code, 0);
}

TEST(Protocol, DecodeStatusFrameRejectsWrongCanId)
{
  CanFrame frame;
  frame.id = buildCanId(FUNC_ID_STATUS, kTestCanId);
  frame.dlc = 8;
  frame.data = {0, 0, 0, 0, 0, 0, 0, 0};

  EXPECT_FALSE(decodeStatusFrame(frame, kTestCanId + 1).has_value());
}

TEST(Protocol, DecodeExtendedPositionFrame)
{
  CanFrame frame;
  frame.id = buildCanId(FUNC_ID_EXT_POSITION, kTestCanId);
  frame.dlc = 8;
  // int32 position = 1000000 (0.01 deg units) -> 10000.00 deg
  // 1000000 = 0x000F4240
  frame.data = {0x00, 0x0F, 0x42, 0x40, 0, 0, 0, 0};

  auto decoded = decodeExtendedPositionFrame(frame, kTestCanId);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_FLOAT_EQ(decoded->position_deg, 10000.0f);
}

TEST(Protocol, IsStartupFrame)
{
  CanFrame frame;
  frame.id = buildCanId(FUNC_ID_STARTUP, kTestCanId);
  frame.dlc = 4;
  frame.data = {0xFA, 0xFB, 0xFC, 0xFD, 0, 0, 0, 0};
  EXPECT_TRUE(isStartupFrame(frame, kTestCanId));

  frame.data[0] = 0x00;
  EXPECT_FALSE(isStartupFrame(frame, kTestCanId));
}

TEST(Protocol, FaultCodeToString)
{
  EXPECT_EQ(faultCodeToString(0), "none");
  EXPECT_EQ(faultCodeToString(2), "over_current");
  EXPECT_EQ(faultCodeToString(7), "motor_lockup");
  EXPECT_EQ(faultCodeToString(200), "unknown");
}

TEST(Protocol, FloatToUintRoundTrip)
{
  // 12-bit round trip should stay within 1 LSB of the original value.
  const float original = 3.7f;
  const uint32_t coded = floatToUint(original, 0.0f, 5.0f, 12);
  const float decoded = uintToFloat(coded, 0.0f, 5.0f, 12);
  EXPECT_NEAR(decoded, original, 5.0f / 4096.0f);
}

TEST(Protocol, FloatToUintClampsOutOfRange)
{
  EXPECT_EQ(floatToUint(999.0f, 0.0f, 5.0f, 12), floatToUint(5.0f, 0.0f, 5.0f, 12));
  EXPECT_EQ(floatToUint(-999.0f, 0.0f, 5.0f, 12), 0u);
}

TEST(MotorLimits, LookupKnownModelIncludesPolePairsAndGearRatio)
{
  const auto spec = ak_v3_driver::lookupMotorModel("AK80-9");
  ASSERT_TRUE(spec.has_value());
  EXPECT_FLOAT_EQ(spec->velocity_max_rad_s, 65.0f);
  EXPECT_FLOAT_EQ(spec->torque_max_nm, 18.0f);
  // Placeholder values until filled in with real hardware numbers in
  // motor_limits.cpp -- this test documents that expectation rather than
  // asserting any particular "correct" value.
  EXPECT_FLOAT_EQ(spec->pole_pairs, 1.0f);
  EXPECT_FLOAT_EQ(spec->gear_ratio, 1.0f);
}

TEST(MotorLimits, LookupUnknownModelReturnsNullopt)
{
  EXPECT_FALSE(ak_v3_driver::lookupMotorModel("NotARealMotor").has_value());
}
