// AK-V3 CAN protocol encode/decode.
//
// This header has NO dependency on rclcpp or any ROS type. It is meant to be
// unit-testable in isolation (see test/test_protocol.cpp) and reusable from
// any transport (SocketCAN here, but the encode/decode logic doesn't care).
//
// Byte layouts and scaling factors are taken directly from the CubeMars
// AK Series Module Product Manual, V3.2.0, section 4.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace ak_v3_driver
{
namespace protocol
{

// --- Servo-mode control mode IDs (manual section 4.1) ---
constexpr uint8_t MODE_DUTY = 0;
constexpr uint8_t MODE_CURRENT = 1;
constexpr uint8_t MODE_CURRENT_BRAKE = 2;
constexpr uint8_t MODE_VELOCITY = 3;
constexpr uint8_t MODE_POSITION = 4;
constexpr uint8_t MODE_SET_ORIGIN = 5;
constexpr uint8_t MODE_POSITION_VELOCITY = 6;
constexpr uint8_t MODE_MIT = 8;
constexpr uint8_t MODE_DISABLE = 15;
constexpr uint8_t MODE_FEEDBACK_CONFIG = 16;

// --- Feedback / function IDs the motor uses when *sending* frames
// (manual section 4.3.1) ---
constexpr uint8_t FUNC_ID_STATUS = 0x29;         // periodic status frame
constexpr uint8_t FUNC_ID_EXT_POSITION = 0x2A;   // optional 4-byte position frame
constexpr uint8_t FUNC_ID_STARTUP = 0x2C;        // one-shot servo-mode-entered frame

// MIT mode shares fixed position / Kp / Kd ranges across all AK-V3 motor
// models (manual section 4.2); only velocity and torque ranges are
// model-specific (see motor_limits.hpp).
constexpr float MIT_POSITION_MIN_RAD = -12.56f;
constexpr float MIT_POSITION_MAX_RAD = 12.56f;
constexpr float MIT_KP_MIN = 0.0f;
constexpr float MIT_KP_MAX = 500.0f;
constexpr float MIT_KD_MIN = 0.0f;
constexpr float MIT_KD_MAX = 5.0f;

/// A raw CAN frame: 29-bit extended identifier + up to 8 data bytes.
struct CanFrame
{
  uint32_t id = 0;        // 29-bit extended CAN identifier
  uint8_t dlc = 0;        // number of valid bytes in `data`
  std::array<uint8_t, 8> data{};
};

/// Per-motor-model limits needed to pack a valid MIT-mode frame.
/// Position / Kp / Kd ranges are fixed (see MIT_* constants above);
/// velocity and torque ranges differ per model, see motor_limits.hpp.
struct MitLimits
{
  float velocity_min_rad_s;
  float velocity_max_rad_s;
  float torque_min_nm;
  float torque_max_nm;
};

/// Decoded contents of a 0x29 status frame.
struct StatusFrame
{
  float position_deg;   // -3200 .. 3200 deg (int16 * 0.1)
  float speed_erpm;      // electrical RPM (int16 * 10)
  float current_a;       // -60 .. 60 A (int16 * 0.01)
  int8_t temperature_c;  // driver board temperature
  uint8_t error_code;    // 0 = no fault, see decodeFaultCode()
};

/// Decoded contents of an optional 0x2A extended position frame.
struct ExtendedPositionFrame
{
  float position_deg;   // int32 * 0.0001 -> range +-21,474,836.47 deg
};

/// Human-readable fault code text, for diagnostics. Returns "unknown" for
/// any value not documented in the manual (section 4.3.2.1's mc_fault_code
/// enum -- note the manual's 0x29 status byte uses a shorter 0-7 fault set;
/// this covers that shorter set, which is what the status frame reports).
std::string faultCodeToString(uint8_t code);

/// Build the 29-bit extended CAN identifier for a given control/function
/// mode and driver (motor) ID: (mode_id << 8) | can_id.
uint32_t buildCanId(uint8_t mode_id, uint8_t can_id);

// --- Servo-mode packers ---
// `can_id` is this motor's driver ID (0-255). All packers return a frame
// ready to hand to CanSocket::send().

/// duty: -1.0 .. 1.0
CanFrame packDuty(uint8_t can_id, float duty);

/// current: amps, Iq current loop (torque = Iq * Kt)
CanFrame packCurrent(uint8_t can_id, float current_a);

/// current: amps, braking current (holds position)
CanFrame packCurrentBrake(uint8_t can_id, float current_a);

/// erpm: electrical RPM, -100000 .. 100000
CanFrame packVelocityErpm(uint8_t can_id, float erpm);

/// position_deg: degrees, -36000 .. 36000
CanFrame packPositionDeg(uint8_t can_id, float position_deg);

/// permanent: false = temporary origin (erased on power loss),
///            true  = permanent origin (saved to flash)
CanFrame packSetOrigin(uint8_t can_id, bool permanent);

/// Combined position + velocity + acceleration command.
/// position_deg: -36000 .. 36000, speed_erpm/accel_erpm_s2: see manual 4.1.7
CanFrame packPositionVelocity(
  uint8_t can_id, float position_deg, float speed_erpm, float accel_erpm_s2);

/// No payload; motor echoes DATA[7]=0x77 on the next 0x29 frame to confirm.
CanFrame packDisable(uint8_t can_id);

/// add_extended_position_frame: enables the optional 0x2A frame (bit 15)
/// single_turn_mode: position reported 0-360 deg instead of multi-turn (bit 14)
/// NOTE: this writes to the motor's flash memory -- do not call frequently.
CanFrame packFeedbackConfig(
  uint8_t can_id, bool add_extended_position_frame, bool single_turn_mode);

/// Force control (MIT) mode. position_rad/velocity_rad_s/torque_nm are
/// clamped to `limits` (and to the fixed MIT_POSITION_*/KP_*/KD_* ranges)
/// before packing -- out-of-range inputs are silently clamped by the motor
/// too, so clamping here just means you find out about it in ROS logs
/// instead of on the bus.
CanFrame packMit(
  uint8_t can_id, float position_rad, float velocity_rad_s,
  float kp, float kd, float torque_nm, const MitLimits & limits);

// --- Feedback decoders ---

/// Returns std::nullopt if `frame` is not a 0x29 status frame for `can_id`
/// (wrong function ID, wrong driver ID, or wrong DLC).
std::optional<StatusFrame> decodeStatusFrame(const CanFrame & frame, uint8_t can_id);

/// Returns std::nullopt if `frame` is not a 0x2A extended position frame
/// for `can_id`.
std::optional<ExtendedPositionFrame> decodeExtendedPositionFrame(
  const CanFrame & frame, uint8_t can_id);

/// True if `frame` is the one-shot 0x2C servo-mode-entered frame for `can_id`.
bool isStartupFrame(const CanFrame & frame, uint8_t can_id);

// --- Bit-field helpers used by MIT packing (manual section 4.2 / 4.3.2.2) ---

/// Converts a float in [x_min, x_max] to an unsigned integer with `bits`
/// bits of resolution. Input is clamped to range first.
uint32_t floatToUint(float x, float x_min, float x_max, unsigned bits);

/// Inverse of floatToUint.
float uintToFloat(uint32_t x_int, float x_min, float x_max, unsigned bits);

}  // namespace protocol
}  // namespace ak_v3_driver
