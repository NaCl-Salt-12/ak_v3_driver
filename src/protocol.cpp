#include "ak_v3_driver/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ak_v3_driver {
namespace protocol {

namespace {

// --- big-endian buffer helpers, matching the manual's buffer_append_* /
// buffer_get_* functions (section 4.1 / 4.3.2.2) ---
// These write/read multi-byte integers into/from the 8-byte CAN data
// payload in big-endian (most-significant-byte-first) order, and advance
// the caller-owned `idx` cursor by the number of bytes written.

/// Appends a 32-bit signed integer to `buf` at `idx`, big-endian, and
/// advances `idx` by 4.
void appendInt32(std::array<uint8_t, 8> &buf, size_t &idx, int32_t value) {
  buf[idx++] = static_cast<uint8_t>(value >> 24);
  buf[idx++] = static_cast<uint8_t>(value >> 16);
  buf[idx++] = static_cast<uint8_t>(value >> 8);
  buf[idx++] = static_cast<uint8_t>(value);
}

/// Appends a 16-bit signed integer to `buf` at `idx`, big-endian, and
/// advances `idx` by 2.
void appendInt16(std::array<uint8_t, 8> &buf, size_t &idx, int16_t value) {
  buf[idx++] = static_cast<uint8_t>(value >> 8);
  buf[idx++] = static_cast<uint8_t>(value);
}

/// Appends a 16-bit unsigned integer to `buf` at `idx`, big-endian, and
/// advances `idx` by 2.
void appendUint16(std::array<uint8_t, 8> &buf, size_t &idx, uint16_t value) {
  buf[idx++] = static_cast<uint8_t>(value >> 8);
  buf[idx++] = static_cast<uint8_t>(value);
}

/// Reads a big-endian 16-bit signed integer from `buf` starting at `idx`.
/// Does not advance any cursor (caller passes explicit offsets).
int16_t getInt16(const std::array<uint8_t, 8> &buf, size_t idx) {
  return static_cast<int16_t>((static_cast<uint16_t>(buf[idx]) << 8) |
                              buf[idx + 1]);
}

/// Reads a big-endian 32-bit signed integer from `buf` starting at `idx`.
int32_t getInt32(const std::array<uint8_t, 8> &buf, size_t idx) {
  return static_cast<int32_t>((static_cast<uint32_t>(buf[idx]) << 24) |
                              (static_cast<uint32_t>(buf[idx + 1]) << 16) |
                              (static_cast<uint32_t>(buf[idx + 2]) << 8) |
                              static_cast<uint32_t>(buf[idx + 3]));
}

/// Constructs a CanFrame from a mode/function ID, this motor's can_id, an
/// 8-byte data buffer, and the number of valid bytes (dlc) in that buffer.
/// The CAN ID itself is built from (mode_id, can_id) via buildCanId().
CanFrame makeFrame(uint8_t mode_id, uint8_t can_id,
                   const std::array<uint8_t, 8> &data, uint8_t dlc) {
  CanFrame frame;
  frame.id = buildCanId(mode_id, can_id);
  frame.dlc = dlc;
  frame.data = data;
  return frame;
}

} // namespace

/// Builds the 29-bit extended CAN identifier used by the AK-V3 protocol:
/// the high byte carries the mode/function ID and the low byte carries the
/// target motor's can_id, so a single arbitration ID encodes both "what
/// kind of message" and "which motor".
uint32_t buildCanId(uint8_t mode_id, uint8_t can_id) {
  return (static_cast<uint32_t>(mode_id) << 8) | can_id;
}

/// Converts a float in [x_min, x_max] to an unsigned integer with `bits`
/// bits of resolution. Input is clamped to range first, so out-of-range
/// values saturate rather than wrapping/overflowing. Used by packMit() to
/// quantize position/velocity/kp/kd/torque into the fixed-width fields the
/// MIT-mode CAN frame requires.
uint32_t floatToUint(float x, float x_min, float x_max, unsigned bits) {
  x = std::min(std::max(x, x_min), x_max);
  const float span = x_max - x_min;
  const float scale = static_cast<float>(1u << bits) / span;
  return static_cast<uint32_t>((x - x_min) * scale);
}

/// Inverse of floatToUint: expands a `bits`-wide unsigned integer back to
/// a float in [x_min, x_max]. (Not currently used for decoding status
/// frames, but kept alongside floatToUint as its counterpart.)
float uintToFloat(uint32_t x_int, float x_min, float x_max, unsigned bits) {
  const float span = x_max - x_min;
  return static_cast<float>(x_int) * span / static_cast<float>(1u << bits) +
         x_min;
}

/// Maps a raw motor fault/error code (as reported in status frame byte 7)
/// to a short, human-readable string for logging/diagnostics.
std::string faultCodeToString(uint8_t code) {
  switch (code) {
  case 0:
    return "none";
  case 1:
    return "motor_over_temperature";
  case 2:
    return "over_current";
  case 3:
    return "over_voltage";
  case 4:
    return "under_voltage";
  case 5:
    return "encoder_fault";
  case 6:
    return "mosfet_over_temperature";
  case 7:
    return "motor_lockup";
  default:
    return "unknown";
  }
}

// ---------------------------------------------------------------------------
// Servo-mode packers
// ---------------------------------------------------------------------------
// `can_id` is this motor's driver ID (0-255). Each packer builds an 8-byte
// (or shorter) big-endian payload per the AK-V3 manual's spec for that
// control mode and wraps it in a CanFrame ready to hand to
// CanSocket::send(). Scale factors below (e.g. *100000, *1000, *10000)
// convert SI/physical units into the fixed-point integer representation
// the protocol transmits on the wire.

/// Packs a duty-cycle (open-loop PWM) command.
/// duty: -1.0 .. 1.0, sent as duty * 100000 (int32).
CanFrame packDuty(uint8_t can_id, float duty) {
  std::array<uint8_t, 8> buf{};
  size_t idx = 0;
  appendInt32(buf, idx, static_cast<int32_t>(duty * 100000.0f));
  return makeFrame(MODE_DUTY, can_id, buf, static_cast<uint8_t>(idx));
}

/// Packs a closed-loop current (Iq/torque) command.
/// current_a: amps, Iq current loop (torque = Iq * Kt), sent as
/// current_a * 1000 (int32, milliamp resolution).
CanFrame packCurrent(uint8_t can_id, float current_a) {
  std::array<uint8_t, 8> buf{};
  size_t idx = 0;
  appendInt32(buf, idx, static_cast<int32_t>(current_a * 1000.0f));
  return makeFrame(MODE_CURRENT, can_id, buf, static_cast<uint8_t>(idx));
}

/// Packs a braking-current command (holds position against external load).
/// current_a: amps, always a positive magnitude in the protocol -- there is
/// no direction to invert here, unlike packCurrent().
CanFrame packCurrentBrake(uint8_t can_id, float current_a) {
  std::array<uint8_t, 8> buf{};
  size_t idx = 0;
  appendInt32(buf, idx, static_cast<int32_t>(current_a * 1000.0f));
  return makeFrame(MODE_CURRENT_BRAKE, can_id, buf, static_cast<uint8_t>(idx));
}

/// Packs a closed-loop velocity command.
/// erpm: electrical RPM, -100000 .. 100000, sent directly as int32 (no
/// scaling -- the wire unit for this mode already is ERPM).
CanFrame packVelocityErpm(uint8_t can_id, float erpm) {
  std::array<uint8_t, 8> buf{};
  size_t idx = 0;
  appendInt32(buf, idx, static_cast<int32_t>(erpm));
  return makeFrame(MODE_VELOCITY, can_id, buf, static_cast<uint8_t>(idx));
}

/// Packs a closed-loop position command.
/// position_deg: degrees, -36000 .. 36000, sent as position_deg * 10000
/// (int32, 0.0001-degree resolution).
CanFrame packPositionDeg(uint8_t can_id, float position_deg) {
  std::array<uint8_t, 8> buf{};
  size_t idx = 0;
  appendInt32(buf, idx, static_cast<int32_t>(position_deg * 10000.0f));
  return makeFrame(MODE_POSITION, can_id, buf, static_cast<uint8_t>(idx));
}

/// Packs a "set origin" command, zeroing the position sensor at its
/// current physical position.
/// permanent: false = temporary origin (erased on power loss),
///            true  = permanent origin (saved to flash).
/// Single-byte payload: 0 or 1.
CanFrame packSetOrigin(uint8_t can_id, bool permanent) {
  std::array<uint8_t, 8> buf{};
  buf[0] = permanent ? 1 : 0;
  return makeFrame(MODE_SET_ORIGIN, can_id, buf, 1);
}

/// Packs a combined position + velocity + acceleration command.
/// position_deg: -36000 .. 36000, sent as position_deg * 10000 (int32).
/// speed_erpm / accel_erpm_s2: see manual section 4.1.7.
CanFrame packPositionVelocity(uint8_t can_id, float position_deg,
                              float speed_erpm, float accel_erpm_s2) {
  std::array<uint8_t, 8> buf{};
  size_t idx = 0;
  appendInt32(buf, idx, static_cast<int32_t>(position_deg * 10000.0f));
  // Wire units are 1/10th of the physical units (manual 4.1.7 / worked
  // example in 4.4.1: 10000 ERPM on the wire as 0x03E8 == 1000 == 10000/10).
  appendInt16(buf, idx, static_cast<int16_t>(speed_erpm / 10.0f));
  appendInt16(buf, idx, static_cast<int16_t>(accel_erpm_s2 / 10.0f));
  return makeFrame(MODE_POSITION_VELOCITY, can_id, buf,
                   static_cast<uint8_t>(idx));
}

/// Packs a disable ("coast the motor") command.
/// No payload; the motor echoes DATA[7]=0x77 on its next 0x29 status frame
/// to confirm the disable took effect (not checked by this driver).
CanFrame packDisable(uint8_t can_id) {
  std::array<uint8_t, 8> buf{};
  return makeFrame(MODE_DISABLE, can_id, buf, 0);
}

/// Packs a feedback-configuration command, toggling optional status
/// reporting features on the motor controller.
/// add_extended_position_frame: enables the optional 0x2A frame (bit 15).
/// single_turn_mode: position reported as 0-360 deg instead of multi-turn
/// (bit 14).
/// The two flag bits are packed into a uint16 written at buffer offset 6
/// (the manual reserves bytes 0-5 for this command; only the trailing
/// uint16 carries data here).
/// NOTE: this writes to the motor's flash memory -- do not call frequently.
CanFrame packFeedbackConfig(uint8_t can_id, bool add_extended_position_frame,
                            bool single_turn_mode) {
  std::array<uint8_t, 8> buf{};
  uint16_t param = 0;
  if (add_extended_position_frame) {
    param |= 0x8000; // bit 15
  }
  if (single_turn_mode) {
    param |= 0x4000; // bit 14
  }
  size_t idx = 6;
  appendUint16(buf, idx, param);
  return makeFrame(MODE_FEEDBACK_CONFIG, can_id, buf,
                   static_cast<uint8_t>(idx));
}

/// Packs an MIT-mode (impedance/force control) command: a full-state
/// command combining desired position, desired velocity, position/velocity
/// gains (kp/kd), and a feed-forward torque, all quantized into fixed-width
/// bit fields and bit-packed into the 8-byte payload.
///
/// position_rad/velocity_rad_s/torque_nm are clamped to `limits` (and to
/// the fixed MIT_POSITION_*/KP_*/KD_* ranges) before packing -- out-of-
/// range inputs are silently clamped by the motor too, so clamping here
/// just means you find out about it in ROS logs instead of on the bus.
CanFrame packMit(uint8_t can_id, float position_rad, float velocity_rad_s,
                 float kp, float kd, float torque_nm, const MitLimits &limits) {
  // Quantize each field to its protocol-defined bit width via floatToUint,
  // which clamps to range first.
  const uint32_t p_int =
      floatToUint(position_rad, MIT_POSITION_MIN_RAD, MIT_POSITION_MAX_RAD, 16);
  const uint32_t v_int = floatToUint(velocity_rad_s, limits.velocity_min_rad_s,
                                     limits.velocity_max_rad_s, 12);
  const uint32_t kp_int = floatToUint(kp, MIT_KP_MIN, MIT_KP_MAX, 12);
  const uint32_t kd_int = floatToUint(kd, MIT_KD_MIN, MIT_KD_MAX, 12);
  const uint32_t t_int =
      floatToUint(torque_nm, limits.torque_min_nm, limits.torque_max_nm, 12);

  // Bit-pack kp(12) | kd(12) | position(16) | velocity(12) | torque(12)
  // across the 8 payload bytes, per the AK-V3 MIT-mode frame layout.
  std::array<uint8_t, 8> buf{};
  buf[0] = static_cast<uint8_t>(kp_int >> 4);
  buf[1] = static_cast<uint8_t>(((kp_int & 0xF) << 4) | (kd_int >> 8));
  buf[2] = static_cast<uint8_t>(kd_int & 0xFF);
  buf[3] = static_cast<uint8_t>(p_int >> 8);
  buf[4] = static_cast<uint8_t>(p_int & 0xFF);
  buf[5] = static_cast<uint8_t>(v_int >> 4);
  buf[6] = static_cast<uint8_t>(((v_int & 0xF) << 4) | (t_int >> 8));
  buf[7] = static_cast<uint8_t>(t_int & 0xFF);
  return makeFrame(MODE_MIT, can_id, buf, 8);
}

// ---------------------------------------------------------------------------
// Frame decoders
// ---------------------------------------------------------------------------
// Each decoder validates that the incoming CanFrame is the expected type
// for the given can_id (correct function ID and minimum DLC) before
// interpreting its payload, returning std::nullopt on any mismatch so
// callers can cheaply try each decoder in turn.

/// Decodes a periodic motor status frame (function ID 0x29): position,
/// speed, current, temperature, and fault code.
/// Returns std::nullopt if `frame` is not a 0x29 status frame for `can_id`
/// (wrong function ID, wrong driver ID, or too few data bytes).
std::optional<StatusFrame> decodeStatusFrame(const CanFrame &frame,
                                             uint8_t can_id) {
  if (frame.id != buildCanId(FUNC_ID_STATUS, can_id) || frame.dlc < 8) {
    return std::nullopt;
  }
  StatusFrame out;
  // Each field is a big-endian int16 with a fixed scale factor per the
  // manual's status frame layout.
  out.position_deg = static_cast<float>(getInt16(frame.data, 0)) * 0.1f;
  out.speed_erpm = static_cast<float>(getInt16(frame.data, 2)) * 10.0f;
  out.current_a = static_cast<float>(getInt16(frame.data, 4)) * 0.01f;
  out.temperature_c = static_cast<int8_t>(frame.data[6]);
  out.error_code = frame.data[7];
  return out;
}

/// Decodes the optional extended (multi-turn) position frame (function ID
/// 0x2A), only sent if enabled via packFeedbackConfig().
/// Returns std::nullopt if `frame` is not a 0x2A extended position frame
/// for `can_id`.
std::optional<ExtendedPositionFrame>
decodeExtendedPositionFrame(const CanFrame &frame, uint8_t can_id) {
  if (frame.id != buildCanId(FUNC_ID_EXT_POSITION, can_id) || frame.dlc < 4) {
    return std::nullopt;
  }
  ExtendedPositionFrame out;
  // Big-endian int32, 0.01-degree resolution, wide enough to represent
  // multi-turn (unwrapped) position.
  out.position_deg = static_cast<float>(getInt32(frame.data, 0)) * 0.01f;
  return out;
}

/// Checks whether `frame` is the one-shot startup/"servo mode entered"
/// frame (function ID 0x2C) that the motor controller sends for `can_id`
/// once when it boots into servo mode. Payload is a fixed 4-byte marker
/// (0xFA 0xFB 0xFC 0xFD); this is currently just detected, not consumed
/// into any queue elsewhere in the driver.
bool isStartupFrame(const CanFrame &frame, uint8_t can_id) {
  static constexpr std::array<uint8_t, 4> kExpected{0xFA, 0xFB, 0xFC, 0xFD};
  if (frame.id != buildCanId(FUNC_ID_STARTUP, can_id) || frame.dlc < 4) {
    return false;
  }
  return std::equal(kExpected.begin(), kExpected.end(), frame.data.begin());
}

} // namespace protocol
} // namespace ak_v3_driver
