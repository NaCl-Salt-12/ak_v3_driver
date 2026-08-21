// Per-motor-model parameters, taken from the CubeMars AK Series Module
// Product Manual V3.2.0, section 4.2, PLUS pole_pairs/gear_ratio.
//
// Position range and Kp/Kd ranges are shared across all AK-V3 models (see
// MIT_POSITION_*/MIT_KP_*/MIT_KD_* in protocol.hpp); only velocity/torque
// range, KV, and torque coefficient differ per model -- those come
// straight from the manual and are verified.
//
// pole_pairs and gear_ratio are NOT in the manual (it doesn't list them
// per model). The values in motor_limits.cpp are PLACEHOLDERS (1.0 each)
// -- fill in the real values for your specific motors/gearboxes there
// before relying on velocity or position-velocity mode; MIT mode and the
// other servo modes (duty/current/position) don't depend on these two
// fields at all.
#pragma once

#include <optional>
#include <string>

namespace ak_v3_driver
{

struct MotorModelSpec
{
  float kv;
  float torque_coefficient_nm_per_a;   // Kt: T = Kt * Iq
  float velocity_min_rad_s;
  float velocity_max_rad_s;
  float torque_min_nm;
  float torque_max_nm;
  float pole_pairs;   // PLACEHOLDER -- fill in from your motor's datasheet
  float gear_ratio;   // PLACEHOLDER -- fill in from your gearbox (1.0 if direct-drive)
};

/// Looks up a motor model by the manual's model name (e.g. "AK80-9").
/// Returns std::nullopt if the name isn't recognized -- callers should
/// treat that as a configuration error (unknown `motor_type` parameter),
/// not silently fall back to a default.
std::optional<MotorModelSpec> lookupMotorModel(const std::string & motor_type);

}  // namespace ak_v3_driver
