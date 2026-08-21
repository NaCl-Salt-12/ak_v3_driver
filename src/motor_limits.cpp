#include "ak_v3_driver/motor_limits.hpp"

#include <unordered_map>

namespace ak_v3_driver {

namespace {
// Table transcribed from manual section 4.2 "Parameter Ranges".
//
// pole_pairs / gear_ratio columns are PLACEHOLDERS (1.0 each) -- these are
// NOT in the manual and are almost certainly wrong for a real geared
// AK-series motor. Replace them with your actual values before trusting
// velocity or position-velocity mode unit conversions. Everything else in
// this table (kv, kt, velocity/torque range) is transcribed as documented.
//                             kv      kt        v_min   v_max    t_min    t_max
//                             pole_pairs gear_ratio
const std::unordered_map<std::string, MotorModelSpec> kMotorTable = {
    {"AK10-9", {60.0f, 1.3137f, -28.0f, 28.0f, -54.0f, 54.0f, 21.0f, 9.0f}},
    {"AK60-6", {80.0f, 0.5994f, -60.0f, 60.0f, -12.0f, 12.0f, 14.0f, 6.0f}},
    {"AK60-39", {80.0f, 3.4616f, -10.0f, 10.0f, -80.0f, 80.0f, 14.0f, 39.0f}},
    {"AK70-9", {60.0f, 1.0621f, -30.0f, 30.0f, -32.0f, 32.0f, 21.0f, 9.0f}},
    {"AK80-8", {60.0f, 1.0569f, -38.0f, 38.0f, -32.0f, 32.0f, 21.0f, 8.0f}},
    {"AK80-9", {100.0f, 0.5701f, -65.0f, 65.0f, -18.0f, 18.0f, 21.0f, 9.0f}},
};
} // namespace

std::optional<MotorModelSpec> lookupMotorModel(const std::string &motor_type) {
  const auto it = kMotorTable.find(motor_type);
  if (it == kMotorTable.end()) {
    return std::nullopt;
  }
  return it->second;
}

} // namespace ak_v3_driver
