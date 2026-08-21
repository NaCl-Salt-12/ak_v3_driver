#include "ak_v3_driver/motor_driver_node.hpp"

#include <cmath>
#include <stdexcept>

namespace ak_v3_driver {

namespace {
// Mathematical constant used for rad<->deg and rad/s<->ERPM conversions.
constexpr double kPi = 3.14159265358979323846;

// How long CanSocket::receive() blocks per loop iteration in the RX thread.
// A short timeout keeps the thread responsive to rx_thread_running_ going
// false (fast shutdown) while still avoiding a tight busy-loop.
constexpr int kRxPollTimeoutMs = 50;

// Rate (Hz) at which the executor-side timer drains the RX queues and
// publishes MotorState/DiagnosticStatus messages. Decoupled from the CAN
// RX thread's poll rate and from feedback_hz_ (which only affects how often
// the physical motor sends status frames over the bus).
constexpr double kDrainTimerHz = 200.0;
} // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

/// Driver node constructor.
/// Loads and validates parameters, resolves the motor spec, opens the CAN
/// socket, and wires up all ROS interfaces (pub/sub/services) plus the
/// background RX thread and its drain timer.
MotorDriverNode::MotorDriverNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("ak_v3_driver_node", options) {

  // --- parameter loading / validation, run once at construction ---
  loadParameters();

  // Look up the hardware spec table entry for this node's motor_type
  // (e.g. "AK80-9"). This includes velocity/torque limits, pole pairs,
  // gear ratio, and torque coefficient used throughout the driver.
  const auto spec = lookupMotorModel(motor_type_);
  if (!spec.has_value()) {
    // Unknown/misspelled motor_type -- fail fast at construction rather
    // than silently using bad defaults.
    throw std::runtime_error("Unknown motor_type '" + motor_type_ +
                             "'. Check spelling against the AK-V3 manual model "
                             "names (e.g. 'AK80-9').");
  }

  // Cache the resolved spec and copy the MIT-mode limits (velocity/torque)
  // into mit_limits_, which is passed to protocol::packMit() on every MIT
  // command to clamp values into the safe range for this motor model.
  motor_spec_ = *spec;
  mit_limits_.velocity_min_rad_s = motor_spec_.velocity_min_rad_s;
  mit_limits_.velocity_max_rad_s = motor_spec_.velocity_max_rad_s;
  mit_limits_.torque_min_nm = motor_spec_.torque_min_nm;
  mit_limits_.torque_max_nm = motor_spec_.torque_max_nm;

  // Not every entry in the motor spec table has been filled in with real
  // hardware values yet. Placeholder pole_pairs/gear_ratio of 1.0/1.0 would
  // silently produce wrong rad/s <-> ERPM conversions (velocity mode and the
  // velocity/acceleration components of position-velocity mode), so warn
  // loudly at startup if this motor_type still has placeholders.
  if (motor_spec_.pole_pairs == 1.0f && motor_spec_.gear_ratio == 1.0f) {
    RCLCPP_WARN(
        get_logger(),
        "motor_type '%s' is using placeholder pole_pairs/gear_ratio (1.0/1.0) "
        "from "
        "motor_limits.cpp -- velocity and position-velocity mode unit "
        "conversions "
        "will be wrong until these are set to your actual hardware values.",
        motor_type_.c_str());
  }

  // Try to open the CAN socket/interface (e.g. "can0"). This must succeed
  // before any commands can be sent or state received, so a failure here
  // is fatal and the exception is rethrown after logging.
  try {
    can_socket_ = std::make_unique<CanSocket>(can_interface_);
  } catch (const CanSocketError &e) {
    RCLCPP_FATAL(get_logger(), "Failed to open CAN interface '%s': %s",
                 can_interface_.c_str(), e.what());
    throw;
  }

  // --- ROS interfaces ---

  // Incoming motor commands (duty/current/velocity/position/MIT/etc.).
  // SensorDataQoS (best-effort, small depth) favors latest-command-wins
  // behavior over guaranteed delivery, appropriate for a high-rate control
  // loop where a stale queued command is worse than a dropped one.
  cmd_sub_ = create_subscription<msg::MotorCommand>(
      "~/cmd", rclcpp::SensorDataQoS(),
      std::bind(&MotorDriverNode::onCommand, this, std::placeholders::_1));

  // Published motor state (position/velocity/current/torque/temperature/
  // fault) at the drain-timer rate. Also best-effort SensorDataQoS to match
  // the high publish rate.
  state_pub_ =
      create_publisher<msg::MotorState>("~/state", rclcpp::SensorDataQoS());

  // Diagnostics (OK/ERROR + fault text) published alongside each state
  // message. Uses reliable SystemDefaultsQoS since diagnostics should not
  // be silently dropped.
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "~/diagnostics", rclcpp::SystemDefaultsQoS());

  // Service to zero the motor's position sensor at its current position,
  // optionally persisting the new origin across power cycles.
  set_origin_srv_ = create_service<srv::SetOrigin>(
      "~/set_origin", std::bind(&MotorDriverNode::onSetOrigin, this,
                                std::placeholders::_1, std::placeholders::_2));

  // Service to toggle the extended (multi-turn) position feedback frame and
  // single-turn mode on the motor controller.
  set_feedback_config_srv_ = create_service<srv::SetFeedbackConfig>(
      "~/set_feedback_config",
      std::bind(&MotorDriverNode::onSetFeedbackConfig, this,
                std::placeholders::_1, std::placeholders::_2));

  // Service to immediately disable (coast) the motor.
  disable_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/disable", std::bind(&MotorDriverNode::onDisable, this,
                             std::placeholders::_1, std::placeholders::_2));

  // Start the dedicated CAN RX thread. Running RX on its own thread (rather
  // than in a ROS timer) avoids blocking the executor while
  // CanSocket::receive() waits for frames, and keeps ingestion latency low
  // regardless of what else the executor is doing.
  rx_thread_running_ = true;
  rx_thread_ = std::thread(&MotorDriverNode::rxThreadMain, this);

  // Timer that periodically drains whatever the RX thread has queued and
  // publishes it. Decoupling drain/publish from the RX thread keeps all
  // ROS-side interaction (logging, publishing) on the executor thread.
  rx_drain_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(1000.0 / kDrainTimerHz)),
      std::bind(&MotorDriverNode::drainRxQueueAndPublish, this));

  RCLCPP_INFO(get_logger(),
              "ak_v3_driver started: joint='%s' can_interface='%s' can_id=%d "
              "motor_type='%s' "
              "invert=%s pole_pairs=%.3f gear_ratio=%.3f",
              joint_name_.c_str(), can_interface_.c_str(), can_id_,
              motor_type_.c_str(), invert_direction_ ? "true" : "false",
              motor_spec_.pole_pairs, motor_spec_.gear_ratio);
}

/// Destructor: signal the RX thread to stop and join it so the CAN socket
/// and node can be torn down safely without a background thread still
/// touching member state.
MotorDriverNode::~MotorDriverNode() {
  rx_thread_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Parameter loading
// ---------------------------------------------------------------------------

/// Declares and validates all ROS parameters. Runs once at construction.
/// Parameters with no sensible default (can_id, joint_name, motor_type) are
/// required and throw if left unset/invalid, causing node construction to
/// fail fast instead of running with an ambiguous configuration.
void MotorDriverNode::loadParameters() {
  // The SocketCAN interface name, e.g. "can0".
  can_interface_ = declare_parameter<std::string>("can_interface", "can0");

  // CAN ID of this specific motor controller on the bus. Defaults to -1
  // (invalid) so that failing to set it is caught here rather than
  // silently addressing the wrong (or no) motor.
  const int can_id_param = declare_parameter<int>("can_id", -1);
  if (can_id_param < 0 || can_id_param > 255) {
    throw std::runtime_error(
        "Required parameter 'can_id' must be set to a value in [0, 255].");
  }
  can_id_ = static_cast<uint8_t>(can_id_param);

  // Human-readable joint name (e.g. "lf_knee"), used as the frame/joint
  // identifier in published MotorState/DiagnosticStatus messages.
  joint_name_ = declare_parameter<std::string>("joint_name", "");
  if (joint_name_.empty()) {
    throw std::runtime_error(
        "Required parameter 'joint_name' must be set (non-empty).");
  }

  // Motor model identifier used to look up hardware limits/coefficients
  // (e.g. "AK10-9"). Must match an entry in the motor spec table.
  motor_type_ = declare_parameter<std::string>("motor_type", "");
  if (motor_type_.empty()) {
    throw std::runtime_error(
        "Required parameter 'motor_type' must be set (e.g. 'AK80-9'), "
        "see manual section 4.2 for valid model names.");
  }

  // Flips the sign of position/velocity/torque on both outgoing commands
  // and incoming state, letting callers use a consistent convention
  // regardless of how the motor happens to be mounted/wired.
  invert_direction_ = declare_parameter<bool>("invert_direction", false);

  // Informational only -- the actual CAN bitrate is configured at the
  // SocketCAN interface level (see can_socket.hpp), not by this node.
  declare_parameter<int>("bitrate", 1000000);

  // Requested rate at which the motor controller itself should push status
  // frames onto the bus. (Consumed elsewhere; not directly used in this
  // file beyond being declared here.)
  feedback_hz_ = declare_parameter<int>("feedback_hz", 100);
}

// ---------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------
// Motor commands/state in this driver's public API are in SI units
// (radians, rad/s, N*m). The underlying VESC-style protocol instead uses
// degrees for position and ERPM (electrical RPM) for velocity, so these
// helpers convert at the CAN boundary.

/// Converts radians to degrees.
float MotorDriverNode::radToDeg(float rad) const {
  return rad * static_cast<float>(180.0 / kPi);
}

/// Converts degrees to radians.
float MotorDriverNode::degToRad(float deg) const {
  return deg * static_cast<float>(kPi / 180.0);
}

/// Converts an output-shaft angular velocity (rad/s) to ERPM
/// (electrical RPM), accounting for the motor's pole-pair count and any
/// external gear reduction.
float MotorDriverNode::radPerSecToErpm(float rad_s) const {
  // output_rpm = rad_s * 60 / (2*pi); erpm = output_rpm * pole_pairs *
  // gear_ratio
  const double output_rpm = static_cast<double>(rad_s) * 60.0 / (2.0 * kPi);
  return static_cast<float>(output_rpm *
                            static_cast<double>(motor_spec_.pole_pairs) *
                            static_cast<double>(motor_spec_.gear_ratio));
}

/// Converts ERPM (electrical RPM), as reported by the motor controller,
/// back to output-shaft angular velocity (rad/s). Inverse of
/// radPerSecToErpm().
float MotorDriverNode::erpmToRadPerSec(float erpm) const {
  const double denom = static_cast<double>(motor_spec_.pole_pairs) *
                       static_cast<double>(motor_spec_.gear_ratio);
  const double output_rpm = static_cast<double>(erpm) / denom;
  return static_cast<float>(output_rpm * 2.0 * kPi / 60.0);
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------

/// Subscription callback for incoming MotorCommand messages. Packs the
/// command into the appropriate CAN frame for the requested control mode
/// (applying direction inversion and unit conversion as needed) and sends
/// it over the bus.
void MotorDriverNode::onCommand(const msg::MotorCommand::SharedPtr msg) {
  // +1 or -1 depending on invert_direction_, applied to every
  // position/velocity/torque quantity that has a physical direction.
  const float sign = invert_direction_ ? -1.0f : 1.0f;
  protocol::CanFrame frame;

  // Build the outgoing CAN frame based on the requested control mode.
  switch (msg->mode) {
  case msg::MotorCommand::MODE_DUTY:
    // Open-loop duty cycle command, range roughly [-1, 1].
    frame = protocol::packDuty(can_id_, sign * static_cast<float>(msg->duty));
    break;
  case msg::MotorCommand::MODE_CURRENT:
    // Closed-loop current (torque-proportional) command in amps.
    frame =
        protocol::packCurrent(can_id_, sign * static_cast<float>(msg->current));
    break;
  case msg::MotorCommand::MODE_CURRENT_BRAKE:
    // Braking current is a magnitude in the protocol (0..60000 -> 0..60A,
    // manual 4.1.3) -- direction doesn't apply, so `invert_direction`
    // is deliberately NOT used here.
    frame =
        protocol::packCurrentBrake(can_id_, static_cast<float>(msg->current));
    break;
  case msg::MotorCommand::MODE_VELOCITY:
    // Closed-loop velocity command; convert SI rad/s to protocol ERPM.
    frame = protocol::packVelocityErpm(
        can_id_, radPerSecToErpm(sign * static_cast<float>(msg->velocity)));
    break;
  case msg::MotorCommand::MODE_POSITION:
    // Closed-loop position command; convert SI radians to protocol degrees.
    frame = protocol::packPositionDeg(
        can_id_, radToDeg(sign * static_cast<float>(msg->position)));
    break;
  case msg::MotorCommand::MODE_POSITION_VELOCITY:
    // Position command with velocity/acceleration feed-forward limits;
    // position converts to degrees, velocity/acceleration convert to ERPM.
    frame = protocol::packPositionVelocity(
        can_id_, radToDeg(sign * static_cast<float>(msg->position)),
        radPerSecToErpm(sign * static_cast<float>(msg->velocity)),
        radPerSecToErpm(sign * static_cast<float>(msg->acceleration)));
    break;
  case msg::MotorCommand::MODE_MIT:
    // Impedance-style MIT-mode command (position + velocity + kp/kd + feed-
    // forward torque). Left in SI units for packMit(); mit_limits_ clamps
    // velocity/torque to this motor's safe range.
    frame = protocol::packMit(
        can_id_, sign * static_cast<float>(msg->position),
        sign * static_cast<float>(msg->mit_velocity),
        static_cast<float>(msg->kp), static_cast<float>(msg->kd),
        sign * static_cast<float>(msg->mit_torque), mit_limits_);
    break;
    // case msg::MotorCommand::MODE_DISABLE:
    // frame = protocol::packDisable()
  default:
    // Unknown mode value -- log (throttled to avoid flooding) and drop the
    // command rather than sending a garbage frame.
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received MotorCommand with unrecognized mode=%d, ignoring.",
        msg->mode);
    return;
  }

  // Send the packed frame; log (throttled) on failure but don't throw --
  // a single dropped CAN frame shouldn't crash the control loop.
  if (!can_socket_->send(frame)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "CAN send failed, frame dropped.");
  }
}

// ---------------------------------------------------------------------------
// Service handlers
// ---------------------------------------------------------------------------

/// Handles ~/set_origin: sends a command to zero the motor's position
/// sensor at its current location, optionally persisting it across power
/// cycles if request->permanent is set.
void MotorDriverNode::onSetOrigin(
    const std::shared_ptr<srv::SetOrigin::Request> request,
    std::shared_ptr<srv::SetOrigin::Response> response) {
  const auto frame = protocol::packSetOrigin(can_id_, request->permanent);
  response->success = can_socket_->send(frame);
  response->message =
      response->success ? "origin set command sent" : "CAN send failed";
}

/// Handles ~/set_feedback_config: toggles the extended (multi-turn)
/// position frame and single-turn mode on the motor controller. If the
/// extended position frame is being disabled, also clears the cached
/// extended position so state publishing falls back to the single-turn
/// value reported in the status frame.
void MotorDriverNode::onSetFeedbackConfig(
    const std::shared_ptr<srv::SetFeedbackConfig::Request> request,
    std::shared_ptr<srv::SetFeedbackConfig::Response> response) {
  const auto frame = protocol::packFeedbackConfig(
      can_id_, request->add_extended_position_frame, request->single_turn_mode);
  response->success = can_socket_->send(frame);
  response->message =
      response->success ? "feedback config command sent" : "CAN send failed";

  if (!request->add_extended_position_frame) {
    // Stop trusting a now-stale extended position reading.
    last_extended_position_deg_.reset();
  }
}

/// Handles ~/disable: sends an immediate disable/coast command to the
/// motor controller.
void MotorDriverNode::onDisable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const auto frame = protocol::packDisable(can_id_);
  response->success = can_socket_->send(frame);
  response->message =
      response->success ? "disable command sent" : "CAN send failed";
}

// ---------------------------------------------------------------------------
// CAN RX
// ---------------------------------------------------------------------------

/// Body of the dedicated CAN RX thread. Continuously polls the CAN socket
/// for frames (with a short timeout so the thread can notice shutdown
/// promptly), decodes recognized frame types belonging to this node's
/// can_id, and pushes them onto the shared RX queues under lock. The
/// executor-side drain timer later consumes these queues on the ROS thread.
void MotorDriverNode::rxThreadMain() {
  while (rx_thread_running_.load()) {
    protocol::CanFrame frame;
    if (!can_socket_->receive(frame, kRxPollTimeoutMs)) {
      continue; // timeout, or a frame for a filter we don't care about -- just
                // loop
    }

    if (const auto status = protocol::decodeStatusFrame(frame, can_id_)) {
      // Regular status frame (position/speed/current/temperature/fault)
      // for our can_id.
      std::lock_guard<std::mutex> lock(rx_queue_mutex_);
      status_queue_.push_back(*status);
      continue;
    }
    if (const auto ext_pos =
            protocol::decodeExtendedPositionFrame(frame, can_id_)) {
      // Optional multi-turn extended position frame for our can_id (only
      // present if enabled via ~/set_feedback_config).
      std::lock_guard<std::mutex> lock(rx_queue_mutex_);
      ext_position_queue_.push_back(*ext_pos);
      continue;
    }
    // Startup frame (0x2C) and frames for other driver IDs are silently
    // ignored here -- this node only reacts to its own can_id's status data.
  }
}

/// Runs on the executor (wall timer callback) at kDrainTimerHz. Swaps out
/// the RX-thread-populated queues under lock (minimizing time spent
/// holding the mutex), applies any pending extended-position update, then
/// converts each queued status frame into a published MotorState and
/// DiagnosticStatus message.
void MotorDriverNode::drainRxQueueAndPublish() {
  std::deque<protocol::StatusFrame> statuses;
  std::deque<protocol::ExtendedPositionFrame> ext_positions;
  {
    // Briefly lock just to swap the queues, so the RX thread is blocked
    // for as little time as possible.
    std::lock_guard<std::mutex> lock(rx_queue_mutex_);
    statuses.swap(status_queue_);
    ext_positions.swap(ext_position_queue_);
  }

  // Update the cached extended (multi-turn) position from any extended
  // position frames received since the last drain. Only the most recent
  // value matters since it fully supersedes older ones.
  for (const auto &ext : ext_positions) {
    last_extended_position_deg_ = ext.position_deg;
  }

  const float sign = invert_direction_ ? -1.0f : 1.0f;

  // Publish one MotorState + DiagnosticStatus pair per queued status frame,
  // preserving the order/rate at which they were actually received.
  for (const auto &status : statuses) {
    // Prefer the multi-turn extended position if we have one cached;
    // otherwise fall back to the single-turn position from the status
    // frame itself.
    const float position_deg =
        last_extended_position_deg_.value_or(status.position_deg);

    msg::MotorState state_msg;
    state_msg.header.stamp = now();
    state_msg.joint_name = joint_name_;
    // Convert protocol units (deg, ERPM) back to SI units (rad, rad/s),
    // applying the same direction inversion used on outgoing commands so
    // state and command share a consistent sign convention.
    state_msg.position = sign * degToRad(position_deg);
    state_msg.velocity = sign * erpmToRadPerSec(status.speed_erpm);
    state_msg.current = sign * status.current_a;
    // Estimated torque from current using this motor model's torque
    // constant (N*m per amp); not a direct sensor measurement.
    state_msg.estimated_torque =
        state_msg.current * motor_spec_.torque_coefficient_nm_per_a;
    state_msg.temperature_c = status.temperature_c;
    state_msg.error_code = status.error_code;
    state_msg.error_text = protocol::faultCodeToString(status.error_code);
    state_msg.extended_position_valid = last_extended_position_deg_.has_value();
    state_pub_->publish(state_msg);

    // Mirror the fault state into a standard DiagnosticStatus message so
    // this joint shows up correctly in diagnostic aggregators.
    diagnostic_msgs::msg::DiagnosticStatus diag_msg;
    diag_msg.name = joint_name_;
    diag_msg.hardware_id = std::to_string(can_id_);
    if (status.error_code == 0) {
      diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_msg.message = "no fault";
    } else {
      diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_msg.message = state_msg.error_text;
    }
    diag_pub_->publish(diag_msg);
  }
}

} // namespace ak_v3_driver
