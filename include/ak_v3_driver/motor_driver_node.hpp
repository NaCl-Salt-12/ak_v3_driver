/// Main driver node for cubemars ak_v3 motors
#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ak_v3_driver/can_socket.hpp"
#include "ak_v3_driver/motor_limits.hpp"
#include "ak_v3_driver/msg/motor_command.hpp"
#include "ak_v3_driver/msg/motor_state.hpp"
#include "ak_v3_driver/protocol.hpp"
#include "ak_v3_driver/srv/set_feedback_config.hpp"
#include "ak_v3_driver/srv/set_origin.hpp"

namespace ak_v3_driver {

// Driver node
class MotorDriverNode : public rclcpp::Node {
public:
  explicit MotorDriverNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MotorDriverNode() override;

private:
  // --- parameter loading / validation, run once at construction ---
  void loadParameters();

  // --- ROS-side callbacks (run on the executor thread) ---
  void onCommand(const msg::MotorCommand::SharedPtr msg);
  void onSetOrigin(const std::shared_ptr<srv::SetOrigin::Request> request,
                   std::shared_ptr<srv::SetOrigin::Response> response);
  void onSetFeedbackConfig(
      const std::shared_ptr<srv::SetFeedbackConfig::Request> request,
      std::shared_ptr<srv::SetFeedbackConfig::Response> response);
  void onDisable(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // --- CAN RX thread + safe hand-off to the executor ---
  void rxThreadMain();
  void drainRxQueueAndPublish(); // timer callback, runs on executor thread

  // --- unit conversions (rad <-> deg, rad/s <-> ERPM, sign inversion) ---
  float radToDeg(float rad) const;
  float degToRad(float deg) const;
  float radPerSecToErpm(float rad_s) const;
  float erpmToRadPerSec(float erpm) const;

  // parameters (immutable after construction -- reconfiguring can_id or
  // motor_type at runtime would change which physical motor this node
  // owns, which is deliberately not supported; restart the node instead)
  std::string can_interface_;
  uint8_t can_id_ = 0;
  std::string joint_name_;
  std::string motor_type_;
  bool invert_direction_ = false;
  int feedback_hz_ = 100;

  MotorModelSpec motor_spec_{};
  protocol::MitLimits mit_limits_{};

  std::unique_ptr<CanSocket> can_socket_;

  rclcpp::Subscription<msg::MotorCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<msg::MotorState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      diag_pub_;
  rclcpp::Service<srv::SetOrigin>::SharedPtr set_origin_srv_;
  rclcpp::Service<srv::SetFeedbackConfig>::SharedPtr set_feedback_config_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::TimerBase::SharedPtr rx_drain_timer_;

  std::thread rx_thread_;
  std::atomic<bool> rx_thread_running_{false};

  std::mutex rx_queue_mutex_;
  std::deque<protocol::StatusFrame> status_queue_;
  std::deque<protocol::ExtendedPositionFrame> ext_position_queue_;

  // holds the last extended-position reading, if enabled, so a 0x29 frame
  // arriving after a 0x2A frame doesn't clobber the higher-resolution
  // position with the lower-resolution one
  std::optional<float> last_extended_position_deg_;
};

} // namespace ak_v3_driver
