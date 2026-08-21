#include "ak_v3_driver/motor_driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

// Entry point for the ak_v3_driver node executable.
// Creates and spins a single MotorDriverNode, and ensures rclcpp::shutdown()
// is called on every exit path (success, construction failure, or normal
// spin exit).
int main(int argc, char **argv) {
  // Initialize the ROS 2 context (parses --ros-args, sets up signal
  // handling, etc.) before any node is created.
  rclcpp::init(argc, argv);

  try {
    // MotorDriverNode's constructor does the real setup work (parameter
    // validation, motor spec lookup, opening the CAN socket, starting the
    // RX thread) and can throw std::runtime_error / CanSocketError if
    // something is misconfigured or the hardware isn't reachable.
    auto node = std::make_shared<ak_v3_driver::MotorDriverNode>();

    // Blocks here, processing callbacks (command subscription, services,
    // drain timer) until the node is shut down (e.g. Ctrl+C / SIGINT).
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    // Construction failed (bad params, unknown motor_type, CAN interface
    // wouldn't open, etc.). Log the reason at FATAL before the node object
    // itself is available to log through, then shut down cleanly and
    // report failure to the caller/launch system.
    RCLCPP_FATAL(rclcpp::get_logger("ak_v3_driver"), "Startup failed: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Reached on a normal (non-exceptional) spin exit, e.g. after SIGINT
  // triggers an orderly shutdown.
  rclcpp::shutdown();
  return 0;
}
