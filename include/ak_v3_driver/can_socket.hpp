// Thin RAII wrapper around a Linux SocketCAN raw socket (AF_CAN/SOCK_RAW).
//
// Deliberately minimal: no external CAN library dependency, just the
// kernel's native CAN API. The interface (e.g. "can0") must already be
// brought up externally (`ip link set can0 up type can bitrate 1000000`)
// -- this class does not configure bitrate or bring the link up, since
// that's a system/udev concern shared across all processes using the bus,
// not something one motor's ROS node should own.
#pragma once

#include <stdexcept>
#include <string>

#include "ak_v3_driver/protocol.hpp"

namespace ak_v3_driver
{

/// Thrown for any socket setup failure (interface not found, not up,
/// permission denied, etc).
class CanSocketError : public std::runtime_error
{
public:
  explicit CanSocketError(const std::string & what)
  : std::runtime_error(what) {}
};

class CanSocket
{
public:
  /// Opens and binds a raw CAN socket to `interface_name` (e.g. "can0").
  /// Throws CanSocketError on failure.
  explicit CanSocket(const std::string & interface_name);

  ~CanSocket();

  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  /// Sends a frame. Returns false (does not throw) on a transient send
  /// failure, so the caller can log-and-continue rather than crash the
  /// node over one dropped frame.
  bool send(const protocol::CanFrame & frame);

  /// Blocks up to `timeout_ms` waiting for a frame. Returns true and
  /// fills `out_frame` if one arrived; returns false on timeout. Only
  /// extended-frame data frames are surfaced -- remote frames and
  /// error frames are silently skipped internally (a fault is reported
  /// via the motor's own status frame, not via a CAN-level error frame,
  /// per the manual's protocol).
  bool receive(protocol::CanFrame & out_frame, int timeout_ms);

  int fd() const {return fd_;}

private:
  int fd_ = -1;
};

}  // namespace ak_v3_driver
