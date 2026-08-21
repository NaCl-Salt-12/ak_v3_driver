#include "ak_v3_driver/can_socket.hpp"
#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ak_v3_driver {

/// Opens and binds a raw CAN socket to `interface_name` (e.g. "can0").
/// Throws CanSocketError on failure.
CanSocket::CanSocket(const std::string &interface_name) {
  // Create a raw CAN socket. PF_CAN is the protocol family for SocketCAN,
  // SOCK_RAW + CAN_RAW gives us direct access to individual CAN frames
  // (no transport-layer framing).
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw CanSocketError("failed to create CAN socket: " +
                         std::string(std::strerror(errno)));
  }

  // Resolve the interface name (e.g. "can0") to a kernel interface index,
  // which is what sockaddr_can actually needs for binding.
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    // Interface doesn't exist / isn't up -- clean up the socket before
    // throwing.
    const std::string msg =
        "interface '" + interface_name + "' not found: " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw CanSocketError(msg);
  }

  // Bind the socket to the resolved interface so all send/receive calls
  // on fd_ are scoped to this specific CAN bus.
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    const std::string msg =
        "failed to bind to '" + interface_name + "': " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw CanSocketError(msg);
  }
}

CanSocket::~CanSocket() {
  // Only close if construction actually succeeded in acquiring a valid fd
  // (constructor sets fd_ = -1 before throwing on any failure path).
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

/// Sends a frame. Returns false (does not throw) on a transient send
/// failure, so the caller can log-and-continue rather than crash the
/// node over one dropped frame.
bool CanSocket::send(const protocol::CanFrame &frame) {
  // Translate our protocol-level frame into the kernel's can_frame struct.
  struct can_frame raw{};
  raw.can_id = frame.id | CAN_EFF_FLAG; // mark as 29-bit extended identifier
  raw.can_dlc = frame.dlc;
  // Only copy dlc bytes -- frame.data may be a larger fixed-size buffer,
  // and we don't want to read/write past the declared payload length.
  std::memcpy(raw.data, frame.data.data(), frame.dlc);

  // A successful CAN write is all-or-nothing: the kernel either accepts
  // the whole frame struct or it doesn't, so comparing byte counts is a
  // valid success check here.
  const ssize_t written = ::write(fd_, &raw, sizeof(raw));
  return written == static_cast<ssize_t>(sizeof(raw));
}

/// Blocks up to `timeout_ms` waiting for a frame. Returns true and
/// fills `out_frame` if one arrived; returns false on timeout. Only
/// extended-frame data frames are surfaced -- remote frames and
/// error frames are silently skipped internally (a fault is reported
/// via the motor's own status frame, not via a CAN-level error frame,
/// per the manual's protocol).
bool CanSocket::receive(protocol::CanFrame &out_frame, int timeout_ms) {
  // Wait for the socket to become readable, or time out. Using poll()
  // instead of a blocking read lets the caller bound how long it waits.
  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int ready = poll(&pfd, 1, timeout_ms);
  if (ready <= 0) {
    return false; // timeout or interrupted -- caller just retries
  }

  // Data is available; read exactly one can_frame's worth of bytes.
  struct can_frame raw{};
  const ssize_t n = ::read(fd_, &raw, sizeof(raw));
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    // Short/partial read -- treat as no valid frame rather than
    // risking use of a partially-populated struct.
    return false;
  }

  // Filter out frame types the driver doesn't handle, in order:

  if (raw.can_id & CAN_ERR_FLAG) {
    return false; // CAN-level error frame, not a motor status frame -- skip
  }
  if (!(raw.can_id & CAN_EFF_FLAG)) {
    return false; // not an extended frame -- the AK-V3 protocol only uses EFF
  }
  if (raw.can_id & CAN_RTR_FLAG) {
    return false; // remote request frame, no payload -- skip
  }

  // Passed all filters: this is a genuine extended data frame. Strip the
  // flag bits out of can_id (CAN_EFF_MASK keeps only the 29 ID bits) and
  // copy the payload into the caller-supplied output frame.
  out_frame.id = raw.can_id & CAN_EFF_MASK;
  out_frame.dlc = raw.can_dlc;
  std::memcpy(out_frame.data.data(), raw.data, raw.can_dlc);
  return true;
}

} // namespace ak_v3_driver
