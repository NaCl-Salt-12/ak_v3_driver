# Decoded feedback for a single motor, published from the 0x29 status frame
# (and 0x2A extended position frame, if enabled). Units are ROS-standard
# (rad, rad/s, amps, N*m) -- `invert_direction` has already been applied by
# the time this is published, so the sign matches this joint's intended
# positive direction regardless of how the motor is mechanically mounted.

std_msgs/Header header
string joint_name

float64 position              # rad
float64 velocity               # rad/s
float64 current                # A
float64 estimated_torque       # N*m, current * motor's torque coefficient

int8 temperature_c             # driver board temperature, degrees C
uint8 error_code                # raw error code from the motor, 0 = no fault
string error_text               # human-readable decode of error_code

bool extended_position_valid    # true if `position` came from the optional
                                 # 0x2A high-resolution frame rather than the
                                 # lower-resolution 0x29 status frame
