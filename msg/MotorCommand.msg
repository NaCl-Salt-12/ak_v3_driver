# Single generic command message covering every AK-V3 control mode via a
# discriminator field, so one subscription can drive the whole protocol
# instead of one topic per mode.
#
# Only the fields relevant to `mode` are used; the rest are ignored by the
# node. `ros2 topic echo` on this topic shows exactly what's being
# commanded, including which mode, which is the main reason to prefer this
# over silently-typed float topics per mode.

uint8 MODE_DUTY              = 0
uint8 MODE_CURRENT           = 1
uint8 MODE_CURRENT_BRAKE     = 2
uint8 MODE_VELOCITY          = 3
uint8 MODE_POSITION          = 4
uint8 MODE_POSITION_VELOCITY = 6
uint8 MODE_MIT               = 8

uint8 mode

# --- servo-mode fields ---
float64 duty                 # MODE_DUTY: -1.0 .. 1.0
float64 current               # MODE_CURRENT / MODE_CURRENT_BRAKE: amps
float64 velocity              # MODE_VELOCITY / MODE_POSITION_VELOCITY: rad/s
float64 position              # MODE_POSITION / MODE_POSITION_VELOCITY / MODE_MIT: rad
float64 acceleration          # MODE_POSITION_VELOCITY: rad/s^2

# --- MIT-mode-only fields (all sent together in one frame) ---
float64 mit_velocity          # MODE_MIT: rad/s
float64 mit_torque            # MODE_MIT: N*m, feed-forward torque
float64 kp                    # MODE_MIT: position gain
float64 kd                    # MODE_MIT: velocity gain
