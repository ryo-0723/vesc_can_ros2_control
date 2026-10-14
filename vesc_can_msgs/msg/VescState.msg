# ROS publication time; CAN packets are received independently
builtin_interfaces/Time stamp

# Logical ID from YAML/URDF, shared with commands and events
uint8 motor_id

# STATUS1: joint/output velocity [rad/s], pole pairs, gearing and direction applied
float64 velocity_rad_s
# Native motor current [A] and duty ratio, before direction conversion
float32 motor_current_a
float32 duty_cycle

# STATUS2: accumulated input charge [Ah], for monitoring
float32 charge_drawn_ah
float32 charge_regen_ah

# STATUS3: accumulated input energy [Wh], for monitoring
float32 energy_drawn_wh
float32 energy_regen_wh

# STATUS4: temperature [degC], input current [A]
float32 temp_fet_c
float32 temp_motor_c
float32 input_current_a
# Wrapped PID joint position [rad], gear, direction and PID zero offset applied
# Requires the VESC PID angle to represent a calibrated mechanical motor angle
float64 pid_pos_rad

# STATUS5: signed accumulated joint/output angle [rad] and input voltage [V]
# Pole pairs, gearing and direction applied; no single-turn wrapping or auto-zero
float64 position_rad
float32 input_voltage_v

# Selective CAN fault query: -1 unknown/expired, 0 healthy, positive firmware code
int16 fault_code -1

# Missing/expired floating-point feedback is published as NaN
