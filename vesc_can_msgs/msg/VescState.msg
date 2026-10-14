# ROS publication time; each CAN packet is received independently
builtin_interfaces/Time stamp

# Physical VESC CAN ID (0..254); select the motor by its named state topic
uint8 controller_id

# STATUS1: native motor values before pole-pair, gear and direction conversion
float64 erpm
float64 motor_current_a
float64 duty_cycle

# STATUS2: accumulated input charge [Ah]
float64 charge_drawn_ah
float64 charge_regen_ah

# STATUS3: accumulated input energy [Wh]
float64 energy_drawn_wh
float64 energy_regen_wh

# STATUS4: temperature [degC], input current [A], motor PID angle [deg]
float64 temp_fet_c
float64 temp_motor_c
float64 input_current_a
float64 pid_pos_deg

# STATUS5: last signed counter value and input voltage [V]
# The counter is usable as current feedback only when tachometer_fresh is true
int32 tachometer_counts
bool tachometer_fresh
float64 input_voltage_v

# Selective CAN fault query: -1 unknown/expired, 0 healthy, positive firmware code
int16 fault_code -1

# Missing/expired floating-point feedback is published as NaN
