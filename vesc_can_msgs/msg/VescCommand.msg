# ROS motor identifier from YAML, independent of the VESC motor_id
uint8 logical_id

# Unit follows this motor's startup control_mode:
#   current: motor current [A], signed in the joint direction
#   velocity / current_velocity: joint velocity [rad/s]
#   position: joint position [rad]
#   duty_cycle: duty ratio [-1, 1], signed in the joint direction (0.5 = 50%)
float32 value
