# ROS logical_id from YAML, independent of the VESC CAN controller_id
uint8 motor_id

# Unit follows this motor's startup control_mode:
#   current: motor current [A], signed in the joint direction
#   velocity / current_velocity: joint velocity [rad/s]
#   position: joint position [rad]
float32 value
