# Motor names from the motors parameter, independent of VESC CAN controller IDs
string[] names

# Command values in the same order as names, with the same array length
# Units follow each motor's configured control_mode:
#   current: motor current [A], signed in the joint direction
#   velocity / current_velocity: joint velocity [rad/s]
#   position: joint position [rad]
# Omitted motors keep their previous command and its original timeout
float64[] values
