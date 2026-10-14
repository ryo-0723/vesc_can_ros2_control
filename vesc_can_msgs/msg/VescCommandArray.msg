# ROS time when the sender generated this command set; zero means unspecified
# The driver's timeout uses local steady receive time, independent of this stamp
builtin_interfaces/Time stamp

# One or more commands; omitted motors retain their original command timeout
VescCommand[] commands
