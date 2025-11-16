import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nvidia/UGV_Rover/ros2_ws/install/hardware_interface_py'
