import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/voltanie/ros2_ws/src/install/ros_qr_reader'
