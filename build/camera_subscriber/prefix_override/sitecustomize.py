import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/damian/Documents/project_NIODSR_ros2_ws/install/camera_subscriber'
