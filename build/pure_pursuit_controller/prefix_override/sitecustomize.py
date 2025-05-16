import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/junaet/ros2_ws/install/pure_pursuit_controller'
