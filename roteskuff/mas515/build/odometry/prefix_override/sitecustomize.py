import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/entho/ros2_ws/MAS514_H24_group3/src/mas515/install/odometry'
