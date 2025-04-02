import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rishiraj/GSOC_Playmotion2_plugin_based_system/ros2_ws/install/my_robot_contoller'
