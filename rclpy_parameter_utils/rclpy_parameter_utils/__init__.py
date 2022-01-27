from rpyutils import import_c_library
impl = import_c_library('.rclpy_parameter_utils','rclpy_parameter_utils')
from .global_parameter_client import *