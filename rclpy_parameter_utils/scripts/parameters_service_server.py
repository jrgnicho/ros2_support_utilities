#!/usr/bin/env python3

import sys
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

if __name__ == '__main__':
    rclpy.init(args = sys.argv)
    node = rclpy.create_node('parameter_server', 
                             start_parameter_services = True,
                             allow_undeclared_parameters = True,
                             automatically_declare_parameters_from_overrides = True)
    update_rate = node.create_rate(30.0)
    executor = MultiThreadedExecutor(10)
    executor.add_node(node)
    spin_thread = threading.Thread(target= executor.spin, args =(), daemon = True)
    spin_thread.start()
    try:    
        while rclpy.ok():
            update_rate.sleep()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    rclpy.shutdown()
    spin_thread.join()
    sys.exit(0)