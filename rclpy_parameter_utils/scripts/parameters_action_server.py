#!/usr/bin/env python3

import sys

import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import rcl_interfaces
import rcl_interfaces_support
from rcl_interfaces_support.action import *

SPIN_RATE = 30.0


class ParametersActionServer:
    def __init__(self, node):
        self.node = node
        # self.get_parameters_node = rclpy.create_node(node.get_name() + '_get_parameters', 
        #                                              start_parameter_services = False,
        #                                              allow_undeclared_parameters = False,
        #                                              automatically_declare_parameters_from_overrides = False)
        # self.set_parameters_node = rclpy.create_node(node.get_name() + '_set_parameters',
        #                                              start_parameter_services = False, 
        #                                              allow_undeclared_parameters = False,
        #                                              automatically_declare_parameters_from_overrides = False)
        #
        # self.executor = MultiThreadedExecutor(4)
        # self.executor.add_node(self.get_parameters_node)
        # self.executor.add_node(self.set_parameters_node)
        # self.servers_nodes_thread = threading.Thread(target = self.executor.spin, args = (), daemon = False)
        # self.servers_nodes_thread.start()
        
        self.get_parameters_server = ActionServer(self.node,
                                                  rcl_interfaces_support.action.GetParameters,
                                                  self.node.get_name() + '/get_parameters',
                                                  self.get_parameters_callback)
        
        self.set_parameters_server = ActionServer(self.node,
                                                  rcl_interfaces_support.action.SetParameters,
                                                  self.node.get_name() + '/set_parameters',
                                                  self.set_parameters_callback)
        
    def get_parameters_callback(self, goal_handle):
        result = rcl_interfaces_support.action.GetParameters.Result()
        goal = goal_handle.request
        for param_name in goal.names:
            param : rclpy.Parameter
            if not self.node.has_parameter(param_name):
                goal_handle.abort()
                self.node.get_logger().error('Parameter "%s" has not been declared'%(param_name))
                return result
            
            param = self.node.get_parameter(param_name)
            result.values.append(param.to_parameter_msg().value)
        goal_handle.succeed()
        return result
    
    def set_parameters_callback(self, goal_handle):
        result = rcl_interfaces_support.action.SetParameters.Result()
        goal = goal_handle.request
        success = len(goal.parameters) > 0
        for param_msg in goal.parameters:
            param_result  = rcl_interfaces.msg.SetParametersResult()
            try:
                param  = rclpy.Parameter.from_parameter_msg(param_msg)
                if self.node.has_parameter(param.name) : 
                    self.node.undeclare_parameter(param.name)
                res = self.node.set_parameters([param])
                    
                param_result.successful = True
            
            except (rclpy.exceptions.InvalidParameterException, rclpy.exceptions.InvalidParameterValueException) as expt:
                self.node.get_logger().error(str(expt))
                param_result.successful = False
                param_result.reason = str(expt)
                success = False
                
            result.results.append(param_result)
            
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
                
        return result
    
if __name__ == '__main__':
    rclpy.init(args = sys.argv)
    node = rclpy.create_node('global_parameter_server', 
                             start_parameter_services = True,
                             allow_undeclared_parameters = True,
                             automatically_declare_parameters_from_overrides = True)
    spin_rate = node.create_rate(SPIN_RATE)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, args = (), daemon = False)
    spin_thread.start()
    
    try:
        server = ParametersActionServer(node)
        while rclpy.ok():
            spin_rate.sleep()
    except KeyboardInterrupt as exp:
        pass
    rclpy.shutdown()
    spin_thread.join(timeout = 2.0)
    sys.exit(0)
            