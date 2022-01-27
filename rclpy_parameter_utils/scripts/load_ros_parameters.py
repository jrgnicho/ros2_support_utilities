#!/usr/bin/env python3

import sys

import yaml

import threading

import time

import uuid

from collections import namedtuple

from builtins import isinstance

from typing import NamedTuple

from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.node import *

DEFAULT_SERVICE_TIMEOUT = 5.0
SERVICE_WAIT_SLEEP_PAUSE = 0.05

ParamInfo = namedtuple('ParamInfo', 'param_name required default_value')

def construct_params(params_name_prefix : str, params_dict: dict ):
    
    params_list : list[rclpy.parameter.Parameter] = []
    for key_name, val in params_dict.items():
        if isinstance(val, dict):
            inner_param_name_prefix = key_name if params_name_prefix == '' else params_name_prefix + '.' + key_name
            inner_params_list = construct_params(inner_param_name_prefix, val)
            if len(inner_params_list) > 0:
                params_list += inner_params_list
        else:
            param_name = key_name if params_name_prefix == '' else params_name_prefix + '.' + key_name
            param = rclpy.Parameter(name = param_name, value = val)
            params_list.append(param)
            
    return params_list   

class LoadParameters:
    
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        
        params_info_list = [ParamInfo('params_yaml', True, ''),
                            ParamInfo('params_ns', False, ''),
                            ParamInfo('target_node_name', False, 'global_parameter_server'),
                            ParamInfo('service_timeout', False, DEFAULT_SERVICE_TIMEOUT)]
        for param_info in params_info_list:   
            param_name = param_info.param_name
                     
            if not self.__node.has_parameter(param_name):
                if param_info.required:
                    raise RuntimeError('The required parameter "%s" was not found'%(param_info.param_name))
                self.__node.declare_parameter(name = param_name, value = param_info.default_value)
                
        target_node_name = node.get_parameter('target_node_name').value
        params_ns = node.get_parameter('params_ns').value
        params_yaml = node.get_parameter('params_yaml').value
        service_timeout = node.get_parameter('service_timeout').value
        
        # check parameter values
        if target_node_name == '':
            raise RuntimeError('The parameter server node name can not be empty')
        
        # loading yaml
        yaml_data_dic: dict = {}
        with open(params_yaml,'r') as f:
            yaml_data_dic = yaml.load(f, Loader = yaml.Loader)
            
        # create list of parameters from yaml data
        params_list = construct_params(params_ns, yaml_data_dic);
        
        if len(params_list) == 0:
            raise RuntimeError('No parameters were passed')
        
        # create set params client
        set_parameters_service_name = target_node_name + '/set_parameters'
        set_param_client = node.create_client(SetParameters, set_parameters_service_name)
        if not set_param_client.wait_for_service(service_timeout):
            raise RuntimeError('Service "%s" was not found'%(set_parameters_service_name))
        
        # create request
        req = SetParameters.Request()
        for param in params_list:
            req.parameters.append(param.to_parameter_msg())
        
        # calling service
        fut = set_param_client.call_async(req)
        
        total_elapsed_time = 0.0
        while not fut.done() and total_elapsed_time < service_timeout:
            time.sleep(SERVICE_WAIT_SLEEP_PAUSE)
            total_elapsed_time += SERVICE_WAIT_SLEEP_PAUSE
            
        if not fut.done():
            raise RuntimeError('Call to set parameters for node "%s" failed'%(target_node_name)) 

if __name__ == '__main__':
    
    rclpy.init(args = sys.argv)
    node = rclpy.create_node('load_parameters_' + str(uuid.uuid4().hex), start_parameter_services = False,
                             allow_undeclared_parameters = False,
                             automatically_declare_parameters_from_overrides = True)
    spin_thread = threading.Thread(target= rclpy.spin, args = (node, ), daemon = True)
    try:
        spin_thread.start()    
        load_parameters_obj = LoadParameters(node)
    except Exception as ex:
        node.get_logger().error(str(ex))    
    rclpy.shutdown()
    spin_thread.join(timeout = 2.0)
    sys.exit(0)
    