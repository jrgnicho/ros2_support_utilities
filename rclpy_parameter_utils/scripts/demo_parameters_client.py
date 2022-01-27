#!/usr/bin/env python3

import sys

import threading

import rclpy
import rclpy_parameter_utils

from rcl_interfaces.msg import Parameter as ParameterMsg

DEFAULT_PARAMETER_SERVER_NAME = 'global_parameter_server'


def main():    
    
    rclpy.init(args = sys.argv)
    node = rclpy.create_node('test_global_parameters_node')   
    global_param_client = None   
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()  
    
    global_parameter_server_name = node.get_parameter_or('parameter_server_name',
                                                           rclpy.Parameter('parameter_server_name', value = DEFAULT_PARAMETER_SERVER_NAME)).value
    #global_param_client = rclpy_parameter_utils.impl.GlobalParameterClient(node.get_name() + '_client','')
    global_param_client = rclpy_parameter_utils.GlobalParameterClient(node)
    
    
    # Getting Parameters
    nested_double_param = global_param_client.get_parameter(global_parameter_server_name, 'nested_param.another_nested_param.double_val')
    print('get_parameter results:\n\t "%s" = %s'%(nested_double_param.name, str(nested_double_param.value)))
    
    # Getting Multiple Parameters
    params_dict = global_param_client.get_parameters(global_parameter_server_name,['str_list_val',
                                                                                             'nested_param.another_nested_param.double_val',
                                                                                             'str_val',
                                                                                             'double_val'])
    print('\nget_parameter results:')
    print('\t' + str(dict(map(lambda p: (p[0], p[1].value), params_dict.items()))))
    
    # Setting Parameter
    new_param = rclpy.Parameter(nested_double_param.name,type_ = nested_double_param.type_, value = -1 *nested_double_param.value)
    global_param_client.set_parameter(global_parameter_server_name,new_param)
    new_param = global_param_client.get_parameter(global_parameter_server_name, new_param.name)
    print('\nset_parameter results:\n\t"%s" = %s'%(new_param.name, str(new_param.value)))
    
    # Setting Multiple Parameters
    params_list = []
    params_list.append( rclpy.Parameter('str_list_val', value = ['fuga', 'hoge', 'piyo']) )
    params_list.append( rclpy.Parameter('str_val', value = 'fuga'))
    params_list.append( rclpy.Parameter('nested_param.another_nested_param.double_val', value = 6.67E-11) )    
    global_param_client.set_parameters(global_parameter_server_name, params_list)
    print('\nset_parameter results:')
    print('\t' + str( dict(map(lambda p: (p.name, p.value),params_list))))
        
    try:  
        
        loop_rate = node.create_rate(10)
        while rclpy.ok():
            loop_rate.sleep()                  
            
    except KeyboardInterrupt as ex:
        
        # explicitly garbage collecting since rclpy.shutdown() appears to disrupt this process
        #del node
        #del global_param_client
        rclpy.shutdown()
        
    thread.join()    

if __name__ == '__main__':
    
    main()