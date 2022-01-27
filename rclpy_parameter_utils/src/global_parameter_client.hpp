/*
 * global_parameter_client.h
 *
 *  Created on: Dec 2, 2021
 *      Author: jnicho
 */

#ifndef INCLUDE_RCLPY_PARAMETER_UTILS_GLOBAL_PARAMETER_CLIENT_H_
#define INCLUDE_RCLPY_PARAMETER_UTILS_GLOBAL_PARAMETER_CLIENT_H_

#include <pybind11/pybind11.h>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter.hpp>

#include "rclcpp_component.hpp"


namespace rclpy_parameter_utils
{

class GlobalParameterClient: RclCppComponent
{
public:
  GlobalParameterClient(const std::string& node_name, const std::string& node_namespace = "");
  virtual ~GlobalParameterClient();

  pybind11::object getParameter(const std::string& remote_node_name,
                                              const std::string& parameter_name, double timeout_secs = 0.5);

  pybind11::dict getParameters(const std::string& remote_node_name, const std::vector<std::string>& parameter_names,
                               double timeout_secs = 0.5);

  /**
   * @brief sets a parameter value
   * @param remote_node_name name of the remote node holding the parameter
   * @param py_obj a python object of the type rclpy::Parameter
   * @param timeout_secs a timeout in seconds to wait for the global parameter server
   */
  void setParameter(const std::string &remote_node_name, const pybind11::object& py_obj,
                    double timeout_secs = 0.5);

  /**
   * @breif sets multiple parameters
   * @param remote_node_name name name of the remote node holding the parameter
   * @param py_obj_list List of parameters to set. Each item should be of the rclpy::Parameter python type
   * @param timeout_secs a timeout in seconds to wait for the global parameter server
   */
  void setParameters(const std::string &remote_node_name, const std::vector<pybind11::object>& py_obj_list,
                     double timeout_secs = 0.5);

protected:

  rclcpp::SyncParametersClient createParameterClient(rclcpp::Node::SharedPtr node,
                                                     const std::string& remote_node_name,
                                                     double timeout_secs);

  std::vector<rclcpp::Parameter> getParametersImpl(const std::string& remote_node_name,
                                    const std::vector<std::string>& parameter_names,
                                    double timeout_secs);

  void setParametersImpl(const std::string& remote_node_name,
                         const std::vector<rclcpp::Parameter>& parameters,
                         double timeout_secs);


  rclcpp::Node::SharedPtr node_;
};

} /* namespace rclpy */

#endif /* INCLUDE_RCLPY_PARAMETER_UTILS_GLOBAL_PARAMETER_CLIENT_H_ */
