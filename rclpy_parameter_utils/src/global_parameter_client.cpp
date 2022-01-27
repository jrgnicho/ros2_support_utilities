/*
 * global_parameter_client.cpp
 *
 *  Created on: Dec 2, 2021
 *      Author: jnicho
 */

#include <boost/format.hpp>

#include <pybind11/stl.h>

#include "global_parameter_client.hpp"

namespace PythonTypes
{
  namespace py = pybind11;
  static py::object Parameter = py::module::import("rclpy").attr("Parameter");
  static py::object ParameterMsg = py::module::import("rcl_interfaces.msg").attr("Parameter");
  static py::object ParameterValueMsg = py::module::import("rcl_interfaces.msg").attr("ParameterValue");
}

static pybind11::object toPyObj(const rcl_interfaces::msg::ParameterValue& obj)
{
  namespace py = pybind11;
  py::object py_obj = PythonTypes::ParameterValueMsg();

  // copying
  py_obj.attr("type") = py::cast(obj.type);
  py_obj.attr("bool_value") = py::cast(obj.bool_value);
  py_obj.attr("integer_value") = py::cast(obj.integer_value);
  py_obj.attr("double_value") = py::cast(obj.double_value);
  py_obj.attr("string_value") = py::cast(obj.string_value);
  py_obj.attr("byte_array_value") = py::cast(obj.byte_array_value);
  py_obj.attr("bool_array_value") = py::cast(obj.bool_array_value);
  py_obj.attr("integer_array_value") = py::cast(obj.integer_array_value);
  py_obj.attr("double_array_value") = py::cast(obj.double_array_value);
  py_obj.attr("string_array_value") = py::cast(obj.string_array_value);
  return py_obj;
}

static pybind11::object toPyObj(const rcl_interfaces::msg::Parameter& obj)
{
  namespace py = pybind11;
  py::object py_obj = PythonTypes::ParameterMsg();

  // copying
  py_obj.attr("value") = toPyObj(obj.value);
  py_obj.attr("name") = py::cast(obj.name);
  return py_obj;
}

static pybind11::object toPyObj(const rclcpp::Parameter& obj)
{
  namespace py = pybind11;
  py::object py_param_msg = toPyObj(obj.to_parameter_msg());
  py::object py_param = PythonTypes::Parameter.attr("from_parameter_msg")(py_param_msg);
  return py_param;
}

template <class T>
T fromPyObj(const pybind11::object& py_obj)
{
  T obj;
  return obj;
}

template <>
rcl_interfaces::msg::ParameterValue fromPyObj(const pybind11::object& py_obj)
{
  rcl_interfaces::msg::ParameterValue obj;

  // copying
  obj.bool_value = py_obj.attr("bool_value").cast<decltype(obj.bool_value)>();
  obj.integer_value = py_obj.attr("integer_value").cast<decltype(obj.integer_value)>();
  obj.double_value = py_obj.attr("double_value").cast<decltype(obj.double_value)>();
  obj.string_value = py_obj.attr("string_value").cast<decltype(obj.string_value)>();
  obj.byte_array_value = py_obj.attr("byte_array_value").cast<decltype(obj.byte_array_value)>();
  obj.bool_array_value = py_obj.attr("bool_array_value").cast<decltype(obj.bool_array_value)>();
  obj.integer_array_value = py_obj.attr("integer_array_value").cast<decltype(obj.integer_array_value)>();
  obj.double_array_value = py_obj.attr("double_array_value").cast<decltype(obj.double_array_value)>();
  obj.string_array_value = py_obj.attr("string_array_value").cast<decltype(obj.string_array_value)>();
  obj.type = py_obj.attr("type").cast<decltype(obj.type)>();

  return obj;
}

template <>
rcl_interfaces::msg::Parameter fromPyObj(const pybind11::object& py_obj)
{
  rcl_interfaces::msg::Parameter obj;

  // copying
  obj.name = py_obj.attr("name").cast<decltype(obj.name)>();
  obj.value = fromPyObj<rcl_interfaces::msg::ParameterValue>(py_obj.attr("value"));
  return obj;
}

template <>
rclcpp::Parameter fromPyObj(const pybind11::object& py_obj)
{
  rcl_interfaces::msg::Parameter param_msg = fromPyObj<rcl_interfaces::msg::Parameter>(py_obj.attr("to_parameter_msg")());
  return rclcpp::Parameter::from_parameter_msg(param_msg);
}

namespace rclpy_parameter_utils
{

GlobalParameterClient::GlobalParameterClient(const std::string& node_name, const std::string& node_namespace):
    RclCppComponent(),
    node_(std::make_shared<rclcpp::Node>(node_name, node_namespace, rclcpp::NodeOptions()))
{

}

GlobalParameterClient::~GlobalParameterClient()
{

}

pybind11::object GlobalParameterClient::getParameter(const std::string& remote_node_name,
                                                                   const std::string& parameter_name, double timeout_secs)
{
  std::vector<rclcpp::Parameter> parameters = getParametersImpl(remote_node_name, {parameter_name}, timeout_secs);
  if(parameters.empty())
  {
    return toPyObj(rclcpp::Parameter());
  }

  return toPyObj(parameters.front());
}

pybind11::dict GlobalParameterClient::getParameters(const std::string& remote_node_name,
                                                    const std::vector<std::string>& parameter_names, double timeout_secs)
{
  std::vector<rclcpp::Parameter> parameters = getParametersImpl(remote_node_name, parameter_names, timeout_secs);
  pybind11::dict output_dict;
  for(const auto& p : parameters)
  {
    pybind11::object pystr = pybind11::cast(p.get_name(), pybind11::return_value_policy::copy); // TODO: May not be necessary to "copy"
    output_dict[pystr] = toPyObj(p);
  }
  return output_dict;
}

rclcpp::SyncParametersClient GlobalParameterClient::createParameterClient(rclcpp::Node::SharedPtr node,
                                                   const std::string& remote_node_name,
                                                   double timeout_secs)
{
  rclcpp::SyncParametersClient parameter_client = rclcpp::SyncParametersClient(node,remote_node_name);
  if(!parameter_client.wait_for_service(std::chrono::duration<double>(timeout_secs)))
  {
    const std::string error_msg = boost::str(boost::format("Failed to connect to parameter server node \"%s\"") % remote_node_name);
    throw std::runtime_error(error_msg);
  }
  return parameter_client;
}

std::vector<rclcpp::Parameter> GlobalParameterClient::getParametersImpl(const std::string& remote_node_name,
                                                                        const std::vector<std::string>& parameter_names,
                                                                        double timeout_secs)
{
  rclcpp::SyncParametersClient parameter_client = createParameterClient(node_,
                                                                        remote_node_name, timeout_secs);
  std::vector<rclcpp::Parameter> parameters = parameter_client.get_parameters(parameter_names);
  return parameters;
}

void GlobalParameterClient::setParameter(const std::string &remote_node_name, const pybind11::object& py_obj,
                                         double timeout_secs)
{
  setParametersImpl(remote_node_name, {fromPyObj<rclcpp::Parameter>(py_obj)}, timeout_secs);
}

void GlobalParameterClient::setParameters(const std::string &remote_node_name,
                                          const std::vector<pybind11::object>& py_obj_list,
                                          double timeout_secs)
{
  std::vector<rclcpp::Parameter> parameters;
  std::transform(py_obj_list.cbegin(), py_obj_list.cend(), std::back_inserter(parameters), [](const pybind11::object& py_obj){
    return fromPyObj<rclcpp::Parameter>(py_obj);
  });

  setParametersImpl(remote_node_name, parameters, timeout_secs);
}

void GlobalParameterClient::setParametersImpl(const std::string &remote_node_name,
                                              const std::vector<rclcpp::Parameter> &parameters,
                                              double timeout_secs)
{
  rclcpp::SyncParametersClient parameter_client = createParameterClient(node_,remote_node_name, timeout_secs);
  std::vector<rcl_interfaces::msg::SetParametersResult> parameter_results = parameter_client.set_parameters(parameters);
  for(const auto& res : parameter_results)
  {
    if(!res.successful)
    {
      throw std::runtime_error(res.reason);
    }
  }
}

} /* namespace rclpy */
