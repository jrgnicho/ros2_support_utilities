/*
 * rclpy_pybind11.cpp
 *
 *  Created on: Dec 4, 2021
 *      Author: jnicho
 */


#include <pybind11/pybind11.h>

// may not be needed
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include <rcl_interfaces/msg/parameter.hpp>

#include "global_parameter_client.hpp"

namespace rclpy_parameter_utils
{
namespace py = pybind11;

  PYBIND11_MODULE(rclpy_parameter_utils, m)
  {
    m.doc() = "rclpy_parameter_utils library for support with global parameters";
    py::class_<GlobalParameterClient, std::shared_ptr<GlobalParameterClient> >(m,"GlobalParameterClient")
        .def(py::init<const std::string&, const std::string&>(),"GlobalParameterClient constructor",
             py::arg("node_name"), py::arg("node_namespace"))
        .def("get_parameter",&GlobalParameterClient::getParameter,"get parameter from global server",
             py::arg("remote_node_name"), py::arg("parameter_name"), py::arg("timeout_secs") = 1.0)
        .def("get_parameters", &GlobalParameterClient::getParameters,"get parameters from global server",
             py::arg("remote_node_name"), py::arg("parameter_names_list"), py::arg("timeout_secs") = 1.0)
        .def("set_parameter", &GlobalParameterClient::setParameter,"set parameter in global server",
             py::arg("remote_node_name"), py::arg("parameter_msg"), py::arg("timeout_secs") = 1.0)
        .def("set_parameters", &GlobalParameterClient::setParameters,"set several parameters in global server",
            py::arg("remote_node_name"), py::arg("parameters_msg_list"), py::arg("timeout_secs") = 1.0);
  }
}
