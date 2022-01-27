/*
 * rclcpp_helper.h
 *
 *  Created on: Dec 4, 2021
 *      Author: jnicho
 */

#ifndef INCLUDE_RCLPY_PARAMETER_UTILS_RCLCPP_COMPONENT_H_
#define INCLUDE_RCLPY_PARAMETER_UTILS_RCLCPP_COMPONENT_H_

#include <rclcpp/rclcpp.hpp>

namespace rclpy_parameter_utils
{
/**
 * @class rclpy_parameter_utils::RclCppComponent
 * @details class used to initialize rclcpp, all classes that use ros in any way should inherit from this
 */
class RclCppComponent
{
public:
  RclCppComponent();

  virtual ~RclCppComponent();
};
}

#endif /* INCLUDE_RCLPY_PARAMETER_UTILS_RCLCPP_COMPONENT_H_ */
