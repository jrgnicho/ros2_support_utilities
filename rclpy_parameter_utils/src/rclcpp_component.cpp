/*
 * rclcpp_component.cpp
 *
 *  Created on: Dec 4, 2021
 *      Author: jnicho
 */

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_component.hpp"

const static int RCLCPP_ARGC = 0;
const static std::vector<char*> RCLCPP_ARGV = {};

rclpy_parameter_utils::RclCppComponent::RclCppComponent()
{
  if(rclcpp::ok())
  {
    return;
  }
  rclcpp::init(RCLCPP_ARGC, RCLCPP_ARGV.data());
  RCLCPP_DEBUG(rclcpp::get_logger("rclpy_parameter_utils"), "initialized rclcpp");

}

rclpy_parameter_utils::RclCppComponent::~RclCppComponent()
{

}
