/*
 * RclComponent.cpp
 *
 *  Created on: Dec 29, 2021
 *      Author: Jorge Nicho
 */

#include <rviz2_py/ros_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>


const static int RCLCPP_ARGC = 0;
static std::vector<char*> RCLCPP_ARGV = {};

namespace rviz2_py {

RosComponent::RosComponent(const std::string node_name):
    ros_client_abstraction_(std::make_unique<rviz_common::ros_integration::RosClientAbstraction>())
{
  if(rclcpp::ok())
  {
    node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name);
  }
  else
  {
    node_ = ros_client_abstraction_->init(RCLCPP_ARGC, RCLCPP_ARGV.data(), node_name, false);
  }
}

RosComponent::~RosComponent()
{

}

rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr RosComponent::getNode()
{
  return node_;
}

} /* namespace rviz2_py */
