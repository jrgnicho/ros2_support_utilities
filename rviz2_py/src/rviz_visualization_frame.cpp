/*
 * rviz_visualization_frame.cpp
 *
 *  Created on: Dec 27, 2021
 *      Author: Jorge Nicho
 */

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include "rviz2_py/rviz_visualization_frame.hpp"

namespace rviz2_py
{

RvizVisualizationFrame::RvizVisualizationFrame(const QString& node_name, const QString& node_namespace, QWidget *parent):
    //node_(std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name.toStdString())),
    RosComponent(node_name.toStdString()),
    rviz_common::VisualizationFrame(getNode(), parent)
{
}

RvizVisualizationFrame::~RvizVisualizationFrame()
{
}

void  RvizVisualizationFrame::initialize(const QString & display_config_file)
{

  auto logger = rclcpp::get_logger(getNode().lock()->get_node_name());
  // install logging handlers to route logging through ROS's logging system
  rviz_common::set_logging_handlers(
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", msg.c_str());
    }
  );
  rviz_common::VisualizationFrame::initialize(getNode(), display_config_file);
}

QWidget* RvizVisualizationFrame::getRenderPanel()
{
  return static_cast<QWidget*>(getManager()->getRenderPanel());
}

} /* namespace rviz2_py */
