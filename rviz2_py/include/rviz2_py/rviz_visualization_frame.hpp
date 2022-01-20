/*
 * rviz_visualization_frame.hpp
 *
 *  Created on: Dec 27, 2021
 *      Author: Jorge Nicho
 */

#ifndef RVIZ2_PY_RVIZ_VISUALIZATION_FRAME_HPP_
#define RVIZ2_PY_RVIZ_VISUALIZATION_FRAME_HPP_


#include <rviz2_py/ros_component.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>


namespace rviz2_py
{

class RvizVisualizationFrame: public rviz_common::VisualizationFrame, rviz2_py::RosComponent
{
  Q_OBJECT
public:
  RvizVisualizationFrame(const QString& node_name, const QString& node_namespace, QWidget* parent = 0 );
  virtual ~RvizVisualizationFrame();

  /**
   * @brief call only after setting the Qt Application with the setApp method
   */
  void  initialize(const QString & display_config_file = "");

  /**
   * @brief Use only after initialization
   */
  QWidget* getRenderPanel();
};

} /* namespace rviz2_py */

#endif /* RVIZ2_PY_RVIZ_VISUALIZATION_FRAME_HPP_ */
