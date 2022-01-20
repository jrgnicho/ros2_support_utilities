/*
 * RclComponent.hpp
 *
 *  Created on: Dec 29, 2021
 *      Author: Jorge Nicho
 */

#ifndef INCLUDE_RVIZ2_PY_ROS_COMPONENT_HPP_
#define INCLUDE_RVIZ2_PY_ROS_COMPONENT_HPP_

#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

namespace rviz2_py {

class RosComponent
{
public:
	RosComponent(const std::string node_name);
	virtual ~RosComponent();

	rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr getNode();

private:
	rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_;
	std::unique_ptr<rviz_common::ros_integration::RosClientAbstraction> ros_client_abstraction_;
};

} /* namespace rviz2_py */

#endif /* INCLUDE_RVIZ2_PY_ROS_COMPONENT_HPP_ */
