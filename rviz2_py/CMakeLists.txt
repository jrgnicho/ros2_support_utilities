cmake_minimum_required(VERSION 3.8)
project(rviz2_py)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

#find_package(rclpy REQUIRED)
find_package(rviz_common REQUIRED)
find_package(python_qt_binding REQUIRED)
include(${python_qt_binding_DIR}/sip_helper.cmake)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

##############
# Build
##############

# creating build rules for calling QT's meta object compiler
# More on this here https://doc.qt.io/qt-5/qtcore-cmake-qt5-wrap-cpp.html
set(${PROJECT_NAME}_HEADERS_TO_MOC 
  include/rviz2_py/rviz_visualization_frame.hpp	
)
foreach(header "${${PROJECT_NAME}_HEADERS_TO_MOC}")
  qt5_wrap_cpp(${PROJECT_NAME}_MOC_FILES "${header}")
endforeach()

# Build the C++ library first
set(${PROJECT_NAME}_SRC 
  src/rviz_visualization_frame.cpp
  src/ros_component.cpp
)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_MOC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
 "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} rviz_common::rviz_common Qt5::Core Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} rviz_common)


## Install python stuff before doing anything with the sip build
ament_python_install_package(${PROJECT_NAME})

##############
# SIP Build Setup
##############
set(SIP_PROJECT_NAME ${PROJECT_NAME}_sip)
set("${SIP_PROJECT_NAME}_DEPENDENT_FILES"
  src/sip/property.sip
  src/sip/config.sip
  src/sip/display.sip
  src/sip/view_controller.sip
  src/sip/view_manager.sip
  src/sip/display_group.sip
  src/sip/visualization_manager.sip
  src/sip/panel_dock_widget.sip
  src/sip/tool.sip
  src/sip/yaml_config_reader.sip
  src/sip/rviz_visualization_frame.sip 
)

set(${SIP_PROJECT_NAME}_LDFLAGS_OTHER -std=c++17 -w)
set(${SIP_PROJECT_NAME}_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/include"
  ${EXPORT_HEADER_DIR}
  ${rviz_common_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

# These variables tell SIP where to find the C++ library built in this package
set(PROJECT_LIBRARY "lib${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
set(${SIP_PROJECT_NAME}_LIBRARIES  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_LIBRARY})
#set(${SIP_PROJECT_NAME}_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX})

# Finally build the python bindings
build_sip_binding(${SIP_PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/sip/rviz2_py.sip
  SIP_CONFIGURE ${PROJECT_SOURCE_DIR}/src/sip/sip_configure.py
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/sip
  LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  DEPENDS ${${SIP_PROJECT_NAME}_DEPENDENT_FILES}
  DEPENDENCIES rviz_common::rviz_common ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS rviz2_py/simple_rviz_app.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
