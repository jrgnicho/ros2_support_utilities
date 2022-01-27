cmake_minimum_required(VERSION 3.5)
project(rclpy_parameter_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON) 
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

# Find python before pybind11
find_package(python_cmake_module REQUIRED)
find_package(PythonExtra REQUIRED)
set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

##############
# Build
##############
set(${PROJECT_NAME}_CPP_FILES src/global_parameter_client.cpp src/rclpy_pybind11.cpp src/rclcpp_component.cpp)
pybind11_add_module(${PROJECT_NAME} SHARED 
  ${${PROJECT_NAME}_CPP_FILES}) 
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME} PRIVATE 
  rclcpp::rclcpp
  rcpputils::rcpputils
  rcutils::rcutils
  pybind11::pybind11
  ${PythonExtra_LIBRARIES})

##############
# Install
##############
ament_python_install_package(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
  
install(PROGRAMS scripts/demo_parameters_client.py scripts/parameters_service_server.py scripts/load_ros_parameters.py
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")

ament_package()
