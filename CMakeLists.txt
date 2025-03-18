cmake_minimum_required(VERSION 3.10)
project(params_manager_cpp)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

# Params Manager library configuration
add_library(paramsmanager SHARED src/paramsmanager.cpp)
add_library(params_manager_cpp::params_manager_cpp ALIAS paramsmanager)
target_compile_features(paramsmanager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(paramsmanager PRIVATE "PARAMS_MANAGER_BUILDING_LIBRARY")
target_include_directories(paramsmanager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(paramsmanager
  rclcpp)

# Libraries installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS paramsmanager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# CMake files installation
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})

# Python scripts installation
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export all dependencies and library targets for this package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

# Configure package with ament extesions
ament_package(CONFIG_EXTRAS
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateInitParameters.cmake")
