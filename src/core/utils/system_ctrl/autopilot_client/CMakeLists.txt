cmake_minimum_required(VERSION 3.22)
project(autopilot_client)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

## ROS2 相关依赖
find_package(type_description_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rmcs_executor REQUIRED)
find_package(rmcs_msgs REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(rmcs_utility REQUIRED)
find_package(fast_tf REQUIRED)

## 目录包含
include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/src/

    ${type_description_interfaces_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${ament_index_cpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}

    ${rmcs_executor_INCLUDE_DIRS}
    ${rmcs_utility_INCLUDE_DIRS}
    ${rmcs_msgs_INCLUDE_DIRS}
    ${rmcs_description_INCLUDE_DIRS}
    ${fast_tf_INCLUDE_DIRS}
)

# file(GLOB_RECURSE AUTO_AIM_KERNEL
#     CONFIGURE_DEPENDS
#     ${PROJECT_SOURCE_DIR}/src/*.cpp
#     ${PROJECT_SOURCE_DIR}/src/*.cc
# )

# add_library(
#     ${PROJECT_NAME}_kernel SHARED
#     ${AUTO_AIM_KERNEL}
# )
# target_link_libraries(
#     ${PROJECT_NAME}_kernel
#     ${OpenCV_LIBS}
#     yaml-cpp::yaml-cpp
# )

## 组件构建: component
add_library(
    ${PROJECT_NAME}_component SHARED
    ${PROJECT_SOURCE_DIR}/src/component.cpp
)
target_link_libraries(
    ${PROJECT_NAME}_component
    # ${PROJECT_NAME}_kernel
    ${rmcs_executor_LIBRARIES}
)

## 安装配置
install(
    TARGETS
        ${PROJECT_NAME}_component
    DESTINATION
        lib/
)
# install(
#     DIRECTORY
#         config/
#         launch/
#     DESTINATION
#         share/${PROJECT_NAME}
# )

## plugins for rmcs_executor
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
    rmcs_executor plugins.xml
)

find_package(ament_cmake REQUIRED)
ament_package()
