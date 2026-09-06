cmake_minimum_required(VERSION 3.12)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

# MuJoCo is OPTIONAL: only simulation backends use it. When it is not available
# (e.g. building on the real robot), the MuJoCo simulation sources are excluded
# and rmcs_core still builds with the fake (dependency-free) backend.
find_package(mujoco_vendor QUIET)

file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.c
)

if(NOT mujoco_vendor_FOUND)
  list(FILTER PROJECT_SOURCE EXCLUDE REGEX ".*/simulation/mujoco_[^/]*\\.cpp$")
endif()

ament_auto_add_library (
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/librmcs)

include_directories(SYSTEM "/usr/include/libusb-1.0")

target_link_libraries(${PROJECT_NAME} -lusb-1.0)

if(mujoco_vendor_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE RMCS_SIM_HAS_MUJOCO=1)
  target_link_libraries(${PROJECT_NAME} mujoco::mujoco)
endif()

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

# Simulation assets (MJCF models) for the MuJoCo backend.
install(DIRECTORY src/simulation/mjcf DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)