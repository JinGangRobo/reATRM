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

# MuJoCo is OPTIONAL: the simulation subsystem is MuJoCo-only. When it is not
# available (e.g. building on the real robot), the simulation sources are simply
# excluded and rmcs_core still builds for production hardware.
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

  # Optional native MuJoCo GUI (GLFW + OpenGL) viewer, compiled into the
  # MuJoCo backend when both are present. On systems without a display/GL the
  # GUI code is simply not built and the sim runs headless.
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(GLFW3 QUIET IMPORTED_TARGET glfw3)
  endif()
  if(GLFW3_FOUND)
    find_package(OpenGL QUIET)
  endif()
  if(GLFW3_FOUND AND OpenGL_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE RMCS_SIM_HAS_GUI=1)
    target_link_libraries(${PROJECT_NAME} PkgConfig::GLFW3 OpenGL::GL)
    message(STATUS "rmcs_core: MuJoCo native GUI enabled (GLFW ${GLFW3_VERSION})")
  else()
    message(STATUS "rmcs_core: MuJoCo native GUI disabled (glfw3/OpenGL not found)")
  endif()
endif()

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

# Simulation assets (MJCF models) for the MuJoCo backend.
install(DIRECTORY src/simulation/mjcf DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)