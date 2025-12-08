# Copyright (c) 2023, AgiBot Inc.
# All rights reserved

set_root_namespace("aimrt_mujoco_sim")

add_subdirectory(protocols)

add_subdirectory(module/mujoco_sim_module)

add_subdirectory(module/joy_stick_module)

add_subdirectory(pkg/mujoco_sim_pkg)

# install
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/linux)
else()
  message(FATAL_ERROR "Unsupport os")
endif()

install(
  DIRECTORY ${CUR_INSTALL_SOURCE_DIR}/bin
  DESTINATION ./
  USE_SOURCE_PERMISSIONS)
