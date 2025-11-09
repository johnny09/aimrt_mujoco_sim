#!/bin/bash

# exit on error and print each command
set -e

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

# cmake
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DAIMRT_MUJOCO_SIM_INSTALL=ON \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=OFF \
    -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF \
    -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    $@

cmake --build build --config Release --target install --parallel $(nproc)
