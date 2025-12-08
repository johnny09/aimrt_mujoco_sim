#!/bin/bash

# exit on error and print each command
set -e

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

# cmake
deploy_folder=./deploy

conan install . --build=missing --profile:build=./profiles/local_release --profile:host=./profiles/local_release --deployer=full_deploy -of=${deploy_folder}

cmake -B ${deploy_folder}/build/Release/generators \
    -DCMAKE_INSTALL_PREFIX=${deploy_folder}/install \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5  \
    -DCMAKE_TOOLCHAIN_FILE=${deploy_folder}/build/Release/generators/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DAIMRT_MUJOCO_SIM_INSTALL=ON \
    -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=OFF \
    -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF
    $@

cmake --build ${deploy_folder}/build/Release/generators --target install --parallel $(nproc)


# cmake -B build \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DAIMRT_MUJOCO_SIM_INSTALL=ON \
#     -DCMAKE_INSTALL_PREFIX=./build/install \
#     -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=OFF \
#     -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF \
#     -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=ON \
#     -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
#     $@

# cmake --build build --config Release --target install --parallel $(nproc)
