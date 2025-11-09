# Mujoco 倒立摆仿真示例

## 基于 PID 算法的倒立摆控制

一个基于 Mujoco 仿真器的倒立摆控制示例，演示内容包括：

- 如何使用 AimRT_Mujoco_Sim 模块进行物理仿真；
- 如何编写控制算法模块并和 AimRT_Mujoco_Sim 仿真模块进行通信；
- 一个基础的单环 PID 控制算法实现对倒立摆的控制；


消息协议：
- [joint.proto](../../protocols/hardware/joint/joint.proto)
- [pid_control.proto](../../protocols/examples/inverted_pendulum/pid_control.proto)

核心代码：
- [pid_control_module.cc](./module/pid_control_module/pid_control_module.cc)
- [pid_control_algorithm.h](./module/pid_control_module/pid_control_algorithm.h)

配置文件：
- 通信配置文件：
  - [examples_inverted_pendulum_with_pid_control_cfg.yaml](./install/linux/bin/cfg/examples_inverted_pendulum_with_pid_control_cfg.yaml)
- 模型配置文件：
  - [inverted_pendulum_flat.yaml](./install/linux/bin/cfg/model/inverted_pendulum_flat.xml)

参数调节工具：
- [inverted_pendulum_pid_gui.py](./install/linux/bin/tools/inverted_pendulum_pid_gui.py)
  
运行方式（linux）：
- 开启 `AIMRT_MUJOCO_SIM_BUILD_EXAMPLES` 选项， 在项目根目录下执行`./build.sh -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=ON`， 编译成功后，进入build 目录；
- 在终端运行  `start_examples_inverted_pendulum_with_pid_control.sh` 启动AimRT_Mujoco_Sim 仿真模块和 PID 控制算法模块；
- 开启新的终端运行 `python3 ./tool/inverted_pendulum_pid_gui.py` 启动控制算法参数调节工具,程序运行默认设置 P=25000, I=0, D=120；
- 分别在开启的终端键入 `ctrl-c` 停止对应进程；

说明：
- 此示例在一个进程中创建了以下两个模块：
  - `InvertedPendulumSimModule`：基于 Mujoco 仿真模块，模拟倒立摆系统的动力学行为，接收控制模块的控制指令，并发布关节状态信息；
  - `InvertedPendulumControlModule`：实现 PID 控制算法，订阅关节状态并计算控制输出；
- 此示例将两个模块分别集成到 `mujoco_sim_pkg` 和 `inverted_pendulum_pkg` 两个 Pkg 中；
- 通过 protobuf 消息进行模块间的数据通信；

## 运行效果

- 仿真器启动后会显示 Mujoco 可视化窗口
- 通过调节控制算法参数调节工具，可以调整 PID 控制算法的参数，观察仿真器的运行效果；