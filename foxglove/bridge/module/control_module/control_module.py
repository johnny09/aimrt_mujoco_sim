# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import aimrt_py
import yaml

import pndsdk2py.protocol.pb.sensor.joint_state_pb2 as joint_state_msg
import pndsdk2py.protocol.pb.actuator.joint_command_pb2 as joint_command_msg


import time
import math


class ControlModule(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.core = aimrt_py.CoreRef()
        self.logger = aimrt_py.LoggerRef()
        self.work_executor = aimrt_py.ExecutorRef()
        self.control_dt = 0.0025  # 400Hz
        self.robot_is_valid = False

    def Info(self):
        info = aimrt_py.ModuleInfo()
        info.name = "ControlModule"
        return info

    def Initialize(self, core):
        assert isinstance(core, aimrt_py.CoreRef)

        self.core = core
        self.logger = self.core.GetLogger()

        # log
        aimrt_py.info(self.logger, "Module initialize")

        try:
            # configure
            module_cfg_file_path = self.core.GetConfigurator().GetConfigFilePath()
            with open(module_cfg_file_path, "r") as file:
                data = yaml.safe_load(file)
                joint_state_topic_name = data["joint_state_topic_name"]
                joint_cmd_topic_name = data["joint_cmd_topic_name"]
                aimrt_py.info(self.logger, str(data))

            # subscriber
            subscriber = self.core.GetChannelHandle().GetSubscriber(
                joint_state_topic_name
            )
            assert (
                subscriber
            ), f"Get subscriber for topic '{joint_state_topic_name}' failed."

            def JointStateCallback(msg, logger=self.logger):
                

            aimrt_py.Subscribe(
                subscriber, joint_state_msg.JointStateArray, JointStateCallback
            )

            # publisher
            # Register publish type
            self.publisher = self.core.GetChannelHandle().GetPublisher(
                joint_cmd_topic_name
            )
            assert self.publisher, "Get publisher for topic '{}' failed.".format(
                joint_cmd_topic_name
            )
            aimrt_py.RegisterPublishType(
                self.publisher, joint_command_msg.JointCommandArray
            )

            # executor
            self.work_executor = self.core.GetExecutorManager().GetExecutor(
                "control_module_executor"
            )
            if not self.work_executor:
                aimrt_py.error(
                    self.logger, "Get executor 'control_module_executor' failed."
                )
                return False

        except Exception as e:
            aimrt_py.error(self.logger, "Initialize failed. {}".format(e))
            return False

        return True

    def Start(self):
        aimrt_py.info(self.logger, "Module start")
        self.run_flag = True
        try:
            self.work_executor.Execute(self.MainLoop)
        except Exception as e:
            aimrt_py.error(self.logger, "Start failed. {}".format(e))
            return False

        return True

    def MainLoop(self):
        step = 0.0

        while self.run_flag:
            loop_start_time = time.time()
            joints_cmd = joint_command_msg.JointCommandArray()
            joints_cmd.header.time_stamp = int(time.time() * 1e9)
            joints_cmd.header.frame_id = "joint_command_array"
            joints_cmd.joints.extend(
                [
                    joint_command_msg.JointCommand(
                        name="left_hip_pitch_joint",
                        position=math.pi * math.sin(step * 0.0025),
                        velocity=0.0,
                        effort=0.0,
                    )
                ]
            )

            aimrt_py.Publish(self.publisher, joints_cmd)

            step += 1.0

            loop_end_time = time.time()
            delta_time = loop_end_time - loop_start_time
            if delta_time < self.control_dt:
                time.sleep(self.control_dt - delta_time)
                self.counter_over_time = 0
            else:
                print("control loop over time.")
                self.counter_over_time += 1
            pass

    def Shutdown(self):
        self.run_flag = False
        aimrt_py.info(self.logger, "Module shutdown")
