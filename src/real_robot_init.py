# import sys
import numpy as np
import time
from ..third_party import robot_interface as sdk

# sys.path.append("../third_party/unitree_legged_sdk/lib/python/amd64")
# sys.path.append()


class RealRobotInit:
    def __init__(self, udp, kp, kd) -> None:
        print("RealRobotInit")
        self.q_curr = np.zeros(12)
        self.qdot_curr = np.zeros(12)

        self.starting_config = np.zeros(12)
        self.desired_config = np.zeros(12)
        self.jpos_cmd = np.zeros(12)

        self.curr_time = 0
        self.motion_dur = 1500  # 5 seconds

        self.kp = kp
        self.kd = kd

        self.desired_config = np.array([
            -0., 0.8, -1.5,  # FR
            0., 0.8, -1.5,  # FL
            -0., 1.0, -1.5,  # RR
            0., 1.0, -1.5])  # RL

        self.udp = udp
        self.lowcmd = sdk.LowCmd()
        self.udp.InitCmdData(self.lowcmd)

        self.get_init_config()

        self.flag = True
        # getch.getch()

    def smooth_changing(self, idx) -> None:
        # print("smooth_changing idx = ", idx)
        # print("desired_config[idx] = ", self.desired_config[idx])
        # print("starting_config[idx] = ", self.starting_config[idx])
        self.jpos_cmd[idx] = self.starting_config[idx] + \
            (self.desired_config[idx] - self.starting_config[idx]) * \
            0.5 * (1. - np.cos(self.curr_time / self.motion_dur * np.pi))
        if self.curr_time > self.motion_dur:
            self.jpos_cmd[idx] = self.desired_config[idx]
        # print("curr_time = ", self.curr_time)
        # print("motion_dur = ", self.motion_dur)
        # print("jpos_cmd[idx] = ", self.jpos_cmd[idx])

    def get_init_config(self) -> None:
        for idx in range(500):
            time.sleep(0.002)
            lowstate = sdk.LowState()
            self.udp.Recv()
            self.udp.GetRecv(lowstate)
            actuators = lowstate.motorState[:12]
            self.starting_config = np.array([motor.q for motor in actuators])
        self.curr_time = 0

    def set_desired_config(self, _config) -> None:
        self.desired_config = _config

    def get_robot_config(self) -> None:
        time.sleep(0.002)
        lowstate = sdk.LowState()
        self.udp.Recv()
        self.udp.GetRecv(lowstate)
        # print("lowstate = ", lowstate.motorState[0].q)
        actuators = lowstate.motorState[:12]
        self.q_curr = np.array([motor.q for motor in actuators])
        self.qdot_curr = np.array([motor.dq for motor in actuators])
        # print("q_curr = ", self.q_curr)

    def init_motion(self) -> None:
        # if self.flag:
        #   self.flag = False
        #   self.get_init_config()

        # We run controller at 100Hz
        k = 4
        while (self.curr_time < self.motion_dur):
            self.curr_time += 2

            for i in range(12):
                self.smooth_changing(i)
            # getch.getch()
            if k == 4:
                self.get_robot_config()
                for motor_id in range(12):
                    self.lowcmd.motorCmd[motor_id].q = self.jpos_cmd[motor_id]
                    self.lowcmd.motorCmd[motor_id].Kp = self.kp
                    self.lowcmd.motorCmd[motor_id].Kd = self.kd
                    k = 0
            else:
                k += 1
            self.udp.SetSend(self.lowcmd)
            self.udp.Send()

            time.sleep(0.002)

    def hold_pose(self, hold_time_sec) -> None:
        pass
        # while self.curr_time <= ((hold_time_sec * 1000) + self.motion_dur):
        #     self.curr_time += 2
        #     self.interface.send_command(self.command)
        #     # print(self.curr_time)
        #     threading.Event().wait(0.002)
