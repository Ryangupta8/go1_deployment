import sys
import os
import numpy as np
import threading

# Pybind of the Unitree Legged SDK
sys.path.append('../third_party/unitree_legged_sdk-3.5.1')
# sys.path.append('../third_party/unitree_legged_sdk-3.8.0')
from robot_interface import RobotInterface

class RealRobotInit:

    def __init__(self, interface, kp, kd):
        print("RealRobotInit Constructor")
        self.q_curr = np.zeros(12)
        self.qdot_curr = np.zeros(12)

        self.starting_config = np.zeros(12)
        self.desired_config = np.zeros(12)
        self.jpos_cmd = np.zeros(12)

        self.curr_time = 0
        self.motion_dur = 1500 # 5 seconds

        self.kp = kp
        self.kd = kd
        
        ##########                      FR               FL              RR               RL
        self.desired_config = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])

        self.command = np.zeros(60, dtype=np.float32)
        self.interface = interface

        self.flag = True
        # getch.getch()

    def smooth_changing(self, idx):
        # print("smooth_changing idx = ", idx)
        # print("self.desired_config[idx] = ", self.desired_config[idx])
        # print("self.starting_config[idx] = ", self.starting_config[idx])
        self.jpos_cmd[idx] = self.starting_config[idx] + (self.desired_config[idx] - self.starting_config[idx]) * \
                             0.5 * (1. - np.cos(self.curr_time / self.motion_dur * np.pi))
        if self.curr_time > self.motion_dur:
            self.jpos_cmd[idx] = desired_config[idx]
        # print("curr_time = ", self.curr_time)
        # print("motion_dur = ", self.motion_dur)
        # print("self.jpos_cmd[idx] = ", self.jpos_cmd[idx])


    def get_init_config(self):

        for idx in range(50000):
            low_state = self.interface.receive_observation()
            self.starting_config = np.array([motor.q for motor in low_state.motorState[:12]])
            #threading.Event().wait(0.002)
        self.curr_time = 0

    def set_desired_config(self, _config):
        self.desired_config = _config


    def get_robot_config(self):
        self.low_state = self.interface.receive_observation()
        self.q_curr = np.array([motor.q for motor in self.low_state.motorState[:12]])
        self.qdot_curr = np.array([motor.dq for motor in self.low_state.motorState[:12]])


    def init_motion(self):
        # if self.flag:
            # self.flag = False
            # self.get_init_config()
        
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
                    self.command[motor_id * 5] = self.jpos_cmd[motor_id]
                    self.command[motor_id * 5 + 1] = self.kp
                    self.command[motor_id * 5 + 3] = self.kd
                    k = 0
            else:
                k += 1
            self.interface.send_command(self.command)
            raBodyQ = np.array([self.low_state.imu.quaternion])
            # print("raBodyQ = ", raBodyQ)
            threading.Event().wait(0.002)

    def hold_pose(self, hold_time_sec):
        while self.curr_time <= ((hold_time_sec * 1000) + self.motion_dur):
            self.curr_time += 2
            self.interface.send_command(self.command)
            # print(self.curr_time)
            threading.Event().wait(0.002)
