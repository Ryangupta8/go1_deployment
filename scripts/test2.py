#!/usr/bin/python3

import os
import time
import sys
import math
import getch
import numpy as np
# import torch


sys.path.append('../third_party/unitree_legged_sdk/lib/python/amd64')
sys.path.append('../src')
sys.path.append('../models')

import robot_interface as sdk

## Constants

# Time history of observation buffer
H = 5
CONTROL_STEP = 0.002 # 500 Hz ~ 0.002 sec
POLICY_STEP = 0.02 # 50 Hz ~ 0.02 sec

PosStopF  = math.pow(10,9)
VelStopF  = 16000.0
LOWLEVEL  = 0xff

def smooth_changing(q_start, q_stand, idx, cur_time, motion_dur):
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


if __name__ == '__main__':
    
    ##########                      FR               FL              RR               RL
    q_stand = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
    vel_cmd = np.zeros(3) # the policy input, used in concatenated obs
    a_cmd = np.zeros(12) # accel cmd from policy

    q = np.zeros(12)
    dq = np.zeros(12)
    projected_gavity = np.zeros(3)

    # PD Gains for robot
    # Kp=20, Kd=0.5, Ka=0.25
    kp = 20
    kd = 0.5
    ka = 0.25

    motion_dur = 1500 # 5 seconds for robot init
    cur_time = 0

    # joint angles q [12] (self.q)
    # joint velocities qdot [12] (self.dq)
    # projected gravity [3] (self.projected_gravity)
    # velocity command (xdot, ydot, yaw rate) [3] (self.vel_cmd)
    # last policy output [12] (self.a_cmd)
    obs = np.zeros(42)
    
    
    udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
    safe = sdk.Safety(sdk.LeggedType.Go1)
    
    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)

    init_flag = 0

    while not init_flag:
        udp.Recv()
        udp.GetRecv(state)
        q_start = np.array([motor.q for motor in state.motorState[:12]])
        print("q_start = ", q_start)
        print("q_des = ", q_stand)

        while cur_time < motion_dur:

            cur_time += 2

            for i in range(12):
                smooth_changing(q_start, q_stand, i, cur_time, motion_dur)