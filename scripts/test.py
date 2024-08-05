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
from real_robot_init import RealRobotInit


## Constants

# Time history of observation buffer
H = 5
CONTROL_STEP = 0.002 # 500 Hz ~ 0.002 sec
POLICY_STEP = 0.02 # 50 Hz ~ 0.02 sec

PosStopF  = math.pow(10,9)
VelStopF  = 16000.0
LOWLEVEL  = 0xff

class Go1Env():

    def __init__(self):
        self.q_stand = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
        self.vel_cmd = np.zeros(3) # the policy input, used in concatenated obs
        self.a_cmd = np.zeros(12) # accel cmd from policy

        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.projected_gavity = np.zeros(3)

        # joint angles q [12] (self.q)
        # joint velocities qdot [12] (self.dq)
        # projected gravity [3] (self.projected_gravity)
        # velocity command (xdot, ydot, yaw rate) [3] (self.vel_cmd)
        # last policy output [12] (self.a_cmd)
        self.obs = np.zeros(42)

        self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)

        self.lowcmd = sdk.LowCmd()
        self.lowstate = sdk.LowState()
        self.udp.InitCmdData(self.lowcmd)

        for motor_id in range(12):
            self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id] # q_des
            self.lowcmd.motorCmd[motor_id].Kp = 0 # kp
        self.udp.SetSend(self.lowcmd)
        self.udp.Send()



        # PD Gains for robot
        # Kp=20, Kd=0.5, Ka=0.25
        self.kp = 20
        self.kd = 0.5
        self.ka = 0.25
        # Robot Startup object
        self.robot_init = RealRobotInit(self.udp, self.kp, self.kd)

        # # Safety Confirmation
        print("robot_init.starting_config = ", self.robot_init.starting_config)
        print("**************************************")
        print("*********    IMPORTANT    ************")
        print("    ENSURE STARTING CONFIG NONZERO    ")
        print("**************************************")
        print("**************************************")
        print("Then press any key ...")
        getch.getch()
        
        # Make Robot Stand
        self.robot_init.init_motion()

        print("Robot Init completed")
    
    def quat_rot_inv(self, body_quat, gravity):
        # print("body_quat.shape = ",body_quat.flatten().shape)
        q_w = body_quat.flatten()[0]
        # print("q_w = ",q_w)
        q_vec = body_quat.flatten()[1:]
        # print("q_vec.shape = ", q_vec.shape)
        # print("gravity.shape = ", gravity.shape)
        a = gravity * (2.0 * q_w**2 - 1.0)
        b = np.cross(q_vec, gravity) * q_w * 2.0
        c = q_vec * q_vec * gravity
        return a - b + c

    def step(self, vel_cmd, a_cmd):
        self.vel_cmd = vel_cmd
        self.a_cmd = a_cmd

        step_time = time.time()
        
        while ((time.time() - step_time) < POLICY_STEP):
            self.run_robot()

        # print("self.q.shape = ", self.q.shape)
        # print("self.dq.shape = ", self.dq.shape)
        # print("self.projected_gravity.shape = ", self.projected_gravity.shape)
        # print("self.vel_cmd.shape = ", self.vel_cmd.shape)
        # print("self.a_cmd.shape = ", self.a_cmd.shape)
        # Update self.obs and send back to policy
        self.obs = np.concatenate((self.q, self.dq, self.projected_gravity, self.vel_cmd, self.a_cmd), axis=0).flatten()
        # print("self.obs.shape = ", self.obs.shape)
        return self.obs

    def get_obs(self):
        self.udp.Recv()
        self.udp.GetRecv(self.lowstate)
        # print("q = ",np.array([motor.q for motor in self.lowstate.motorState[:12]]))
        # Joint pos
        self.q = np.array([motor.q for motor in self.lowstate.motorState[:12]])
        # Joint vel
        self.dq = np.array([motor.dq for motor in self.lowstate.motorState[:12]])
        # Body quaternion, normalized, (w,x,y,z)
        quat = np.array([self.lowstate.imu.quaternion]) 
        self.projected_gravity = self.quat_rot_inv(quat, np.array([0,0,-9.81]))
    
    def run_robot(self):
        cur_time = time.time()
        control_time = time.time()
        while ((time.time() - control_time) < CONTROL_STEP):
            pass

        self.get_obs()

        ## Option 1
        for motor_id in range(12):
            self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id] + self.ka*self.a_cmd[motor_id] # q_des
            self.lowcmd.motorCmd[motor_id].Kp = self.kp # kp
        ## Option 2
        # for motor_id in range(12):
        #     self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id] # q_des
        #     self.lowcmd.motorCmd[motor_id].Kp = self.kp # kp
        #     self.lowcmd.motorCmd[motor_id].dq = 0 # dq_des
        #     self.lowcmd.motorCmd[motor_id].Kd = self.kd # kd
        #     self.lowcmd.motorCmd[motor_id].tau = self.kp*self.ka*self.a_cmd[motor_id]  # FF torque
        ## Option 3
        # for motor_id in range(12):
        #    self.lowcmd.motorCmd[motor_id].tau = self.kp*(self.q_stand[motor_id] - self.q[motor_id]) + self.kd*(-self.dq[motor_id]) + self.kp*self.ka*self.a_cmd[motor_id]  # FF torque

        self.safe.PowerProtect(self.lowcmd, self.lowstate, 1)
        self.udp.SetSend(self.lowcmd)
        self.udp.Send()

def main():

    ## todo load the model

    env = Go1Env()

    steps = 0
    obs_history = np.zeros((42, H))

    vel_cmd = np.array([0,0,0]) # policy input
    q_cmd = np.zeros(12)# policy output
    ## todo 
    #     - init obs_history from real readings
    #     - check dims on everything

    while steps < 1000:

        obs = env.step(vel_cmd, q_cmd)
        obs = np.expand_dims(obs, axis=1)
        # print("obs_history.shape = ", obs_history.shape)
        # print("obs.shape = ", obs.shape)

        ## Organized such that newest obs goes on top
        ## and oldest at the bottom
        obs_history = np.delete(obs_history, -1, axis=1)
        # print("obs_history.shape = ", obs_history.shape)
        obs_history = np.append(obs, obs_history, axis=1)
        # print("obs_history.shape = ", obs_history.shape)

        ## Call policy; update vel_cmd and q_cmd


if __name__ == "__main__":
    main()
