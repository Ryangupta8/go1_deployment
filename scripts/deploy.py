#!/usr/bin/python3

import os
import time
import sys
import math
import getch
import numpy as np
import pickle
# import torch
import datetime
import onnxruntime as ort


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
        self.vel_cmd = np.zeros(3, dtype=float) # the policy input, used in concatenated obs
        self.a_cmd = np.zeros(12, dtype=float) # accel cmd from policy

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
        # self.udp.SetSend(self.lowcmd)
        # self.udp.Send()



        # PD Gains for robot
        # Kp=20, Kd=0.5, Ka=0.25
        self.kp = 30
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
        # self.robot_init.init_motion()

        self.stance_trigger = 1

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

    def step(self, a_cmd):
        self.a_cmd = a_cmd

        step_time = time.time()
        
        while ((time.time() - step_time) < POLICY_STEP):
            self.run_robot()
            # print("POLICY STEP", (time.time() - step_time))

        # print("self.q.shape = ", self.q.shape)
        # print("self.dq.shape = ", self.dq.shape)
        # print("self.projected_gravity.shape = ", self.projected_gravity.shape)
        # print("self.vel_cmd.shape = ", self.vel_cmd.shape)
        # print("self.a_cmd.shape = ", self.a_cmd.shape)
        # Update self.obs and send back to policy
        self.obs = np.concatenate((self.q, self.dq, self.projected_gravity, self.vel_cmd, self.a_cmd), axis=0).flatten()
        # print("self.obs.shape = ", self.obs.shape)


        return self.obs, self.lowstate

    def get_obs(self):
        self.udp.Recv()
        self.udp.GetRecv(self.lowstate)
        # print("q = ",np.array([motor.q for motor in self.lowstate.motorState[:12]]))
        # Joint pos
        self.q = np.array([motor.q for motor in self.lowstate.motorState[:12]]) - self.q_stand  # q is relative to default config
        # Joint vel
        self.dq = np.array([motor.dq for motor in self.lowstate.motorState[:12]])
        # Body quaternion, normalized, (w,x,y,z)
        quat = np.array([self.lowstate.imu.quaternion]) 
        self.projected_gravity = self.quat_rot_inv(quat, np.array([0,0,-9.81]))

        wirelessRemote = self.lowstate.wirelessRemote
        ## Left Stick forward/backward == xdot command
        # 23 is left stick forward/backwards (can get direction 58-63 / 186-191)
        # 22 is left stick foward/backwards (can get intensity 129-0jump256-0-256jump0-128)
        # 58-63 --> pushing forward (62 to 63 is where we go from 256 to 0)
        # 186-191 --> pushing backwards (190 to 191 is where we go from 256 to 0)
        # if left stick being pressed forward
        #print(type(wirelessRemote[23]))
        #print(type(float(wirelessRemote[22])))
        print(float(wirelessRemote[22]))
        if (59 <= wirelessRemote[23] <= 63):
            if wirelessRemote[23] != 63:
                intensity = float(wirelessRemote[22]) / 384.
            elif wirelessRemote[23] == 63:
                intensity = float((wirelessRemote[22] + 256.)) / 384.
            print(intensity)
            self.vel_cmd[0] = intensity
        # if left stick being pressed backward
        elif (187 <= wirelessRemote[23] <= 191):
            if wirelessRemote[23] != 191:
                intensity = float(wirelessRemote[22]) / 384.
            elif wirelessRemote[23] == 191:
                intensity = float((wirelessRemote[22] + 256.)) / 384.
            self.vel_cmd[0] = -intensity
        else:
            self.vel_cmd[0] = 0.0

        ## Left Stick left/right == ydot command
        # 7 is left stick left/right (can get direction 58-63 / 188-191)
        # 6 is left stick left/right (can get intensity)
        # 58-63 --> pushing right (62 to 63 is where we go from 256 to 0)
        # 188-191 (controller slightly less sensitive) 
            # --> pushing left (190 to 191 is where we go from 256 to 0)
        # if left stick being pushed right
        if (59 <= wirelessRemote[7] <= 63):
            if wirelessRemote[7] != 63:
                intensity = float(wirelessRemote[6] / 384)
            elif wirelessRemote[7] == 63:
                intensity = float((wirelessRemote[6] + 256) / 384)
            self.vel_cmd[1] = -intensity
        # if left stick beign pushed left
        elif (188 <= wirelessRemote[7] <= 191):
            if wirelessRemote[7] != 191:
                intensity = float(wirelessRemote[6] / 384)
            elif wirelessRemote[7] == 191:
                intensity = float((wirelessRemote[6] + 256) / 384)
            self.vel_cmd[1] = intensity
        else:
            self.vel_cmd[1] = 0.0

        ## Right Stick left/right == omega command
        # 11 is right stick left/right (can get direction) 60; 188
        # 10 is right stick left/right (can get intensity) 
        # if right stick being pushed right
        if (60 <= wirelessRemote[11] <= 63):
            if wirelessRemote[11] != 63:
                intensity = float(wirelessRemote[10] / 384)
            elif wirelessRemote[11] == 63:
                intensity = float((wirelessRemote[10] + 256) / 384)
            self.vel_cmd[2] = -intensity
        # if right stick beign pushed left
        elif (188 <= wirelessRemote[11] <= 191):
            if wirelessRemote[11] != 191:
                intensity = float(wirelessRemote[10] / 384)
            elif wirelessRemote[11] == 191:
                intensity = float((wirelessRemote[10] + 256) / 384)
            self.vel_cmd[2] = intensity
        else:
            self.vel_cmd[2] = 0.0

        # print("velocity command = ", self.vel_cmd)

        # print("wirelessRemote = ", wirelessRemote[2])
        if wirelessRemote[2] == 16:
            ## R2 Right Trigger
            print("E-STOP TRIGGERED")
            exit(1)
        if wirelessRemote[2] == 32:
            ## L2 Left Trigger
            self.stance_trigger = 0
        else:
            self.stance_trigger = 1
        ## Unused
        # 39 appears to roughly be the right forwards/backwards
        # 15 is right stick forward/backwards (can get direction)
        # 14 is right stick forward/backwards (can maybe get intensity)
        # 21 is left strick diagonals but unclear mapping

    def run_robot(self):
        cur_time = time.time()
        control_time = time.time()
        while ((time.time() - control_time) < CONTROL_STEP):
            # print((time.time() - control_time))
            pass

        self.get_obs()

        ## Option 1
        # for motor_id in range(12):
        #     self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id] + self.ka*self.a_cmd[motor_id] # q_des
        #     self.lowcmd.motorCmd[motor_id].Kp = self.kp # kp
        ## Option 2
        for motor_id in range(12):
            self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id] # q_des
            self.lowcmd.motorCmd[motor_id].Kp = self.kp # kp
            self.lowcmd.motorCmd[motor_id].dq = 0 # dq_des
            self.lowcmd.motorCmd[motor_id].Kd = self.kd # kd
            self.lowcmd.motorCmd[motor_id].tau = self.stance_trigger*self.kp*self.ka*self.a_cmd[motor_id]  # FF torque
        ## Option 3 ## NOT STABLE
        # for motor_id in range(12):
        #    self.lowcmd.motorCmd[motor_id].tau = self.kp*(self.q_stand[motor_id] - self.q[motor_id]) + self.kd*(-self.dq[motor_id]) + self.kp*self.ka*self.a_cmd[motor_id]  # FF torque

        self.safe.PowerProtect(self.lowcmd, self.lowstate, 1)
        # self.udp.SetSend(self.lowcmd)
        # self.udp.Send()

def main():

    env = Go1Env()

    # Load onnx policy
    ort_session = ort.InferenceSession("policy.onnx")


    steps = 0
    obs_history = np.zeros((42, H))

    a_cmd = np.zeros(12, dtype=float) # policy output

    save_logs = []
    now = datetime.datetime.now()
    date = now.strftime("%b-%d-%Y_%H%M")

    # stand for 1 second before activating the policy
    for _ in range(50):
        obs, lowstate = env.step(a_cmd)
        # TODO add logging for initial stance as well

    while steps < 1000:

        # obs = np.concatenate(q, dq, projected_gravity, vel_cmd, a_cmd)
        # lowstate = unitree_legged_sdk::LowState
        obs, lowstate = env.step(a_cmd)
        obs = np.expand_dims(obs, axis=1)

        cur_time = time.time()

        # print("obs_history.shape = ", obs_history.shape)
        # print("obs.shape = ", obs.shape)

        ## Organized such that newest obs goes on top
        ## and oldest at the bottom
        obs_history = np.delete(obs_history, -1, axis=1)
        # print("obs_history.shape = ", obs_history.shape)
        obs_history = np.append(obs, obs_history, axis=1)
        # print("obs_history.shape = ", obs_history.shape)

        ## Call policy; update a_cmd
        a_cmd = ort_session.run(None, {"obs": obs_history.reshape(1, -1)})[0]

        save_logs.append({"time": cur_time, "a_cmd": a_cmd[:], "q": obs[:12], "dq": obs[12:24], \
                "ddq": lowstate.motorState.ddq[:], "tauEst": lowstate.motorState.tauEst[:], \
                "projected_gravity": obs[24:27], "vel_cmd": obs[27:30], "a_cmd": obs[-12:], \
                "wirelessRemote": lowstate.wirelessRemote[:], "footforce": lowstate.footForce[:], \
                "footforceEst": lowstate.footForceEst[:], "quaternion": lowstate.imu.quaternion[:], \
                "gyroscope": lowstate.imu.gyroscope[:], "accelerometer": lowstate.imu.accelerometer[:], \
                "rpy": lowstate.imu.rpy[:]})

    ## TODO does estop kill the logs? we should just break out of the loop
    ## and kill the motors but the logs should still be saved
    with open('{date}.pickle'.format(date=date), 'wb') as handle:
        pickle.dump(save_logs, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == "__main__":
    main()
