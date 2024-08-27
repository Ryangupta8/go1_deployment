#!/usr/bin/python3

import time
# import sys
import math
import getch
import numpy as np
import pickle
import datetime
import onnxruntime as ort
from typing import Any

# TODO: make this a proper python module so we dont need this
# sys.path.append("../third_party/unitree_legged_sdk/lib/python/amd64")
# sys.path.append("../src")
# sys.path.append("../models")

from ..third_party import robot_interface as sdk
from .. import RealRobotInit

# Constants

H = 5  # observation history length
CONTROL_STEP = 0.002  # 500 Hz ~ 0.002 sec
POLICY_STEP = 0.02  # 50 Hz ~ 0.02 sec
OBS_LEN = 46

PosStopF = math.pow(10, 9)
VelStopF = 16000.0
LOWLEVEL = 0xff


class Go1Env():
    def __init__(self) -> None:
        """
        Observation Space:
            - projected gravity [3] (self.projected_gravity)
            - velocity command (vx, vy, yaw rate) [3] (self.vel_cmd)
            - joint angles q [12] (self.q)
            - joint velocities dq [12] (self.dq)
            - last policy output [12] (self.a_cmd)
        Action Space:
            - joint position offsets [12]
        """
        self.q_stand = np.array([
            -0.1, 0.8, -1.5,
            0.1, 0.8, -1.5,
            -0.1, 1.0, -1.5,
            0.1, 1.0, -1.5,
        ])
        self.vel_cmd = np.zeros(3, dtype=np.float32)
        self.a_cmd = np.zeros(12, dtype=np.float32)
        self.gravity = np.array([0, 0, -9.81])
        self.gravity /= np.norm(self.gravity)  # normalized

        self.q = np.zeros(12, dtype=np.float32)
        self.dq = np.zeros(12, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)

        self.obs = np.zeros(OBS_LEN, dtype=np.float32)

        self.estop = 0

        self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)

        self.lowcmd = sdk.LowCmd()
        self.lowstate = sdk.LowState()
        self.udp.InitCmdData(self.lowcmd)

        for motor_id in range(12):
            self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id]
            self.lowcmd.motorCmd[motor_id].Kp = 0
        # self.udp.SetSend(self.lowcmd)
        # self.udp.Send()

        # PD Gains for robot
        # Kp=30, Kd=0.5, Ka=0.25
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
        self.robot_init.init_motion()

        self.stance_trigger = 1

        print("Robot Init completed")

    def quat_rot_inv(
            self,
            body_quat: np.ndarray,
            gravity: np.ndarray
    ) -> np.ndarray:
        q_w = body_quat.flatten()[0]
        q_vec = body_quat.flatten()[1:]
        a = gravity * (2.0 * q_w**2 - 1.0)
        b = np.cross(q_vec, gravity) * q_w * 2.0
        c = q_vec * q_vec * gravity
        return a - b + c

    def step(
            self,
            a_cmd: np.ndarray,
            gait_mode: np.ndarray,
            vel_cmd_override: np.ndarray = None,
    ) -> tuple[np.ndarray, Any]:
        self.a_cmd = a_cmd
        self.gait_mode = gait_mode
        self.vel_cmd_override = vel_cmd_override

        step_time = time.time()
        while ((time.time() - step_time) < POLICY_STEP):
            self.run_robot()

        # print("projected_gravity.shape = ", self.projected_gravity.shape)
        # print("vel_cmd.shape = ", self.vel_cmd.shape)
        # print("q.shape = ", self.q.shape)
        # print("dq.shape = ", self.dq.shape)
        # print("a_cmd.shape = ", self.a_cmd.shape)

        # Update self.obs and send back to policy
        self.obs = np.concatenate((
            self.projected_gravity,
            self.vel_cmd,
            self.q,
            self.dq,
            self.a_cmd,
            self.gait_mode,
        ), axis=0).flatten()
        # print("obs.shape = ", self.obs.shape)

        # TODO: find type(self.lowstate)
        return self.obs, self.lowstate

    def get_obs(self) -> None:
        self.udp.Recv()
        self.udp.GetRecv(self.lowstate)
        actuators = self.lowstate.motorState[:12]
        # Joint pos
        q = np.array([motor.q for motor in actuators])
        # print("q = ", q)
        self.q = q - self.q_stand  # q is relative to stance config
        self.q = self.robot_to_policy_joint_reorder(self.q)
        # Joint vel
        self.dq = np.array([motor.dq for motor in actuators])
        self.dq = self.robot_to_policy_joint_reorder(self.dq)
        # Body quaternion, normalized, (w,x,y,z)
        quat = np.array([self.lowstate.imu.quaternion])
        self.projected_gravity = self.quat_rot_inv(quat, self.gravity)

        wirelessRemote = self.lowstate.wirelessRemote
        # Left Stick forward/backward == vx command
        # 23 is left stick forward/backwards
        #   (can get direction 58-63 / 186-191)
        # 22 is left stick forward/backwards
        #   (can get intensity 129-0jump256-0-256jump0-128)
        # 58-63 --> pushing forward
        #   (62 to 63 is where we go from 256 to 0)
        # 186-191 --> pushing backwards
        #   (190 to 191 is where we go from 256 to 0)

        # if left stick being pressed forward
        if (59 <= wirelessRemote[23] <= 63):
            if wirelessRemote[23] != 63:
                intensity = float(wirelessRemote[22]) / 384.
            elif wirelessRemote[23] == 63:
                intensity = float((wirelessRemote[22] + 256.)) / 384.
            # print(intensity)
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

        # Left Stick left/right == vy command
        # 7 is left stick left/right
        #   (can get direction 58-63 / 188-191)
        # 6 is left stick left/right
        #   (can get intensity)
        # 58-63 --> pushing right
        #   (62 to 63 is where we go from 256 to 0)
        # 188-191
        #   (controller slightly less sensitive)
        # --> pushing left
        #   (190 to 191 is where we go from 256 to 0)

        # if left stick being pushed right
        if (59 <= wirelessRemote[7] <= 63):
            if wirelessRemote[7] != 63:
                intensity = float(wirelessRemote[6] / 384)
            elif wirelessRemote[7] == 63:
                intensity = float((wirelessRemote[6] + 256) / 384)
            self.vel_cmd[1] = -intensity
        # if left stick being pushed left
        elif (188 <= wirelessRemote[7] <= 191):
            if wirelessRemote[7] != 191:
                intensity = float(wirelessRemote[6] / 384)
            elif wirelessRemote[7] == 191:
                intensity = float((wirelessRemote[6] + 256) / 384)
            self.vel_cmd[1] = intensity
        else:
            self.vel_cmd[1] = 0.0

        # Right Stick left/right == yaw rate command
        # 11 is right stick left/right
        #   (can get direction 60; 188)
        # 10 is right stick left/right
        #   (can get intensity)

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
            # R2 Right Trigger
            print("E-STOP TRIGGERED")
            self.estop = 1
            # exit(1)
        if wirelessRemote[2] == 32:
            # L2 Left Trigger
            self.stance_trigger = 0
        else:
            self.stance_trigger = 1

        # Unused:
        # 39 appears to roughly be the right forwards/backwards
        # 15 is right stick forward/backwards (can get direction)
        # 14 is right stick forward/backwards (can maybe get intensity)
        # 21 is left strick diagonals but unclear mapping
        if self.vel_cmd_override is not None:
            self.vel_cmd = np.float32(self.vel_cmd_override.reshape((3,)))

    def run_robot(self) -> None:
        control_time = time.time()
        while ((time.time() - control_time) < CONTROL_STEP):
            # print((time.time() - control_time))
            pass

        self.get_obs()

        a_cmd_robot = self.policy_to_robot_joint_reorder(self.a_cmd)

        # Option 1:
        # for motor_id in range(12):
        #     q_act = self.stance_trigger * self.ka * a_cmd_robot[motor_id]
        #     self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id] + q_act
        #     self.lowcmd.motorCmd[motor_id].Kp = self.kp # kp
        # Option 2:
        for motor_id in range(12):
            q_act = self.stance_trigger * self.ka * a_cmd_robot[motor_id]
            self.lowcmd.motorCmd[motor_id].q = self.q_stand[motor_id]
            self.lowcmd.motorCmd[motor_id].Kp = self.kp
            self.lowcmd.motorCmd[motor_id].dq = 0
            self.lowcmd.motorCmd[motor_id].Kd = self.kd
            self.lowcmd.motorCmd[motor_id].tau = self.kp * q_act

        self.safe.PowerProtect(self.lowcmd, self.lowstate, 1)
        self.udp.SetSend(self.lowcmd)
        self.udp.Send()

    def robot_to_policy_joint_reorder(self, inp) -> np.ndarray:
        policy_joint_order = np.zeros(12, dtype=np.float32)
        policy_joint_order[0] = inp[3]  # FL Hip
        policy_joint_order[1] = inp[0]  # FR Hip
        policy_joint_order[2] = inp[9]  # RL Hip
        policy_joint_order[3] = inp[6]  # RR Hip
        policy_joint_order[4] = inp[4]  # FL Thigh
        policy_joint_order[5] = inp[1]  # FR Thigh
        policy_joint_order[6] = inp[10]  # RL Thigh
        policy_joint_order[7] = inp[7]  # RR Thigh
        policy_joint_order[8] = inp[5]  # FL Calf
        policy_joint_order[9] = inp[2]  # FR Calf
        policy_joint_order[10] = inp[11]  # RL Calf
        policy_joint_order[11] = inp[8]  # RR Calf

        return policy_joint_order

    def policy_to_robot_joint_reorder(self, inp) -> np.ndarray:
        robot_joint_order = np.zeros(12, dtype=np.float32)
        robot_joint_order[0] = inp[1]  # FR Hip
        robot_joint_order[1] = inp[5]  # FR Thigh
        robot_joint_order[2] = inp[9]  # FR Calf
        robot_joint_order[3] = inp[0]  # FL Hip
        robot_joint_order[4] = inp[4]  # FL Thigh
        robot_joint_order[5] = inp[8]  # FL Calf
        robot_joint_order[6] = inp[3]  # RR Hip
        robot_joint_order[7] = inp[7]  # RR Thigh
        robot_joint_order[8] = inp[11]  # RR Calf
        robot_joint_order[9] = inp[2]  # RL Hip
        robot_joint_order[10] = inp[6]  # RL Thigh
        robot_joint_order[11] = inp[10]  # RL Calf

        return robot_joint_order


def flatten_for_policy(obs) -> np.ndarray[np.float32]:
    proj_g = obs[0:3, :].reshape((1, -1))
    vel_cmd = obs[3:6, :].reshape((1, -1))
    q = obs[6:18, :].reshape((1, -1))
    dq = obs[18:30, :].reshape((1, -1))
    last_action = obs[30:42, :].reshape((1, -1))
    gait_mode = obs[42:46, :].reshape((1, -1))
    print("proj_g.shape = ", proj_g.shape)
    print("vel_cmd.shape = ", vel_cmd.shape)
    print("q.shape = ", q.shape)
    print("dq.shape = ", dq.shape)
    print("last_action.shape = ", last_action.shape)
    print("gait_mode.shape = ", gait_mode.shape)
    obs_flat = np.concatenate((proj_g, vel_cmd, q, dq, last_action, gait_mode), axis=1)
    print("obs_flat.shape = ", obs_flat.shape)
    print("obs_flat.flatten().shape = ", obs_flat.flatten().shape)
    return np.float32(obs_flat)


def vel_cmd_generator():
    commands = [
        np.array([0, 0, 0], dtype=np.float32),  # stance
        np.array([0.5, 0, 0], dtype=np.float32),  # forward
        np.array([-0.5, 0, 0], dtype=np.float32),  # backward
        np.array([0, 0.5, 0], dtype=np.float32),  # left
        np.array([0, -0.5, 0], dtype=np.float32),  # right
        np.array([0, 0, 0.5], dtype=np.float32),  # ccw
        np.array([0, 0, -0.5], dtype=np.float32),  # cw
        np.array([0, 0, 0], dtype=np.float32),  # stance
    ]
    for command in commands:
        yield command
    return "Test Commands Finished: Done."


def main() -> None:

    # Load onnx policy
    # ort_session = ort.InferenceSession("../models/model_94000.onnx")
    ort_session = ort.InferenceSession("../onnx_models/hist5_trot_model_25000.onnx")

    env = Go1Env()

    obs_history = np.zeros((OBS_LEN, H), dtype=np.float32)

    a_cmd = np.zeros(12, dtype=np.float32)
    gait_mode = np.array([1, 0, 0, 0], dtype=np.float32)

    save_logs = []
    now = datetime.datetime.now()
    date = now.strftime("%b-%d-%Y_%H%M")
    current_time = time.time()

    # stand for 5 second before activating the policy
    for _ in range(5 * int(1 / POLICY_STEP)):
        obs, lowstate = env.step(a_cmd, gait_mode)
        ddq = np.array([motor.ddq for motor in lowstate.motorState[:12]])
        tauEst = np.array([motor.tauEst for motor in lowstate.motorState[:12]])
        save_logs.append({
            "time": current_time,
            "q": obs[6:18],
            "dq": obs[18:30],
            "ddq": ddq,
            "tauEst": tauEst,
            "projected_gravity": obs[0:3],
            "vel_cmd": obs[3:6],
            "a_cmd": obs[30:42],
            "gait_mode": obs[42:46],
            "wirelessRemote": lowstate.wirelessRemote[:],
            "footforce": lowstate.footForce[:],
            "footforceEst": lowstate.footForceEst[:],
            "quaternion": lowstate.imu.quaternion[:],
            "gyroscope": lowstate.imu.gyroscope[:],
            "accelerometer": lowstate.imu.accelerometer[:],
            "rpy": lowstate.imu.rpy[:],
        })

    print("Policy Start...")

    vel_cmd = np.array([0, 0, 0], dtype=np.float32)
    counter = 0
    while not env.estop:
        if counter % (5 * int(1 / POLICY_STEP)) == 0:
            vel_cmd = vel_cmd_generator()
        if isinstance(vel_cmd, str):
            print(vel_cmd)
            break
        counter += 1

        # obs = np.concatenate(q, dq, projected_gravity, vel_cmd, a_cmd)
        # lowstate = unitree_legged_sdk::LowState
        obs, lowstate = env.step(a_cmd, gait_mode, vel_cmd_override=vel_cmd)
        obs = np.expand_dims(obs, axis=1)

        current_time = time.time()

        # print("obs_history.shape = ", obs_history.shape)
        # print("obs.shape = ", obs.shape)
        print(obs[3:6])
        print()
        # Organized such that newest obs goes on top
        # and oldest at the bottom is deleted
        obs_history = np.delete(obs_history, -1, axis=1)
        # print("obs_history.shape = ", obs_history.shape)
        obs_history = np.append(obs, obs_history, axis=1)
        # print("obs_history.shape = ", obs_history.shape)

        # Kyle's method of flattening the obs
        obs_flat = flatten_for_policy(obs_history)

        # Call policy; update a_cmd
        a_cmd = ort_session.run(None, {"obs": obs_flat})[0].flatten()

        ddq = np.array([motor.ddq for motor in lowstate.motorState[:12]])
        tauEst = np.array([motor.tauEst for motor in lowstate.motorState[:12]])
        # TODO: pick convention, either unitree or isaac for all logs
        save_logs.append({
            "time": current_time,
            "a_cmd": a_cmd[:],
            "q": obs[6:18],
            "dq": obs[18:30],
            "ddq": ddq,
            "tauEst": tauEst,
            "projected_gravity": obs[0:3],
            "vel_cmd": obs[3:6],
            "last_a_cmd": obs[30:42],
            "gait_mode": obs[42:46],
            "wirelessRemote": lowstate.wirelessRemote[:],
            "footforce": lowstate.footForce[:],
            "footforceEst": lowstate.footForceEst[:],
            "quaternion": lowstate.imu.quaternion[:],
            "gyroscope": lowstate.imu.gyroscope[:],
            "accelerometer": lowstate.imu.accelerometer[:],
            "rpy": lowstate.imu.rpy[:]
        })

    with open('{date}.pickle'.format(date=date), 'wb') as handle:
        pickle.dump(save_logs, handle, protocol=pickle.HIGHEST_PROTOCOL)


if __name__ == "__main__":
    main()
