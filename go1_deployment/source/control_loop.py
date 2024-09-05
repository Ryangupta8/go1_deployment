import math
import numpy as np
from typing import Any, Literal

from .utils import (
    quat_rot_inv,
    robot_to_policy_joint_reorder,
    policy_to_robot_joint_reorder
)

from go1_deployment import robot_interface as sdk

PosStopF = math.pow(10, 9)
VelStopF = 16000.0
LOWLEVEL = 0xff


class Go1Env():
    def __init__(self) -> None:
        """
        Observation Space:
            - projected gravity [3] (projected_gravity)
            - velocity command (vx, vy, yaw rate) [3] (vel_cmd)
            - joint angles q [12] (q)
            - joint velocities dq [12] (dq)
            - last policy output [12] (action)
            - gait mode [4] (gait_mode)
        Action Space:
            - joint position offsets [12]
        Notation:
            - all vectors in robot joint order will be denoted by
              robot_<var_name>
            - all vectors in policy joint order will be denoted by
              policy_<var_name>
        """
        # Environment Data
        self.robot_q_stand = np.array([
            -0.1, 0.8, -1.5,
            0.1, 0.8, -1.5,
            -0.1, 1.0, -1.5,
            0.1, 1.0, -1.5,
        ])
        self.policy_q_stand = robot_to_policy_joint_reorder(self.robot_q_stand)
        self.gravity = np.array([0, 0, -9.81])
        self.gravity /= np.linalg.norm(self.gravity)  # normalized
        self.kp = 30
        self.kd = 0.5
        self.ka = 0.25
        self.stance_trigger = 1

        # Unitree Interface
        self.estop = 0
        self.udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe_level = 1  # int from 1 (10%) to 9 (100%)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)
        self.lowcmd = sdk.LowCmd()
        self.lowstate = sdk.LowState()
        self.udp.InitCmdData(self.lowcmd)

    def step(
            self,
            policy_action: np.ndarray,
            gait_mode: np.ndarray,
            vel_cmd_override: np.ndarray = None,
            control_mode: Literal["pd", "hybrid", "direct"] = "hybrid",
    ) -> tuple[np.ndarray, Any]:
        self.send_commands(policy_action, mode=control_mode)
        policy_obs = self.get_obs(
            policy_action,
            gait_mode,
            vel_cmd_override,
            control_mode,
        )
        # self.policy_last_obs = obs  # only used in direct torque control
        return policy_obs, self.lowstate

    def get_obs(
            self,
            policy_action: np.ndarray,
            gait_mode: np.ndarray,
            vel_cmd_override: np.ndarray,
            control_mode: Literal["pd", "hybrid", "direct"] = "hybrid",
    ) -> np.ndarray:
        self.udp.Recv()
        self.udp.GetRecv(self.lowstate)
        wirelessRemote = self.lowstate.wirelessRemote
        modifiers = self.get_modifiers_from_controller(wirelessRemote)
        self.estop, self.stance_trigger = modifiers
        if self.estop:
            self.trigger_estop()
        # Normalized Body Quaternion [w,x,y,z]
        quat = np.array([self.lowstate.imu.quaternion])
        projected_gravity = quat_rot_inv(quat, self.gravity)
        # Velocity Command
        if vel_cmd_override is not None:
            # Velocity Command from Generator
            vel_cmd = np.float32(vel_cmd_override.reshape((3,)))
        else:
            # Velocity Command from Controller
            vel_cmd = self.get_vel_cmd_from_controller(wirelessRemote)
        motors = self.lowstate.motorState[:12]
        # (Relative) Joint Position
        robot_q = np.array([motor.q for motor in motors])
        robot_q_rel = robot_q - self.robot_q_stand
        policy_q_rel = robot_to_policy_joint_reorder(robot_q_rel)
        # Joint Velocity
        robot_dq = np.array([motor.dq for motor in motors])
        policy_dq = robot_to_policy_joint_reorder(robot_dq)
        # Action
        if control_mode == "direct":
            policy_action -= self.policy_q_stand

        return np.concatenate((
            projected_gravity,
            vel_cmd,
            policy_q_rel,
            policy_dq,
            policy_action,
            gait_mode,
        ), axis=0).flatten()

    def send_commands(
            self,
            policy_action: np.ndarray,
            policy_q_rel: np.ndarray = None,
            policy_dq: np.ndarray = None,
            mode: Literal["pd", "hybrid", "direct"] = "hybrid",
    ) -> None:
        robot_action = policy_to_robot_joint_reorder(policy_action)

        # Mode 1 - Joint PD Control:
        if mode == "pd":
            for motor_id in range(12):
                q_default = self.robot_q_stand[motor_id]
                q_act = self.stance_trigger * self.ka * robot_action[motor_id]
                self.lowcmd.motorCmd[motor_id].q = q_default + q_act
                self.lowcmd.motorCmd[motor_id].Kp = self.kp
                self.lowcmd.motorCmd[motor_id].dq = 0
                self.lowcmd.motorCmd[motor_id].Kd = self.kd
        # Mode 2 - Hybrid Control:
        elif mode == "hybrid":
            for motor_id in range(12):
                q_default = self.robot_q_stand[motor_id]
                q_act = self.stance_trigger * self.ka * robot_action[motor_id]
                self.lowcmd.motorCmd[motor_id].q = q_default
                self.lowcmd.motorCmd[motor_id].Kp = self.kp
                self.lowcmd.motorCmd[motor_id].dq = 0
                self.lowcmd.motorCmd[motor_id].Kd = self.kd
                self.lowcmd.motorCmd[motor_id].tau = self.kp * q_act
        # Mode 3 - Direct Position Control:
        elif mode == "direct":
            for motor_id in range(12):
                self.lowcmd.motorCmd[motor_id].q = robot_action[motor_id]
                self.lowcmd.motorCmd[motor_id].Kp = self.kp
                self.lowcmd.motorCmd[motor_id].dq = 0
                self.lowcmd.motorCmd[motor_id].Kd = self.kd
        # Mode 4 - Direct Torque Control:
        # robot_q_rel = policy_to_robot_joint_reorder(policy_q_rel)
        # robot_dq = policy_to_robot_joint_reorder(policy_dq)
        # for motor_id in range(12):
        #     q_act = self.stance_trigger * self.ka * robot_action[motor_id]
        #     q_err = q_act - robot_q_rel[motor_id]  # note: q_rel
        #     dq_err = - robot_dq[motor_id]
        #     torque = self.kp * q_err + self.kd * dq_err
        #     self.lowcmd.motorCmd[motor_id].Kp = 0
        #     self.lowcmd.motorCmd[motor_id].Kd = 0
        #     self.lowcmd.motorCmd[motor_id].tau = torque
        else:
            print(f"Error: control mode {mode} undefined")
            self.trigger_estop()
        unsafe = self.safe.PowerProtect(
            self.lowcmd,
            self.lowstate,
            self.safe_level
        )
        if unsafe < 0:
            self.trigger_estop()
        self.udp.SetSend(self.lowcmd)
        self.udp.Send()

    def trigger_estop(self) -> None:
        self.estop = 1
        for motor_id in range(12):
            self.lowcmd.motorCmd[motor_id].Kp = 0
            self.lowcmd.motorCmd[motor_id].Kd = 0
            self.lowcmd.motorCmd[motor_id].tau = 0
        unsafe = self.safe.PowerProtect(
            self.lowcmd,
            self.lowstate,
            self.safe_level
        )
        if unsafe >= 0:
            self.udp.SetSend(self.lowcmd)
            self.udp.Send()
        raise SystemExit  # TODO remove

    @property
    def is_stopped(self) -> int:
        return self.estop

    def get_vel_cmd_from_controller(self, wirelessRemote) -> np.ndarray:
        vel_cmd = np.zeros(3, dtype=np.float32)
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
            vel_cmd[0] = intensity
        # if left stick being pressed backward
        elif (187 <= wirelessRemote[23] <= 191):
            if wirelessRemote[23] != 191:
                intensity = float(wirelessRemote[22]) / 384.
            elif wirelessRemote[23] == 191:
                intensity = float((wirelessRemote[22] + 256.)) / 384.
            vel_cmd[0] = -intensity
        else:
            vel_cmd[0] = 0.0

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
            vel_cmd[1] = -intensity
        # if left stick being pushed left
        elif (188 <= wirelessRemote[7] <= 191):
            if wirelessRemote[7] != 191:
                intensity = float(wirelessRemote[6] / 384)
            elif wirelessRemote[7] == 191:
                intensity = float((wirelessRemote[6] + 256) / 384)
            vel_cmd[1] = intensity
        else:
            vel_cmd[1] = 0.0

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
            vel_cmd[2] = -intensity
        # if right stick begin pushed left
        elif (188 <= wirelessRemote[11] <= 191):
            if wirelessRemote[11] != 191:
                intensity = float(wirelessRemote[10] / 384)
            elif wirelessRemote[11] == 191:
                intensity = float((wirelessRemote[10] + 256) / 384)
            vel_cmd[2] = intensity
        else:
            vel_cmd[2] = 0.0

        # Unused:
        # 39 appears to roughly be the right forwards/backwards
        # 15 is right stick forward/backwards (can get direction)
        # 14 is right stick forward/backwards (can maybe get intensity)
        # 21 is left strick diagonals but unclear mapping

        return vel_cmd

    def get_modifiers_from_controller(self, wirelessRemote) -> tuple[int, int]:
        estop = 0
        stance_trigger = 1
        if wirelessRemote[2] == 16:
            # R2 Right Trigger
            print("E-STOP TRIGGERED")
            estop = 1
            # exit(1)
        if wirelessRemote[2] == 32:
            # L2 Left Trigger
            stance_trigger = 0

        return estop, stance_trigger
