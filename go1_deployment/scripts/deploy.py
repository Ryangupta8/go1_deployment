#!/usr/bin/python3

import time
import numpy as np
import pickle
import datetime
import onnxruntime as ort

# import robot_interface as sdk
"""
from go1_deployment import (
    # H, POLICY_STEP, OBS_LEN,
    # flatten_for_policy,
    # Go1Env
"""
from constants import H, POLICY_STEP, OBS_LEN
from control_loop import Go1Env
from utils import flatten_for_policy


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


class Runner():
    def __init__(self, onnx_file: str) -> None:
        # Load Onnx Policy
        self.ort_session = ort.InferenceSession(onnx_file)

        # Instantiate Go1 Environment
        self.env = Go1Env()

        # Instantiate tracking Arrays
        self.obs_history = np.zeros((OBS_LEN, H), dtype=np.float32)
        self.obs_flat = flatten_for_policy(self.obs_history)
        output = self.ort_session.run(None, {"obs": self.obs_flat})
        self.action = np.zeros(output[0].flatten().shape, dtype=np.float32)
        self.estimates = np.zeros(output[1].flatten().shape, dtype=np.float32)
        self.date = datetime.datetime.now().strftime("%b-%d-%Y_%H%M")
        self.logs = []

        # Initialize High-Level Commands
        # one hot: [trot, pace, bound, pronk]
        self.gait_mode = np.array([1, 0, 0, 0], dtype=np.float32)
        self.vel_cmd_gen = vel_cmd_generator()
        self.vel_cmd = next(self.vel_cmd_gen)

    def init_stance(self, duration: float = 5.0) -> None:
        # stand for 'duration' seconds before activating the policy
        start_time = time.time()
        while time.time() - start_time < duration:
            current_time = time.time()
            # Actuate Robot
            while time.time() - current_time < POLICY_STEP:
                obs, lowstate = self.env.step(
                    self.action,
                    self.gait_mode,
                    self.vel_cmd
                )
            # Write Logs
            motors = lowstate.motorState[:12]
            self.logs.append({
                # policy order
                "time": current_time,
                "action": self.action,
                "q": obs[6:18],
                "dq": obs[18:30],
                "projected_gravity": obs[0:3],
                "vel_cmd": obs[3:6],
                "last_action": obs[30:42],
                "gait_mode": obs[42:46],
                "estimates": self.estimates,
                # robot order
                "ddq": np.array([motor.ddq for motor in motors]),
                "tauEst": np.array([motor.tauEst for motor in motors]),
                "wirelessRemote": lowstate.wirelessRemote[:],
                "footforce": lowstate.footForce[:],
                "footforceEst": lowstate.footForceEst[:],
                "quaternion": lowstate.imu.quaternion[:],
                "gyroscope": lowstate.imu.gyroscope[:],
                "accelerometer": lowstate.imu.accelerometer[:],
                "rpy": lowstate.imu.rpy[:],
            })
        print("=== Policy Start ===")

    def run(self) -> list:
        print("=== Policy Running ===")
        counter = 1
        time_pre_policy = time.time()
        time_post_policy = time.time()
        while not self.env.is_stopped:
            # Get Velocity Command
            if counter % (5 * int(1 / POLICY_STEP)) == 0:
                vel_cmd = next(self.vel_cmd_gen)
                print("New Command: {}".format(vel_cmd))
            if isinstance(vel_cmd, str):
                print(vel_cmd)
                break
            counter += 1

            # Actuate Robot
            time_pre_step = time.time()
            step_delta_time = POLICY_STEP - (time_pre_step - time_pre_policy)
            while time.time() - time_pre_step < step_delta_time:
                obs, lowstate = self.env.step(
                    self.action,
                    self.gait_mode,
                    self.vel_cmd
                )

            # Update Observation History
            obs = np.expand_dims(obs, axis=1)
            self.obs_history = np.delete(self.obs_history, -1, axis=1)
            self.obs_history = np.append(obs, self.obs_history, axis=1)
            obs_flat = flatten_for_policy(self.obs_history)

            # Update Action
            time_pre_policy = time.time()
            output = self.ort_session.run(None, {"obs": obs_flat})  # ~5000 Hz
            self.action = output[0].flatten()
            self.estimates = output[1].flatten()

            # Print Analytics
            print("Step {}:".format(counter))
            print("Loop: {} Hz".format(
                1 / (time_post_policy - time_pre_policy + 1e-8)
            ))
            time_post_policy = time.time()
            print("Policy: {} Hz".format(
                1 / (time_post_policy - time_pre_policy + 1e-8)
            ))
            print()

            # Write Logs TODO: convert all to policy order
            motors = lowstate.motorState[:12]
            self.logs.append({
                # policy order
                "time": time_pre_policy,
                "action": self.action,
                "q": obs[6:18],
                "dq": obs[18:30],
                "projected_gravity": obs[0:3],
                "vel_cmd": obs[3:6],
                "last_action": obs[30:42],
                "gait_mode": obs[42:46],
                "estimates": self.estimates,
                # robot order
                "ddq": np.array([motor.ddq for motor in motors]),
                "tauEst": np.array([motor.tauEst for motor in motors]),
                "wirelessRemote": lowstate.wirelessRemote[:],
                "footforce": lowstate.footForce[:],
                "footforceEst": lowstate.footForceEst[:],
                "quaternion": lowstate.imu.quaternion[:],
                "gyroscope": lowstate.imu.gyroscope[:],
                "accelerometer": lowstate.imu.accelerometer[:],
                "rpy": lowstate.imu.rpy[:]
            })
        print("=== Shutting Down ===")
        return self.logs

    def trigger_estop(self) -> None:
        print("=== Shutting Down ===")
        self.env.trigger_estop()

    def save_logs(self) -> None:
        print("=== Saving Logs ===")
        with open("logs/{date}.pickle".format(date=self.date), "wb") as handle:
            pickle.dump(self.logs, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("=== Done ===")


if __name__ == "__main__":
    onnx_file = "../onnx_models/hist5_trot_model_25000.onnx"
    runner = Runner(onnx_file)
    try:
        runner.init_stance()
        runner.run()
    except (SystemExit, KeyboardInterrupt):
        runner.trigger_estop()
    runner.save_logs()
