import datetime
import numpy as np
import onnxruntime as ort
import pickle
import time
from typing import Iterator

from .constants import H, OBS_LEN, POLICY_STEP
from .control_loop import Go1Env
from .utils import flatten_for_policy, robot_to_policy_joint_reorder

from go1_deployment import DIR_PATH


def vel_cmd_generator() -> Iterator[np.ndarray]:
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

    def start_robot(
            self,
            init_duration: float = 1.0,
            stance_duration: float = 5.0
    ) -> None:
        # init for 'init_duration' seconds before
        # standing up for 'stance_duration seconds
        zero_action = np.zeros_like(self.action)
        zero_vel = np.zeros_like(self.vel_cmd)
        start_time = time.time()
        while time.time() - start_time < init_duration:
            obs = self.env.get_obs(
                zero_action,
                self.gait_mode,
                zero_vel
            )
        q = (obs[6:18] + self.env.policy_q_stand).tolist()
        print("=== Robot Start ===")
        print("Initial Joint State:")
        print(q)
        print("Important: ensure nonzero joint State")
        print("Press enter to continue...")
        input()
        print("=== Begin Stance ===")
        self.init_stance(stance_duration, startup=True)
        print("=== Hold Stance ===")
        self.init_stance(stance_duration, startup=False)
        print("=== Policy Start ===")

    def action_smoothing(
            self,
            q_rel: np.ndarray,
            q_ref: np.ndarray,
            q_des: np.ndarray,
            steps: int,
            mode: str = "linear",
            action_space: str = "offset"
    ) -> np.ndarray:
        q = q_rel + q_ref
        delta_q = q_des - q
        if mode == "linear":
            q_next = q + (delta_q / steps)
        else:
            print(f"Error: action smoothing mode {mode} undefined")
            self.trigger_estop()
        print(f"q: {q}")
        print(f"q_next: {q_next}")
        print(f"q_next-q: {q_next-q}")
        if action_space == "offset":
            action = q_next - q_ref
        elif action_space == "position":
            action = q_next
        else:
            print(f"Error: action smoothing action space {action_space} undefined")
            self.trigger_estop()
        return action

    def init_stance(
            self,
            duration: float = 5.0,
            startup: bool = False
    ) -> None:
        # stand for 'duration' seconds before activating the policy
        init_action = np.zeros_like(self.action)
        zero_vel = np.zeros_like(self.vel_cmd)
        obs = self.env.get_obs(
            init_action,
            self.gait_mode,
            zero_vel,
        )
        start_time = time.time()
        while time.time() - start_time < duration:
            current_time = time.time()
            if startup:
                init_action = self.action_smoothing(
                    q_rel=obs[6:18],
                    q_ref=self.env.policy_q_stand,
                    q_des=self.env.policy_q_stand,
                    steps=100,
                    action_space="position"
                )
            # Actuate Robot
            while time.time() - current_time < POLICY_STEP:
                obs, lowstate = self.env.step(
                    init_action,
                    self.gait_mode,
                    zero_vel,
                    "position" if startup else "offset",
                )
            # Write Logs
            motors = lowstate.motorState[:12]
            self.logs.append({
                # policy order
                "time": current_time,
                "action": init_action,
                "q": obs[6:18],
                "dq": obs[18:30],
                "projected_gravity": obs[0:3],
                "vel_cmd": obs[3:6],
                "last_action": obs[30:42],
                "gait_mode": obs[42:46],
                "estimates": self.estimates,
                "ddq": robot_to_policy_joint_reorder(
                    np.array([motor.ddq for motor in motors])),
                "tauEst": robot_to_policy_joint_reorder(
                    np.array([motor.tauEst for motor in motors])),
                # robot order
                "wirelessRemote": lowstate.wirelessRemote[:],
                "footforce": lowstate.footForce[:],
                "footforceEst": lowstate.footForceEst[:],
                "quaternion": lowstate.imu.quaternion[:],
                "gyroscope": lowstate.imu.gyroscope[:],
                "accelerometer": lowstate.imu.accelerometer[:],
                "rpy": lowstate.imu.rpy[:],
            })

    def run(self) -> list:
        print("=== Policy Running ===")
        counter = 1
        time_pre_policy = time.time()
        time_post_policy = time.time()
        while not self.env.is_stopped:
            # Get Velocity Command
            if counter % (5 * int(1 / POLICY_STEP)) == 0:
                try:
                    self.vel_cmd = next(self.vel_cmd_gen)
                    print("New Command: {}".format(self.vel_cmd))
                except StopIteration:
                    self.env.trigger_estop()
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
            output = self.ort_session.run(None, {"obs": obs_flat})  # ~13000 Hz
            self.action = output[0].flatten()
            self.estimates = output[1].flatten()

            # Print Analytics
            print("Step {}:".format(counter))
            print("Loop: {} Hz".format(
                1 / (time_pre_policy - time_post_policy + 1e-8)
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
                "ddq": robot_to_policy_joint_reorder(
                    np.array([motor.ddq for motor in motors])),
                "tauEst": robot_to_policy_joint_reorder(
                    np.array([motor.tauEst for motor in motors])),
                # robot order
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
        with open(f"{DIR_PATH}/logs/{self.date}.pickle", "wb") as handle:
            pickle.dump(self.logs, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("=== Done ===")
