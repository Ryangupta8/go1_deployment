import matplotlib.pyplot as plt
import numpy as np
import pickle

from utils import list_to_dict


class Logger:
    def __init__(self, Ka: float = 0.25, f: float = 50) -> None:
        # Data
        self.Ka = Ka
        # Time
        self.dt = 1. / f
        self.t = []
        self.init_time = None
        # Projected Gravity
        self.proj_g = {
            "gx": [],
            "gy": [],
            "gz": [],
        }
        # Velocity Command
        self.vel_cmd = {
            "vx": [],
            "vy": [],
            "yaw rate": [],
        }
        # Joint Position
        self.q = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        # Default Joint Position
        self.q_des = {
            "FR_hip_joint": -0.1,
            "FL_hip_joint": 0.1,
            "RR_hip_joint": -0.1,
            "RL_hip_joint": 0.1,
            "FR_thigh_joint": 0.8,
            "FL_thigh_joint": 0.8,
            "RR_thigh_joint": 1.0,
            "RL_thigh_joint": 1.0,
            "FR_calf_joint": -1.5,
            "FL_calf_joint": -1.5,
            "RR_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
        }
        # Joint Velocity
        self.dq = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        # Action
        self.action = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        self.torque_des = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        self.torque_applied = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        self.estimate = {
            "vx": [],
            "vy": [],
            "vz": [],
            "avx": [],
            "avy": [],
            "avz": [],
        }

        # Plotters
        subplot_kw_args = {
            "sharex": True,
            # "sharey": True,
        }
        # Projected Gravity
        self.fig_proj_g, self.ax_proj_g = plt.subplots()
        # Velocity Command
        self.fig_vel_cmd, self.ax_vel_cmd = plt.subplots()
        # Joint Position
        self.fig_q, self.ax_q = plt.subplots(3, 1, **subplot_kw_args)
        # Joint Velocity
        self.fig_dq, self.ax_dq = plt.subplots(3, 1, **subplot_kw_args)
        # Action
        self.fig_action, self.ax_action = plt.subplots(3, 1, **subplot_kw_args)
        # Torque
        self.fig_torque, self.ax_torque = plt.subplots(3, 1, **subplot_kw_args)
        # Estimate
        self.fig_est, self.ax_est = plt.subplots(2, 1, **subplot_kw_args)

    def log(
            self,
            time_stamp: float,
            proj_g: list,
            vel_cmd: list,
            q: dict,
            dq: dict,
            action: dict,
            q_offset: dict,
            torque_des: dict,
            torque_applied: dict,
            estimate: list,
    ) -> None:
        # Time
        if self.init_time is None:
            self.init_time = time_stamp
        self.t.append(time_stamp - self.init_time)
        # Projected Gravity
        self.proj_g["gx"].append(proj_g[0])
        self.proj_g["gy"].append(proj_g[1])
        self.proj_g["gz"].append(proj_g[2])
        # Velocity Command
        self.vel_cmd["vx"].append(vel_cmd[0])
        self.vel_cmd["vy"].append(vel_cmd[1])
        self.vel_cmd["yaw rate"].append(vel_cmd[2])
        # Joint Position
        for k, v in q.items():
            self.q[k].append(v + q_offset[k])
        # Joint Velocity
        for k, v in dq.items():
            self.dq[k].append(v)
        # Action
        for k, v in action.items():
            self.action[k].append(self.Ka * v)
        # Torque
        for k, v in torque_des.items():
            self.torque_des[k].append(v)
        for k, v in torque_applied.items():
            self.torque_applied[k].append(v)
        # Estimate
        self.estimate["vx"].append(estimate[0])
        self.estimate["vy"].append(estimate[1])
        self.estimate["vz"].append(estimate[2])
        self.estimate["avx"].append(estimate[3])
        self.estimate["avy"].append(estimate[4])
        self.estimate["avz"].append(estimate[5])

    def plot(self) -> None:
        styles = {
            "FR": "tab:blue",
            "FL": "tab:orange",
            "RR": "tab:green",
            "RL": "tab:red",
        }
        # Projected Gravity
        for k, v in self.proj_g.items():
            self.ax_proj_g.plot(self.t, v, label=k)
        self.ax_proj_g.legend(loc="upper left")
        self.ax_proj_g.set_title("Projected Gravity")
        self.ax_proj_g.set_ylabel("Force [N]")
        self.ax_proj_g.set_xlabel("Time [s]")
        # Velocity Command
        for k, v in self.vel_cmd.items():
            self.ax_vel_cmd.plot(self.t, v, label=k)
        self.ax_vel_cmd.legend(loc="upper left")
        self.ax_vel_cmd.set_title("Velocity Command")
        self.ax_vel_cmd.set_ylabel("Velocity [m/s, rad/s]")
        self.ax_vel_cmd.set_xlabel("Time [s]")
        # Joint Position
        for k, v in self.q.items():
            c = styles[k.split("_")[0]]
            q_desired = np.array(self.action[k]) + self.q_des[k]
            if "hip" in k:
                self.ax_q[0].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_q[0].plot(self.t, q_desired, c=c, ls="--")
                self.ax_q[0].legend(loc="upper left")
            elif "thigh" in k:
                self.ax_q[1].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_q[1].plot(self.t, q_desired, c=c, ls="--")
                self.ax_q[1].legend(loc="upper left")
            elif "calf" in k:
                self.ax_q[2].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_q[2].plot(self.t, q_desired, c=c, ls="--")
                self.ax_q[2].legend(loc="upper left")
        self.ax_q[0].set_title("Joint Position")
        self.ax_q[1].set_ylabel("Angle [rad]")
        self.ax_q[2].set_xlabel("Time [s]")
        # Joint Velocity
        for k, v in self.dq.items():
            if "hip" in k:
                self.ax_dq[0].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_dq[0].legend(loc="upper left")
            elif "thigh" in k:
                self.ax_dq[1].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_dq[1].legend(loc="upper left")
            elif "calf" in k:
                self.ax_dq[2].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_dq[2].legend(loc="upper left")
        self.ax_dq[0].set_title("Joint Velocity")
        self.ax_dq[1].set_ylabel("Angular Rate [rad/s]")
        self.ax_dq[2].set_xlabel("Time [s]")
        # Action
        for k, v in self.action.items():
            if "hip" in k:
                self.ax_action[0].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_action[0].legend(loc="upper left")
            elif "thigh" in k:
                self.ax_action[1].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_action[1].legend(loc="upper left")
            elif "calf" in k:
                self.ax_action[2].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_action[2].legend(loc="upper left")
        self.ax_action[0].set_title("Action")
        self.ax_action[1].set_ylabel("Angle Offset [rad]")
        self.ax_action[2].set_xlabel("Time [s]")
        # Torque
        for k, v in self.torque_applied.items():
            c = styles[k.split("_")[0]]
            if "hip" in k:
                self.ax_torque[0].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_torque[0].plot(self.t, self.torque_des[k], c, ls="--")
                self.ax_torque[0].legend(loc="upper left")
            elif "thigh" in k:
                self.ax_torque[1].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_torque[1].plot(self.t, self.torque_des[k], c, ls="--")
                self.ax_torque[1].legend(loc="upper left")
            elif "calf" in k:
                self.ax_torque[2].plot(self.t, v, label=k.split("_joint")[0])
                self.ax_torque[2].plot(self.t, self.torque_des[k], c, ls="--")
                self.ax_torque[2].legend(loc="upper left")
        self.ax_torque[0].set_title("Applied Torque")
        self.ax_torque[1].set_ylabel("Torque [Nm]")
        self.ax_torque[2].set_xlabel("Time [s]")
        # Estimate
        for k, v in self.estimate.items():
            if "a" in k:
                self.ax_est[1].plot(self.t, v, label=k)
            else:
                self.ax_est[0].plot(self.t, v, label=k)
        self.ax_est[0].set_title("Velocity Estimates")
        self.ax_est[0].legend(loc="upper left")
        self.ax_est[1].legend(loc="upper left")
        self.ax_est[0].set_ylabel("Velocity [m/s]")
        self.ax_est[1].set_ylabel("Velocity [rad/s]")
        self.ax_est[1].set_xlabel("Time [s]")
        plt.show()


def read_pickled_data(
        logger: Logger,
        pickle_file: str,
        Kp: float = 30,
        Kd: float = 0.5,
        Ka: float = 0.25,
) -> None:
    with open(pickle_file, "rb") as file:
        data = pickle.load(file)
    for obs in data:
        t = obs["time"]
        proj_g = obs["projected_gravity"].flatten().tolist()
        vel_cmd = obs["vel_cmd"].flatten().tolist()
        q = list_to_dict(obs["q"].flatten().tolist())
        dq = list_to_dict(obs["dq"].flatten().tolist())
        action = list_to_dict(obs["action"].flatten().tolist())
        q_offset = {
            "FR_hip_joint": -0.1,
            "FL_hip_joint": 0.1,
            "RR_hip_joint": -0.1,
            "RL_hip_joint": 0.1,
            "FR_thigh_joint": 0.8,
            "FL_thigh_joint": 0.8,
            "RR_thigh_joint": 1.0,
            "RL_thigh_joint": 1.0,
            "FR_calf_joint": -1.5,
            "FL_calf_joint": -1.5,
            "RR_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
        }
        torque_des = {
            k: Kp * (q_offset[k] + Ka * action[k] - q[k]) + Kd * dq[k]
            for k in q_offset.keys()
        }
        torque_applied = list_to_dict(obs["tauEst"].tolist())
        estimate = obs["estimates"].flatten().tolist()
        logger.log(
            t,
            proj_g,
            vel_cmd,
            q,
            dq,
            action,
            q_offset,
            torque_des,
            torque_applied,
            estimate,
        )
    logger.plot()
