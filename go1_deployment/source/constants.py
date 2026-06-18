import math
import numpy as np

POS_STOP_F = math.pow(10, 9)  # set q for torque control
VEL_STOP_F = 16000.0  # set dq for torque control
LOWLEVEL = 0xff
CONTROL_STEP = 0.005
POLICY_STEP = 0.02
OBS_LEN = 46
H = 5
KP = 30
robot_KP = np.array([
    30, 40, 55,  # FR
    30, 40, 55,  # FL
    30, 50, 70,  # RR
    30, 50, 70,  # RL
])
KD = 0.5
robot_KD = np.array([
    0.5, 0.6, 0.75,  # FR
    0.5, 0.6, 0.75,  # FL
    0.5, 0.6, 0.75,  # RR
    0.5, 0.6, 0.75,  # RL
])
KA = 0.25
INIT_STEPS = 100
INTERP_MODE = "linear"
INIT_CONTROL_MODE = "direct"
CONTROL_MODE = "hybrid"
SAFE_LEVEL = 4  # int from 1 (10%) to 9 (100%)
robot_Q_STANCE = np.array([
    -0.1, 0.8, -1.5,
    0.1, 0.8, -1.5,
    -0.1, 1.0, -1.5,
    0.1, 1.0, -1.5,
])
SIM_CFG = {
    "vel_lim": 0,
    "saturation_lim": 0,
    "tau_lim": 0,
    "init_pose": [0, 0, 0.4],
    "init_ori": [0, 0, 0],
    "lateral_friction": 0,
    "rolling_friction": 0,
    "policy_joint_order": {
        "FR_hip_joint": 1,
        "FL_hip_joint": 0,
        "RR_hip_joint": 3,
        "RL_hip_joint": 2,
        "FR_thigh_joint": 5,
        "FL_thigh_joint": 4,
        "RR_thigh_joint": 7,
        "RL_thigh_joint": 6,
        "FR_calf_joint": 9,
        "FL_calf_joint": 8,
        "RR_calf_joint": 11,
        "RL_calf_joint": 10,
    },
    "robot_joint_order": {
        "FR_hip_joint": 0,
        "FL_hip_joint": 3,
        "RR_hip_joint": 6,
        "RL_hip_joint": 9,
        "FR_thigh_joint": 1,
        "FL_thigh_joint": 4,
        "RR_thigh_joint": 7,
        "RL_thigh_joint": 10,
        "FR_calf_joint": 2,
        "FL_calf_joint": 5,
        "RR_calf_joint": 8,
        "RL_calf_joint": 11,
    },
    "pybullet_joint_order": {
        "RL_hip_joint": 0,
        "RL_thigh_joint": 1,
        "RL_calf_joint": 2,
        "FR_hip_joint": 3,
        "FR_thigh_joint": 4,
        "FR_calf_joint": 5,
        "RR_hip_joint": 6,
        "RR_thigh_joint": 7,
        "RR_calf_joint": 8,
        "FL_hip_joint": 9,
        "FL_thigh_joint": 10,
        "FL_calf_joint": 11,
    }

}
