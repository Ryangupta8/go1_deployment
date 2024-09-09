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
