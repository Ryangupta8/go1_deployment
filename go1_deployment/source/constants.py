import math

POS_STOP_F = math.pow(10, 9)  # set q for torque control
VEL_STOP_F = 16000.0  # set dq for torque control
LOWLEVEL = 0xff
CONTROL_STEP = 0.005
POLICY_STEP = 0.02
OBS_LEN = 46
H = 5
KP = 30
KD = 0.5
KA = 0.25
INIT_STEPS = 100
INTERP_MODE = "linear"
INIT_CONTROL_MODE = "direct"
CONTROL_MODE = "hybrid"
SAFE_LEVEL = 1  # int from 1 (10%) to 9 (100%)
