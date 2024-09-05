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
