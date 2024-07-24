import os
import getch
import numpy as np
import torch


sys.path.append('../third_party/unitree_legged_sdk-3.5.1')
# sys.path.append('../third_party/unitree_legged_sdk-3.8.0')
sys.path.append('../src')
sys.path.append('../models')

from robot_interface import RobotInterface
from real_robot_init import RealRobotInit


## Constants

# Time history of observation buffer
H = 5
CONTROL_STEP = 0.002 # 500 Hz ~ 0.002 sec
POLICY_STEP = 0.02 # 50 Hz ~ 0.02 sec

class Go1Env():

    def __init__(self):
        self.q_stand = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
        self.vel_cmd = np.zeros(3)
        self.q_cmd = np.zeros(12)

        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.projected_gavity = np.zeros(3)

        # joint angles q [12] (self.q)
        # joint velocities qdot [12] (self.dq)
        # projected gravity (quat_rot_inv(robot_base_quat, [0,0,-9.81]) ) [3] (self.projected_gravity)
        # velocity command (xdot, ydot, yaw rate) [3] (self.vel_cmd)
        # last policy output [12] (self.q_cmd)
        self.obs = np.zeros(42)

        self.interface = RobotInterface()

        # PD Gains for robot
        self.kp = np.array([130 , 110 , 140 , 130 , 110 , 140 , 130, 110, 140 , 130 , 110, 140])*0.725# 0.6 # 0.8
        self.kd = np.array([0.5, 0.3, 0.2, 0.5, 0.3, 0.2, 0.5, 0.3, 0.2, 0.5, 0.3, 0.2])*5 # 5 # 7
        # Robot Startup object
        self.robot_init = RealRobotInit(self.interface, self.kp, self.kd)
        self.robot_init.get_init_config()

        # Safety Confirmation
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

        print("Robot Init completed")
    
    def quat_rot_inv(self, body_quat):
        pass
        ## todo

    def step(self, vel_cmd, q_cmd):
        self.vel_cmd = vel_cmd
        self.q_cmd = q_cmd

        step_time = time.time()
        
        while ((time.time() - step_time) < POLICY_STEP):
            self.run_robot()

        # Update self.obs and send back to policy
        self.obs = np.flatten(np.concatenate((self.q, self.dq, self.projected_gravity, self.vel_cmd, self.q_cmd)))
        
        return self.obs

    def get_obs(self):
        low_state = self.interface.receive_observation()
        # Joint pos
        self.q = np.array([motor.q for motor in low_state.motorState[:12]])
        # Joint vel
        self.dq = np.array([motor.dq for motor in low_state.motorState[:12]])
        # Body quat
        quat = np.array([low_state.imu.quaternion])
        self.projected_gravity = quat_rot_inv(quat, [0,0,-9.81])
    
    def run_robot(self):
        cur_time = time.time()
        control_time = time.time()
        while ((time.time() - control_time) < CONTROL_STEP):
            pass

        self.get_obs()

        ## todo we want to do this two ways

def main():

    ## Todo load the model

    env = Go1Env()

    steps = 0
    obs_history = np.zeros((42, H))

    ## todo 
    #     - init obs_history from real readings
    #     - check dims on everything

    while steps < 1000:

        obs = env.step(vel_cmd, q_cmd)

        ## Organized such that newest obs goes on top
        ## and oldest at the bottom
        obs_history = np.delete(obs_history, H-1, 0)
        obs_history = np.append(obs, obs_history, axis=0) 

        


if __name__ == "__main__":
    main()