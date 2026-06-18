import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
from typing import Any, Literal

from .constants import (
    LOWLEVEL, robot_KP, robot_KD,
    KA, SAFE_LEVEL, POS_STOP_F, VEL_STOP_F,
    robot_Q_STANCE, POLICY_STEP, CONTROL_STEP,
    SIM_CFG
)


class Go1_Sim:
    def __init__(self, mode: Literal["direct", "gui"] = "gui") -> None:
        # Get PyBullet client
        if mode == "gui":
            self.client = bc.BulletClient(connection_mode=p.GUI)
            self.client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        elif mode == "direct":
            self.client = bc.BulletClient(connection_mode=p.DIRECT)

        # Initiate simulation
        self.client.resetSimulation()
        self.client.setPhysicsEngineParameter(enableConeFriction=0)
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set up simulation
        self.sim_f = int(1. / CONTROL_STEP)
        self.sim_dt = CONTROL_STEP
        self.control_f = int(1. / POLICY_STEP)
        self.control_dt = POLICY_STEP
        self.repeat = int(self.sim_f / self.control_f)
        self.client.setTimeStep(self.sim_dt)
        self.gravity = [0, 0, -9.81]
        self.client.setGravity(*self.gravity)

        # Set up ground
        self.ground = self.client.loadURDF("plane.urdf")
        # self.client.changeDynamics(
        #     self.ground, -1,
        #     lateralFriction=2, rollingFriction=2
        # )

        # Set up robot
        # self.Kp, self.Kd, self.Ka = cfg["Kp"], cfg["Kd"], cfg["Ka"]
        self.vel_lim = SIM_CFG["vel_lim"]
        self.saturation_lim = SIM_CFG["saturation_lim"]
        self.tau_lim = SIM_CFG["tau_lim"]
        init_pose = SIM_CFG["init_pose"]
        init_ori = p.getQuaternionFromEuler(SIM_CFG["init_ori"])
        flags = p.URDF_MERGE_FIXED_LINKS  # \
        #     | p.URDF_USE_SELF_COLLISION  # \
        #   | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.robot = self.client.loadURDF(
            "robots/go1.urdf", init_pose, init_ori, flags=flags
        )
        self.n = self.client.getNumJoints(self.robot)
        self.joints = {}
        self.links = {}
        for j in range(self.n):
            self.client.setJointMotorControl2(
                self.robot, j, p.VELOCITY_CONTROL,
                force=0
            )
            self.client.enableJointForceTorqueSensor(
                self.robot, j, 1
            )
            # self.client.changeDynamics(
            #     self.robot, j, linearDamping=0, angularDamping=0,
            #     lateralFriction=cfg["lateral_friction"],
            #     rollingFriction=cfg["rolling_friction"],
            # )
            info = self.client.getJointInfo(self.robot, j)
            joint_name = info[1].decode("utf8")
            joint_type = info[2]
            if (joint_type in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                self.joints[joint_name] = j
                self.links[joint_name.split("_joint")[0]] = j
        info = self.client.getDynamicsInfo(self.robot, -1)
        self.client.changeDynamics(
            self.robot, -1, linearDamping=0, angularDamping=0,
            lateralFriction=SIM_CFG["lateral_friction"],
            rollingFriction=SIM_CFG["rolling_friction"],
        )
        self.links["trunk"] = -1
        self.joints_isaac = SIM_CFG["policy_joint_order"]
        self.q_init = cfg["q_stance"]
        self.q_init_arr = np.array(dict_to_list(self.q_init, self.joints))
        self.dq_init_arr = np.array([0 for _ in range(self.n)])
        self.tau_lim_arr = np.array([self.tau_lim for _ in range(self.n)])
        self.Kp = np.array(dict_to_list(cfg["Kp_joints"], self.joints))
        self.Kd = np.array(dict_to_list(cfg["Kd_joints"], self.joints))
        self.Ka = cfg["Ka"]
        # self.Kp_arr = [self.Kp] * self.n
        # self.Kd_arr = [self.Kd] * self.n
        self.Kp_arr = self.Kp.tolist()
        self.Kd_arr = self.Kd.tolist()
        self.action_init = [0 for _ in range(self.n)]

        for j in range(self.n):
            self.client.resetJointState(
                self.robot, j, self.q_init_arr[j]
            )
