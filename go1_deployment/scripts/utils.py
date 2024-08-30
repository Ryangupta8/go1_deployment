import numpy as np


def quat_rot_inv(
    body_quat: np.ndarray,
    gravity: np.ndarray
) -> np.ndarray:
    # assumes quat [w, x, y, z]
    q_w = body_quat.flatten()[0]
    q_vec = body_quat.flatten()[1:]
    a = gravity * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, gravity) * q_w * 2.0
    c = q_vec * q_vec * gravity
    return a - b + c


def flatten_for_policy(obs) -> np.ndarray:
    # obs [42, H]
    proj_g = obs[0:3, :].reshape((1, -1))
    vel_cmd = obs[3:6, :].reshape((1, -1))
    q = obs[6:18, :].reshape((1, -1))
    dq = obs[18:30, :].reshape((1, -1))
    last_action = obs[30:42, :].reshape((1, -1))
    gait_mode = obs[42:46, :].reshape((1, -1))
    obs_flat = np.concatenate((
        proj_g, vel_cmd, q, dq, last_action, gait_mode
        ), axis=1)
    return np.float32(obs_flat)


def robot_to_policy_joint_reorder(robot_joint_order: np.ndarray) -> np.ndarray:
    policy_joint_order = np.zeros(12, dtype=np.float32)
    policy_joint_order[0] = robot_joint_order[3]  # FL Hip
    policy_joint_order[1] = robot_joint_order[0]  # FR Hip
    policy_joint_order[2] = robot_joint_order[9]  # RL Hip
    policy_joint_order[3] = robot_joint_order[6]  # RR Hip
    policy_joint_order[4] = robot_joint_order[4]  # FL Thigh
    policy_joint_order[5] = robot_joint_order[1]  # FR Thigh
    policy_joint_order[6] = robot_joint_order[10]  # RL Thigh
    policy_joint_order[7] = robot_joint_order[7]  # RR Thigh
    policy_joint_order[8] = robot_joint_order[5]  # FL Calf
    policy_joint_order[9] = robot_joint_order[2]  # FR Calf
    policy_joint_order[10] = robot_joint_order[11]  # RL Calf
    policy_joint_order[11] = robot_joint_order[8]  # RR Calf

    return policy_joint_order


def policy_to_robot_joint_reorder(policy_joint_order) -> np.ndarray:
    robot_joint_order = np.zeros(12, dtype=np.float32)
    robot_joint_order[0] = policy_joint_order[1]  # FR Hip
    robot_joint_order[1] = policy_joint_order[5]  # FR Thigh
    robot_joint_order[2] = policy_joint_order[9]  # FR Calf
    robot_joint_order[3] = policy_joint_order[0]  # FL Hip
    robot_joint_order[4] = policy_joint_order[4]  # FL Thigh
    robot_joint_order[5] = policy_joint_order[8]  # FL Calf
    robot_joint_order[6] = policy_joint_order[3]  # RR Hip
    robot_joint_order[7] = policy_joint_order[7]  # RR Thigh
    robot_joint_order[8] = policy_joint_order[11]  # RR Calf
    robot_joint_order[9] = policy_joint_order[2]  # RL Hip
    robot_joint_order[10] = policy_joint_order[6]  # RL Thigh
    robot_joint_order[11] = policy_joint_order[10]  # RL Calf

    return robot_joint_order
