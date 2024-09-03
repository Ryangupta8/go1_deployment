import os
import sys

unitree_dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(f"{unitree_dir_path}/unitree_legged_sdk/lib/python/amd64")

import robot_interface

__all__ = ["robot_interface"]
