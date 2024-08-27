import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append("{}/unitree_legged_sdk/lib/python/amd64".format(dir_path))

import robot_interface

__all__ = ["robot_interface"]
