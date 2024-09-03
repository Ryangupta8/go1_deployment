import os

from go1_deployment.third_party import robot_interface

DIR_PATH = os.path.dirname(os.path.realpath(__file__))

__all__ = [
    "robot_interface",
    "DIR_PATH",
]
