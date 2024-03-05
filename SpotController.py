import argparse
import sys
import time

import bosdyn.api.gripper_camera_param_pb2
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient

# TODO: figure out how the heck this works

class SpotController:
    def __init__(self, hostname="192.168.80.3"):
        self.hostname = hostname
        bosdyn.client.util.setup_logging(config.verbose)
        self.sdk = bosdyn.client.create_standard_sdk('SpotControllerClient')
        self.robot = sdk.create_robot()
        bosdyn.client.util.authenticate(robot)
        self.robot.time_sync.wait_for_sync()

    def setGripperState(openState):
        # true = open
        # false = closed
    def enablePatrol():
        # enable and disable can be the same function, just separating them rn to mirror what the website looks like
        # do stuff
    def disablePatrol():
        # do other stuff

