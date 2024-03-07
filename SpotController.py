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
        self.sdk = bosdyn.client.create_standard_sdk('SpotControllerClient')
        self.robot = self.sdk.create_robot(self.hostname)
        bosdyn.client.util.authenticate(self.robot)
        self.robot.time_sync.wait_for_sync()
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)



    def setGripperState(self, openState):
        print("to be implemented by iker")
        # true = open
        # false = closed
    def enablePatrol(self):
        print("to be implemented by iker")
        # enable and disable can be the same function, just separating them rn to mirror what the website looks like
        # do stuff
    def disablePatrol(self):
        print("to be implemented by iker")
        # do other stuff

