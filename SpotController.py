import argparse
import sys
import time

import bosdyn.api.gripper_camera_param_pb2
import bosdyn.client
import bosdyn.client.lease
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, block_until_arm_arrives, blocking_stand)

import bosdyn.client.util
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_state import RobotStateClient
import estop_nogui as EstopNoGui

# TODO: figure out how the heck this works
class SpotController:
    def __init__(self, hostname="192.168.80.3"):
        self.hostname = hostname 
        self.sdk = bosdyn.client.create_standard_sdk('SpotControllerClient')
        self.robot = self.sdk.create_robot(self.hostname)
        bosdyn.client.util.authenticate(self.robot)

        self.lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        self.robot.time_sync.wait_for_sync()
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        # Create estop client for the robot
        self.estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        # Create nogui estop
        self.estop_nogui = EstopNoGui.EstopNoGui(self.estop_client, int(20), 'Estop NoGUI') 

    def setGripperState(self, openState):
        print("things")
        # true = open
        # false = closed
    def enablePatrol(self):
        print("things")
        # enable and disable can be the same function, just separating them rn to mirror what the website looks like
        # do stuff
    def disablePatrol(self):
        print("things")
        # do other stuff
    def estop(self):
        print("Estopping...") 
        self.estop_nogui.stop()
    def unestop(self):
        print("Removing estop...")
        self.estop_nogui.allow()
    def getestopstatus(self):
        states = self.robot_state_client.get_robot_state().estop_states
        print(states)
