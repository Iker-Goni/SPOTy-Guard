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


class EstopNoGui():
    """Provides a software estop without a GUI.

    To use this estop, create an instance of the EstopNoGui class and use the stop() and allow()
    functions programmatically.
    """

    def __init__(self, client, timeout_sec, name=None):

        # Force server to set up a single endpoint system
        ep = EstopEndpoint(client, name, timeout_sec)
        ep.force_simple_setup()

        # Begin periodic check-in between keep-alive and robot
        self.estop_keep_alive = EstopKeepAlive(ep)

        # Release the estop
        self.estop_keep_alive.allow()

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanly shut down estop on exit."""
        self.estop_keep_alive.end_periodic_check_in()

    def stop(self):
        self.estop_keep_alive.stop()

    def allow(self):
        self.estop_keep_alive.allow()

    def settle_then_cut(self):
        self.estop_keep_alive.settle_then_cut()


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
    def estop(self):
        print("Estopping...")
         # Create estop client for the robot
        estop_client = self.robot.ensure_client(EstopClient.default_service_name)

        # Create nogui estop
        estop_nogui = EstopNoGui(estop_client, int(20), 'Estop NoGUI') 
        estop_nogui.stop()
    def unestop(self):
        print("Removing estop...")
