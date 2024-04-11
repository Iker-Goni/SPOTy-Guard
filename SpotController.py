import argparse
import sys
import time
import facerecog

import cv2
import numpy as np
from scipy import ndimage

import bosdyn.api.gripper_camera_param_pb2
import bosdyn.client
import bosdyn.client.lease
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, block_until_arm_arrives, blocking_stand

import bosdyn.client.util
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2
from bosdyn.api import robot_command_pb2

# sound functionality
from playsound import playsound
# patrol thread imports
# TODO: fix requirements.txt to add these maybe? not sure
import threading
import time

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
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.robot.time_sync.wait_for_sync()
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.recognizer = facerecog.FaceRecognizer()

        self._patrolStatus = False
        self._patrolThread = threading.Thread(target=patrol_loop, name="Patrol Thread")

    def setGripperState(self, openState: bool):
        print("things")
        # true = open
        # false = closed
        gripperAngle: float = 0.0
        if (openState):
            gripperAngle = 1.0
        with bosdyn.client.lease.LeaseKeepAlive(self.lease_client, must_acquire=True, return_at_exit=True):
            assert self.robot.is_powered_on(), 'failed power on'
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(gripperAngle)
            command = RobotCommandBuilder.build_synchro_command(gripper_command)
            cmd_id = self.command_client.robot_command(command)

    def patrol_loop(): # TODO: untested
        while _patrolStatus == True:
            patrolRecognize()
            time.sleep(5)

    def enablePatrol(self): # TODO: untested
        self._patrolStatus = True
        self._patrolThread.run()
    def disablePatrol(self): # TODO: untested
        self._patrolStatus = False
        self._patrolThread.join()

    def estop(self):
        print("Estopping...")
         # Create estop client for the robot
        estop_client = self.robot.ensure_client(EstopClient.default_service_name)

        # Create nogui estop
        estop_nogui = EstopNoGui(estop_client, int(20), 'Estop NoGUI') 
        estop_nogui.stop()
    def unestop(self):
        print("Removing estop...")
    def flashLightOn(self):
        #TODO: finish
        print("things")
        arm = self.robot_state_client.get_robot_state().arm_states[0]
        flashlight = arm.flashlight
        flashlight_command = RobotCommandBuilder.synchro_command(robot_command_pb2.ArmFlashlightCommand.Request.ON)
    def bark(self):
        playsound("sounds/bark.mp3")
    def scanNewFace(self, person_name=""):
        "scans a user's face and adds it to the database."
        print ("Scanning face...")
        #TODO: put the arm up, instruct user to stand infront of spot
        image_client = self.robot.ensure_client(ImageClient.default_service_name)
        image_request = [
            build_image_request(image_source_name='hand_color_image', quality_percent=100)
        ]
        image_responses = image_client.get_image(image_request)
        #TODO: flash spot's flashlight while taking images
        for image in image_responses:
            num_bytes = 1
            img = np.frombuffer(image.shot.image.data, dtype=np.uint8)
            img = cv2.imdecode(img, -1)
            image_saved_path = image.source.name
            image_saved_path = image_saved_path.replace(
                '/', ''
            )
            #add a random number to the name of the image to prevent duplicates
            image_path = person_name + image_saved_path + str(np.random.randint(0,10000)) + '.png'
            cv2.imwrite(image_path, img)
            
        print ("Saved image as " + image_path)
        self.recognizer.IdentifyFaces(image_path)
        self.recognizer.SaveNewFaces()
    def patrolRecognize(self):
        "Takes an image with SPOT's hand cam, sees if it contains faces. If it does, we try to match them to known faces in the database."
        print ("Taking patrol image...")
        #self.setGripperState(True)
        #TODO: have spot be swiveling the arm back and forth?
        image_client = self.robot.ensure_client(ImageClient.default_service_name)
        image_request = [
            build_image_request(image_source_name='hand_color_image', quality_percent=100)
        ]
        image_responses = image_client.get_image(image_request)
        for image in image_responses:
            num_bytes = 1
            img = np.frombuffer(image.shot.image.data, dtype=np.uint8)
            img = cv2.imdecode(img, -1)
            image_saved_path = image.source.name
            image_saved_path = image_saved_path.replace(
                '/', ''
            )
            #add a random number to the name of the image to prevent duplicates
            image_path = image_saved_path + str(np.random.randint(0,10000)) + '.png'
            cv2.imwrite(image_path, img)
            
        print ("Saved image as " + image_path)
        #self.setGripperState(False)
        # Guard clause
        if not (self.recognizer.ContainsFaces(image_path)):
            return # return out of the function if we didn't find any faces in the image
        
        print ("Hey, we see somebody!")

        # Grab the faces in the image, and prep them for analysis.
        self.recognizer.IdentifyFaces(image_path)
        
        # Check to see if any of the faces in the patrol image are people we recognize. If they are, we don't have to bark.
        if (self.recognizer.RecognizeFaces()):
            print("We recognize at least one person in the given image, no need to bark")
        else:
            # The only people in the image are strangers, time to bark.
            # TODO discord message for intruder
            bark()
            print("Stranger detected! Woof woof!")
        


    
