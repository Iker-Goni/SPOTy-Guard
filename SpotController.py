import argparse
import sys
import time
import facerecog
import os
import cv2
import numpy as np
from scipy import ndimage

import bosdyn.api.gripper_camera_param_pb2
import bosdyn.client
import bosdyn.client.lease
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, block_until_arm_arrives, blocking_stand, block_for_trajectory_cmd

import bosdyn.client.util
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2
from bosdyn.api import robot_command_pb2

# animation imports
from bosdyn.api import geometry_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers

from google.protobuf import duration_pb2
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.api import estop_pb2 as estop_protos
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2

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
        self._patrolThread = threading.Thread(target=self.patrol_loop, name="Patrol Thread")

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
        os.system("ffplay -nodisp -autoexit sounds/bark.mp3")
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
            image_path = str(person_name) + str(np.random.randint(0,10000)) + '.png'
            cv2.imwrite(image_path, img)
            
        print ("Saved image as " + image_path)
        self.recognizer.IdentifyFaces(image_path)
        self.recognizer.SaveNewFaces()
    def patrolRecognize(self):
        "Takes an image with SPOT's hand cam, sees if it contains faces. If it does, we try to match them to known faces in the database."
        print ("Taking patrol image...")
        #self.setGripperState(True)
        #TODO: have spot be swiveling the arm back and forth?
        with bosdyn.client.lease.LeaseKeepAlive(self.lease_client, must_acquire=True, return_at_exit=True):
            self.robot.power_on(timeout_sec=60)
            #stand
            blocking_stand(self.command_client, timeout_sec=10)

            #unstow arm
            unstow = RobotCommandBuilder.arm_ready_command()

            # Issue the command via the RobotCommandClient
            unstow_command_id = self.command_client.robot_command(unstow)
            self.robot.logger.info('Unstow command issued.')

            # Convert the location from the moving base frame to the world frame.
            robot_state = self.robot_state_client.get_robot_state()
            self.odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                             ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
            
            # MOVE ARM TO CENTRAL POSITION

            x = 0.75
            y = 0.0
            z = .8
            hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

            # Rotation as a quaternion
            qw = 1
            qx = 0
            qy = 0
            qz = 0
            flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

            flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                                    rotation=flat_body_Q_hand)

            robot_state = self.robot_state_client.get_robot_state()
            self.odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                             ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

            odom_T_hand = self.odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

            # duration in seconds
            seconds = 2

            arm_position_1 = RobotCommandBuilder.arm_pose_command(
                odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
                odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

            # Make the open gripper RobotCommand
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

            # Combine the arm and gripper commands into one RobotCommand
            arm_command_1 = RobotCommandBuilder.build_synchro_command(gripper_command, arm_position_1)

            # Send the request
            cmd_id = self.command_client.robot_command(arm_command_1)
            self.robot.logger.info('Moving arm to position 1.')

            # Wait until the arm arrives at the goal.
            self.block_until_arm_arrives_with_prints(cmd_id)

            time.sleep(2)

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
            self.bark()
            print("Stranger detected! Woof woof!")
    
    def block_until_arm_arrives_with_prints(self, cmd_id):
        """Block until the arm arrives at the goal and print the distance remaining.
            Note: a version of this function is available as a helper in robot_command
            without the prints.
        """
        while True:
            feedback_resp = self.command_client.robot_command_feedback(cmd_id)
            measured_pos_distance_to_goal = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_pos_distance_to_goal
            measured_rot_distance_to_goal = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_rot_distance_to_goal
            self.robot.logger.info('Distance to go: %.2f meters, %.2f radians',
                              measured_pos_distance_to_goal, measured_rot_distance_to_goal)

            if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
                self.robot.logger.info('Move complete.')
                break
            time.sleep(0.1)

    def patrol_loop(self): # TODO: untested
        while self._patrolStatus == True:
            # PATROL LEFT TO RIGHT
            # Look to the left and the right with the hand.
            # Robot's frame is X+ forward, Z+ up, so left and right is +/- in Y.
            x = 3  # look 2 meters ahead
            start_y = 4
            end_y = -4
            z = 0.3  # Look ahead, not up or down

            traj_time = 6  # take 4 seconds to look from left to right.

            start_pos_in_odom_tuple = self.odom_T_flat_body.transform_point(x=x, y=start_y, z=z)
            start_pos_in_odom = geometry_pb2.Vec3(x=start_pos_in_odom_tuple[0],
                                                y=start_pos_in_odom_tuple[1],
                                                z=start_pos_in_odom_tuple[2])

            end_pos_in_odom_tuple = self.odom_T_flat_body.transform_point(x=x, y=end_y, z=z)
            end_pos_in_odom = geometry_pb2.Vec3(x=end_pos_in_odom_tuple[0], y=end_pos_in_odom_tuple[1],
                                                z=end_pos_in_odom_tuple[2])

            # Create the trajectory points
            point1 = trajectory_pb2.Vec3TrajectoryPoint(point=start_pos_in_odom)

            duration_seconds = int(traj_time)
            duration_nanos = int((traj_time - duration_seconds) * 1e9)

            point2 = trajectory_pb2.Vec3TrajectoryPoint(
                point=end_pos_in_odom,
                time_since_reference=duration_pb2.Duration(seconds=duration_seconds,
                                                        nanos=duration_nanos))

            # Build the trajectory proto
            traj_proto = trajectory_pb2.Vec3Trajectory(points=[point1, point2])

            # Build the proto
            gaze_cmd = arm_command_pb2.GazeCommand.Request(target_trajectory_in_frame1=traj_proto,
                                                        frame1_name=ODOM_FRAME_NAME,
                                                        frame2_name=ODOM_FRAME_NAME)
            arm_command = arm_command_pb2.ArmCommand.Request(arm_gaze_command=gaze_cmd)
            synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
                arm_command=arm_command)
            command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

            # Make the open gripper RobotCommand
            gripper_command = RobotCommandBuilder.claw_gripper_open_command()

            # Combine the arm and gripper commands into one RobotCommand
            synchro_command = RobotCommandBuilder.build_synchro_command(gripper_command, command)

            # Send the request
            gaze_command_id = self.command_client.robot_command(command)
            self.robot.logger.info('Sending gaze trajectory.')

            # Wait until the robot completes the gaze before issuing the next command.
            block_until_arm_arrives(self.command_client, gaze_command_id, timeout_sec=traj_time + 3.0)

            self.patrolRecognize() # look for people 

    def enablePatrol(self): # TODO: untested 
        # back legs should bend down, arm should move up to head level
        with bosdyn.client.lease.LeaseKeepAlive(self.lease_client, must_acquire=True, return_at_exit=True):
            self.robot.power_on(timeout_sec=60)
            #stand
            blocking_stand(self.command_client, timeout_sec=10)

            #unstow arm
            unstow = RobotCommandBuilder.arm_ready_command()

            # Issue the command via the RobotCommandClient
            unstow_command_id = self.command_client.robot_command(unstow)
            self.robot.logger.info('Unstow command issued.')

            # Convert the location from the moving base frame to the world frame.
            robot_state = self.robot_state_client.get_robot_state()
            self.odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                             ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
            
            # MOVE ARM TO CENTRAL POSITION

            x = 0.75
            y = 0.0
            z = .8
            hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

            # Rotation as a quaternion
            qw = 1
            qx = 0
            qy = 0
            qz = 0
            flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

            flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                                    rotation=flat_body_Q_hand)

            robot_state = self.robot_state_client.get_robot_state()
            self.odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                             ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

            odom_T_hand = self.odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

            # duration in seconds
            seconds = 2

            arm_position_1 = RobotCommandBuilder.arm_pose_command(
                odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
                odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

            # Make the open gripper RobotCommand
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

            # Combine the arm and gripper commands into one RobotCommand
            arm_command_1 = RobotCommandBuilder.build_synchro_command(gripper_command, arm_position_1)

            # Send the request
            cmd_id = self.command_client.robot_command(arm_command_1)
            self.robot.logger.info('Moving arm to position 1.')

            # Wait until the arm arrives at the goal.
            self.block_until_arm_arrives_with_prints(cmd_id)

            time.sleep(2)
        self._patrolStatus = True
        self.patrol_loop()
        ##self._patrolThread.run()
    def disablePatrol(self): # TODO: untested
        self._patrolStatus = False
        self._patrolThread.join()
