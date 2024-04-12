# animation for taking images
# animation for patrol mode
# password: sh384s6nnk6q    
# ip: 192.168.80.3
import argparse
import sys
import time
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.gripper_command_pb2
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
import bosdyn.client.estop
from bosdyn.api import estop_pb2
from bosdyn.client.lease import LeaseClient, ResourceAlreadyClaimedError
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, block_until_arm_arrives,
                                         blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
import bosdyn.api.gripper_camera_param_pb2
from google.protobuf import duration_pb2
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.api import (arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2,
                        trajectory_pb2)
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.api import estop_pb2 as estop_protos
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.geometry import EulerZXY
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
HEIGHT_MAX = 0.3  # m
ROLL_OFFSET_MAX = 0.4  # rad
YAW_OFFSET_MAX = 0.7805  # rad
PITCH_OFFSET_MAX = 0.7805  # rad
HEIGHT_CHANGE = 0.1  # m per command

def block_until_arm_arrives_with_prints(robot, command_client, cmd_id):
    """Block until the arm arrives at the goal and print the distance remaining.
        Note: a version of this function is available as a helper in robot_command
        without the prints.
    """
    while True:
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        measured_pos_distance_to_goal = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_pos_distance_to_goal
        measured_rot_distance_to_goal = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_rot_distance_to_goal
        robot.logger.info('Distance to go: %.2f meters, %.2f radians',
                          measured_pos_distance_to_goal, measured_rot_distance_to_goal)

        if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
            robot.logger.info('Move complete.')
            break
        time.sleep(0.1)
# Function for animating SPOT when registering faces into the database
# SPOT raises arm and gazes forward, then positions are to the left and to the right
def registering_face(config):

    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    estop_client = robot.ensure_client('estop')

    estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=estop_client, name='my_estop', estop_timeout=9.0)
    estop_endpoint.force_simple_setup()

    estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)

    assert not robot.is_estopped(), 'SPOT is currently estopped.'

    assert robot.has_arm(), 'Robot requires an arm'

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):

        #power on spot
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Spot power on failed.'
        
        #stand
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)

        #unstow arm
        unstow = RobotCommandBuilder.arm_ready_command()
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        # Issue the command via the RobotCommandClient
        unstow_command_id = command_client.robot_command(unstow)
        robot.logger.info('Unstow command issued.')

        # Convert the location from the moving base frame to the world frame.
        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
    

######################################################################################################
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

        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

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
        cmd_id = command_client.robot_command(arm_command_1)
        robot.logger.info('Moving arm to position 1.')

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives_with_prints(robot, command_client, cmd_id)

        # GAZE TOWARDS CENTER

        # Make the open gripper RobotCommand
        gripper_command = RobotCommandBuilder.claw_gripper_open_command()
        
        # Look at a point 3 meters in front and 4 meters to the left.
        # We are not specifying a hand location, the robot will pick one.
        gaze_target_in_odom = odom_T_flat_body.transform_point(x=4.0, y=0.0, z=0.3)

        gaze_command2 = RobotCommandBuilder.arm_gaze_command(gaze_target_in_odom[0],
                                                            gaze_target_in_odom[1],
                                                            gaze_target_in_odom[2], ODOM_FRAME_NAME)
        
        # Combine the arm and gripper commands into one RobotCommand
        gaze_command_2 = RobotCommandBuilder.build_synchro_command(gripper_command1, gaze_command1)

        # Send the request
        robot.logger.info('Requesting gaze.')
        gaze_command_2 = command_client.robot_command(gaze_command_2)

        block_until_arm_arrives(command_client, gaze_command_2, 4.0)

        time.sleep(2.5)

        # POSITION ARM TO THE LEFT
        
        x = 0.75
        y = .15
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

        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

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
        cmd_id = command_client.robot_command(arm_command_1)
        robot.logger.info('Moving arm to position 1.')

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives_with_prints(robot, command_client, cmd_id)

        # GAZE TO THE RIGHT

        # Make the open gripper RobotCommand
        gripper_command = RobotCommandBuilder.claw_gripper_open_command()
        
        # Look at a point 3 meters in front and 4 meters to the left.
        # We are not specifying a hand location, the robot will pick one.
        gaze_target_in_odom = odom_T_flat_body.transform_point(x=6.0, y=-3.0, z=.3)

        gaze_command = RobotCommandBuilder.arm_gaze_command(gaze_target_in_odom[0],
                                                            gaze_target_in_odom[1],
                                                            gaze_target_in_odom[2], ODOM_FRAME_NAME)
        
        # Combine the arm and gripper commands into one RobotCommand
        gaze_command = RobotCommandBuilder.build_synchro_command(gripper_command, gaze_command)

        # Send the request
        robot.logger.info('Requesting gaze.')
        gaze_command = command_client.robot_command(gaze_command)

        block_until_arm_arrives(command_client, gaze_command, 4.0)

        time.sleep(2.5)

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

        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

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
        cmd_id = command_client.robot_command(arm_command_1)
        robot.logger.info('Moving arm to position 1.')

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives_with_prints(robot, command_client, cmd_id)

        # GAZE TO THE LEFT

        # Make the open gripper RobotCommand
        gripper_command = RobotCommandBuilder.claw_gripper_open_command()
        
        # Look at a point 3 meters in front and 4 meters to the left.
        # We are not specifying a hand location, the robot will pick one.
        gaze_target_in_odom = odom_T_flat_body.transform_point(x=6.0, y=6.0, z=0.3)

        gaze_command2 = RobotCommandBuilder.arm_gaze_command(gaze_target_in_odom[0],
                                                            gaze_target_in_odom[1],
                                                            gaze_target_in_odom[2], ODOM_FRAME_NAME)
        
        # Combine the arm and gripper commands into one RobotCommand
        gaze_command_2 = RobotCommandBuilder.build_synchro_command(gripper_command1, gaze_command1)

        # Send the request
        robot.logger.info('Requesting gaze.')
        gaze_command_2 = command_client.robot_command(gaze_command_2)

        block_until_arm_arrives(command_client, gaze_command_2, 4.0)

        time.sleep(2.5)

        # MOVE TO THIRD POSITION

        x = 0.75
        y = -.15
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

        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

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
        cmd_id = command_client.robot_command(arm_command_1)
        robot.logger.info('Moving arm to position 1.')

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives_with_prints(robot, command_client, cmd_id)

#######################################################################################################

        time.sleep(3)

        robot.logger.info('done')

        robot.power_off(cut_immediately=False, timeout_sec=20)
        

def main(argv):
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        registering_face(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception('Threw an exception')
        return False
    
if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)