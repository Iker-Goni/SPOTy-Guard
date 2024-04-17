# animation for taking images
# animation for patrol mode
# password: sh384s6nnk6q    
# ip: 192.168.80.3
import argparse
import sys
import time

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
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.api import (arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2,
                        trajectory_pb2)
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import estop_pb2 as estop_protos
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.geometry import EulerZXY
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_command_pb2, synchronized_command_pb2

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

def registering_face(config, time_for_patrol):

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

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    # back legs should bend down, arm should move up to head level
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

        time.sleep(2)

        start_time = time.time()
        while time.time() < start_time + time_for_patrol:
            # PATROL LEFT TO RIGHT
            # Look to the left and the right with the hand.
            # Robot's frame is X+ forward, Z+ up, so left and right is +/- in Y.
            x = 3  # look 2 meters ahead
            start_y = 4
            end_y = -4
            z = 0.3  # Look ahead, not up or down

            traj_time = 6  # take 4 seconds to look from left to right.

            start_pos_in_odom_tuple = odom_T_flat_body.transform_point(x=x, y=start_y, z=z)
            start_pos_in_odom = geometry_pb2.Vec3(x=start_pos_in_odom_tuple[0],
                                                y=start_pos_in_odom_tuple[1],
                                                z=start_pos_in_odom_tuple[2])

            end_pos_in_odom_tuple = odom_T_flat_body.transform_point(x=x, y=end_y, z=z)
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
            gaze_command_id = command_client.robot_command(command)
            robot.logger.info('Sending gaze trajectory.')

            # Wait until the robot completes the gaze before issuing the next command.
            block_until_arm_arrives(command_client, gaze_command_id, timeout_sec=traj_time + 3.0)

            time.sleep(3)

        robot.logger.info('patrol completed')

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