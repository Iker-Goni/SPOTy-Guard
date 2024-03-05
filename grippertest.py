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


def gripper_test(config):

    #just a test to see how to utilize spot's gripper
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert not robot.is_estopped(), 'SPOT is currently estopped.'

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        #power on spot
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Spot power on failed.'
        
        #open the gripper
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

        #send open request
        cmd_id = command_client.robot_command(gripper_command)
        robot.logger.info('Opening gripper')

        time.sleep(3)

        #close gripper
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)

        cmd_id = command_client.robot_command(gripper_command)
        robot.logger.info('closing gripper')

        time.sleep(3)

        robot.logger.info('done')

        robot.power_off(cut_immediately=False, timeout_sec=20)




def main(argv):
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        gripper_test(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception('Threw an exception')
        return False
    

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)