#!/usr/bin/env python
# Import ROS stuff
import rospy
import rospkg
import actionlib

from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectory, CartesianTrajectoryPoint, CartesianTolerance, CartesianPosture
from geometry_msgs.msg import Vector3, Twist, Accel, Pose, Point, Quaternion 

import simple_ur_move.utils as utils
from simple_ur_move.controller_handler import ControllerHandler

# Import other stuff
import time
import yaml
import os
import sys
import copy

filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')

class TrajectoryRunner():
    '''
    A ROS Action server to run single tests.

    Parameters
    ----------
    name : str
        Name of the Action Server
    '''

    def __init__(self, name, controller, debug):
        # Get parameters from roslaunch
        self.DEBUG = rospy.get_param(rospy.get_name()+"/debug",debug)
        self.robot_name = rospy.get_param(rospy.get_name()+"/robot_name",name)
        self.traj_file = rospy.get_param(rospy.get_name()+"/traj",None)
        self.controller_to_use = rospy.get_param(rospy.get_name()+"/controller",controller)

        if self.traj_file is not None:
            self.load_config(self.traj_file)
        else:
            self._parse_config(None)


        # Set ROS controllers
        self.controller_handler = ControllerHandler(self.robot_name)
        self.controller_handler.set_controller(self.controller_to_use)

        # Make an action client for trajectories
        action_name = self.robot_name+'/cartesian_trajectory_controller/follow_cartesian_trajectory'

        self.trajectory_client = actionlib.SimpleActionClient(
                    action_name,
                    FollowCartesianTrajectoryAction)
        try:
            self.trajectory_client.wait_for_server(2.0)
        except rospy.exceptions.ROSException as err:
            print("Could not reach controller action. Msg:")
            print(err)


    def load_config(self, filename):
        self.traj_file=filename
        self.traj_path = os.path.join(filepath_config,filename)
        config = utils.load_yaml(self.traj_path)
        self._parse_config(self.config)


    def _parse_config(self, config):
        if config is None:
            config = {}
        self.settings = config.get('settings', None)
        self.trajectory = config.get('trajectory', None)

        if self.settings is None or self.trajectory is None:
            return

        self.units = self.settings.get('units',None)
        self.path_tolerance=self.settings.get('path_tolerance',None)
        self.goal_tolerance=self.settings.get('goal_tolerance',None)
        self.goal_time_tolerance=self.settings.get('goal_time_tolerance',None)
        self.controlled_frame=self.settings.get('controlled_frame',None)


    def pack_trajectory(self, trajectory):
        traj_packed = []
        for waypoint in trajectory:
            pt = CartesianTrajectoryPoint()
            pose = Pose()
            pose.position= Point(waypoint['position'])
            pose.orientation = Quaternion(waypoint['orientation'])
            pt.pose=pose

            lin_vel = None
            ang_vel = None
            if 'linear_velocity' in waypoint.keys() and 'angular_velocity' in waypoint.keys():
                lin_vel = waypoint['linear_velocity']
                ang_vel = waypoint['angular_velocity']
                pt.twist=Twist(linear=Vector3(lin_vel), angular=Vector3(ang_vel))

            pt.time_from_start = rospy.Duration(waypoint['time'])
            traj_packed.append(pt)

        traj_ros = CartesianTrajectory(points=traj_packed, controlled_frame=self.controlled_frame)
        return traj_ros


    def run_trajectory(self, blocking=True):
        if self.trajectory is None:
            rospy.logerr("Cannot run trajectory. It was not defined.")
            return
        
        traj= self.pack_trajectory(self.trajectory)
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory = traj
        if self.path_tolerance is not None:
            goal.path_tolerance = self.path_tolerance
        if self.goal_tolerance is not None:
            goal.goal_tolerance = self.goal_tolerance
        if self.goal_time_tolerance is not None:
            goal.goal_time_tolerance = self.goal_time_tolerance

        self.trajectory_client.send_goal(goal)

        if blocking:
            self.trajectory_client.wait_for_result()


    def shutdown(self):
        self.trajectory_client.cancel_all_goals()


if __name__ == '__main__':
    try:
        rospy.init_node('ur_cartesian_traj_runner', disable_signals=True)
        sender = TrajectoryRunner(rospy.get_name())
        sender.run_trajectory()
        sender.shutdown()

    except KeyboardInterrupt:
        print("ur_cartesian_traj_runner: Shutting Down")
        sender.shutdown()

    except rospy.ROSInterruptException:
        print("ur_cartesian_traj_runner: Shutting Down")
        sender.shutdown()

   



        