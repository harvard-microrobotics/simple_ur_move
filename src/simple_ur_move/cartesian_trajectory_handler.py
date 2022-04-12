#!/usr/bin/env python3
# Import ROS stuff
from distutils.log import debug
import rospy
import rospkg
import actionlib

from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectory, CartesianTrajectoryPoint, CartesianTolerance, CartesianPosture
from geometry_msgs.msg import Vector3, Twist, Accel, Pose, Point, Quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import simple_ur_move.utils as utils
from simple_ur_move.controller_handler import ControllerHandler


# Import other stuff
import time
import yaml
import os
import sys
import copy
import numpy as np

filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')

class CartesianTrajectoryHandler():
    """
    The cartesian trajectory interface

    Parameters
    ----------
    name : str
        Name of the Action Server
    controller : str
        The cartesian controller to use. Options are:
        ``forward_cartesian_traj_controller``,
        ``pose_based_cartesian_traj_controller``, and
        ``joint_based_cartesian_traj_controller``.
    debug : bool
        Turn on debug print statements
    """
    def __init__(self, name, controller=None, debug=False):
        # Get parameters from roslaunch
        self.debug = debug
        self.robot_name = name
        self.controller_to_use = controller
        self._parse_config(None)
        self.initialize_time = 4.0 # [sec]
        self.speed_factor = 1.0


        # Set ROS controllers
        self.controller_handler = ControllerHandler(self.robot_name)
        self.controller_handler.set_controller(self.controller_to_use)

        # Make an action client for trajectories
        action_name = self.robot_name+'/'+self.controller_to_use+'/follow_cartesian_trajectory'

        self.trajectory_client = actionlib.SimpleActionClient(
                    action_name,
                    FollowCartesianTrajectoryAction)
        try:
            self.trajectory_client.wait_for_server(rospy.Duration(2.0))
        except rospy.exceptions.ROSException as err:
            print("Could not reach controller action. Msg:")
            print(err)


    def load_config(self, filename, directory=None):
        """
        Load a trajectory configuration from a file

        Parameters
        ----------
        filename : str
            Filename of configuration to load
        directory : str
            Directory where config files should be loaded from. Default
            is the config folder of this package
        """
        self.traj_file=filename
        if directory is None:
            directory = filepath_config
        self.traj_path = os.path.join(directory,filename)
        config = utils.load_yaml(self.traj_path)
        self.set_config(config)

    
    def set_config(self, config):
        """
        Set the trajectory configuration

        Parameters
        ----------
        config : dict
            Configuration to set
        """
        self._parse_config(config)


    def _parse_config(self, config):
        """
        Parse a trajectory configuration

        Parameters
        ----------
        config : dict
            Configuration to parse. If ``None``, config is initialized empty.
        """
        if config is None:
            config = {}
        self.settings = config.get('settings', None)
        self.trajectory = config.get('trajectory', None)

        if self.settings is None:
            return

        self.units = self.settings.get('units',None)
        self.path_tolerance=self.settings.get('path_tolerance',None)
        self.goal_tolerance=self.settings.get('goal_tolerance',None)
        self.goal_time_tolerance=self.settings.get('goal_time_tolerance',None)
        self.controlled_frame=self.settings.get('controlled_frame',None)


    def set_initialize_time(self, time):
        """
        Set the time the robot takes to get to its initial poisiton

        Parameters
        ----------
        time : float
            Initialization time in seconds. Must be greater than or equal to 0 
        """
        if time<0:
            raise ValueError("Initialization time cannot be set less than 0")

        if time ==0:
            time =0.001

        if time<1:
            if self.debug:
                rospy.loginfo("Setting initialization time to less than 1 second" + \
                    "can lead to dangerous robot motion")
        
        self.initialize_time = time


    def set_speed_factor(self, speed_factor):
        """
        Set the speed multiplier

        Parameters
        ----------
        speed_factor : float
            Speed multiplier to use
        """
        if speed_factor<=0:
            raise ValueError("Speed_factor time must be strictly greater than 0")
        else:
            self.speed_factor=float(speed_factor)


    def set_trajectory(self, trajectory):
        """
        Set the current trajectory 

        Parameters
        ----------
        trajectory : list or cartesian_control_msgs/CartesianTrajectory
            Trajectory to parse

        Raises
        ------
        ValueError : If a trajectory of incorrect type is passed
        """
        
        if not isinstance(trajectory, CartesianTrajectory) and not isinstance(trajectory, list):
            raise ValueError("Joint trajectories must be of type list or trajectory_msgs/JointTrajectory")
        else:
            self.trajectory = trajectory


    def pack_trajectory(self, trajectory):
        """
        Pack a trajectory into a ROS ``CartesianTrajectory``

        Parameters
        ----------
        trajectory : list
            Trajectory to parse
        
        Returns
        -------
        ros_traj : cartesian_control_msgs/CartesianTrajectory
            A ROS trajectory
        """
        traj_to_use = copy.deepcopy(trajectory)
        # Convert mm to m
        traj_to_use = self.convert_units(traj_to_use, direction='to_ros')
    
        traj_packed = []
        for waypoint in traj_to_use:
            if self.debug:
                print('quaternion: ',waypoint['orientation'])
            pt = CartesianTrajectoryPoint()
            pose = Pose()
            pose.position= Point(x=waypoint['position'][0],
                                 y=waypoint['position'][1],
                                 z=waypoint['position'][2],)
            pose.orientation = Quaternion(x=waypoint['orientation'][0],
                                          y=waypoint['orientation'][1],
                                          z=waypoint['orientation'][2],
                                          w=waypoint['orientation'][3])
            pt.pose=pose

            lin_vel = None
            ang_vel = None
            if 'linear_velocity' in waypoint.keys() and 'angular_velocity' in waypoint.keys():
                lin_vel = waypoint['linear_velocity']
                ang_vel = waypoint['angular_velocity']
                pt.twist=Twist(linear=Vector3(x=lin_vel[0], y=lin_vel[1], z=lin_vel[2]),
                               angular=Vector3(x=ang_vel[0], y=ang_vel[1], z=ang_vel[2]))

            if 'posture' in waypoint.keys():
                posture= CartesianPosture()
                posture.posture_joint_values = waypoint['posture']['posture_joint_values']
                posture.posture_joint_names = waypoint['posture']['posture_joint_names']
                pt.posture=posture

            pt.time_from_start = rospy.Duration(waypoint['time'])
            traj_packed.append(pt)

        if len(traj_packed)>0:
            # Trajectories cannot start at time=0 becasue of how the UR ROS driver is written
            traj_packed[0].time_from_start+=rospy.Duration(0.001)

        traj_ros = CartesianTrajectory(points=traj_packed, controlled_frame=self.controlled_frame)
        return traj_ros

    
    def convert_units(self, trajectory, direction='to_ros'):
        """
        Convert units from the units defined in the trajectory config
        to ROS default units (or vice versa).

        Parameters
        ----------
        trajectory : list
            Trajectory to parse
        direction : str
            Which direction to convert. Options are ``to_ros`` or ``from_ros``.
        
        Returns
        -------
        trajectory : list
            Converted trajectory
        """
        if direction == 'to_ros':
            # Convert mm to m
            if self.units['position']=='mm':
                for waypoint in trajectory:
                    waypoint['position'] = (np.array(waypoint['position'])/1000).tolist()
                    if waypoint.get('linear_velocity', False):
                        waypoint['linear_velocity'] = (np.array(waypoint['linear_velocity'])/1000).tolist()

            # Convert degrees to radians
            if 'degree' in self.units['orientation']:
                for waypoint in trajectory:
                    waypoint['orientation'] = np.deg2rad(waypoint['orientation']).tolist()
                    if waypoint.get('linear_velocity', False):
                        waypoint['linear_velocity'] = np.deg2rad(waypoint['linear_velocity']).tolist()

            # Convert euler angles to quaternians
            if 'euler' in self.units['orientation']:
                for waypoint in trajectory:
                    waypoint['orientation'] = quaternion_from_euler(*waypoint['orientation'])
                    if waypoint.get('linear_velocity', False):
                        waypoint['linear_velocity'] = quaternion_from_euler(*waypoint['linear_velocity'])

        elif direction == 'from_ros':
            # Convert mm to m
            if self.units['position']=='mm':
                for waypoint in trajectory:
                    waypoint['position'] = (np.array(waypoint['position'])*1000).tolist()
                    if waypoint.get('linear_velocity', False):
                        waypoint['linear_velocity'] = (np.array(waypoint['linear_velocity'])*1000).tolist()

            # Convert quaternion to euler angles
            if 'euler' in self.units['orientation']:
                for waypoint in trajectory:
                    waypoint['orientation'] = euler_from_quaternion(waypoint['orientation'])
                    if waypoint.get('linear_velocity', False):
                        waypoint['linear_velocity'] = euler_from_quaternion(waypoint['linear_velocity'])

            # Convert radians to degrees
            if 'degree' in self.units['orientation']:
                for waypoint in trajectory:
                    waypoint['orientation'] = np.rad2deg(waypoint['orientation']).tolist()
                    if waypoint.get('linear_velocity', False):
                        waypoint['linear_velocity'] = np.rad2deg(waypoint['linear_velocity']).tolist()

        return trajectory


    def build_goal(self, trajectory):
        """
        Build the trajectory goal

        Parameters
        ----------
        trajectory : dict or cartesian_control_msgs/CartesianTrajectory
            Trajectory to parse

        Returns
        -------
        goal : cartesian_control_msgs/FollowCartesianTrajectoryGoal
            The goal built from the trajectory
        """
        if trajectory is None:
            rospy.logerr("Cannot build trajectory goal. It was not defined.")
            return

        if isinstance(trajectory, CartesianTrajectory):
            traj = trajectory
        else:
            traj= self.pack_trajectory(trajectory)

        if self.debug:
            print("")
            for row in traj.points:
                print(row.time_from_start.to_sec(), row.pose.position.x, row.pose.position.y, row.pose.position.z)

        # Pack the trajectory into a trajectory goal        
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory = traj
        if self.path_tolerance is not None:
            goal.path_tolerance = self.path_tolerance
        if self.goal_tolerance is not None:
            goal.goal_tolerance = self.goal_tolerance
        if self.goal_time_tolerance is not None:
            goal.goal_time_tolerance = self.goal_time_tolerance

        return goal

    
    def _run_trajectory(self, trajectory, blocking=True):
        """
        Run a single trajectory

        Parameters
        ----------
        trajectory : dict or cartesian_control_msgs/CartesianTrajectory
            Trajectory to parse
        blocking : bool
            Whether to wait for the trajectory to finish.
        """
        if trajectory is None:
            rospy.logerr("Cannot run trajectory. It was not defined.")
            return

        if isinstance(trajectory, FollowCartesianTrajectoryGoal):
            goal = trajectory

        else:
            goal = self.build_goal(trajectory)     

        self.trajectory_client.send_goal(goal)

        if blocking:
            self.trajectory_client.wait_for_result()


    def go_to_point(self, point):
        """
        Move the arm from its current pose to a new pose.

        Parameters
        ----------
        point : dict or cartesian_control_msgs/CartesianTrajectoryPoint
            Trajectory point to go to
        """
        if isinstance(point, CartesianTrajectoryPoint):
            start_pt = copy.deepcopy(point)
            start_pt.time_from_start = rospy.Duration(self.initialize_time)
            traj = CartesianTrajectory(points=[start_pt], controlled_frame=self.controlled_frame)
        
        else:
            start_pt = copy.deepcopy(point)
            start_pt['time'] = self.initialize_time
            traj = [start_pt]


        self._run_trajectory(trajectory=traj, blocking=True) 


    def run_trajectory(self, trajectory=None, blocking=True, perform_init=True):
        """
        Run a trajectory.

        Parameters
        ----------
        trajectory : dict, CartesianTrajectory, FollowCartesianTrajectoryGoal
            Trajectory to run. If excluded, the currently loaded trajectory is run
        blocking : bool
            Whether to wait for the trajectory to finish.
        """
        if trajectory is None:
            if self.trajectory is None:
                rospy.logerr("Cannot run trajectory. It was not defined.")
                return
            else:
                trajectory =  copy.deepcopy(self.trajectory)
        else:
            trajectory = copy.deepcopy(trajectory)

        if isinstance(trajectory, FollowCartesianTrajectoryGoal):
            first_pt = trajectory.trajectory.points[0]
            #Adjust speeds
            for pt in trajectory.trajectory.points:
                pt.time_from_start = rospy.Duration(pt.time_from_start.to_sec()/self.speed_factor)
                if pt.twist is not None:
                    pt.twist.linear.x = pt.twist.linear.x*self.speed_factor
                    pt.twist.linear.y = pt.twist.linear.y*self.speed_factor
                    pt.twist.linear.z = pt.twist.linear.z*self.speed_factor
                    pt.twist.angular.x = pt.twist.angular.x*self.speed_factor
                    pt.twist.angular.y = pt.twist.angular.y*self.speed_factor
                    pt.twist.angular.z = pt.twist.angular.z*self.speed_factor

        elif isinstance(trajectory, CartesianTrajectory):
            first_pt = trajectory.points[0]
            #Adjust speeds
            for pt in trajectory.points:
                pt.time_from_start = pt.time_from_start/self.speed_factor
                if pt.twist is not None:
                    pt.twist.linear.x = pt.twist.linear.x*self.speed_factor
                    pt.twist.linear.y = pt.twist.linear.y*self.speed_factor
                    pt.twist.linear.z = pt.twist.linear.z*self.speed_factor
                    pt.twist.angular.x = pt.twist.angular.x*self.speed_factor
                    pt.twist.angular.y = pt.twist.angular.y*self.speed_factor
                    pt.twist.angular.z = pt.twist.angular.z*self.speed_factor

        else:
            first_pt = trajectory[0]
            #Adjust speeds
            for pt in trajectory:
                pt['time'] = pt['time']/self.speed_factor

                if 'linear_velocity' in pt.keys() and 'angular_velocity' in pt.keys():
                    pt['linear_velocity'] = (np.array(pt['linear_velocity'])*self.speed_factor).tolist() 
                    pt['angular_velocity'] = (np.array(pt['angular_velocity'])*self.speed_factor).tolist()

        # Go to the beginning of the trajectory
        if perform_init:
            self.go_to_point(first_pt)

        # Run the trajectory
        self._run_trajectory(trajectory=trajectory, blocking=blocking)


    def shutdown(self):
        """
        Shut down gracefully
        """
        self.trajectory_client.cancel_all_goals()


    def __del__(self):
        """
        Shut down gracefully upon object deletion
        """
        self.shutdown()
