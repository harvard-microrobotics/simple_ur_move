#!/usr/bin/env python
# Import ROS stuff
from distutils.log import debug
import rospy
import rospkg
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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

class JointTrajectoryHandler():
    """
    The joint trajectory interface

    Parameters
    ----------
    name : str
        Name of the Action Server
    controller : str
        The joint controller to use. Options are:
        ``scaled_pos_joint_traj_controller``,
        ``scaled_vel_joint_traj_controller``,
        ``pos_joint_traj_controller``,
        ``vel_joint_traj_controller``, and
        ``forward_joint_traj_controller``.
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
        action_name = self.robot_name+'/'+self.controller_to_use+'/follow_joint_trajectory'

        self.trajectory_client = actionlib.SimpleActionClient(
                    action_name,
                    FollowJointTrajectoryAction)
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
        self.joint_names=self.settings.get('joint_names',None)


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
        trajectory : list or trajectory_msgs/JointTrajectory
            Trajectory to parse

        Raises
        ------
        ValueError : If a trajectory of incorrect type is passed
        """
        
        if not isinstance(trajectory, JointTrajectory) and not isinstance(trajectory, list):
            raise ValueError("Joint trajectories must be of type list or trajectory_msgs/JointTrajectory")
        else:
            self.trajectory = trajectory


    def pack_trajectory(self, trajectory):
        """
        Pack a trajectory into a ROS ``JointTrajectory``

        Parameters
        ----------
        trajectory : list
            Trajectory to parse
        
        Returns
        -------
        ros_traj : trajectory_msgs/JointTrajectory
            A ROS trajectory
        """
        traj_to_use = copy.deepcopy(trajectory)
        # Convert mm to m
        traj_to_use = self.convert_units(traj_to_use, direction='to_ros')
    
        traj_packed = []
        for waypoint in traj_to_use:
            if self.debug:
                print('orientation: ',waypoint['orientation'])
            pt = JointTrajectoryPoint()
            pt.positions = waypoint['positions']

            if 'velocities' in waypoint.keys():
                pt.velocities = waypoint['velocities']
            if 'accelerations' in waypoint.keys():
                pt.accelerations = waypoint['accelerations']
            if 'effort' in waypoint.keys():
                pt.effort = waypoint['effort']

            pt.time_from_start = rospy.Duration(waypoint['time'])
            traj_packed.append(pt)

        if len(traj_packed)>0:
            # Trajectories cannot start at time=0 becasue of how the UR ROS driver is written
            traj_packed[0].time_from_start+=rospy.Duration(0.001)

        traj_ros = JointTrajectory(points=traj_packed, joint_names=self.joint_names)
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
            # Convert degrees to radians
            if 'degree' in self.units['orientation']:
                parts_to_convert=['positions', 'velocities', 'accelerations']
                for waypoint in trajectory:
                    for part in parts_to_convert:
                        if part in waypoint.keys():
                            waypoint[part] = np.deg2rad(waypoint[part]).tolist()

        elif direction == 'from_ros':
            # Convert radians to degrees
            if 'degree' in self.units['orientation']:
                parts_to_convert=['positions', 'velocities', 'accelerations']
                for waypoint in trajectory:
                    for part in parts_to_convert:
                        if part in waypoint.keys():
                            waypoint[part] = np.rad2deg(waypoint[part]).tolist()

        return trajectory


    def build_goal(self, trajectory):
        """
        Build the trajectory goal

        Parameters
        ----------
        trajectory : dict or trajectory_msgs/JointTrajectory
            Trajectory to parse

        Returns
        -------
        goal : trajectory_msgs/FollowJointTrajectoryGoal
            The goal built from the trajectory
        """
        if trajectory is None:
            rospy.logerr("Cannot build trajectory goal. It was not defined.")
            return

        if isinstance(trajectory, JointTrajectory):
            traj = trajectory
        else:
            traj= self.pack_trajectory(trajectory)

        if self.debug:
            print("")
            for row in traj.points:
                print(row.time_from_start.to_sec(), row.positions)

        # Pack the trajectory into a trajectory goal        
        goal = FollowJointTrajectoryGoal()
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
        trajectory : dict or trajectory_msgs/JointTrajectory
            Trajectory to parse
        blocking : bool
            Whether to wait for the trajectory to finish.
        """
        if trajectory is None:
            rospy.logerr("Cannot run trajectory. It was not defined.")
            return

        if isinstance(trajectory, FollowJointTrajectoryGoal):
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
        point : dict or trajectory_msgs/JointTrajectoryPoint
            Trajectory point to go to
        """
        if isinstance(point, JointTrajectoryPoint):
            start_pt = copy.deepcopy(point)
            start_pt.time_from_start = rospy.Duration(self.initialize_time)
            traj = JointTrajectory(points=[start_pt], joint_names=self.joint_names)
        
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
        trajectory : dict, JointTrajectory, FollowJointTrajectoryGoal
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

        if isinstance(trajectory, FollowJointTrajectoryGoal):
            first_pt = trajectory.trajectory.points[0]
            #Adjust speeds
            for pt in trajectory.trajectory.points:
                pt.time_from_start = rospy.Duration(pt.time_from_start.to_sec()/self.speed_factor)
                if pt.velocities is not None:
                    pt.velocities = (np.array(pt.velocities)*self.speed_factor).tolist()
                if pt.accelerations is not None:
                    pt.accelerations = (np.array(pt.accelerations)*self.speed_factor).tolist()
                
        elif isinstance(trajectory, JointTrajectory):
            first_pt = trajectory.points[0]

            #Adjust speeds
            for pt in trajectory.points:
                pt.time_from_start = rospy.Duration(pt.time_from_start.to_sec()/self.speed_factor)
                if pt.velocities is not None:
                    pt.velocities = (np.array(pt.velocities)*self.speed_factor).tolist()
                if pt.accelerations is not None:
                    pt.accelerations = (np.array(pt.accelerations)*self.speed_factor).tolist()
        else:
            first_pt = trajectory[0]

            #Adjust speeds
            for pt in trajectory:
                pt['time'] = pt['time']/self.speed_factor
                if 'velocities' in pt.keys():
                    pt['velocities'] = (np.array(pt['velocities'])*self.speed_factor).tolist()
                if 'accelerations' in pt.keys():
                    pt['accelerations'] = (np.array(pt['accelerations'])*self.speed_factor).tolist()

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