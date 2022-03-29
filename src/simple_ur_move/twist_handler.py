#!/usr/bin/env python
# Import ROS stuff
from distutils.log import debug
import rospy
import rospkg
from geometry_msgs.msg import Vector3, Twist

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

class TwistHandler():
    """
    The twist interface

    Parameters
    ----------
    name : str
        Name of the Action Server
    controller : str
        The twist controller to use. Options are:
        ``twist_controller``
    debug : bool
        Turn on debug print statements
    self_contained : bool
        Decide whether to set jog speed back to zero when object is deleted.
    """
    def __init__(self, name, controller='twist_controller', debug=False, self_contained=False):
        # Get parameters from roslaunch
        self.debug = debug
        self.robot_name = name
        self.controller_to_use = controller
        self._parse_config(None)
        self.speed_factor = 1.0
        self.self_contained=self_contained


        # Set ROS controllers
        if self.debug:
            print("Setting controller: %s"%(controller))
        self.controller_handler = ControllerHandler(self.robot_name)
        self.controller_handler.set_controller(self.controller_to_use)

        # Publish to the twist command topic
        topic_name = self.robot_name+'/'+self.controller_to_use+'/command'
        if self.debug:
            print("Subscribing to topic: %s"%(topic_name))
        self.twist_publisher = rospy.Publisher(topic_name, Twist, queue_size=10, latch=True)
        rospy.sleep(0.5)


    def load_config(self, filename, directory=None):
        """
        Load a configuration from a file

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
        Set the configuration

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

        if self.settings is None:
            return

        self.units = self.settings.get('units',None)


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

    
    def convert_units(self, twist, direction='to_ros'):
        """
        Convert units from the units defined in the trajectory config
        to ROS default units (or vice versa).

        Parameters
        ----------
        twist : dict
            twist to parse
        direction : str
            Which direction to convert. Options are ``to_ros`` or ``from_ros``.
        
        Returns
        -------
        twist : dict
            Converted twist
        """
        if direction == 'to_ros':
            # Convert mm to m
            if self.units['linear']=='mm/s':
                twist['linear'] = (np.array(twist['linear'])/1000).tolist()

            # Convert degrees to radians
            if 'degree' in self.units['angular']:
                twist['angular'] = np.deg2rad(twist['angular']).tolist()

        elif direction == 'from_ros':
            # Convert mm to m
            if self.units['linear']=='mm/s':
                twist['linear'] = (np.array(twist['linear'])*1000).tolist()

            # Convert radians to degrees
            if 'degree' in self.units['angular']:
                twist['angular'] = np.rad2deg(twist['angular']).tolist()

        return twist


    def build_twist(self, linear, angular):
        '''
        Build a twist message from vectors

        Parameters
        ----------
        linear : list
            The linear twist components [x,y,z]
        angular : list
            The angular twist components [x,y,z]

        Returns
        -------
        twist : geometry_msgs/Twist
            The resulting twist message
        '''
        linear_out = Vector3()
        linear_out.x=linear[0]
        linear_out.y=linear[1]
        linear_out.z=linear[2]

        angular_out = Vector3()
        angular_out.x=angular[0]
        angular_out.y=angular[1]
        angular_out.z=angular[2]

        twist_out = Twist()
        twist_out.linear=linear_out
        twist_out.angular=angular_out

        return twist_out

    
    def _set_twist(self, twist):
        """
        Run a single trajectory

        Parameters
        ----------
        twist : dict or geometrty_msgs/Twist
            Twist to set
        blocking : bool
            Whether to wait for the trajectory to finish.
        """
        if twist is None:
            rospy.logerr("Cannot run trajectory. It was not defined.")
            return

        if isinstance(twist, Twist):
            msg = twist

        else:
            msg = self.build_twist(twist['linear'],twist['angular'])    

        if self.debug:
            print(msg) 

        self.twist_publisher.publish(msg)


    def set_twist(self, twist):
        """
        Run a trajectory.

        Parameters
        ----------
        twist : dict or geometrty_msgs/Twist
            Twist to set.
        """
        if twist is None:
            rospy.logerr("Cannot run trajectory. It was not defined.")
            return
        else:
            twist = copy.deepcopy(twist)

        #Adjust speeds
        if isinstance(twist, Twist):
            twist.linear.x = twist.linear.x*self.speed_factor
            twist.linear.y = twist.linear.y*self.speed_factor
            twist.linear.z = twist.linear.z*self.speed_factor
            twist.angular.x = twist.angular.x*self.speed_factor
            twist.angular.y = twist.angular.y*self.speed_factor
            twist.angular.z = twist.angular.z*self.speed_factor

        elif isinstance(twist, dict):
            twist['linear'] = (np.array(twist['linear'])*self.speed_factor).tolist() 
            twist['angular'] = (np.array(twist['angular'])*self.speed_factor).tolist()

            twist = self.convert_units(twist, direction='to_ros')

        # Set the twist
        self._set_twist(twist=twist)


    def shutdown(self):
        """
        Shut down gracefully
        """
        if self.self_contained:
            self._set_twist({'linear':[0,0,0], 'angular':[0,0,0]})


    def __del__(self):
        """
        Shut down gracefully upon object deletion
        """
        self.shutdown()