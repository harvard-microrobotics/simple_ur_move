#!/usr/bin/env python
# Import ROS stuff
import rospy
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController, ListControllers
import simple_ur_move.utils as utils
   
reserved_controllers = ['joint_state_controller', 'robot_status_controller', 'force_torque_sensor_controller', 'speed_scaling_state_controller']

class ControllerHandler():
    '''
    Handle ROS controllers

    Parameters
    ----------
    robot_name : str
        Name of the robot
    '''
    def __init__(self, robot_name, debug=False):
        self.robot_name = robot_name
        self.reserved_controllers = reserved_controllers
        self.controller_list = self.get_controller_list()
        self.debug = debug

    
    def get_controller_list(self):
        '''
        Get the list of currently loaded controllers

        Returns
        -------
        controllers : list
            List of controllers
        '''
        name = self.robot_name+'/controller_manager/list_controllers'
        result = utils.call_service(name, ListControllers)
        return result.controller

    
    def set_reserved_controllers(self, controllers):
        '''
        Set which controllers are reserved (always running)

        Parameters
        ----------
        controllers : list
            List of controller names
        '''
        self.reserved_controllers = controllers


    def load_controller(self, controller):
        '''
        Load a ROS controller

        Parameters
        ----------
        controller : str
            Name of the controller to load

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        name = self.robot_name+'/controller_manager/load_controller'
        result = utils.call_service(name, LoadController, name=str(controller))
        return result


    def unload_controller(self, controller):
        '''
        Unload a ROS controller

        Parameters
        ----------
        controller : str
            Name of the controller to unload

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        name = self.robot_name+'/controller_manager/unload_controller'
        result = utils.call_service(name, UnloadController, name=str(controller))
        return result

    
    def switch_controller(self, start_controllers, stop_controllers, strictness=1, start_asap=False, timeout=0):
        '''
        Switch ROS controllers

        Parameters
        ----------
        start_controllers : list
            Names of the controllers to start
        stop_controllers : list
            Names of the controllers to stop
        strictness : int
            Strictness of controller switching
        start_asap : bool
            Decide whether controllers should be started immediately
        timeout : int
            Timeout (in seconds)

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        name = self.robot_name+'/controller_manager/switch_controller'
        result = utils.call_service(name, SwitchController,
                                    start_controllers = start_controllers,
                                    stop_controllers = stop_controllers,
                                    strictness = strictness,
                                    start_asap = start_asap,
                                    timeout = float(timeout))
        return result


    def set_controller(self,controller):
        '''
        Set which ROS controller is started, and stop all others

        Parameters
        ----------
        controller : str
            Name of the controller to start

        Returns
        -------
        response : str
            Service response from the controller manager
        '''

        controllers_to_unload = []

        for ctrl in self.controller_list:
            if (controller != ctrl.name) and (ctrl.name not in self.reserved_controllers):
                controllers_to_unload.append(str(ctrl.name))

        if self.debug:
            print(controllers_to_unload)
                
        self.load_controller(controller)
        self.switch_controller([controller],controllers_to_unload)