#!/usr/bin/env python3
# Import ROS stuff
import rospy
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController, ListControllers
import simple_ur_move.utils as utils
from ur_msgs.srv import SetSpeedSliderFraction
from std_srvs.srv import TriggerRequest
   
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
        self.update_controller_list()
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


    def update_controller_list(self):
        '''
        Update the controller list via a ROS service call.
        '''
        self.controller_list = self.get_controller_list()


    def get_controllers_with_state(self, states=None):
        '''
        Get a list of controllers that have a particular state

        Parameters
        ----------
        states : list or str
            List of states (or s single state) to check for
            (``uninitialized``, ``initialized``, ``running``, ``stopped``,
            ``waiting``, ``aborted``, ``unknown``)

        Returns
        -------
        controllers : list
            List of controller names matching the states
        '''
        if states is None:
            states = ['all']
        elif isinstance(states, str):
            states=[states]

        self.update_controller_list()
        controllers=[]
        for ctrl in self.controller_list:
            if ('all' in states) or(ctrl.state in states):
                controllers.append(ctrl.name)
        
        return controllers

    
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
        # Check if the controller is already loaded
        all_controllers = self.get_controllers_with_state()
        loaded_controllers = self.get_controllers_with_state(['running','stopped','initialized'])
        if self.debug:
            print("ALL CONTROLLERS")
            print(all_controllers)
            print('\n'+"LOADED CONTROLLERS")
            print(loaded_controllers)

        if str(controller) in loaded_controllers:
            return True
        
        if self.debug:
            print("LOADING CONTROLLER: %s"%(controller))
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
        self.update_controller_list()
        controllers_to_unload = []

        for ctrl in self.controller_list:
            if (controller != ctrl.name) and (ctrl.name not in self.reserved_controllers):
                controllers_to_unload.append(str(ctrl.name))

        if self.debug:
            print(controllers_to_unload)
                
        self.load_controller(controller)
        self.switch_controller([controller],controllers_to_unload)


    def set_speed_slider(self, fraction):
        '''
        Set the speed slider fraction

        Parameters
        ----------
        fraction : float
            Slider fraction to set (0.02 to 1.00)

        Returns
        -------
        result : srv
            The result of the service call
        '''
        name = self.robot_name+'/ur_hardware_interface/set_speed_slider'
        result = utils.call_service(name, SetSpeedSliderFraction, speed_slider_fraction=fraction)
        return result


    def play_program(self):
        '''
        Start the program on the teach pendant.
        This ony works if you are in remote control mode

        Returns
        -------
        result : srv
            The result of the service call
        '''
        name = self.robot_name+'/ur_hardware_interface/dashboard/play'
        result = utils.call_service(name, TriggerRequest)
        return result


    def stop_program(self):
        '''
        Stop the program on the teach pendant.
        This ony works if you are in remote control mode

        Returns
        -------
        result : srv
            The result of the service call
        '''
        name = self.robot_name+'/ur_hardware_interface/dashboard/stop'
        result = utils.call_service(name, TriggerRequest)
        return result
