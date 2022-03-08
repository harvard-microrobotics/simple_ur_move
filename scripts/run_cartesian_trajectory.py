#!/usr/bin/env python
# Import ROS stuff
import rospy
import rospkg
import os

from simple_ur_move.trajectory_handler import TrajectoryHandler


filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')

class TrajectoryRunner():
    """
    A ROS Action server to run single tests.

    Parameters
    ----------
    name : str
        Name of the Action Server
    """
    def __init__(self, name, controller=None, debug=False):
        # Get parameters from roslaunch
        self.debug = rospy.get_param(rospy.get_name()+"/debug",debug)
        self.robot_name = rospy.get_param(rospy.get_name()+"/robot_name",name)
        self.traj_file = rospy.get_param(rospy.get_name()+"/traj",None)
        self.controller_to_use = rospy.get_param(rospy.get_name()+"/controller",controller)

        self.traj_handler = TrajectoryHandler(self.robot_name, self.controller_to_use, self.debug)
        self.traj_handler.load_config(self.traj_file, filepath_config)
        self.traj_handler.set_initialize_time(3.0)

    def run_trajectory(self):
        self.traj_handler.run_trajectory(blocking=True)
    
    def shutdown(self):
        self.traj_handler.shutdown()



if __name__ == '__main__':
    # Load in parameters from roslaunch
    try:
        rospy.init_node('ur_cartesian_traj_runner', disable_signals=True)

        robot_name = rospy.get_param(rospy.get_name()+"/robot_name",rospy.get_name())
        traj_file = rospy.get_param(rospy.get_name()+"/traj",None)
        controller_to_use = rospy.get_param(rospy.get_name()+"/controller")
        debug = rospy.get_param(rospy.get_name()+"/debug", False)

    except:
        raise

    # Create a traje handler
    try:
        traj_handler = TrajectoryHandler(robot_name, controller_to_use, debug)
    except:
        raise

    # Run the trajectory
    try:
        traj_handler.load_config(traj_file, filepath_config)
        traj_handler.set_initialize_time(3.0)

        traj_handler.run_trajectory(blocking=True)
        traj_handler.shutdown()

    except KeyboardInterrupt:
        print("ur_cartesian_traj_runner: Shutting Down")
        sender.shutdown()

    except rospy.ROSInterruptException:
        print("ur_cartesian_traj_runner: Shutting Down")
        sender.shutdown()

   



        