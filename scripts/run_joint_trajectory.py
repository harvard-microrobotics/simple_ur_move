#!/usr/bin/env python3
# Import ROS stuff
import rospy
import rospkg
import os

from simple_ur_move.joint_trajectory_handler import JointTrajectoryHandler


filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')


if __name__ == '__main__':
    # Load in parameters from roslaunch
    try:
        rospy.init_node('ur_joint_traj_runner', disable_signals=True)

        robot_name = rospy.get_param(rospy.get_name()+"/robot_name",rospy.get_name())
        traj_file = rospy.get_param(rospy.get_name()+"/traj",None)
        controller_to_use = rospy.get_param(rospy.get_name()+"/controller")
        debug = rospy.get_param(rospy.get_name()+"/debug", False)

    except:
        raise

    # Create a traje handler
    try:
        traj_handler = JointTrajectoryHandler(robot_name, controller_to_use, debug)
    except:
        raise

    # Run the trajectory
    try:
        traj_handler.load_config(traj_file, filepath_config)
        traj_handler.set_initialize_time(3.0)

        traj_handler.run_trajectory(blocking=True)
        traj_handler.shutdown()

    except KeyboardInterrupt:
        print("ur_joint_traj_runner: Shutting Down")
        traj_handler.shutdown()

    except rospy.ROSInterruptException:
        print("ur_joint_traj_runner: Shutting Down")
        traj_handler.shutdown()

   



        