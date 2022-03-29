#!/usr/bin/env python
# Import ROS stuff
import os
import time
import rospy
import rospkg
from simple_ur_move.twist_handler import TwistHandler


filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')


if __name__ == '__main__':
    # Load in parameters from roslaunch
    try:
        rospy.init_node('ur_twist_runner', disable_signals=True)

        robot_name = rospy.get_param(rospy.get_name()+"/robot_name",rospy.get_name())
        config_file = rospy.get_param(rospy.get_name()+"/config","twist_config.yaml")
        controller_to_use = rospy.get_param(rospy.get_name()+"/controller", 'twist_controller')
        debug = rospy.get_param(rospy.get_name()+"/debug", False)

    except:
        raise


    # Create a twist handler
    try:
        
        twist_handler = TwistHandler(
            name="",
            controller=controller_to_use,
            debug=debug)
    except:
        raise

    # Run the trajectory
    try:

        # Load config from a file
        filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
        twist_handler.load_config(filename=config_file, directory=filepath_config)

        # Set a twist of 10 mm/sec in the z-direction
        twist_handler.set_speed_factor(1.0)
        twist={'linear':[0,0,0.01], 'angular':[0,0,0]}
        twist_handler.set_twist(twist)
        print("Moving at 5mm/s in the z-direction")
        rospy.sleep(4)
        twist_handler.shutdown() # Sets speeds back to zero

    except KeyboardInterrupt:
        print("ur_twist_runner: Shutting Down")
        twist_handler.shutdown()

    except rospy.ROSInterruptException:
        print("ur_twist_runner: Shutting Down")
        twist_handler.shutdown()

   



        