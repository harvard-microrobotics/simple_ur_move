.. _start_arm:

=============
Start the Arm
=============

1. Set up the arm
    a. *(Teach Pendant)* Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
    b. *(Teach Pendant)* Load the desired installation (Load >> Installation)
    c. *(Teach Pendant)* Start the robot (tap the small red dot on the bottom left corner)
2. Bringup the arm (ROS)
    a. *(Host Computer)* In a new terminal:
        - If your arm is calibrated (for example, as in `this package <https://github.com/harvard-microrobotics/ur_user_calibration>`_ ):  ``roslaunch ur_user_calibration bringup_armando.launch``
        - If your arm is not calibrated, use the `standard bringup command <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#quick-start>`_ : ``roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=<robot_ip>``
    b. *(Teach Pendant)* Move the arm around manually to set things up.
    c. *(Teach Pendant)* Once you are ready to test, run the "EXTERNAL_CONTROL.urp" program. (press "play" in the bottom bar) 