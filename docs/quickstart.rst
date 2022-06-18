.. _quickstart:

================
Quickstart Guide
================
*(from "* `README.md <https://github.com/harvard-microrobotics/simple_ur_move/blob/main/README.md>`_ *" in github repo)*

Installation
____________

1. Clone the `Universal Robot ROS Driver <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>`_ and associated dependencies
2. Clone this package to the `src` folder of your catkin workspace
3. In the root folder of your workspace, install dependencies:
    - ``rosdep install --from-paths src --ignore-src -r -y``
4. Build your workspace (``catkin_make``)


Usage
_____

This package has some useful python objects you can import into your own nodes to send trajectories to the robot.

- ``JointTrajectoryHandler``: Sends joint trajectories to the robot.
    - Choose between several `UR ROS controllers <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md>`_: ``scaled_pos_joint_traj_controller``, ``scaled_vel_joint_traj_controller``, ``pos_joint_traj_controller``, ``vel_joint_traj_controller``, and ``forward_joint_traj_controller``
    - You can also go to specific joint configurations via the ``go_to_point()`` function.
- ``CartesianTrajectoryHandler``: Sends end effector pose trajectories to the robot.
    - Choose between several `UR ROS controllers <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md>`_: ``pose_based_cartesian_traj_controller``, ``joint_based_cartesian_traj_controller``, and ``forward_cartesian_traj_controller``
    - You can also go to specific cartesian poses via the ``go_to_point()`` function.
- ``TwistHandler``: Sends cartesian end effector velocities to the robot.
    - This uses the ``twist_controller`` from the `UR ROS controllers <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md>`_.

Check out the :ref:`Examples <examples>`, the :ref:`full API reference <api_reference>`, or run any of the launch files in the ``launch`` folder.