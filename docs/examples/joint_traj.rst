.. _joint_traj:

======================
Run Joint Trajectories
======================


Lets walk through how to run joint trajectories using a `JointTrajectoryHandler`. We will also see how the existing "run_joint_trajectory.py" script works (specify trajectory in a yaml file, then use `roslaunch` to run it on the robot.)


.. contents:: Contents:
    :local:
    :depth: 1

Overview
________

``JointTrajectoryHandler`` sends joint trajectories to the robot.

We can choose between several `UR ROS controllers <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md>`_ :

- ``scaled_pos_joint_traj_controller`` (default)
- ``scaled_vel_joint_traj_controller``
- ``pos_joint_traj_controller``
- ``vel_joint_traj_controller``
- ``forward_joint_traj_controller``

You can also go directly to specific joint configurations via the ``go_to_point()`` function.


Build a Trajectory
__________________

Configuration
-------------

We have several settings to choose for our trajectory configuration. These tell the package how to build trajectories into "ROS" format.

Config files are stored in the `config folder <https://github.com/harvard-microrobotics/simple_ur_move/tree/devel-noetic/config>`_. Config files can contain a trajectory (i.e. config and trajectory stored all in the same place), but typically we just want to set up the ``settings`` part, then dynamically set trajectories in Python instead.

Configuration settings:

- **units**
    - **orientation**: Units to use for joint angles. Options are either ``degrees`` or ``radians``
- **joint_names**: Names of the joints of your arm.
- **path_tolerance**: A set of tolerances for the trajectory based on `CartesianTolerance <http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/msg/CartesianTolerance.html>`_. *Typically the default will work fine.*
- **goal_tolerance**: A set of tolerances for the trajectory goal point based on `CartesianTolerance <http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/msg/CartesianTolerance.html>`_. *Typically the default will work fine.*
- **goal_time_tolerance**: A tolerance (in sec) for the trajectory goal time. *Typically the default will work fine.*


Trajectory
----------

A trajectory is just a list of waypoints in time. For joint trajectories, each waypoint has five parts:

- **time**: The time point (in sec). The trajectory implicitly starts at *0 sec* even if no point exists.
- **positions**: Joint positions at this time point (in [orientation_units]). These are required.
- **velocities**: Joint velocities at this time point (in [orientation_units]/s). If left out, zero velocity is used
- **accelerations**: Joint accelerations at this time point (in [orientation_units]/s^2). If left out, zero acceleration is used
- **effort**: Joint "efforts" (in arbitrary units). If left out, these are computed by the robot on-the-fly.


**Example of a typical config file with settings and a joint trajectory:**

.. literalinclude:: ../../config/test_traj_joint.yaml


Run Joint Trajectories (the simple way)
_______________________________________

This package has a simple script that loads a trajectory config file and runs it on your arm. We invoke it using `run_joint_trajectory.launch <https://github.com/harvard-microrobotics/simple_ur_move/blob/devel-noetic/launch/run_joint_trajectory.launch>`_, which itself runs a simple python script (`run_joint_trajectory.py <https://github.com/harvard-microrobotics/simple_ur_move/blob/devel-noetic/scripts/run_joint_trajectory.py>`_) with your given settings

1. :ref:`Startup the arm <start_arm>`
2. In a new terminal, ``roslaunch simple_ur_move run_joint_trajectory.launch traj:=test_traj_joint.yaml`` *(Check out these launch files for information on different settings.)*

The trajectory will be run one time. Tada!


Run Joint Trajectories (with python)
____________________________________

The big feature of this package is a very simple python interface that allows you to run trajectories directly via python (so you can focus on writing other, more-useful control software).

To run trajectories, do the following steps:

1. Import necessary packages
2. Create a trajectory handler
3. Load the configuration from a file *OR* set the configuration directly
4. Move directly to a point
5. Set up other settings
6. Run the trajectory

You can set and run trajectories on-the-fly, in a loop, etc. 

.. code-block:: python

    import os
    import rospkg
    from simple_ur_move.joint_trajectory_handler import JointTrajectoryHandler

    # Create a trajectory handler
    traj_handler = JointTrajectoryHandler(
        name="",
        controller="scaled_pos_joint_traj_controller",
        debug=False)

    # Load the configuration from a file
    filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
    traj_file="test_traj_joint.yaml"
    traj_handler.load_config(filename=traj_file, directory=filepath_config)

    # OR set the configuration directly
    # config = {CONFIG_DICT}
    # traj_handler.set_config(config)

    # Go directly to a point
    point={'positions',[0,0,0,0,0,0]}    # Define the home configuration
    traj_handler.set_initialize_time(3.0) # Set the transition time here
    traj_handler.go_to_point(point)      # Go to the point

    # Set up other settings
    traj_handler.set_trajectory(traj)
    traj_handler.set_initialize_time(3.0) # Time the robot should take to get to the starting position.
    traj_handler.set_speed_factor(1.0) # Speed multiplier (smaller=slower, larger=faster)

    # Run the trajectory
    traj=[{'time':1.0, 'positions',[0,0,0,0,0,0]},
          {'time':3.0, 'positions',[1.57,0,0,0,0,0.3]}]

    traj_handler.run_trajectory(trajectory=traj)
