.. _cartesian_traj:

==========================
Run Cartesian Trajectories
==========================


Lets walk through how to run cartesian trajectories using a `CartesianTrajectoryHandler`. We will also see how the existing "run_cartesian_trajectory.py" script works (specify trajectory in a yaml file, then use `roslaunch` to run it on the robot.)


.. contents:: Contents:
    :local:
    :depth: 1

Overview
________

``CartesianTrajectoryHandler`` sends cartesian trajectories to the robot.

We can choose between several `UR ROS controllers <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md>`_ :

- ``pose_based_cartesian_traj_controller``,
- ``joint_based_cartesian_traj_controller`` (default)
- ``forward_cartesian_traj_controller``

You can also go directly to specific tool poses via the ``go_to_point()`` function.


Build a Trajectory
__________________

Configuration
-------------

We have several settings to choose for our trajectory configuration. These tell the package how to build trajectories into "ROS" format.

Config files are stored in the `config folder <https://github.com/harvard-microrobotics/simple_ur_move/tree/devel-noetic/config>`_. Config files can contain a trajectory (i.e. config and trajectory stored all in the same place), but typically we just want to set up the ``settings`` part, then dynamically set trajectories in Python instead.

Configuration settings:

- **units**
    - **position**: Units to use for positions. Options are either ``m`` (meter) or ``mm`` (millimeter)
    - **orientation**: Units to use for orientations. Options are ``euler_degrees``, ``euler_radians``, or ``quaternion``
- **controlled_frame**: Names of the tf frame you are controlling. Usually this is ``tool0``.
- **interpolate**: Whether or not to interpolate the trajectory.  Options are ``smooth``, ``linear``, or ``False`` (turn off interpolation).
- **interp_time**: Interpolation time (default is 0.005 sec)
- **path_tolerance**: A set of tolerances for the trajectory based on `CartesianTolerance <http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/msg/CartesianTolerance.html>`_. *Typically the default will work fine.*
- **goal_tolerance**: A set of tolerances for the trajectory goal point based on `CartesianTolerance <http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/msg/CartesianTolerance.html>`_. *Typically the default will work fine.*
- **goal_time_tolerance**: A tolerance (in sec) for the trajectory goal time. *Typically the default will work fine.*

.. note::

    **A note on trajectory interpolation:**

    The builtin inverse kinematic solvers for the UR's seem to sometimes jump in configuration space, causing large motions for seemingly close cartesian waypoints. To prevent this from happening, this package implements a very fine-grained interpolation between waypoints (every 0.005 sec). This leads to reliable results.


.. warning::

    **Known Issue:**

    If waypoint orientations are defined in euler angles, occasionally the robot can flip around to alternative configurations. This is because the conversion between euler angles and quaternions happens in ROS without knowledge of the current robot configuration. If waypoint orientations are defined in quaternions directly, this problem does not exist.


Trajectory
----------

A trajectory is just a list of waypoints in time. For cartesian trajectories, each waypoint has five parts:

- **time**: The time point (in sec). The trajectory implicitly starts at *0 sec* even if no point exists.
- **position**: Tool position (in [position_units]). This is required.
- **orientation**: Tool orientation (in [orientation_units]). This is required.
- **linear_velocity**: Tool's linear velocity (in [position_units]/s). If left out, zero velocity is used.
- **angular_velocity**: Tool's angular velocity(in [orientation_units]/s). If left out, zero velocity is used.
- **posture**: Joint posture to use. *(This is typically omitted).*
    - **posture_joint_names**: List of the robot's joint names
    - **posture_joint_values**: List of joint orientations (in rad)


**Example of a typical config file with settings and a cartesian trajectory:**

.. literalinclude:: ../../config/test_traj.yaml


Run Cartesian Trajectories (the simple way)
___________________________________________

This package has a simple script that loads a trajectory config file and runs it on your arm. We invoke it using `run_cartesian_trajectory.launch <https://github.com/harvard-microrobotics/simple_ur_move/blob/devel-noetic/launch/run_cartesian_trajectory.launch>`_, which itself runs a simple python script (`run_cartesian_trajectory.py <https://github.com/harvard-microrobotics/simple_ur_move/blob/devel-noetic/scripts/run_cartesian_trajectory.py>`_) with your given settings

1. :ref:`Startup the arm <start_arm>`
2. In a new terminal, ``roslaunch simple_ur_move run_cartesian_trajectory.launch traj:=test_traj.yaml`` *(Check out these launch files for information on different settings.)*

The trajectory will be run one time. Tada!


Run Cartesian Trajectories (with python)
________________________________________

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
    from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler

    # Create a trajectory handler
    traj_handler = CartesianTrajectoryHandler(
        name="",
        controller="pose_based_cartesian_traj_controller",
        debug=False)

    # Load the configuration from a file
    filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
    traj_file="test_traj.yaml"
    traj_handler.load_config(filename=traj_file, directory=filepath_config)

    # OR set the configuration directly
    # config = {CONFIG_DICT}
    # traj_handler.set_config(config)

    # Go directly to a point
    point={'position',[0.5, 0.5, 0.2], 'orientation',[180, 0, 90]}    # Define the home configuration
    traj_handler.set_initialize_time(3.0) # Set the transition time here
    traj_handler.go_to_point(point)      # Go to the point

    # Set up other settings
    traj_handler.set_trajectory(traj)
    traj_handler.set_initialize_time(3.0) # Time the robot should take to get to the starting position.
    traj_handler.set_speed_factor(1.0) # Speed multiplier (smaller=slower, larger=faster)

    # Run the trajectory
    traj=[{'time':1.0, 'position',[-0.5, -0.5, 0.2], 'orientation',[180, 0, 90]},
          {'time':3.0, 'position',[-0.3, -0.3, 0.15], 'orientation',[180, 0, 90]}]

    traj_handler.run_trajectory(trajectory=traj)
