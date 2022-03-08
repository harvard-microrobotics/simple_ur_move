# Simple UR Move
A minimal example of how to move a UR robot with the new [Universal Robot ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). This package also includes some useful utilities to take care of common tasks.


## Installation
1. Clone this package to the `src` folder of your catkin workspace
2. In the root folder of your workspace, install dependencies:
    - `rosdep install --from-paths src --ignore-src -r -y`
3. Build your workspace (`catkin_make`)


## Usage

### Bringup the robot
1. _(Teach Pendant)_ Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
2. _(Host Computer)_ `roslaunch ur_user_calibration bringup_armando.launch`
3. _(Teach Pendant)_ Run the "EXTERNAL_CONTROL.urp" program.

### Run a trajectory
1. Set up a trajectory config (see config folder for examples)
2. `roslaunch simple_ur_move run_cartesian_trajectory.launch traj:=test_traj.yaml`
2. `roslaunch simple_ur_move run_cartesian_trajectory.launch traj:=pick_place.yaml`

### Use the cartesian trajectory handler
```python
import os
import rospkg
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="pick_place.yaml"

# Create a trajectory handler and load trajectory
traj_handler = CartesianTrajectoryHandler(
    name="",
    controller="pose_based_cartesian_traj_controller",
    debug=False)
traj_handler.load_config(filename=traj_file, directory=filepath_config)
traj_handler.set_initialize_time(3.0)

# Run trajectory
traj_handler.run_trajectory(blocking=True)

# Shutdown
traj_handler.shutdown()
```