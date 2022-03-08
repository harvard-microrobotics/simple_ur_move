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
1. Import nessecary packages
```python
import os
import rospkg
from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler
```
2. Create a trajectory handler and load trajectory
```python
traj_handler = CartesianTrajectoryHandler(
    name="",
    controller="pose_based_cartesian_traj_controller",
    debug=False)
```
3a. Load the trajectory config from a file
```python
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="pick_place.yaml"
traj_handler.load_config(filename=traj_file, directory=filepath_config)
```
3b. **_OR_** Set trajectory config directly
```python
config={TRAJECTORY CONFIG DICT}
traj_handler.set_config(config)
```
4. Run the trajectory
```python
traj_handler.set_initialize_time(3.0)
traj_handler.run_trajectory(blocking=True)
traj_handler.shutdown()
```

**All together:**
```python
import os
import rospkg
from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler

# Create a trajectory handler
traj_handler = CartesianTrajectoryHandler(
    name="",
    controller="pose_based_cartesian_traj_controller",
    debug=False)

# Load trajectory config from a file
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="pick_place.yaml"
traj_handler.load_config(filename=traj_file, directory=filepath_config)

# OR Set trajectory config directly
#config={TRAJECTORY CONFIG DICT}
#traj_handler.set_config(config)

# Run the trajectory
traj_handler.set_initialize_time(3.0)
traj_handler.run_trajectory(blocking=True)
traj_handler.shutdown()
```
