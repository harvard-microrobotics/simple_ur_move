# Simple UR Move
A minimal example of how to move a UR robot with the new [Universal Robot ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). This package also includes some useful utilities to take care of common tasks such as running joint or cartesian waypoint trajectories.


## Installation
1. Clone this package to the `src` folder of your catkin workspace
2. In the root folder of your workspace, install dependencies:
    - `rosdep install --from-paths src --ignore-src -r -y`
3. Build your workspace (`catkin_make`)


## Usage
This package has some useful python objects you can import into your own nodes to send trajectories to the robot.

- `JointTrajectoryHandler`: Sends joint trajectories to the robot.
    - Choose between several [UR ROS controllers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md): `scaled_pos_joint_traj_controller`, `scaled_vel_joint_traj_controller`, `pos_joint_traj_controller`, `vel_joint_traj_controller`, and `forward_joint_traj_controller`
    - You can also go to specific joint configurations via the `go_to_point()` function.
- `CartesianTrajectoryHandler`: Sends end effector pose trajectories to the robot.
    - Choose between several [UR ROS controllers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md): `pose_based_cartesian_traj_controller`, `joint_based_cartesian_traj_controller`, and `forward_cartesian_traj_controller`
    - You can also go to specific cartesian poses via the `go_to_point()` function.
- `TwistHandler`: Sends cartesian end effector velocities to the robot.
    - This uses the `twist_controller` from the [UR ROS controllers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md).


### Bringup the robot
1. _(Teach Pendant)_ Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
2. _(Host Computer)_ `roslaunch ur_user_calibration bringup_armando.launch`
3. _(Teach Pendant)_ Run the "EXTERNAL_CONTROL.urp" program.


### Using the joint command interface

#### Run a joint trajectory
1. Set up a trajectory config (see config folder for examples)
2. `roslaunch simple_ur_move run_joint_trajectory.launch traj:=test_traj_joint.yaml`

**Check out these launch files for information on different joint controllers you can use.**

#### Use the joint trajectory handler
1. Import necessary packages
```python
import os
import rospkg
from simple_ur_move.joint_trajectory_handler import JointTrajectoryHandler
```
2. Create a trajectory handler
```python
traj_handler = JointTrajectoryHandler(
    name="",
    controller="scaled_pos_joint_traj_controller",
    debug=False)
```
3a. Load the trajectory/config from a file
```python
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="test_traj_joint.yaml"
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
5. Move directly to a point
```python
point={'position',[0,0,0,0,0,0]}    # Define the home configuration
traj_handler.set_initialize_time(3.0) # Set the transition time
traj_handler.go_to_point(point)      # Go to the point
```

**All together:**
```python
import os
import rospkg
from simple_ur_move.joint_trajectory_handler import JointTrajectoryHandler

# Create a trajectory handler
traj_handler = JointTrajectoryHandler(
    name="",
    controller="scaled_pos_joint_traj_controller",
    debug=False)

# Load trajectory config from a file
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="test_traj_joint.yaml"
traj_handler.load_config(filename=traj_file, directory=filepath_config)

# OR Set trajectory config directly
#config={TRAJECTORY CONFIG DICT}
#traj_handler.set_config(config)

# Run the trajectory
traj_handler.set_initialize_time(3.0)
traj_handler.run_trajectory(blocking=True)
traj_handler.shutdown()

# OR: Go directly to a single point
point={'position',[0,0,0,0,0,0]}    # Define the home configuration
traj_handler.set_initialize_time(3.0) # Set the transition time here
traj_handler.go_to_point(point)      # Go to the point
```

### Using the cartesian command interface

#### Run a cartesian trajectory
1. Set up a trajectory config (see config folder for examples)
2. `roslaunch simple_ur_move run_cartesian_trajectory.launch traj:=test_traj.yaml`
3. `roslaunch simple_ur_move run_cartesian_trajectory.launch traj:=pick_place.yaml`

**Check out these launch files for information on different cartesian controllers you can use.**

#### Use the cartesian trajectory handler
1. Import nessecary packages
```python
import os
import rospkg
from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler
```
2. Create a trajectory handler
```python
traj_handler = CartesianTrajectoryHandler(
    name="",
    controller="pose_based_cartesian_traj_controller",
    debug=False)
```
3a. Load the trajectory/config from a file
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
5. Move directly to a point
```python
point={'position',[0.5, 0.5, 0.2]}    # Define the point
traj_handler.set_initialize_time(3.0) # Set the transition time
traj_handler.go_to_point(point)      # Go to the point
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

# OR: Go directly to a single point
point={'position',[0.5, 0.5, 0.2]}    # Define the point
traj_handler.set_initialize_time(3.0) # Set the transition time here
traj_handler.go_to_point(point)      # Go to the point
```


### Using the twist command interface

#### Use the twist handler
1. Import nessecary packages
```python
import os
import time
import rospkg
from simple_ur_move.twist_handler import TwistHandler
```
2. Create a twist handler
```python
traj_handler = TwistHandler(
    name="",
    debug=False)
```
3a. Load the config from a file
```python
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="twist_config.yaml"
traj_handler.load_config(filename=traj_file, directory=filepath_config)
```
3b. **_OR_** Set config directly
```python
config={TRAJECTORY CONFIG DICT}
traj_handler.set_config(config)
```
4. Set a twist of 5 mm/sec in the z-direction
```python
traj_handler.set_speed_factor(1.0)
twist={'linear':[0,0,0.005], 'angular':[0,0,0]}
traj_handler.set_twist(twist)
time.sleep(4)
traj_handler.shutdown() # Sets speeds back to zero
```

**All together:**
```python
import os
import time
import rospkg
from simple_ur_move.twist_handler import TwistHandler

# Create a twist handler
traj_handler = TwistHandler(
    name="",
    debug=False)

# Load config from a file
filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')
traj_file="twist_config.yaml"
traj_handler.load_config(filename=traj_file, directory=filepath_config)

# OR Set config directly
#config={TWIST CONFIG DICT}
#traj_handler.set_config(config)

# Set a twist of 5 mm/sec in the z-direction
traj_handler.set_speed_factor(1.0)
twist={'linear':[0,0,0.005], 'angular':[0,0,0]}
traj_handler.set_twist(twist)
time.sleep(4)
traj_handler.shutdown() # Sets speeds back to zero
```