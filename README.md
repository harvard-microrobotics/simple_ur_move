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
1. `roslaunch simple_ur_move run_cartesian_trajectory.launch traj:=test_traj.yaml`

### Useful commands for debugging
- Show the controller manager: `rosrun rqt_controller_manager rqt_controller_manager`
- Enable sending of single messages
    - `rqt`
    - Go to _Plugins_ >> _Topics_ >> _Message Publisher_
