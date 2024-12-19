# gazebo_simulation


## Installation

Developed on Ubuntu 22.04.5 LTS ("Jammy") with ROS2 humble and Gazebo Fortress LTS (see here for a version overview https://gazebosim.org/docs/latest/ros_installation/).

### ROS2 Installation
Follow the installation instructions from the ROS2 humble Docs: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
sudo apt update
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
```

### Gazebo Installation
Follow the installation instructions from the Gazebo Docs: https://gazebosim.org/docs/fortress/install/
```
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

## Usage

### Basic Usage

As always (from the root of your ROS2 workspace)
```
colcon build --packages-select gazebo_simulation
source install/setup.bash
```

Start the basic simulation setup in gazebo:
```
ros2 launch gazebo_simulation setup_sim.launch.py controller:='joint_trajectory_controller'
```
Currently there are two controllers implemented: A JointTrajectoryController and a PidController. You can specify the controller to be used with the launch argument "controller". Possible values are 'pid_controller' (default) and 'joint_trajectory_controller'.
The latter one enables us to specify a whole trajectory over time within one message, interpolates between the timepoints and uses the interpolated values as reference for a PID controller (per Joint). The latter one is used to directly dictate the current reference value to a PID controller (per Joint).
If we want to follow a trajectory, the JointTrajectoryController should be used. If we want to hold a static position, the PidController is the simpler choice.

Information about the JointTrajectory message type (used by the JointTrajectoryController):
https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectory.html
https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html
https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html
https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html

Information about the MultiDOFCommand message type (used by the PidController):
https://docs.ros.org/en/humble/p/control_msgs/interfaces/msg/MultiDOFCommand.html
https://control.ros.org/humble/doc/ros2_controllers/pid_controller/doc/userdoc.html
https://control.ros.org/humble/doc/ros2_controllers/pid_controller/doc/userdoc.html#list-of-parameters

There are two publisher nodes, which publish some test signals to the corresponding controller. They can be started with the corresponding command:
```
ros2 run gazebo_simulation trajectory_publisher.py
ros2 run gazebo_simulation pid_controller_publisher.py
```
They can be used to implement own reference values/trajectories.

If one just wants to publish a value directly, e.g. for debugging purposes, one can use one of the following commands:
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['base_rotation_joint', 'arm_height_joint', 'arm_rotation_joint'], points: [{positions: [.0, .0, .0], time_from_start: {sec: 0}}, {positions: [.0.3, -.2, -.3], time_from_start: {sec: 1}}]}"
ros2 topic pub /pid_controller/reference control_msgs/msg/MultiDOFCommand "{dof_names: ['base_rotation_joint', 'arm_height_joint', 'arm_rotation_joint'], values: [.0, .0, .0]}"
```

### PID parameter

The parameter of the per-joint-PID-controllers used with both controller methods can be adjusted in the config/controller_manager.yaml.
The current values are not tuned, but chosen rather random (just to see some movement). They are probably too high and cause some osciallations.

### Accessing values of the simulation

The topic /joint_trajectory_controller/controller_state or /pid_controller/controller_state publishes information about the current position ("feedback") and velocity ("feedback_dot") of the joints, as well as their current reference value (position) and the output of the controller, which is in this case the effective effort of the joint, i.e. torque or force.

### Applying a wrench in Gazebo

There is a Node which calls a service from gazebo to apply a wrench to the endeffector of the robot. Run it with
```
ros2 run gazebo_simulation apply_wrench
```
It is specified to be applied in the center of the arm_third_link. 
(Caution: Gazebo summarizes the three arm links into a single one, since they are connected through fixed joints. The resulting single link is named after the first one "robot::arm_first_link" where the "robot::" prefix is used in Gazebo for all frames and links based on the name of the robot in the urdf file. For this reason, the Node applies the wrench at a reference point, which describes the center of the original arm_third_link inside the frame of the summarized robot::arm_first_link)