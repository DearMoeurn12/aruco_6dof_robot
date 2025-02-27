
# Manipulator

#### create a new workspace
``` bash
export COLCON_WS=~/manipulator_ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src
git clone https://github.com/DearMoeurn12/aruco_6dof_robot.git
```

#### install moveit2

``` bash
sudo apt install ros-humble-moveit
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
echo " export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```


#### install dependencies packages
``` bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-ur-msgs ros-humble-gazebo-ros ros-humble-moveit-servo
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-realtime-tools
sudo apt install ros-humble-warehouse-ros-sqlite
sudo apt install ros-humble-hardware-interface-testing
```

#### build the workspace
``` bash
cd $COLCON_WS
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```


#### run the simulation
``` bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
ros2 run execute_pose send_goal_pose
```



#### Reference
- [MoveIt2](https://moveit.ros.org/install-moveit2/source/)
- [Pymoveit2](https://github.com/AndrejOrsula/pymoveit2)
- [ROS2 Control](https://ros-controls.github.io/control.ros.org/)
- [ROS2 Controllers](https://ros-controls.github.io/control.ros.org/)
- [Universal Robots Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Universal Robots URDF](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [Universal Robots Gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation)