# Turtlebot3 in Warehouse Environment Simulation
This guide will walk you through setting up the Autostore-Robot package. Follow these steps to get started:


## 1. Clone the Package
First, navigate to your ROS workspace where you want to install the package. Then, clone the `warehouse_pkg` package from its GitHub repository into the `src` folder:
```
mkdir -p <your_ros_workspace>/src
cd <your_ros_workspace>/src
git clone https://github.com/truongtran19315/Autostore-Robot warehouse_pkg
``` 


## 2. Build the Package
After cloning the package, navigate to your ROS workspace and build it using `catkin_make`:

```
cd <your_ros_workspace>/src && catkin_make
```
## 3. Set Environment Variables
Add the following lines to your `.bashrc` file and update it to include the ROS setup and your workspace setup:

```
source /opt/ros/noetic/setup.bash
source ~/<your_ros_workspace>/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```
After adding these lines, update your `.bashrc` by running:

```
source ~/bashrc
```
## 4. Start ROS Core
Open a terminal and start the ROS core:

```
cd <your_ros_workspace>
roscore
```

## 5. Launch Gazebo Simulation
In a new terminal, launch the Gazebo simulation for the TurtleBot3:

```
roslaunch warehouse_pkg warehouse.launch
```

## 6. Run the Avoider Node
Finally, in another terminal, run the `avoider_done.py` node from the `warehouse_pkg` package:

```
rosrun warehouse_pkg runRobot.py
```
