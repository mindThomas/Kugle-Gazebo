# Kugle-Gazebo
Gazebo Simulation model of the Kugle robot

Notice that this model is not a complete simulation model of the ballbot but is only used as an abstraction to simulate and test navigation and planning algorithms for the Kugle robot using its' holonomic properties.

# Install catkin tool
```bash
sudo apt-get install python-catkin-tools
```

# Cloning
To set up the simulation environment you need to clone the necessary repositories into an existing or new catkin workspace.
Follow the steps below to set up a new catkin workspace and clone:
```bash
mkdir -p ~/kugle_ws/src
cd ~/kugle_ws/src
catkin_init_workspace
git clone https://github.com/mindThomas/Kugle-Gazebo
git clone https://github.com/mindThomas/Kugle-ROS
git clone https://github.com/mindThomas/realsense_gazebo_plugin
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

# Building
Build the project with catkin build
```bash
cd ~/kugle_simulation_ws
catkin build
source devel/setup.bash
```

# Launch simulation
The Gazebo simulation can be launched with
```bash
roslaunch kugle_gazebo gazebo.launch
```

But it is recommended to launch the simulation within the Kugle-ROS driver. See more at https://github.com/mindThomas/Kugle-ROS#simulation


