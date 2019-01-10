# Kugle-Gazebo
Gazebo Simulation model of the Kugle robot

Notice that this model is not a complete simulation model of the ballbot but is only used as an abstraction to simulate and test navigation and planning algorithms for the Kugle robot using its' holonomic properties.

# Install catkin tool
```bash
sudo apt-get install python-catkin-tools

# Cloning
To set up the simulation environment you need to clone the necessary repositories into an existing or new catkin workspace.
Follow the steps below to set up a new catkin workspace and clone:
```bash
mkdir -p ~/kugle_simulation_ws/src
cd ~/kugle_simulation_ws/src
catkin_init_workspace
git clone https://github.com/mindThomas/Kugle-Gazebo
git clone https://github.com/mindThomas/Kugle-ROS
git clone https://github.com/mindThomas/realsense_gazebo_plugin
```

# Building
Build the project with catkin build
```bash
cd ~/kugle_simulation_ws
catkin build
source devel/setup.bash
```

# Launch simulation
```roslaunch kugle_gazebo gazebo.launch```
