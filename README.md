# Kugle-Gazebo
Gazebo Simulation model of the Kugle robot

Notice that this model is not a complete simulation model of the ballbot but is only used as an abstraction to simulate and test navigation and planning algorithms for the Kugle robot using its' holonomic properties.


# Cloning
Clone this repository into an existing catkin workspace:
```bash
mkdir -p ~/kugle_simulation_ws/src
cd ~/kugle_simulation_ws/src
catkin_init_workspace
git clone https://github.com/mindThomas/Kugle-Gazebo
```

# Building
Build the project with catkin build
```bash
cd ~/kugle_simulation_ws
catkin build
source devel/setup.zsh
```

# Launch simulation
```roslaunch kugle_gazebo gazebo.launch```
