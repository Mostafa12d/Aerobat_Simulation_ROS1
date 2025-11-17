#!/bin/bash

# Navigate to the workspace directory
cd /path/to/your/ros_flappy_sim

# Source the ROS setup.bash file
source /opt/ros/noetic/setup.bash

# Source the workspace setup.bash file
source devel/setup.bash

# Run the ROS launch file to start the simulation
roslaunch flappy_sim.launch

# Keep the terminal open after the simulation ends
exec bash