# README for ROS Flappy Simulation

## Overview
The ROS Flappy Simulation project integrates the MuJoCo physics engine with the Robot Operating System (ROS) to simulate a flapping wing model. This project provides a framework for real-time simulation and data publishing, including IMU sensor data, camera frames, and axes information.

## Project Structure
```
ros_flappy_sim
├── src
│   ├── mujoco_ros_bridge
│   │   ├── __init__.py
│   │   ├── mujoco_ros_bridge.py
│   │   ├── imu_publisher.py
│   │   ├── camera_publisher.py
│   │   ├── axes_publisher.py
│   │   └── utils.py
│   ├── Flappy_v9_RB.xml
│   ├── v9_flappy.py
│   └── JointAngleData.csv
├── launch
│   └── flappy_sim.launch
├── msg
│   ├── Axes.msg
│   └── CameraFrame.msg
├── scripts
│   └── run_sim.sh
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Installation
1. Ensure you have ROS installed on your system.
2. Clone this repository into your ROS workspace's `src` directory:
   ```
   git clone <repository-url> ros_flappy_sim
   ```
3. Navigate to the root of your ROS workspace and build the package:
   ```
   cd ~/catkin_ws
   catkin_make
   ```

## Usage
1. Source your ROS workspace:
   ```
   source devel/setup.bash
   ```
2. Launch the simulation:
   ```
   roslaunch ros_flappy_sim flappy_sim.launch
   ```

## Features
- **IMU Data Publishing**: The `IMUPublisher` class publishes IMU sensor data to a specified ROS topic.
- **Camera Stream**: The `CameraPublisher` class captures and publishes frames from the onboard camera.
- **Axes Data**: The `AxesPublisher` class publishes axes information to a ROS topic.
- **MuJoCo Integration**: The main simulation logic is handled in `v9_flappy.py`, which integrates with ROS for real-time data exchange.

## Custom Messages
- **Axes.msg**: Defines the structure for axes data.
- **CameraFrame.msg**: Defines the structure for camera frame data.

## Running the Simulation
To run the simulation, execute the provided shell script:
```
cd ros_flappy_sim/scripts
chmod +x run_sim.sh
./run_sim.sh
```

## Dependencies
- ROS (version)
- MuJoCo (version)
- Python packages: mujoco, mujoco_viewer, numpy, pandas, etc.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for details.