# Fetchbot-Grasping-Simulation

Controls the fetch bot to find, move to, and grasp an object in front of it.

## Contributors

- **Nick (75%)**
  - Object Searching (matlab/BlueScanner.m)
  - Object Coordinate Conversion (Pixels -> Camera Coordinate -> Robot Reference Frame Coordinates)
  - Robot Movement Script (entire matlab/main.m script)
  - Matlab moveit! Bridge node (matlab_move_it_bridge, package setup and python script)
  - README file

- **Aiden (25%)**
  - Object Detection (detectBlue() function in matlab/BlueScanner.m)
  - Matlab Code Comments

- **Nimmo Napier-Rowney (0%)**

## Setup Instructions

### 1. Matlab Setup
- Download and place all contents of the 'matlab' folder inside a directory in Matlab. 
- It may require the robotics toolbox which can be found [here](https://canvas.uts.edu.au/courses/27375/files/5451349?wrap=1).

### 2. Linux Setup
- Download the `matlab_moveit_bridge` folder and place it into your `~/catkin_ws/src` folder.
- Execute the following commands:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-melodic-fetch-moveit-config
  cd ~/catkin_ws
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
- If you don't have the fetch bot, follow the guide [here](https://canvas.uts.edu.au/courses/28447/files/5257344?module_item_id=1405074). 
### 3. Running the Grasping Simulation
- Download the `run.sh` file.
- In the same directory, run `./run.sh` and wait until gazebo has launched and the robot arm is set in place.
- Run the `main.m` matlab code.

## Possible Issues and Troubleshooting:
- If `bridge_node.py` does not run, try installing necessary packages with:
  ```bash
  sudo apt-get install ros-melodic-moveit-msgs
- The robot's accuracy is dependent on the speed of the computer. For instance, my laptop only succeeds about 50% of the time, but my PC runs it perfectly every time.
  - After research, the issue seems to arise from the physics simulation lagging, and occasionally, MoveIt! not finding joint configurations swiftly enough.

## For any questions, email me at nicholas.surmon@student.uts.edu.au    
