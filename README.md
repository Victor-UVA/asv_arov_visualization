# asv_arov_visualization
## Install ROS 2 Jazzy and Gazebo Harmonic
```
sudo apt install ros-jazzy-desktop
sudo apt-get install ros-jazzy-ros-gz
```
## STL Meshes
Grab STL files from the vehicle models folder and add them to the meshes folder. The file paths in the world .sdf shouldn't need renaming (I hope).

## Opening the world file
```
ros2 launch asv_arov_visualization sim_launch.py
```
