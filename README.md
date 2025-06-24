# asv_arov_visualization
## Install ROS 2 Jazzy and Gazebo Harmonic
```
sudo apt install ros-jazzy-desktop
sudo apt-get install ros-jazzy-ros-gz
```
## STL Meshes
Grab STL files from the 'Gazebo Models' folder on OneDrive and add them to the meshes folder. The file paths in tank_world.sdf shouldn't need renaming, hopefully.
## Opening the world file
```
ros2 launch asv_arov_visualization sim_launch.py
```
## Helpful stuff
[Gazebo Harmonic Tutorials](https://gazebosim.org/docs/harmonic/tutorials/)
