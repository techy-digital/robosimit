## RoboSimIT
Robot Simulation Verification Tool

## Stack
- Ubuntu 20.04.2 LTS (Host and guest)
- ROS Noetic Desktop Full
- Gazebo 11
- NVIDIA Driver (465)

## Building Image
```
git clone https://github.com/techy-digital/robosimit.git
cd robosimit
docker build -t valu3s:robosimit .
```
### Running a single container
```
sh start_container.sh
```