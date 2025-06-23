# Hornet
Hornet is an autonomous UAV developed as part of the Aeromodelling Club at IIT Guwahati. The drone is capable of real-time obstacle detection and avoidance using a 2D LiDAR system and onboard processing.

# Features
2D LiDAR Integration for dynamic obstacle detection (up to 5m range)
Autonomous Flight Control using MAVLink, PX4, and PID tuning
Sensor Data Processing with ROS for real-time navigation
Failsafe Behavior to halt or reroute on encountering obstacles

# Tech Stack
Jetson Nano, PX4 Flight Stack
MAVSDK, MAVLink, PID Controller
ROS (Robot Operating System), OpenCV
 2D LiDAR, Gazebo (for simulation & testing)
 
# Highlights
Implemented real-time sensor fusion to enable smooth and responsive navigation
Developed and tested control algorithms in simulation before real-world deployment
Designed with modularity to support upgrades like 3D mapping or object tracking
