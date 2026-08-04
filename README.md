# Autonomous Drone with PX4 and ROS 2

An autonomous drone project built with ROS 2, PX4, and ran on Gazebo as the simulation environment. It demonstrates how ROS 2 nodes can communicate with the PX4 flight controller to send commands and receive vehicle telemetry 
in order to create flight paths so ground troops can easily scan areas.

The project consists of 3 different ROS 2 nodes, with a straight flight path job for live-time movement, a forward path with an orbit at the end and a return, and a simple forwrad/return path.
Commands can be issued using a GUI with gesture detection which send the ROS 2 command in the terminal to activate the nodes.

PX4 provides the autopilot and low-level flight control, while ROS 2 supports higher-level autonomous behavior and system coordination. Gazebo provides the simulated drone, physics, sensors, and environment for safe virtual testing.

The project serves as a foundation for experimenting with autonomous flight, navigation, and drone control without requiring physical hardware.


