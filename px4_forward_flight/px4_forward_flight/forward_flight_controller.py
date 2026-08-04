#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import Float32
import numpy as np
import time

class ForwardFlightController(Node):

    def __init__(self):
        super().__init__('forward_flight_controller')

        # QoS profiles for PX4 compatibility
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)
        
        # Command subscriber
        self.command_subscriber = self.create_subscription(
            Float32, '/drone_command/distance',
            self.command_callback, 10)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.mission_distance = 0.0
        self.command_received = False
        self.start_position = None
        self.start_yaw = None
        self.mission_started = False
        self.mission_completed = False
        self.start_time = None
        self.nav_state = None
        self.timestamp = 0

        # Create timers
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('=== SIMPLE FORWARD/BACKWARD CONTROLLER ===')
        self.get_logger().info('Send command: ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: 10.0"')
        self.get_logger().info('Positive = Forward, Negative = Backward')

    def local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.timestamp = msg.timestamp

    def command_callback(self, msg):
        """Receive movement command"""
        self.mission_distance = msg.data
        self.command_received = True
        direction = "FORWARD" if msg.data > 0 else "BACKWARD"
        self.get_logger().info(f'Command: {direction} {abs(msg.data):.1f}m')
        
        # Reset mission if new command
        if self.mission_started:
            self.mission_started = False
            self.mission_completed = False

    def status_callback(self):
        """Status every second"""
        if self.mission_started and not self.mission_completed:
            elapsed = time.time() - self.start_time
            progress = min(elapsed / 3.0, 1.0) * 100  # 5 second mission
            self.get_logger().info(f'Mission: {progress:.1f}% complete')
        else:
            state_names = {4: "HOLD", 6: "OFFBOARD", 14: "LANDING"}
            state = state_names.get(self.nav_state, f"STATE_{self.nav_state}")
            cmd = "Ready" if self.command_received else "Waiting for command"
            self.get_logger().info(f'Status: {state} | {cmd}')

    def switch_to_offboard_mode(self):
        """Switch to offboard mode"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0  # OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('Switching to OFFBOARD mode')

    def switch_to_hold_mode(self):
        """Switch back to HOLD mode"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 4.0  # AUTO_LOITER (HOLD)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('Switching back to HOLD mode')

    def calculate_smooth_trajectory(self, t):
        """Simple smooth trajectory"""
        mission_duration = 3  # 5 seconds
        
        if t >= mission_duration:
            return abs(self.mission_distance), 0.0, 0.0
        
        # Simple smooth curve
        progress = t / mission_duration
        smooth_progress = 0.5 * (1 - np.cos(np.pi * progress))
        distance = abs(self.mission_distance) * smooth_progress
        
        return distance, 0.0, 0.0

    def publish_offboard_control_heartbeat_signal(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish position setpoint"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.timestamp
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self):
        """Main control loop"""
        # Always publish heartbeat
        self.publish_offboard_control_heartbeat_signal()

        # Start mission when: ARMED + (HOLD, OFFBOARD, or LANDING) + Command received
        if (self.vehicle_status.arming_state >= 2 and  # Armed
            self.nav_state in [4, 6, 14] and  # HOLD, OFFBOARD, or LANDING mode
            self.command_received and
            not self.mission_started and 
            not self.mission_completed):
            
            # Save starting position
            self.start_position = [
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ]
            self.start_yaw = self.vehicle_local_position.heading
            
            direction = "forward" if self.mission_distance > 0 else "backward"
            self.get_logger().info(f'Starting: {abs(self.mission_distance):.1f}m {direction}')
            
            # Start mission (switch to offboard if not already)
            self.mission_started = True
            self.start_time = time.time()
            if self.nav_state != 6:  # If not already in OFFBOARD
                self.switch_to_offboard_mode()
            
        elif self.mission_started and not self.mission_completed:
            # Execute mission
            elapsed = time.time() - self.start_time
            
            if elapsed < 3.0:  # 5 second mission
                # Calculate movement
                distance, _, _ = self.calculate_smooth_trajectory(elapsed)
                
                # Apply direction (negative for backward)
                if self.mission_distance < 0:
                    distance = -distance
                
                # Calculate target position using drone's heading
                target_x = self.start_position[0] + distance * np.cos(self.start_yaw)
                target_y = self.start_position[1] + distance * np.sin(self.start_yaw)
                target_z = self.start_position[2]
                
                # Send command
                self.publish_position_setpoint(target_x, target_y, target_z, self.start_yaw)
            else:
                # Mission complete - hold current position
                self.mission_completed = True
                self.command_received = False  # Reset for next command
                
                # Keep publishing final position to maintain OFFBOARD mode
                final_distance = self.mission_distance if self.mission_distance > 0 else self.mission_distance
                target_x = self.start_position[0] + final_distance * np.cos(self.start_yaw)
                target_y = self.start_position[1] + final_distance * np.sin(self.start_yaw)
                target_z = self.start_position[2]
                
                self.publish_position_setpoint(target_x, target_y, target_z, self.start_yaw)
                self.get_logger().info('Mission completed! Holding position...')
        
        elif self.mission_completed and not self.command_received:
            # Keep holding position until new command
            if self.start_position is not None:
                final_distance = self.mission_distance if self.mission_distance > 0 else self.mission_distance
                target_x = self.start_position[0] + final_distance * np.cos(self.start_yaw)
                target_y = self.start_position[1] + final_distance * np.sin(self.start_yaw)
                target_z = self.start_position[2]
                
                self.publish_position_setpoint(target_x, target_y, target_z, self.start_yaw)

def main(args=None):
    rclpy.init(args=args)
    controller = ForwardFlightController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()