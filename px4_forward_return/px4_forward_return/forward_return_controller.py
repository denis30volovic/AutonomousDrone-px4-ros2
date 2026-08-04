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
import std_msgs.msg
import numpy as np
import time

class ForwardReturnController(Node):

    def __init__(self):
        super().__init__('forward_return_controller')

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
            Float32, '/drone_command/forward_return',
            self.command_callback, 10)
        
        # Mission lock publisher and subscriber for inter-node coordination
        self.mission_lock_publisher = self.create_publisher(
            std_msgs.msg.String, '/drone_mission_lock', 10)
        self.mission_lock_subscriber = self.create_subscription(
            std_msgs.msg.String, '/drone_mission_lock',
            self.mission_lock_callback, 10)

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
        self.mission_locked = False
        self.phase = 'forward'  # 'forward', 'rotate', 'return', 'done'
        self.forward_start_time = None
        self.rotate_start_time = None
        self.return_start_time = None
        self.forward_duration = 4.0  # seconds to reach target
        self.rotate_duration = 8.0   # seconds to complete 360 rotation
        self.return_duration = 4.0   # seconds to return

        # Create timers
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('=== FORWARD-RETURN CONTROLLER ===')
        self.get_logger().info('Send command: ros2 topic pub -1 /drone_command/forward_return std_msgs/msg/Float32 "data: 10.0"')
        self.get_logger().info('Drone will fly forward X meters, rotate 360°, and return to start position')

    def local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.timestamp = msg.timestamp

    def command_callback(self, msg):
        """Receive movement command"""
        if self.mission_locked:
            self.get_logger().info(f'📡 Command ignored: Mission locked by another node')
            return
            
        if self.mission_started and not self.mission_completed:
            self.get_logger().info(f'📡 Command ignored: Mission already in progress (Phase: {self.phase})')
            return
        
        if msg.data <= 0:
            self.get_logger().info(f'📡 Command ignored: Distance must be positive, got {msg.data}')
            return
        
        self.mission_distance = msg.data
        self.command_received = True
        self.get_logger().info(f'📡 Command: FORWARD-RETURN {msg.data:.1f}m')
        
        # Reset mission if new command and previous mission was completed
        if self.mission_started and self.mission_completed:
            self.mission_started = False
            self.mission_completed = False
            self.phase = 'forward'
            self.rotate_start_time = None
            self.get_logger().info('🔄 Resetting for new mission...')

    def mission_lock_callback(self, msg):
        if msg.data == "lock":
            self.mission_locked = True
            self.command_received = False
            self.get_logger().info('🔒 Mission locked by another node')
        elif msg.data == "unlock":
            self.mission_locked = False
            self.get_logger().info('🔓 Mission unlocked')

    def status_callback(self):
        """Status every second"""
        if self.mission_started and not self.mission_completed:
            if self.phase == 'forward' and self.forward_start_time:
                elapsed = time.time() - self.forward_start_time
                progress = min(elapsed / self.forward_duration, 1.0) * 100
                self.get_logger().info(f'Forward phase: {progress:.1f}% complete')
            elif self.phase == 'rotate' and self.rotate_start_time:
                elapsed = time.time() - self.rotate_start_time
                progress = min(elapsed / self.rotate_duration, 1.0) * 100
                degrees = progress * 360 / 100
                self.get_logger().info(f'Rotation phase: {degrees:.1f}° of 360° complete')
            elif self.phase == 'return' and self.return_start_time:
                elapsed = time.time() - self.return_start_time
                progress = min(elapsed / self.return_duration, 1.0) * 100
                self.get_logger().info(f'Return phase: {progress:.1f}% complete')
            else:
                self.get_logger().info(f'Mission phase: {self.phase}')
        else:
            state_names = {4: "HOLD", 6: "OFFBOARD", 14: "LANDING"}
            state = state_names.get(self.nav_state, f"STATE_{self.nav_state}")
            if self.mission_locked:
                cmd = "🔒 Locked by another node"
            else:
                cmd = "✅ Ready" if self.command_received else "⏳ Waiting for command"
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
        self.get_logger().info('🚁 Switching to OFFBOARD mode')

    def calculate_smooth_trajectory(self, t, duration):
        """Calculate smooth trajectory using cosine interpolation"""
        if t >= duration:
            return 1.0
        
        # Smooth curve using cosine interpolation
        progress = t / duration
        smooth_progress = 0.5 * (1 - np.cos(np.pi * progress))
        
        return smooth_progress

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
            not self.mission_completed and
            not self.mission_locked):
            
            # Save starting position
            self.start_position = [
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ]
            self.start_yaw = self.vehicle_local_position.heading
            
            self.get_logger().info(f'🚀 Starting: Forward {self.mission_distance:.1f}m, rotate 360°, and return mission')
            
            # Lock the mission
            lock_msg = std_msgs.msg.String()
            lock_msg.data = "lock"
            self.mission_lock_publisher.publish(lock_msg)
            
            # Start mission
            self.mission_started = True
            self.start_time = time.time()
            self.phase = 'forward'
            self.forward_start_time = time.time()
            
            if self.nav_state != 6:  # If not already in OFFBOARD
                self.switch_to_offboard_mode()
            
        elif self.mission_started and not self.mission_completed:
            # Execute mission phases
            if self.phase == 'forward':
                # Forward phase
                if self.forward_start_time is None:
                    self.forward_start_time = time.time()
                
                elapsed = time.time() - self.forward_start_time
                
                if elapsed < self.forward_duration:
                    # Calculate smooth forward movement
                    progress = self.calculate_smooth_trajectory(elapsed, self.forward_duration)
                    distance = self.mission_distance * progress
                    
                    # Calculate target position using drone's heading
                    target_x = self.start_position[0] + distance * np.cos(self.start_yaw)
                    target_y = self.start_position[1] + distance * np.sin(self.start_yaw)
                    target_z = self.start_position[2]
                    
                    # Send command
                    self.publish_position_setpoint(target_x, target_y, target_z, self.start_yaw)
                else:
                    # Forward phase complete, start rotation
                    self.phase = 'rotate'
                    self.rotate_start_time = time.time()
                    self.get_logger().info('🔄 Forward complete, starting 360° rotation...')
            
            elif self.phase == 'rotate':
                # Rotation phase
                if self.rotate_start_time is None:
                    self.rotate_start_time = time.time()
                
                elapsed = time.time() - self.rotate_start_time
                
                if elapsed < self.rotate_duration:
                    # Calculate smooth rotation (360 degrees)
                    progress = self.calculate_smooth_trajectory(elapsed, self.rotate_duration)
                    rotation_angle = progress * 2 * np.pi  # 360 degrees in radians
                    target_yaw = self.start_yaw + rotation_angle
                    
                    # Stay at forward position while rotating
                    target_x = self.start_position[0] + self.mission_distance * np.cos(self.start_yaw)
                    target_y = self.start_position[1] + self.mission_distance * np.sin(self.start_yaw)
                    target_z = self.start_position[2]
                    
                    # Send command with updated yaw
                    self.publish_position_setpoint(target_x, target_y, target_z, target_yaw)
                else:
                    # Rotation phase complete, start return
                    self.phase = 'return'
                    self.return_start_time = time.time()
                    self.get_logger().info('🔄 360° rotation complete, returning to start position...')
            
            elif self.phase == 'return':
                # Return phase
                if self.return_start_time is None:
                    self.return_start_time = time.time()
                
                elapsed = time.time() - self.return_start_time
                
                if elapsed < self.return_duration:
                    # Calculate smooth return movement
                    progress = self.calculate_smooth_trajectory(elapsed, self.return_duration)
                    
                    # Calculate current position (start from forward position to start position)
                    forward_target_x = self.start_position[0] + self.mission_distance * np.cos(self.start_yaw)
                    forward_target_y = self.start_position[1] + self.mission_distance * np.sin(self.start_yaw)
                    
                    # Interpolate from forward position back to start
                    target_x = forward_target_x + (self.start_position[0] - forward_target_x) * progress
                    target_y = forward_target_y + (self.start_position[1] - forward_target_y) * progress
                    target_z = self.start_position[2]
                    
                    # Send command
                    self.publish_position_setpoint(target_x, target_y, target_z, self.start_yaw)
                else:
                    # Return phase complete
                    self.phase = 'done'
                    self.mission_completed = True
                    self.command_received = False
                    
                    # Unlock the mission
                    unlock_msg = std_msgs.msg.String()
                    unlock_msg.data = "unlock"
                    self.mission_lock_publisher.publish(unlock_msg)
                    
                    self.get_logger().info('✅ Mission completed! Returned to start position.')
            
            elif self.phase == 'done':
                # Hold at start position
                self.publish_position_setpoint(
                    self.start_position[0], 
                    self.start_position[1], 
                    self.start_position[2], 
                    self.start_yaw
                )
        
        elif self.mission_completed and not self.command_received:
            # Keep holding position until new command
            if self.start_position is not None:
                self.publish_position_setpoint(
                    self.start_position[0], 
                    self.start_position[1], 
                    self.start_position[2], 
                    self.start_yaw
                )

def main(args=None):
    rclpy.init(args=args)
    controller = ForwardReturnController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()