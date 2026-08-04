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
import std_msgs.msg

class OrbitController(Node):
    def __init__(self):
        super().__init__('orbit_controller')

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)
        
        # Command subscriber
        self.command_subscriber = self.create_subscription(
            Float32, '/drone_command/orbit',
            self.command_callback, 10)
        
        # Mission lock publisher and subscriber for inter-node coordination
        self.mission_lock_publisher = self.create_publisher(
            std_msgs.msg.String, '/drone_mission_lock', 10)
        self.mission_lock_subscriber = self.create_subscription(
            std_msgs.msg.String, '/drone_mission_lock',
            self.mission_lock_callback, 10)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.nav_state = None
        self.timestamp = 0
        self.start_position = [None, None, None]
        self.start_yaw = None
        self.mission_started = False
        self.mission_completed = False
        self.start_time = None
        self.command_received = False
        self.phase = 'takeoff'  # takeoff, orbit, descent, return, done
        self.mission_locked = False  # True if another node has mission control

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('=== PX4 ORBIT CONTROLLER ===')
        self.get_logger().info('⏳ WAITING FOR COMMAND - Send command to start mission:')
        self.get_logger().info('   ros2 topic pub -1 /drone_command/orbit std_msgs/msg/Float32 "data: 1.0"')
        self.get_logger().info('The drone will take off, orbit, and return to start.')

    def local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.timestamp = msg.timestamp

    def command_callback(self, msg):
        """Receive orbit command"""
        if self.mission_locked:
            self.get_logger().info(f'📡 Command ignored: Mission locked by another node')
            return
            
        if self.mission_started and not self.mission_completed:
            self.get_logger().info(f'📡 Command ignored: Mission already in progress (Phase: {self.phase})')
            return
        
        # Only process command if not locked and not in progress
        self.command_received = True
        self.get_logger().info(f'📡 Command: ORBIT mission initiated')
        
        # Reset mission if new command and previous mission was completed
        if self.mission_started and self.mission_completed:
            self.mission_started = False
            self.mission_completed = False
            self.phase = 'takeoff'
            self.get_logger().info('🔄 Resetting for new mission...')

    def status_callback(self):
        state_names = {4: "HOLD", 6: "OFFBOARD", 14: "LANDING"}
        state = state_names.get(int(self.nav_state) if self.nav_state is not None else -1, f"STATE_{self.nav_state}")
        if self.mission_started and not self.mission_completed:
            self.get_logger().info(f'Status: {state} | Phase: {self.phase}')
        else:
            if self.mission_locked:
                cmd = "🔒 Locked by another node"
            else:
                cmd = "✅ Ready" if self.command_received else "⏳ Waiting for command"
            self.get_logger().info(f'Status: {state} | {cmd}')

    def switch_to_offboard_mode(self):
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

    def switch_to_land_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('🛬 Initiating LAND mode')

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.timestamp
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()

        # Wait for vehicle to be armed and ready
        if not self.mission_started:
            if (self.vehicle_status.arming_state >= 2 and  # Armed
                self.nav_state in [4, 6, 14] and  # HOLD, OFFBOARD, or LANDING mode
                self.command_received and
                not self.mission_completed and
                not self.mission_locked):  # Check mission lock
                
                if self.vehicle_local_position.x is None or self.vehicle_local_position.y is None or self.vehicle_local_position.z is None or self.vehicle_local_position.heading is None:
                    return  # Wait for valid position
                
                # Lock the mission
                lock_msg = std_msgs.msg.String()
                lock_msg.data = "lock"
                self.mission_lock_publisher.publish(lock_msg)
                
                self.start_position = [
                    float(self.vehicle_local_position.x),
                    float(self.vehicle_local_position.y),
                    float(self.vehicle_local_position.z)
                ]
                self.start_yaw = float(self.vehicle_local_position.heading)
                self.start_time = time.time()
                self.mission_started = True
                self.phase = 'takeoff'
                if self.nav_state != 6:
                    self.switch_to_offboard_mode()
                self.get_logger().info('🚀 Mission started!')
            else:
                return

        # If mission is already in progress, ignore new commands until current mission completes
        elif self.mission_started and not self.mission_completed:
            # Mission is in progress, continue with current mission
            pass
        else:
            # Mission completed, ready for new command
            return

        if self.start_position is None or self.start_yaw is None:
            return  # Wait for valid start position

        elapsed = time.time() - self.start_time

        # Takeoff phase: ascend to 5m above start
        if self.phase == 'takeoff':
            sx, sy, sz, syaw = self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw
            if sx is None or sy is None or sz is None or syaw is None:
                return  # Wait for valid start position/altitude/yaw
            start_x = float(sx)
            start_y = float(sy)
            start_z = float(sz)
            start_yaw = float(syaw)
            target_z = start_z - 5.0  # NED: down is positive
            self.publish_position_setpoint(start_x, start_y, target_z, start_yaw)
            
            # Debug altitude check
            if self.vehicle_local_position.z is not None:
                altitude_diff = abs(self.vehicle_local_position.z - target_z)
                takeoff_timeout = 30.0  # 30 seconds timeout
                
                if altitude_diff < 0.5:  # Increased tolerance from 0.2 to 0.5
                    self.phase = 'orbit'
                    self.orbit_start_time = time.time()
                    self.get_logger().info(f'🛸 Reached 5m altitude (diff: {altitude_diff:.2f}m), starting orbit...')
                elif elapsed > takeoff_timeout:
                    # Force transition if timeout reached
                    self.phase = 'orbit'
                    self.orbit_start_time = time.time()
                    self.get_logger().info(f'⏰ Takeoff timeout reached ({takeoff_timeout}s), forcing orbit start...')
                else:
                    # Debug log every few seconds
                    if int(time.time()) % 3 == 0:  # Log every 3 seconds
                        self.get_logger().info(f'🛫 Takeoff: Current Z={self.vehicle_local_position.z:.2f}, Target Z={target_z:.2f}, Diff={altitude_diff:.2f}m, Time={elapsed:.1f}s')

        # Orbit phase: 1m radius circle at 5m altitude, 1 revolution
        elif self.phase == 'orbit':
            sx, sy, sz, syaw = self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw
            if sx is None or sy is None or sz is None or syaw is None:
                return  # Wait for valid start position/altitude/yaw
            start_x = float(sx)
            start_y = float(sy)
            start_z = float(sz)
            start_yaw = float(syaw)
            orbit_duration = 10.0  # seconds for 1 revolution
            t = time.time() - self.orbit_start_time
            if t > orbit_duration:
                self.phase = 'descent'
                self.descent_start_time = time.time()
                self.get_logger().info('🔄 Orbit complete, descending to original height...')
                return
            angle = 2 * np.pi * (t / orbit_duration)
            radius = 1.0
            # Center orbit around start position at 5m altitude
            x = start_x + radius * np.cos(angle)
            y = start_y + radius * np.sin(angle)
            z = start_z - 5.0  # 5m altitude
            yaw = float(angle + start_yaw)
            self.publish_position_setpoint(x, y, z, yaw)

        # Descent phase: lower back to original altitude
        elif self.phase == 'descent':
            sx, sy, sz, syaw = self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw
            if sx is None or sy is None or sz is None or syaw is None:
                return  # Wait for valid start position/altitude/yaw
            start_x = float(sx)
            start_y = float(sy)
            target_z = float(sz)  # Original altitude
            start_yaw = float(syaw)
            self.publish_position_setpoint(start_x, start_y, target_z, start_yaw)
            if self.vehicle_local_position.z is not None and abs(self.vehicle_local_position.z - target_z) < 0.2:
                self.phase = 'return'
                self.get_logger().info('🛬 Reached original altitude, returning to start...')

        # Return phase: go back to start position at original altitude
        elif self.phase == 'return':
            sx, sy, sz, syaw = self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw
            if sx is None or sy is None or sz is None or syaw is None:
                return  # Wait for valid start position/altitude/yaw
            x, y = float(sx), float(sy)
            z = float(sz)  # Original altitude
            yaw = float(syaw)
            self.publish_position_setpoint(x, y, z, yaw)
            # If close to start, mission complete
            if self.vehicle_local_position.x is not None and self.vehicle_local_position.y is not None:
                dist = np.linalg.norm([
                    self.vehicle_local_position.x - x,
                    self.vehicle_local_position.y - y
                ])
                if dist < 0.2:
                    self.phase = 'done'
                    self.mission_completed = True
                    self.command_received = False  # Reset for next command
                    
                    # Unlock the mission
                    unlock_msg = std_msgs.msg.String()
                    unlock_msg.data = "unlock"
                    self.mission_lock_publisher.publish(unlock_msg)
                    
                    self.get_logger().info('✅ Mission completed! Holding position at start...')

        # Done phase: keep holding position in OFFBOARD mode
        elif self.phase == 'done':
            sx, sy, sz, syaw = self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw
            if sx is None or sy is None or sz is None or syaw is None:
                return  # Wait for valid start position/altitude/yaw
            x, y = float(sx), float(sy)
            z = float(sz)  # Original altitude
            yaw = float(syaw)
            self.publish_position_setpoint(x, y, z, yaw)

        elif self.mission_completed and not self.command_received:
            # Keep holding position until new command
            if self.start_position is not None:
                sx, sy, sz, syaw = self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw
                if sx is not None and sy is not None and sz is not None and syaw is not None:
                    x, y = float(sx), float(sy)
                    z = float(sz)  # Original altitude
                    yaw = float(syaw)
                    self.publish_position_setpoint(x, y, z, yaw)

    def mission_lock_callback(self, msg):
        if msg.data == "lock":
            self.mission_locked = True
            self.command_received = False  # Clear any pending commands
            self.get_logger().info('🔒 Mission locked by another node')
        elif msg.data == "unlock":
            self.mission_locked = False
            self.get_logger().info('🔓 Mission unlocked')

def main(args=None):
    rclpy.init(args=args)
    controller = OrbitController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 