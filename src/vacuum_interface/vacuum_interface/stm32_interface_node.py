#!/usr/bin/env python3
import rclpy
import rclpy.logging # Added for explicit logger level setting
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist # Added for cmd_vel
from tf_transformations import quaternion_from_euler
import serial
import time
import struct  # Assuming binary data format
import random # For simulation
import math   # For simulation

class Stm32InterfaceNode(Node):
    def __init__(self):
        super().__init__('stm32_interface_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('simulate', False) # Add simulate parameter

        # Get parameters
        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/wheel_odom', 10)

        # Subscriber for velocity commands
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("Subscribed to /cmd_vel")

        # Serial port setup or simulation timer
        self.serial_conn = None
        self.timer = None
        self.timer_period = 0.02 # seconds (50Hz) - Use a reasonable rate for simulation

        if self.simulate:
            self.get_logger().info("Running in SIMULATION mode. No serial connection.")
            # Simulation state variables
            self.sim_x = 0.0
            self.sim_y = 0.0
            self.sim_theta = 0.0
            self.sim_vx = 0.0 # Target linear velocity
            self.sim_vyaw = 0.0 # Target angular velocity
            self.last_sim_time = self.get_clock().now()
            # Create timer for simulation callback
            self.timer = self.create_timer(self.timer_period, self.simulation_callback)
            self.get_logger().info("Simulation timer started.")
        else:
            self.get_logger().info("Running in HARDWARE mode. Attempting serial connection.")
            try:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=1.0  # Read timeout in seconds
                )
                self.get_logger().info(f"Successfully connected to serial port: {self.serial_port} at {self.baud_rate} baud")
                # Timer for reading serial data
                self.timer = self.create_timer(self.timer_period, self.serial_read_callback)
                # Placeholder for incomplete data buffer
                self.read_buffer = b''
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
                # Consider exiting or retrying - rclpy.shutdown() called in main now
                # No return here, let main handle the shutdown based on timer status
            except Exception as e:
                 self.get_logger().error(f"Unexpected error during serial setup: {e}")
                 # Let main handle shutdown

        self.get_logger().info("STM32 Interface Node initialization sequence complete.")

    def cmd_vel_callback(self, msg):
        """
        Callback for receiving Twist messages from /cmd_vel.
        Extracts linear.x and angular.z and sends them to STM32 via serial.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f"Received cmd_vel: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}") # Changed to info

        if not self.simulate and self.serial_conn and self.serial_conn.is_open:
            try:
                # Pack the two floats (linear.x, angular.z) into bytes
                # Using '<2f' for little-endian, two floats
                # Consider adding start/end bytes or a checksum for more robust communication
                data_to_send = struct.pack('<2f', linear_x, angular_z)
                self.serial_conn.write(data_to_send)
                self.get_logger().info(f"Sent {len(data_to_send)} bytes to STM32: {data_to_send.hex()}") # Changed to info
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
                # Potentially handle reconnection or flag an error state
            except struct.error as e:
                self.get_logger().error(f"Error packing cmd_vel data: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error sending cmd_vel data: {e}", exc_info=True)
        elif self.simulate:
            # In simulation mode, we might want to update sim_vx and sim_vyaw
            # based on cmd_vel for more interactive simulation.
            # This part is currently handled by the simulation_callback's own logic.
            # If you want cmd_vel to drive the simulation, update self.sim_vx and self.sim_vyaw here.
            self.sim_vx = linear_x # Update simulated target linear velocity
            self.sim_vyaw = angular_z # Update simulated target angular velocity
            self.get_logger().info(f"SIMULATION: Updated target velocities: vx={self.sim_vx:.2f}, vyaw={self.sim_vyaw:.2f}") # Changed to info

    def serial_read_callback(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Read available data
                if self.serial_conn.in_waiting > 0:
                    data_bytes = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.read_buffer += data_bytes
                    self.get_logger().info(f"Read {len(data_bytes)} bytes. Buffer size: {len(self.read_buffer)}")

                    # --- Protocol Parsing Placeholder ---
                    # This is where you need to implement the logic to parse your specific
                    # custom serial protocol from STM32.
                    # Example: Look for start/end bytes, checksums, message IDs, etc.
                    # You might need to handle incomplete messages arriving across multiple reads.

                    # --- Example Parsing Logic (Replace with your actual protocol) ---
                    # Let's assume a simple protocol:
                    # Start Byte (0xA5) | Type (1=IMU, 2=Odom) | Length | Data | Checksum
                    HEADER = b'\xA5' # Start frame byte 0xA5
                    MIN_PACKET_LEN = 4 # Header + Type + Length + Checksum (at least)

                    while len(self.read_buffer) >= MIN_PACKET_LEN:
                        start_index = self.read_buffer.find(HEADER)
                        if start_index == -1:
                            # No header found, discard buffer (or keep last few bytes?)
                            # self.get_logger().warn("No header found, discarding buffer.")
                            self.read_buffer = b'' # Discard everything if no header
                            break

                        # Move buffer start to the header
                        if start_index > 0:
                            self.get_logger().warn(f"Discarding {start_index} bytes before header.")
                            self.read_buffer = self.read_buffer[start_index:]

                        # Check if we have enough data for header + type + length
                        if len(self.read_buffer) < 3:
                            break # Need more data for basic structure

                        msg_type = self.read_buffer[1]
                        msg_len = self.read_buffer[2] # Length of the 'Data' part
                        total_packet_len = 1 + 1 + 1 + msg_len + 1 # Header + Type + Length + Data + Checksum

                        if len(self.read_buffer) >= total_packet_len:
                            # We might have a full packet
                            packet = self.read_buffer[:total_packet_len]
                            # TODO: Add checksum validation here
                            # Example checksum: simple XOR sum
                            # calculated_checksum = 0
                            # for byte in packet[:-1]: # Exclude checksum byte itself
                            #     calculated_checksum ^= byte
                            # received_checksum = packet[-1]
                            # if calculated_checksum != received_checksum:
                            #     self.get_logger().warn("Checksum mismatch! Discarding packet.")
                            #     self.read_buffer = self.read_buffer[total_packet_len:] # Discard corrupted packet
                            #     continue # Try parsing next potential packet

                            # --- Process valid packet ---
                            data_payload = packet[3:-1] # Extract data payload

                            if msg_type == 1: # IMU Data
                                self.parse_and_publish_imu(data_payload)
                            elif msg_type == 2: # Odometry Data
                                self.parse_and_publish_odom(data_payload)
                            else:
                                self.get_logger().warn(f"Unknown message type: {msg_type}")

                            # Remove processed packet from buffer
                            self.read_buffer = self.read_buffer[total_packet_len:]
                        else:
                            # Not enough data for the full packet yet
                            break # Wait for more data
                    # --- End Example Parsing Logic ---

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                # Attempt to close and reopen?
                if self.serial_conn:
                    self.serial_conn.close()
                self.serial_conn = None # Indicate connection lost
                self.timer.cancel() # Stop timer if serial fails
                self.get_logger().info("Serial connection lost. Stopping timer.")
                # Maybe try reconnecting after a delay?
            except Exception as e:
                self.get_logger().error(f"Error processing serial data: {e}", exc_info=True)
                # Clear buffer to potentially recover from corrupted data
                self.read_buffer = b''

    def parse_and_publish_imu(self, data):
        # --- Placeholder IMU Parsing (Replace with your actual data structure) ---
        # Example: Assume data is 10 floats: qx, qy, qz, qw, gx, gy, gz, ax, ay, az
        # Requires 10 * 4 = 40 bytes
        expected_len = 40
        if len(data) != expected_len:
            self.get_logger().warn(f"IMU data length mismatch. Expected {expected_len}, got {len(data)}")
            return

        try:
            # '<' means little-endian, '10f' means 10 floats
            qx, qy, qz, qw, gx, gy, gz, ax, ay, az = struct.unpack('<10f', data)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link" # Or your actual IMU frame

            # Orientation
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            # Angular Velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            # Linear Acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            # --- Covariance Matrices (PLACEHOLDERS - TUNE THESE) ---
            # Set reasonable default covariances. These should reflect the sensor's noise characteristics.
            # If unknown, start with small values on the diagonal assuming uncorrelated noise.
            # Units: orientation (rad^2), angular_velocity (rad/s)^2, linear_acceleration (m/s^2)^2

            # Example: Low covariance for orientation if well-calibrated, higher for others
            # If orientation is reliable from STM32:
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            # If orientation is NOT reliable or not provided by STM32:
            # imu_msg.orientation_covariance[0] = -1.0

            imu_msg.angular_velocity_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001
            ]
            imu_msg.linear_acceleration_covariance = [
                0.005, 0.0, 0.0,
                0.0, 0.005, 0.0,
                0.0, 0.0, 0.005
            ]

            self.imu_publisher.publish(imu_msg)
            self.get_logger().debug("Published IMU message")

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack IMU data: {e}")
        except Exception as e:
             self.get_logger().error(f"Error in parse_and_publish_imu: {e}", exc_info=True)

    def parse_and_publish_odom(self, data):
        # --- Placeholder Odometry Parsing (Replace with your actual data structure) ---
        # Example: Assume data is 6 floats: x, y, yaw, vx, vy, vyaw (Pose + Twist)
        # Requires 6 * 4 = 24 bytes
        # OR: Assume data is 2 floats: vx, vyaw (Twist only)
        # Requires 2 * 4 = 8 bytes

        # --- Option 1: Parsing Pose + Twist (e.g., 24 bytes) ---
        # expected_len = 24
        # if len(data) != expected_len:
        #     self.get_logger().warn(f"Odom data length mismatch (Pose+Twist). Expected {expected_len}, got {len(data)}")
        #     return
        # try:
        #     x, y, yaw, vx, vy, vyaw = struct.unpack('<6f', data)
        #     publish_pose = True
        # except struct.error as e:
        #     self.get_logger().error(f"Failed to unpack Odom data (Pose+Twist): {e}")
        #     return

        # --- Option 2: Parsing Twist only (e.g., 8 bytes) ---
        expected_len = 8
        if len(data) != expected_len:
            self.get_logger().warn(f"Odom data length mismatch (Twist only). Expected {expected_len}, got {len(data)}")
            return
        try:
            vx, vyaw = struct.unpack('<2f', data)
            x, y, yaw, vy = 0.0, 0.0, 0.0, 0.0 # Set pose and unmeasured twist to zero
            publish_pose = False # Indicate we only have twist
        except struct.error as e:
            self.get_logger().error(f"Failed to unpack Odom data (Twist only): {e}")
            return
        # --- End Options ---

        try:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"       # Standard odom frame
            odom_msg.child_frame_id = "base_link" # Or your robot's base frame

            # --- Fill Pose ---
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0 # Assuming 2D

            q = quaternion_from_euler(0.0, 0.0, yaw)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # --- Fill Twist ---
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy # Often 0 for non-holonomic robots
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = vyaw

            # --- Covariance Matrices (PLACEHOLDERS - TUNE THESE) ---
            P_POSE_DEFAULT = 1e-3
            P_TWIST_DEFAULT = 1e-3
            P_HIGH_VARIANCE = 1e9 # Use a large number for unmeasured/highly uncertain dimensions

            # Set pose covariance based on whether pose was received
            if publish_pose:
                odom_msg.pose.covariance = [
                    P_POSE_DEFAULT, 0.0, 0.0, 0.0, 0.0, 0.0,           # X pos
                    0.0, P_POSE_DEFAULT, 0.0, 0.0, 0.0, 0.0,           # Y pos
                    0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,           # Z pos
                    0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,           # Roll
                    0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,           # Pitch
                    0.0, 0.0, 0.0, 0.0, 0.0, P_POSE_DEFAULT * 10.0    # Yaw
                ]
            else: # Only twist received, set pose covariance very high
                 odom_msg.pose.covariance = [
                    P_HIGH_VARIANCE, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE
                ]

            # Set twist covariance (adjust based on sensor reliability)
            odom_msg.twist.covariance = [
                P_TWIST_DEFAULT, 0.0, 0.0, 0.0, 0.0, 0.0,          # X vel
                0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0, 0.0,          # Y vel (Set high if non-holonomic/not measured)
                0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,          # Z vel
                0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,          # Roll rate
                0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,          # Pitch rate
                0.0, 0.0, 0.0, 0.0, 0.0, P_TWIST_DEFAULT * 10.0   # Yaw rate
            ]

            self.odom_publisher.publish(odom_msg)
            self.get_logger().debug("Published Odometry message")

        except ImportError:
             self.get_logger().error("Failed to import tf_transformations. Install it: sudo apt install python3-tf-transformations-pip")
             # Consider stopping the node if essential library is missing
             rclpy.shutdown()
        except Exception as e:
             self.get_logger().error(f"Error in parse_and_publish_odom: {e}", exc_info=True)

    def simulation_callback(self):
        """Generates and publishes simulated IMU and Odometry data."""
        current_time = self.get_clock().now()
        try:
            dt = (current_time - self.last_sim_time).nanoseconds / 1e9
        except AttributeError: # Handle case where last_sim_time might not be initialized if __init__ fails early
             dt = self.timer_period # Use default period as fallback
        self.last_sim_time = current_time

        # --- Simulate Motion ---
        # Create a simple movement pattern for mapping
        time_in_cycle = (current_time.nanoseconds / 1e9) % 10.0  # 10-second cycle
        
        if time_in_cycle < 5.0:  # First 5 seconds: move forward and rotate
            self.sim_vx = 0.2  # m/s - moderate forward speed
            self.sim_vyaw = 0  # rad/s - gentle rotation
        else:  # Next 5 seconds: rotate in place
            self.sim_vx = -0.2
            self.sim_vyaw = 0  # rad/s - rotate a bit faster in place
        # Add some noise to the actual movement for realism?
        # actual_vx = self.sim_vx + random.gauss(0, 0.01)
        # actual_vyaw = self.sim_vyaw + random.gauss(0, 0.02)

        # Update simulated pose using target velocities
        delta_x = self.sim_vx * math.cos(self.sim_theta) * dt
        delta_y = self.sim_vx * math.sin(self.sim_theta) * dt
        delta_theta = self.sim_vyaw * dt

        self.sim_x += delta_x
        self.sim_y += delta_y
        self.sim_theta += delta_theta
        # Normalize theta to be within [-pi, pi]
        self.sim_theta = math.atan2(math.sin(self.sim_theta), math.cos(self.sim_theta))

        # --- Publish Simulated Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"       # Standard odom frame
        odom_msg.child_frame_id = "base_link" # Or your robot's base frame

        # Position (using integrated pose)
        odom_msg.pose.pose.position.x = self.sim_x
        odom_msg.pose.pose.position.y = self.sim_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (from integrated Yaw)
        try:
            q = quaternion_from_euler(0.0, 0.0, self.sim_theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
        except ImportError:
             self.get_logger().error("Failed to import tf_transformations for odom sim. Install it: sudo apt install python3-tf-transformations-pip")
             return # Skip publishing if library missing

        # Velocity (report the target velocities, maybe add noise here?)
        odom_msg.twist.twist.linear.x = self.sim_vx + random.gauss(0, 0.01)
        odom_msg.twist.twist.linear.y = 0.0 # Non-holonomic
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.sim_vyaw + random.gauss(0, 0.02)

        # Covariances (Reflect uncertainty in the *reported* values)
        P_POSE_DEFAULT = 1e-3
        P_TWIST_DEFAULT = 1e-3
        P_HIGH_VARIANCE = 1e9
        # Pose covariance should be relatively low as we are integrating it directly
        odom_msg.pose.covariance = [
            P_POSE_DEFAULT, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, P_POSE_DEFAULT, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, P_POSE_DEFAULT * 10.0 # Yaw might drift more
        ]
        # Twist covariance reflects noise added to reported twist
        odom_msg.twist.covariance = [
            P_TWIST_DEFAULT * 2, 0.0, 0.0, 0.0, 0.0, 0.0, # Slightly higher due to noise
            0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0, 0.0, # Y vel not applicable
            0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, P_TWIST_DEFAULT * 20.0 # Slightly higher due to noise
        ]
        self.odom_publisher.publish(odom_msg)

        # --- Publish Simulated IMU ---
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link" # Or your actual IMU frame

        # Orientation (Set to identity, EKF should use odom/mag for heading)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0
        imu_msg.orientation_covariance[0] = -1.0 # Indicate orientation is not available from this source

        # Angular Velocity (Use simulated yaw rate + noise)
        imu_msg.angular_velocity.x = 0.0 + random.gauss(0, 0.01)
        imu_msg.angular_velocity.y = 0.0 + random.gauss(0, 0.01)
        imu_msg.angular_velocity.z = self.sim_vyaw + random.gauss(0, 0.02) # Match odom twist noise level?

        # Linear Acceleration (Simulate gravity + noise + potentially centripetal accel)
        # Centripetal acceleration = v^2 / r = v * omega
        centripetal_accel = self.sim_vx * self.sim_vyaw
        imu_msg.linear_acceleration.x = 0.0 + random.gauss(0, 0.05) # Accel in robot's x (forward)
        imu_msg.linear_acceleration.y = centripetal_accel + random.gauss(0, 0.05) # Accel in robot's y (left)
        imu_msg.linear_acceleration.z = 9.81 + random.gauss(0, 0.05) # Gravity

        # Covariances
        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.002 # Slightly higher Z variance due to noise
        ]
        imu_msg.linear_acceleration_covariance = [
            0.005, 0.0, 0.0,
            0.0, 0.005, 0.0, # Y accel has centripetal component + noise
            0.0, 0.0, 0.005
        ]
        self.imu_publisher.publish(imu_msg)
        # self.get_logger().debug("Published simulated IMU and Odometry") # Reduce log spam

    def destroy_node(self):
        # Stop timer if it exists
        if self.timer:
            self.timer.cancel()
            self.get_logger().info("Timer cancelled.")
        # Close serial if it exists and is open
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()
        self.get_logger().info("STM32 Interface node destroyed.")

def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    initialization_successful = False
    try:
        # Attempt to create the node instance
        node = Stm32InterfaceNode()
        # Set logger level to DEBUG for this node
        if node:
            rclpy.logging.set_logger_level(node.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)
            node.get_logger().info(f"Logger level set to DEBUG for {node.get_logger().name}")

        # Check if the timer was successfully created. This happens in both
        # successful hardware connection and simulation mode. If it's None,
        # it implies serial connection failed in hardware mode and __init__ might have exited early.
        if hasattr(node, 'timer') and node.timer is not None:
            initialization_successful = True
            node.get_logger().info("Node initialized successfully. Spinning...")
            rclpy.spin(node)
        else:
            # This case handles when __init__ fails to create the timer (e.g., serial error)
            if node: # Check if node object exists even if timer doesn't
                 node.get_logger().error("Node initialization failed (timer not created). Shutting down.")
            else:
                 print("Failed to create Stm32InterfaceNode object.") # Should not happen if __init__ runs

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        # Log any other exceptions during initialization or spin
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}", exc_info=True)
        else:
            # If node creation itself failed before __init__ finished
            print(f"Failed during node creation or early initialization: {e}")
    finally:
        # Ensure cleanup happens regardless of success or failure
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown complete.")

if __name__ == '__main__':
    main()
