#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import serial
import time
import struct  # Assuming binary data format

class Stm32InterfaceNode(Node):
    def __init__(self):
        super().__init__('stm32_interface_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/wheel_odom', 10)

        # Serial port setup
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0  # Read timeout in seconds
            )
            self.get_logger().info(f"Successfully connected to serial port: {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            # Consider exiting or retrying
            rclpy.shutdown()
            return # Stop initialization if serial fails

        # Timer for reading serial data
        self.timer_period = 0.01  # seconds (adjust based on expected data rate)
        self.timer = self.create_timer(self.timer_period, self.serial_read_callback)

        # Placeholder for incomplete data buffer
        self.read_buffer = b''

        self.get_logger().info("STM32 Interface Node started.")

    def serial_read_callback(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Read available data
                if self.serial_conn.in_waiting > 0:
                    data_bytes = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.read_buffer += data_bytes
                    self.get_logger().debug(f"Read {len(data_bytes)} bytes. Buffer size: {len(self.read_buffer)}")

                    # --- Protocol Parsing Placeholder ---
                    # This is where you need to implement the logic to parse your specific
                    # custom serial protocol from STM32.
                    # Example: Look for start/end bytes, checksums, message IDs, etc.
                    # You might need to handle incomplete messages arriving across multiple reads.

                    # --- Example Parsing Logic (Replace with your actual protocol) ---
                    # Let's assume a simple protocol:
                    # Start Byte (0xA5) | Type (1=IMU, 2=Odom) | Length | Data | Checksum
                    HEADER = b'\xA5'
                    MIN_PACKET_LEN = 4 # Header + Type + Length + Checksum (at least)

                    while len(self.read_buffer) >= MIN_PACKET_LEN:
                        start_index = self.read_buffer.find(HEADER)
                        if start_index == -1:
                            # No header found, discard buffer (or keep last few bytes?)
                            self.get_logger().warn("No header found, discarding buffer.")
                            self.read_buffer = b''
                            break

                        # Move buffer start to the header
                        if start_index > 0:
                            self.get_logger().warn(f"Discarding {start_index} bytes before header.")
                            self.read_buffer = self.read_buffer[start_index:]

                        # Check if we have enough data for header + type + length
                        if len(self.read_buffer) < 3:
                            break # Need more data

                        msg_type = self.read_buffer[1]
                        msg_len = self.read_buffer[2] # Length of the 'Data' part
                        total_packet_len = 1 + 1 + 1 + msg_len + 1 # Header + Type + Length + Data + Checksum

                        if len(self.read_buffer) >= total_packet_len:
                            # We might have a full packet
                            packet = self.read_buffer[:total_packet_len]
                            # TODO: Add checksum validation here

                            if msg_type == 1: # IMU Data
                                self.parse_and_publish_imu(packet[3:-1]) # Pass only the data part
                            elif msg_type == 2: # Odometry Data
                                self.parse_and_publish_odom(packet[3:-1]) # Pass only the data part
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
                # Maybe try reconnecting after a delay?
            except Exception as e:
                self.get_logger().error(f"Error processing serial data: {e}")
                # Clear buffer to potentially recover from corrupted data
                self.read_buffer = b''


    def parse_and_publish_imu(self, data):
        # --- Placeholder IMU Parsing (Replace with your actual data structure) ---
        # Example: Assume data is 6 floats: qx, qy, qz, qw, gx, gy, gz, ax, ay, az
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
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            # Mark orientation as unknown if not provided or unreliable
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
             self.get_logger().error(f"Error in parse_and_publish_imu: {e}")


    def parse_and_publish_odom(self, data):
        # --- Placeholder Odometry Parsing (Replace with your actual data structure) ---
        # Example: Assume data is 6 floats: x, y, yaw, vx, vy, vyaw
        # Requires 6 * 4 = 24 bytes
        expected_len = 24
        if len(data) != expected_len:
            self.get_logger().warn(f"Odom data length mismatch. Expected {expected_len}, got {len(data)}")
            return

        try:
            # '<' means little-endian, '6f' means 6 floats
            x, y, yaw, vx, vy, vyaw = struct.unpack('<6f', data)

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"       # Standard odom frame
            odom_msg.child_frame_id = "base_link" # Or your robot's base frame

            # Position
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0 # Assuming 2D

            # Orientation (from Yaw) - Convert Yaw to Quaternion
            # Note: This assumes yaw is the only rotation.
            # If STM32 provides quaternion directly, use that.
            q = quaternion_from_euler(0.0, 0.0, yaw)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Velocity
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy # Often 0 for non-holonomic robots
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = vyaw

            # --- Covariance Matrices (PLACEHOLDERS - TUNE THESE) ---
            # Odometry covariance reflects uncertainty in the motion estimate.
            # Diagonal elements represent variance, off-diagonal represent covariance.
            # Units: pose (m^2 for position, rad^2 for orientation), twist (m/s)^2, (rad/s)^2

            # Example: Higher uncertainty in Y and Yaw for differential drive
            # Set high variance for dimensions not measured (e.g., Z pos, Roll/Pitch orientation, Z/X/Y angular vel)
            P_POSE_DEFAULT = 1e-3 # Default small variance for well-measured values
            P_TWIST_DEFAULT = 1e-3
            P_HIGH_VARIANCE = 1e9 # Use a large number for unmeasured/highly uncertain dimensions

            odom_msg.pose.covariance = [
                P_POSE_DEFAULT, 0.0, 0.0, 0.0, 0.0, 0.0,           # X pos
                0.0, P_POSE_DEFAULT, 0.0, 0.0, 0.0, 0.0,           # Y pos
                0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,           # Z pos (unmeasured)
                0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,           # Roll (unmeasured)
                0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,           # Pitch (unmeasured)
                0.0, 0.0, 0.0, 0.0, 0.0, P_POSE_DEFAULT * 10.0    # Yaw (often less certain than position)
            ]
            odom_msg.twist.covariance = [
                P_TWIST_DEFAULT, 0.0, 0.0, 0.0, 0.0, 0.0,          # X vel
                0.0, P_TWIST_DEFAULT, 0.0, 0.0, 0.0, 0.0,          # Y vel (might be higher if measured, or P_HIGH_VARIANCE if not)
                0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0, 0.0,          # Z vel (unmeasured)
                0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0, 0.0,          # Roll rate (unmeasured)
                0.0, 0.0, 0.0, 0.0, P_HIGH_VARIANCE, 0.0,          # Pitch rate (unmeasured)
                0.0, 0.0, 0.0, 0.0, 0.0, P_TWIST_DEFAULT * 10.0   # Yaw rate (often less certain)
            ]


            self.odom_publisher.publish(odom_msg)
            self.get_logger().debug("Published Odometry message")

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack Odom data: {e}")
        except ImportError:
             self.get_logger().error("Failed to import tf_transformations. Install it: sudo apt install python3-tf-transformations-pip")
        except Exception as e:
             self.get_logger().error(f"Error in parse_and_publish_odom: {e}")

    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32InterfaceNode()
    # Only spin if serial port was opened successfully
    if node.serial_conn:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received, shutting down...")
        finally:
            # Explicitly destroy the node upon exit
            node.destroy_node()
            rclpy.shutdown()
    else:
         node.get_logger().error("Node initialization failed due to serial port error. Shutting down.")
         # Ensure shutdown even if spin wasn't called
         if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
