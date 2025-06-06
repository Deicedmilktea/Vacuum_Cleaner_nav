import rclpy
from rclpy.node import Node
import socket
import threading
import re
import json
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

class WifiNode(Node):
    def __init__(self):
        super().__init__('wifi_node')  # Initialize the ROS 2 node with the name 'wifi_node'
        self.get_logger().info('Wifi node started')  # Log that the node has started
        
        # Declare and retrieve parameters for the TCP server's IP and port
        self.declare_parameter('tcp_server_ip', '0.0.0.0')  # Default IP: 0.0.0.0 (bind to all interfaces)
        self.declare_parameter('tcp_server_port', 8080)  # Default port: 8080
        self.declare_parameter('map_send_interval', 1.0)  # Default: send map every 5 seconds
        
        self.server_ip = self.get_parameter('tcp_server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('tcp_server_port').get_parameter_value().integer_value
        self.map_send_interval = self.get_parameter('map_send_interval').get_parameter_value().double_value
        
        # Initialize ROS2 publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_processed', 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Initialize server and client socket variables
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.latest_map = None
        
        # Create timer for periodic map sending
        self.map_timer = self.create_timer(self.map_send_interval, self.send_map_to_client)
        
        # Start the server in a separate thread
        self.thread = threading.Thread(target=self.start_server)
        self.thread.daemon = True  # Ensure the thread exits when the main program exits
        self.thread.start()

    def start_server(self):
        # Create a TCP server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of the address
        try:
            # Bind the server socket to the specified IP and port
            self.server_socket.bind((self.server_ip, self.server_port))
            self.server_socket.listen(1)  # Listen for incoming connections (1 client at a time)
            self.get_logger().info(f'TCP Server listening on {self.server_ip}:{self.server_port}')
            
            while rclpy.ok():  # Keep running while ROS 2 is active
                try:
                    # Accept a new client connection
                    self.client_socket, self.client_address = self.server_socket.accept()
                    self.get_logger().info(f'Accepted connection from {self.client_address}')
                    
                    # Handle client communication
                    self.handle_client()
                    
                except socket.error as e:
                    # Log socket errors if the node is still running
                    if rclpy.ok():
                        self.get_logger().error(f'Socket error: {e}')
                    break  # Exit the loop on socket error
                except Exception as e:
                    # Log other exceptions
                    if rclpy.ok():
                        self.get_logger().error(f'Error accepting connection: {e}')
                    break
        except Exception as e:
            # Log errors during server startup
            self.get_logger().error(f'Could not start server: {e}')
        finally:
            # Ensure the server socket is closed when the server stops
            if self.server_socket:
                self.server_socket.close()
            self.get_logger().info('TCP Server stopped.')

    def handle_client(self):
        try:
            while rclpy.ok() and self.client_socket:  # Keep handling client while ROS 2 is active
                # Receive data from the client
                data = self.client_socket.recv(1024)
                if not data:  # If no data is received, the client has disconnected
                    self.get_logger().info(f'Client {self.client_address} disconnected.')
                    break
                
                # Decode and log the received message
                message = data.decode().strip()
                self.get_logger().info(f'Received from {self.client_address}: {message}')
                
                # Parse velocity command in format "v=linear,angular"
                self.parse_and_publish_velocity(message)
                
                # # Send acknowledgment back to the client
                # ack_message = "OK\n"
                # self.client_socket.sendall(ack_message.encode())
                
        except socket.error as e:
            # Log socket errors during client communication
            if rclpy.ok():
                self.get_logger().error(f'Socket error during client communication: {e}')
        except Exception as e:
            # Log other exceptions during client handling
            if rclpy.ok():
                self.get_logger().error(f'Error handling client {self.client_address}: {e}')
        finally:
            # Ensure the client socket is closed when done
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            self.get_logger().info(f'Connection with {self.client_address} closed.')
            self.client_address = None

    def parse_and_publish_velocity(self, message):
        """Parse velocity command in format 'v=linear,angular' and publish as Twist message"""
        try:
            # Use regex to match the pattern v=number,number
            pattern = r'v=([+-]?\d*\.?\d+),([+-]?\d*\.?\d+)'
            match = re.match(pattern, message)
            
            if match:
                linear_vel = float(match.group(1))
                angular_vel = float(match.group(2))
                
                # Create and publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_vel
                twist_msg.angular.z = angular_vel
                
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info(f'Published velocity: linear={linear_vel}, angular={angular_vel}')
            else:
                self.get_logger().warn(f'Invalid velocity format: {message}. Expected format: v=linear,angular')
                
        except ValueError as e:
            self.get_logger().error(f'Error parsing velocity values: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing velocity command: {e}')
    
    def map_callback(self, msg):
        """Callback function for /map topic subscription"""
        self.latest_map = msg
        self.get_logger().debug('Received map update')
    
    def send_map_to_client(self):
        """Send the latest map data to connected client"""
        if self.client_socket and self.latest_map:
            try:
                # Convert map data to a simplified format for transmission
                map_data = {
                    'type': 'map',
                    'width': self.latest_map.info.width,
                    'height': self.latest_map.info.height,
                    'resolution': self.latest_map.info.resolution,
                    'origin': {
                        'x': self.latest_map.info.origin.position.x,
                        'y': self.latest_map.info.origin.position.y,
                        'z': self.latest_map.info.origin.position.z
                    },
                    'data': list(self.latest_map.data)  # Convert to list for JSON serialization
                }
                
                # Serialize to JSON and send
                json_data = json.dumps(map_data)
                message = f"MAP:{json_data}\n"
                
                self.client_socket.sendall(message.encode())
                self.get_logger().info(f'Sent map data to client: {self.latest_map.info.width}x{self.latest_map.info.height}')
                
            except socket.error as e:
                self.get_logger().error(f'Error sending map to client: {e}')
            except Exception as e:
                self.get_logger().error(f'Error preparing map data: {e}')
        elif not self.client_socket:
            self.get_logger().debug('No client connected, skipping map send')
        elif not self.latest_map:
            self.get_logger().debug('No map data available, skipping map send')

    def destroy_node(self):
        # Gracefully shut down the TCP server
        self.get_logger().info("Shutting down TCP server...")
        if self.client_socket:
            try:
                # Inform the client about the shutdown if possible
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except socket.error as e:
                self.get_logger().warn(f"Error closing client socket: {e}")
        if self.server_socket:
            try:
                # Close the server socket to stop accepting new connections
                self.server_socket.close()
            except socket.error as e:
                self.get_logger().warn(f"Error closing server socket: {e}")
        if self.thread.is_alive():
            # Wait for the server thread to finish
            self.thread.join(timeout=1.0)
        super().destroy_node()  # Call the parent class's destroy_node method
        self.get_logger().info("Wifi node shutdown complete.")

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    wifi_node = WifiNode()  # Create an instance of the WifiNode
    try:
        rclpy.spin(wifi_node)  # Keep the node running
    except KeyboardInterrupt:
        wifi_node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        wifi_node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shut down the ROS 2 client library

if __name__ == '__main__':
    main()  # Run the main function
