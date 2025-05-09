import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import os # For path manipulation

class MapExporterNode(Node):
    def __init__(self):
        super().__init__('map_exporter_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Map Exporter Node started, subscribing to /map topic...')
        self.map_received = False # Flag to ensure we only save the map once

    def map_callback(self, msg):
        if self.map_received:
            return

        self.get_logger().info('Received map data. Resolution: %f m/pixel' % msg.info.resolution)
        self.get_logger().info('Map width: %d pixels, height: %d pixels' % (msg.info.width, msg.info.height))
        self.get_logger().info('Origin: [x: %f, y: %f, z: %f]' % (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.position.z
        ))

        # Define the path for the TXT file
        # For simplicity, saving in the user's home directory.
        # You might want to change this to a more specific path.
        file_path = os.path.expanduser('/home/reeve/Vacuum_Cleaner_nav/map_data.txt')

        try:
            with open(file_path, 'w') as txtfile:
                self.get_logger().info(f'Writing full map message to {file_path}...')
                txtfile.write(str(msg)) # Write the string representation of the entire message
            
            self.get_logger().info(f'Successfully exported full map message to {file_path}')
            self.map_received = True # Set flag to true after saving
            self.get_logger().info('Map saved. Shutting down node to prevent re-saving.')
            rclpy.shutdown() # Shutdown after saving the map

        except IOError as e:
            self.get_logger().error(f'Failed to write map data to TXT: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')


def main(args=None):
    rclpy.init(args=args)
    map_exporter_node = MapExporterNode()
    try:
        rclpy.spin(map_exporter_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        if rclpy.ok():
            map_exporter_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
