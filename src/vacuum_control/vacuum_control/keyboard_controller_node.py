import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d
   x

w/s : move forward/backward (only while pressed)
a/d : turn left/right (only while pressed)
x   : force stop

NOTE: Robot will stop automatically when key is released
CTRL-C to quit (will be detected as raw input)
"""

moveBindings = {
    'w': (1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (-1, 0),
    'x': (0, 0),  # Force stop
}

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller_node')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.2
        self.turn = 0.5
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.original_settings = None

        try:
            self.original_settings = termios.tcgetattr(sys.stdin)
        except termios.error as e:
            self.get_logger().error(f"Fatal: Failed to get terminal attributes: {e}. Ensure node is run in a TTY.")
            raise RuntimeError(f"Failed to get terminal attributes: {e}") from e
            
        try:
            tty.setraw(sys.stdin.fileno())
        except termios.error as e:
            self.get_logger().error(f"Fatal: Failed to set terminal to raw mode: {e}.")
            # Attempt to restore if getattr succeeded, though it might also fail if stdin is problematic
            if self.original_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            raise RuntimeError(f"Failed to set terminal to raw mode: {e}") from e

        self.get_logger().info(msg)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def _get_key(self):
        if not sys.stdin.readable(): # Check if stdin is readable
             self.get_logger().warn_once("stdin is not readable, cannot get key.")
             return ''
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05) # Reduced timeout slightly
        if rlist:
            try:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl+C character
                    self.get_logger().info("Ctrl+C detected as raw input, initiating shutdown.")
                    rclpy.shutdown()  # Signal rclpy.spin() to stop
                    return None  # Indicate shutdown sequence started
                return key
            except IOError as e: # Handle potential IO errors on read
                self.get_logger().warn(f"IOError reading from stdin: {e}")
                return ''
        else:
            return ''

    def timer_callback(self):
        if not rclpy.ok():
            # If shutdown was initiated (e.g., by Ctrl+C in _get_key),
            # stop trying to process keys or publish messages.
            # The timer will be cancelled eventually by rclpy.spin() exiting.
            return

        key = self._get_key()

        if key is None:  # Shutdown initiated by _get_key (Ctrl+C)
            return

        if key in moveBindings:
            # Set velocities based on pressed key
            self.current_linear_x = float(moveBindings[key][0])
            self.current_angular_z = float(moveBindings[key][1])
        else:
            # Reset velocities when no valid key is pressed or when stop commands received
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            
        twist = Twist()
        twist.linear.x = self.current_linear_x * self.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z * self.turn
        self.publisher_.publish(twist)

    def destroy_node(self):
        self.get_logger().info("Restoring terminal settings before shutdown.")
        if self.original_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            except termios.error as e:
                self.get_logger().error(f"Failed to restore terminal settings: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = KeyboardControllerNode()
        rclpy.spin(node)
    except RuntimeError as e:
        if node and hasattr(node, 'get_logger') and node.get_logger().is_info_enabled(): # Check if logger is usable
             node.get_logger().fatal(f"Node execution failed: {e}")
        else:
            print(f"FATAL: Could not start/run KeyboardControllerNode: {e}", file=sys.stderr)
        exit_code = 1
    except KeyboardInterrupt: # This might not be triggered if Ctrl+C is only read as \x03
        if node and hasattr(node, 'get_logger') and node.get_logger().is_info_enabled():
            node.get_logger().info('KeyboardInterrupt (SIGINT) received, shutting down.')
        else:
            print("KeyboardInterrupt (SIGINT) received, shutting down.", file=sys.stderr)
    # No specific 'except Exception' here to let rclpy handle other unexpected errors if it can.
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok(): # Check if rclpy is still ok (e.g. not shut down by Ctrl+C handler)
            rclpy.shutdown()
    return exit_code

if __name__ == '__main__':
    sys.exit(main())
