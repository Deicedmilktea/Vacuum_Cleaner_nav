import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time

msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d
   x

w/s : move forward/backward (toggles state)
a/d : turn left/right (toggles state)
x   : force stop (sets speed to zero)

NOTE: Pressing a movement key sets the robot's state.
      Press 'x' to stop. Other keys override the current action.
CTRL-C to quit (will be detected as raw input)
"""

moveBindings = {
    "w": (1, 0),
    "a": (0, 1),
    "d": (0, -1),
    "s": (-1, 0),
    "x": (0, 0),  # Force stop
}


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__("keyboard_controller_node")

        self.publisher_ = self.create_publisher(Twist, "cmd_vel_processed", 10)
        self.speed = 1
        self.turn = 1
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.original_settings = None

        # 按键命令跟踪
        self.current_key = (
            None  # Stores the current active command key, e.g., 'w', 'a', 'x'
        )

        try:
            self.original_settings = termios.tcgetattr(sys.stdin)
        except termios.error as e:
            self.get_logger().error(
                f"Fatal: Failed to get terminal attributes: {e}. Ensure node is run in a TTY."
            )
            raise RuntimeError(f"Failed to get terminal attributes: {e}") from e

        try:
            tty.setraw(sys.stdin.fileno())
        except termios.error as e:
            self.get_logger().error(f"Fatal: Failed to set terminal to raw mode: {e}.")
            if self.original_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            raise RuntimeError(f"Failed to set terminal to raw mode: {e}") from e

        self.get_logger().info(msg)
        self.timer = self.create_timer(
            0.05, self.timer_callback
        )  # 降低到20Hz，减少网络负载

    def _get_key_improved(self):
        """改进的按键获取方法"""
        if not sys.stdin.readable():
            self.get_logger().warn_once("stdin is not readable, cannot get key.")
            return None

        # 使用更长的超时时间，适应SSH环境
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)

        keys_read = []
        if rlist:
            try:
                # 一次性读取所有可用的字符
                while True:
                    # 再次检查是否有数据可读，避免阻塞
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
                    if not rlist:
                        break

                    char = sys.stdin.read(1)
                    if char == "\x03":  # Ctrl+C
                        self.get_logger().info("Ctrl+C detected, initiating shutdown.")
                        rclpy.shutdown()
                        return None

                    keys_read.append(char)

                    # 限制一次读取的字符数，防止无限循环
                    if len(keys_read) > 10:
                        break

            except IOError as e:
                self.get_logger().warn(f"IOError reading from stdin: {e}")
                return None

        return keys_read

    def timer_callback(self):
        if not rclpy.ok():
            return

        keys_pressed_in_interval = self._get_key_improved()

        if keys_pressed_in_interval is None:  # Shutdown initiated
            return

        # Process new key presses from this interval
        if keys_pressed_in_interval:
            # Take the last valid movement key pressed in this interval
            # to determine the new command.
            new_command_key = None
            for key_char in reversed(keys_pressed_in_interval):
                if key_char in moveBindings:
                    new_command_key = key_char
                    break  # Found the most recent command key from this batch

            if new_command_key:
                if self.current_key != new_command_key:
                    self.get_logger().info(
                        f"Command changed from '{self.current_key}' to '{new_command_key}'"
                    )
                self.current_key = new_command_key  # Update the persistent command

        # Set speeds based on the persistent current_key
        # (which might be None initially, or 'x' for stop, or a movement key)
        if self.current_key and self.current_key in moveBindings:
            self.current_linear_x = float(moveBindings[self.current_key][0])
            self.current_angular_z = float(moveBindings[self.current_key][1])
        else:
            # Initial state (current_key is None)
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0

        # 发布Twist消息
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
        if node and hasattr(node, "get_logger") and node.get_logger().is_info_enabled():
            node.get_logger().fatal(f"Node execution failed: {e}")
        else:
            print(
                f"FATAL: Could not start/run KeyboardControllerNode: {e}",
                file=sys.stderr,
            )
        exit_code = 1
    except KeyboardInterrupt:
        if node and hasattr(node, "get_logger") and node.get_logger().is_info_enabled():
            node.get_logger().info(
                "KeyboardInterrupt (SIGINT) received, shutting down."
            )
        else:
            print(
                "KeyboardInterrupt (SIGINT) received, shutting down.", file=sys.stderr
            )
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
