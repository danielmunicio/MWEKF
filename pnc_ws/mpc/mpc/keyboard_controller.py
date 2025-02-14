import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import termios
import tty
import sys
import select


class WASDPublisher(Node):
    def __init__(self):
        super().__init__("wasd_publisher")
        self.publisher = self.create_publisher(AckermannDriveStamped, "/cmd", 10)

        # Ackermann message initialization
        self.speed = 0.0
        self.steering_angle = 0.0
        self.speed_increment = 0.5
        self.steering_increment = 0.1

        self.get_logger().info("WASD Publisher Node Initialized. Use WASD keys to control.")

    def get_key(self):
        """Reads a keypress without blocking."""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
            else:
                key = None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_ackermann_msg(self):
        """Publishes the current speed and steering angle as an AckermannDriveStamped message."""
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.publisher.publish(msg)

    def run(self):
        """Main loop to process WASD inputs and publish messages."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == "w":
                    self.speed += self.speed_increment
                elif key == "s":
                    self.speed -= self.speed_increment
                elif key == "a":
                    self.steering_angle += self.steering_increment
                elif key == "d":
                    self.steering_angle -= self.steering_increment
                elif key == "q":
                    self.get_logger().info("Exiting WASD control.")
                    break
                elif key == 'b':
                    rclpy.shutdown()
                elif key is None:
                    # No key press; skip publishing
                    continue

                # Log current state
                self.get_logger().info(f"Speed: {self.speed}, Steering Angle: {self.steering_angle}")

                # Publish message
                self.publish_ackermann_msg()

        except KeyboardInterrupt:
            self.get_logger().info("Keyboard Interrupt. Shutting down WASD Publisher.")
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WASDPublisher()
    node.run()


if __name__ == "__main__":
    main()
