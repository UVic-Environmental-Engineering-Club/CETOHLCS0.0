import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # subscribe
        self.subscription = self.create_subscription(Int32, 'cubemx_publisher', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # publish
        self.publisher = self.create_publisher(Int32, 'raspberry_publisher', 10)
        timer_period = 5.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
