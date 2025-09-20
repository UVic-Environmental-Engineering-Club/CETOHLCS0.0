import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, String, Bool, UInt16
from uveec_custom_interfaces.msg import RaspberrySensorsInterface, StmSensorsInterface

from . import sensors, sensordepth

class MinimalSubscriber(Node):

    def __init__(self):
        self.Sensors = sensors.SensorManager()

        super().__init__('minimal_subscriber')
        # subscribe
        self.subscription = self.create_subscription(StmSensorsInterface, 'stm_sensors_topic', self.listener_callback, 10)

        # publish
        self.publisher = self.create_publisher(RaspberrySensorsInterface, 'raspberry_sensors_topic', 10)
        timer_period = 5.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        # msg = StmSensorsInterface()
        self.get_logger().info('I heard: "%d"' % msg.gpslatitude)

    def timer_callback(self):
        msg = self.Sensors.getSensorReadingsMsg()
        self.publisher.publish(msg)
        self.get_logger().info('Publishing raspberry_sensors_interface message %s' % msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
