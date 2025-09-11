import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, String, Bool, UInt16
from uveec_custom_interfaces.msg import RaspberrySensorsInterface, StmSensorsInterface

from . import sensors, sensordepth

RIGHT_PITCH_MM = 0   # TOF >= this, then pitch right end
LEFT_PITCH_MM = 0   # TOF <= this, then pitch is left end

# Roll constraints
ROLL_LIMIT_POS = +80   # max right
ROLL_LIMIT_NEG = -80   # max left
ROLL_STEP = 40    # FG encoder resolution (degrees per pulse)

# Topics
TOPIC_PITCH_EN = '/pitch_enable'
TOPIC_PITCH_DIR = '/pitch_direction' # Int32: 1=right, 0=idle, -1=left
TOPIC_ROLL_TARGET = '/roll_target_deg'  # Int32: absolute roll target (degrees, snapped to 40°)
TOPIC_ROLL_ENCODER = '/roll_encoder' # Int32 from STM32 (degrees, multiples of 40)
TOPIC_TOF_MM = '/tof_mm' # UInt16 from STM32 (mm)

# Directions
RIGHT, IDLE, LEFT = 1, 0, -1

class MinimalSubscriber(Node):

    def __init__(self):
        self.Sensors = sensors.SensorManager()

        super().__init__('minimal_subscriber')
        # subscribe
        self.subscription = self.create_subscription(Int32, 'cubemx_publisher', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        # publish
        self.publisher = self.create_publisher(RaspberrySensorsInterface, 'raspberry_sensors_publisher', 10)
        timer_period = 5.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)

    def timer_callback(self):
        msg = self.Sensors.getSensorReadingsMsg()
        self.publisher.publish(msg)
        self.get_logger().info('Publishing raspberry_sensors_interface message %s' % msg)

class Demo(Node):
    def __init__(self):

        super().__init__('demo')

        # Publishers to STM32 topics (LLCS)
        self.pub_pitch_enable = self.create_publisher(Bool, TOPIC_PITCH_EN, 10)
        self.pub_pitch_direction = self.create_publisher(Int32, TOPIC_PITCH_DIR, 10)
        self.pub_roll_target = self.create_publisher(Int32, TOPIC_ROLL_TARGET, 10)

        # Subscribers from STM32 (LLCS)
        self.sub_tof = self.create_subscription(UInt16, TOPIC_TOF_MM, self.tof_mm, 10)
        self.sub_roll = self.create_subscription(Int32, TOPIC_ROLL_ENCODER, self.on_roll, 10)

        # State
        self.tof_mm_value: int | None = None
        self.rol_deg_value: int | None = None
        self.state = 'GO_RIGHT' 

        # Enable actuators initially
        self.publish_bool(self.pub_pitch_enable, True)

        # Loop
        self.timer = self.create_timer(1.0 / 2.0, self.tick_callback)
        self.get_logger().info('HLCS demo running: ')

    # Publisher
    def publish_bool(self, pub, val: bool):
        msg = Bool()
        msg.data = val
        pub.publish(msg)
    
    def publish_int(self, pub, val: int):
        msg = Int32()
        msg.data = val
        pub.publish(msg)

    # Snap angle to nearest 40° step within + or - 80°
    def snap_to_step(self, angle: int) -> int:
        snapped = round(angle / ROLL_STEP) * ROLL_STEP
        return max(ROLL_LIMIT_NEG, min(ROLL_LIMIT_POS, snapped))

    # TOF 
    def tof_mm(self, msg: UInt16):
        self.tof_mm_value = int(msg.data)

    # Roll
    def on_roll(self, msg: Int32):
        self.roll_encoder_value = int(msg.data)
    
    def send_roll_target(self, target_deg: int):
        snapped = self.snap_to_step(target_deg)
        self.publish_int(self.pub_roll_target, snapped)
        self.get_logger().info(f'Roll target requested: {target_deg}°, snapped to {snapped}°.')

    # Loop
    def tick_callback(self):
        # If no TOF yet, keep the current pitch motion alive
        if self.tof_mm_value is None:
            if self.state == 'GO_RIGHT':
                self.publish_int(self.pub_pitch_direction, RIGHT)
            elif self.state == 'GO_LEFT':
                self.publish_int(self.pub_pitch_direction, LEFT)
            return

        if self.state == 'GO_RIGHT':
            self.publish_int(self.pub_pitch_direction, RIGHT)
            if self.tof_mm_value >= RIGHT_PITCH_MM:
                # Stop pitch, make a roll move to the RIGHT (positive delta), then go LEFT
                self.publish_int(self.pub_pitch_direction, IDLE)
                self.get_logger().info(f'Right end reached (TOF={self.tof_mm_value} mm).')

                # At right end, then roll further right by one step
                if self.roll_deg_value is not None:
                    self.send_roll_target(self.roll_deg_value + ROLL_STEP)

                self.state = 'GO_LEFT'
                self.get_logger().info('Transition: GO_LEFT')

        elif self.state == 'GO_LEFT':
            self.publish_int(self.pub_pitch_direction, LEFT)
            if self.tof_mm_value <= LEFT_PITCH_MM:
                # Stop pitch, make a roll move to the LEFT (negative delta), then go RIGHT
                self.publish_int(self.pub_pitch_direction, IDLE)
                self.get_logger().info(f'Left end reached (TOF={self.tof_mm_value} mm).')

                # At left end, then roll further left by one step
                if self.roll_deg_value is not None:
                    self.send_roll_target(self.roll_deg_value - ROLL_STEP)

                self.state = 'GO_RIGHT'
                self.get_logger().info('Transition: GO_RIGHT')

def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)
        rclpy.spin(Demo())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
