import bluerobotics_navigator as navigator
import sensordepth
from UVEEC_custom_interfaces.msg import raspberry_sensors_interface


class SensorManager:
  def __init__(self):

    self.initialize()
    navigator.init()

  def initialize(self):
    self.depthsensor = sensordepth.ms5837.MS5837_30BA()
    if not self.depthsensor.init():
      print("Depth sensor not initialized")
      exit(1)

  def getSensorReadingsMsg(self) -> raspberry_sensors_interface:
     # TODO: Add actual sensor reading instead of static values
    msg = raspberry_sensors_interface()
    msg.barometer=1013.25
    msg.gyroscopex=0.1
    msg.gyroscopey=-0.2
    msg.gyroscopez=0.05
    msg.leakdetection=False
    msg.temperature=21.5
    msg.accelerometerx=0.01
    msg.accelerometery=0.98
    msg.accelerometerz=-0.05
    msg.magnetometerx=0.3
    msg.magnetometery=-0.1
    msg.magnetometerz=0.5
    msg.depthsensor=self.depthsensor.pressure()
    return msg
