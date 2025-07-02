import bluerobotics_navigator as navigator
import sensordepth

class sensors:
  def __init__(self):

    self.initialize()
    navigator.init()

  def initialize(self):
    self.depthsensor = sensordepth.ms5837.MS5837_30BA()
    if not self.depthsensor.init():
      print("Depth sensor not initialized")
      exit(1)

  def read(self):
    self.readdepth = self.depthsensor.pressure()
