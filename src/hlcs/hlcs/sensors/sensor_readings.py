from dataclasses import dataclass


@dataclass
# Based on RaspberrySensorsInterface.msg
class SensorReadings:
    barometer: float
    gyroscopex: float
    gyroscopey: float
    gyroscopez: float
    leakdetection: bool
    temperature: float
    accelerometerx: float
    accelerometery: float
    accelerometerz: float
    magnetometerx: float
    magnetometery: float
    magnetometerz: float
    depthsensor: float
