from ForceSensor import ForceSensor
import numpy as np

# The only calibration required is an offset calibration that can be done 
# externally by gathering data while the sensor is steady and no change in 
# the dynamic state is happening. After the data is averaged, it can be 
# subtracted from the current measurements.

force_calibration_value = np.array([0,0,0])
force_sensor = ForceSensor("/dev/ttyUSB0", force_calibration_value)
force_sensor.force_sensor_setup()

while True:
    # get force reading from the force sensor
    force = force_sensor.get_force_obs()
    print(force)
