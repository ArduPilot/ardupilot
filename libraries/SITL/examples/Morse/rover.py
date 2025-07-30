'''
This is an example builder script that sets up a rover in Morse to
be driven by ArduPilot.

The rover has the basic set of sensors that ArduPilot needs

To start the simulation use this:

  morse run rover.py

Then connect with ArduPilot like this:

  sim_vehicle.py --model morse --console --map

This model assumes you will setup a steering/throttle rover 

  SERVO1_FUNCTION 26
  SERVO3_FUNCTION 70
'''

# flake8: noqa

from morse.builder import *

# use the Hummer
vehicle = Hummer()
vehicle.properties(Object = True, Graspable = False, Label = "Vehicle")
vehicle.translate(x=0.0, z=0.0)

# add a camera
camera = SemanticCamera(name="Camera")
camera.translate(x=0.2, y=0.3, z=0.9)
vehicle.append(camera)
camera.properties(cam_far=800)
camera.properties(Vertical_Flip=True)

# we could optionally stream the video to a port
#camera.add_stream('socket')

# add sensors needed for ArduPilot operation to a vehicle
pose = Pose()
vehicle.append(pose)

imu = IMU()
vehicle.append(imu)

gps = GPS()
gps.alter('UTM')
vehicle.append(gps)

velocity = Velocity()
vehicle.append(velocity)

# create a compound sensor of all of the individual sensors and stream it
all_sensors = CompoundSensor([imu, gps, velocity, pose])
all_sensors.add_stream('socket')

vehicle.append(all_sensors)

# make the vehicle controllable with steer and force 
# this will be available on port 60001 by default
motion = SteerForce()
vehicle.append(motion)
motion.add_stream('socket')

# this would allow us to control the vehicle with a keyboard
# we don't enable it as it causes issues with sensor consistency
#keyboard = Keyboard()
#keyboard.properties(Speed=3.0)
#vehicle.append(keyboard)

# Environment
env = Environment('land-1/trees')
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.0470, 0, 0.7854])
env.select_display_camera(camera)
env.set_camera_clip(clip_end=1000)

# startup at CMAC. A location is needed for the magnetometer
env.properties(longitude = 149.165230, latitude = -35.363261, altitude = 584.0)