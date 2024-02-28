'''
This is an example builder script that sets up a a set of rovers to
be driven by ArduPilot for demonstrating follow mode

The rover has the basic set of sensors that ArduPilot needs

To start the simulation use this:

  morse run rover_follow.py
'''
from morse.builder import *

num_vehicles = 3

for i in range(num_vehicles):
    vehicle = ATRV('Vehicle%u' % i)
    vehicle.properties(Object = True, Graspable = False, Label = "Vehicle")
    # set rovers 3 meters apart
    vehicle.translate(x=0.0, y=3*i, z=0.0)

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

    # make the vehicle controllable with speed and angular velocity
    motion = MotionVW()
    vehicle.append(motion)
    motion.add_stream('socket')

# Environment
env = Environment('land-1/trees', fastmode=False)
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.0470, 0, 0.7854])
env.set_camera_clip(clip_end=1000)

# startup at CMAC. A location is needed for the magnetometer
env.properties(longitude = 149.165230, latitude = -35.363261, altitude = 584.0)
