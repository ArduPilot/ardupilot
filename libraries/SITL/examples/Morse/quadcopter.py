'''
This is an example builder script that sets up a quadcopter in Morse to
be driven by ArduPilot.

The quadcopter has the basic set of sensors that ArduPilot needs

To start the simulation use this:

  morse run quadcopter.py

Then connect with ArduPilot like this:

  sim_vehicle.py --model morse --console --map

This model assumes an X frame quadcopter, so you will need:

  FRAME_CLASS 1
  FRAME_TYPE 1
'''
from morse.builder import *

# try to import bpy (no need to install the package as it is used by morse.builder)
try:
    import bpy
except ImportError:
    print("Import Error Detected")

# use the ATRV rover
vehicle = Quadrotor()
vehicle.properties(Object = True, Graspable = False, Label = "Vehicle")
vehicle.translate(x=0.0, z=1.0)

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

# make the vehicle controllable via force and torque
# this will be available on port 4000 by default
engines = QuadrotorDynamicControl()
vehicle.append(engines)
engines.add_stream('socket')

# this would allow us to control the vehicle with a keyboard
# we don't enable it as it causes issues with sensor consistency
#keyboard = Keyboard()
#keyboard.properties(Speed=3.0)
#vehicle.append(keyboard)

# Environment. Run in fast mode which gives wire-frame view, but lowers
# CPU load a lot
env = Environment('land-1/trees', fastmode=False)

# Keep camera position as default
env.set_camera_location([0.0, 0.0, 0.0])
env.set_camera_rotation([0.0, 0.0, 0.0])
env.select_display_camera(camera)
env.set_camera_clip(clip_end=1000)

# startup at CMAC. A location is needed for the magnetometer
env.properties(longitude = 149.165230, latitude = -35.363261, altitude = 584.0)
env.create()

#adding 3rd person camera control(translational units : m , rotational units : radians)
cam_transform={"trans":{"X":-1, "Y":0, "Z":+0.1}, "rot":{"X":0.9, "Y":0, "Z":-1.57}}
if bpy:
    objs=bpy.data.objects
    cam=bpy.data.objects['CameraFP']
    rob=bpy.data.objects['vehicle']
    cam.parent=rob
    robLoc=rob.location
    cam.location = (robLoc[0]+cam_transform['trans']['X'], robLoc[1]+cam_transform['trans']['Y'], robLoc[2]+cam_transform['trans']['Z'])
    cam.rotation_euler = (cam_transform['rot']['X'], cam_transform['rot']['Y'], cam_transform['rot']['Z'])
