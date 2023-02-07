# AP_FLAKE8_CLEAN


class VehicleType:
    Plane = 17
    Copter = 23
    Rover = 37


# these should really be "Plane", "Copter" and "Rover", but many
# things use these values as triggers in their code:
VehicleTypeString = {17: "ArduPlane", 23: "ArduCopter", 37: "ArduRover"}
