class VehicleInfo(object):

    def __init__(self):
        """
        waf_target: option passed to waf's --target to create binary
        default_params_filename: filename of default parameters file.  Taken to be relative to autotest dir.
        extra_mavlink_cmds: extra parameters that will be passed to mavproxy
        """
        self.options = {
    "ArduCopter": {
        "default_frame": "quad",
        "frames": {
            # COPTER
            "+": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "quad": {
                "model": "+",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "X": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
                # this param set FRAME doesn't actually work because mavproxy
                # won't set a parameter unless it knows of it, and the
                # param fetch happens asynchronously
                "extra_mavlink_cmds": "param fetch frame; param set FRAME 1;",
            },
            "bfx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-bfx.parm" ],
            },
            "djix": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-djix.parm" ],
            },
            "cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-cwx.parm" ],
            },
            "hexa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-hexa.parm" ],
            },
            "hexa-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-hexa.parm",
                    "default_params/copter-hexa-cwx.parm"
                ],
            },
            "hexa-dji": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-hexa.parm",
                    "default_params/copter-hexa-dji.parm"
                ],
            },
             "octa-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-octa.parm",
                    "default_params/copter-octa-cwx.parm"
                ],
            },
            "octa-quad-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-octaquad.parm",
                    "default_params/copter-octaquad-cwx.parm"
                ],
            },
            "octa-quad": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-octaquad.parm" ],
            },
            "octa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-octa.parm" ],
            },
            "octa-dji": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-octa.parm",
                    "default_params/copter-octa-dji.parm"
                ],
            },
            "deca": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-deca.parm" ],
            },
            "deca-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-deca.parm",
                    "default_params/copter-deca-cwx.parm"
                 ],
            },
            "tri": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-tri.parm" ],
            },
            "y6": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-y6.parm" ],
            },
            "dodeca-hexa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-dodecahexa.parm" ],
            },
            # SIM
            "IrisRos": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
                "external": True,
            },
            "gazebo-iris": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris.parm"],
                "external": True,
            },
            "airsim-copter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/airsim-quadX.parm"],
                "external": True,
            },
            # HELICOPTER
            "heli": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": "default_params/copter-heli.parm",
            },
            "heli-dual": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                                            "default_params/copter-heli-dual.parm"],
            },
            "heli-blade360": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                ],
            },
            "singlecopter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter-single.parm",
            },
            "coaxcopter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter-single.parm",
                                            "default_params/copter-coax.parm"],
            },
            "scrimmage-copter" : {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
                "external": True,
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
                "external": True,  # lies!  OTOH, hard to take off with this
            },
            "Callisto": {
                "model": "octa-quad:@ROMFS/models/Callisto.json",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "models/Callisto.param"],
            },
        },
    },
    "Helicopter": {
        "default_frame": "heli",
        "frames": {
            "heli": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": "default_params/copter-heli.parm",
            },
            "heli-dual": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                                            "default_params/copter-heli-dual.parm"],
            },
            # "heli-compound": {
            #     "waf_target": "bin/arducopter-heli",
            #     "default_params_filename": ["default_params/copter-heli.parm",
            #                                 "default_params/copter-heli-compound.parm"],
            # },
            "heli-blade360": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                ],
            },
        },
    },
    "Blimp": {
        "default_frame": "Blimp",
        "frames": {
            "Blimp": {
                "waf_target": "bin/blimp",
                "default_params_filename": "default_params/blimp.parm",
            },
        },
    },
    "ArduPlane": {
        "default_frame": "plane",
        "frames": {
            # PLANE
            "quadplane-tilttri": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tilttri.parm",
            },
            "quadplane-tilttrivec": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tilttrivec.parm",
            },
            "quadplane-tilthvec": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/quadplane-tilthvec.parm"],
            },
            "quadplane-tri": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tri.parm",
            },
            "quadplane-cl84" : {
                "waf_target" : "bin/arduplane",
                "default_params_filename": "default_params/quadplane-cl84.parm",
            },
            "quadplane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane.parm",
            },
            "quadplane-ice": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/quadplane.parm", "default_params/plane-ice.parm", "default_params/quadplane-ice.parm"],
            },
            "firefly": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/firefly.parm",
            },
            "plane-elevon": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/plane-elevons.parm"],
            },
            "plane-vtail": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/plane-vtail.parm"],
            },
            "plane-tailsitter": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane-tailsitter.parm",
            },
            "plane-jet": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/plane-jet.parm"],
            },
            "plane-ice": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/plane-ice.parm"],
            },
            "plane-3d": {
                "waf_target": "bin/arduplane",
                "default_params_filename": [], # defaults are loaded in SIM_Plane.cpp
            },
            "quadplane-copter_tailsitter": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/quadplane.parm","default_params/quadplane-copter_tailsitter.parm"],
            },
            "plane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "models/plane.parm",
            },
            "plane-dspoilers": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/plane-dspoilers.parm"]
            },
            "plane-soaring": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["models/plane.parm", "default_params/plane-soaring.parm"]
            },
            "gazebo-zephyr": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/gazebo-zephyr.parm",
                "external": True,
            },
            "last_letter": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "models/plane.parm",
                "external": True,
            },
            "CRRCSim": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "models/plane.parm",
                "external": True,
            },
            "jsbsim": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane-jsbsim.parm",
                "external": True,
            },
            "scrimmage-plane" : {
                "waf_target": "bin/arduplane",
                "default_params_filename": "models/plane.parm",
                "external": True,
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
                "external": True,  # lies!  OTOH, hard to take off with this
            },
        },
    },
    "Rover": {
        "default_frame": "rover",
        "frames": {
            # ROVER
            "rover": {
                "waf_target": "bin/ardurover",
                "default_params_filename": "default_params/rover.parm",
            },
            "rover-skid": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm"],
            },
            "rover-vectored": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-vectored.parm"],
            },
            "balancebot": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm",
                                            "default_params/balancebot.parm"],
            },
            "motorboat": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/motorboat.parm"],
            },
            "sailboat": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/sailboat.parm"],
            },
            "sailboat-motor": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/sailboat-motor.parm"],
            },
            "gazebo-rover": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm"],
            },
            "airsim-rover": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/airsim-rover.parm"],
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
            },
        },
    },
    "ArduSub": {
        "default_frame": "vectored",
        "frames": {
            "vectored": {
                "waf_target": "bin/ardusub",
                "default_params_filename": "default_params/sub.parm",
            },
            "vectored_6dof": {
                "waf_target": "bin/ardusub",
                "default_params_filename": "default_params/sub-6dof.parm",
            },
            "gazebo-bluerov2": {
                "waf_target": "bin/ardusub",
                "default_params_filename": "default_params/sub.parm",
            },
        },
    },
    "AntennaTracker": {
        "default_frame": "tracker",
        "frames": {
            "tracker": {
                "waf_target": "bin/antennatracker",
            },
        },
    },
    "sitl_periph_gps": {
        "frames": {
            "gps": {
                "configure_target": "sitl_periph_gps",
                "waf_target": "bin/AP_Periph",
                },
            }
    },
}


    def default_frame(self, vehicle):
        return self.options[vehicle]["default_frame"]

    def default_waf_target(self, vehicle):
        """Returns a waf target based on vehicle type, which is often determined by which directory the user is in"""
        default_frame = self.default_frame(vehicle)
        return self.options[vehicle]["frames"][default_frame]["waf_target"]

    def options_for_frame(self, frame, vehicle, opts):
        """Return informatiom about how to sitl for frame e.g. build-type==sitl"""
        ret = None
        frames = self.options[vehicle]["frames"]
        if frame in frames:
            ret = self.options[vehicle]["frames"][frame]
        else:
            for p in ["octa", "tri", "y6", "firefly", "heli", "gazebo", "last_letter", "jsbsim", "quadplane", "plane-elevon", "plane-vtail", "plane", "airsim"]:
                if frame.startswith(p):
                    ret = self.options[vehicle]["frames"][p]
                    break
        if ret is None:
            if frame.endswith("-heli"):
                ret = self.options[vehicle]["frames"]["heli"]
        if ret is None:
            print("WARNING: no config for frame (%s)" % frame)
            ret = {}

        if "model" not in ret:
            ret["model"] = frame

        if "sitl-port" not in ret:
            ret["sitl-port"] = True

        if opts.model is not None:
            ret["model"] = opts.model

        if (ret["model"].find("xplane") != -1 or ret["model"].find("flightaxis") != -1):
            ret["sitl-port"] = False


        if "waf_target" not in ret:
            ret["waf_target"] = self.default_waf_target(vehicle)

        if opts.build_target is not None:
            ret["waf_target"] = opts.build_target

        return ret



