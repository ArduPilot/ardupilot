class VehicleInfo(object):

    def __init__(self):
        """
        make_target: option passed to make to create binaries.  Usually sitl, and "-debug" may be appended if -D is passed to sim_vehicle.py
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
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-bfx.parm" ],
            },
            "djix": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-djix.parm" ],
            },
            "cwx": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-cwx.parm" ],
            },
            "hexa": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-hexa.parm" ],
            },
            "hexa-cwx": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "hexa-dji": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
             "octa-cwx": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "octa-quad-cwx": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "octa-quad": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-octaquad.parm" ],
            },
            "octa": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-octa.parm" ],
            },
            "octa-dji": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "tri": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-tri.parm" ],
            },
            "y6": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-y6.parm" ],
            },
            "dodeca-hexa": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-dodecahexa.parm" ],
            },
            # SIM
            "IrisRos": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "gazebo-iris": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris.parm"],
            },
            "airsim-copter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/airsim-quadX.parm"],
            },
            # HELICOPTER
            "heli": {
                "make_target": "sitl-heli",
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": "default_params/copter-heli.parm",
            },
            "heli-dual": {
                "make_target": "sitl-heli-dual",
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                                            "default_params/copter-heli-dual.parm"],
            },
            "heli-compound": {
                "make_target": "sitl-heli-compound",
                "waf_target": "bin/arducopter-heli",
            },
            "singlecopter": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter-single.parm",
            },
            "coaxcopter": {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter-single.parm",
                                            "default_params/copter-coax.parm"],
            },
            "scrimmage-copter" : {
                "make_target": "sitl",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
            },
        },
    },
    "ArduPlane": {
        "default_frame": "plane",
        "frames": {
            # PLANE
            "quadplane-tilttri": {
                "make_target": "sitl",
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tilttri.parm",
            },
            "quadplane-tilttrivec": {
                "make_target": "sitl",
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tilttrivec.parm",
            },
            "quadplane-tilthvec": {
                "make_target": "sitl",
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/quadplane-tilthvec.parm"],
            },
            "quadplane-tri": {
                "make_target": "sitl",
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tri.parm",
            },
            "quadplane-cl84" : {
                "make_target" : "sitl",
                "waf_target" : "bin/arduplane",
                "default_params_filename": "default_params/quadplane-cl84.parm",
            },
            "quadplane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane.parm",
            },
            "firefly": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/firefly.parm",
            },
            "plane-elevon": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-elevons.parm"],
            },
            "plane-vtail": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-vtail.parm"],
            },
            "plane-tailsitter": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane-tailsitter.parm",
            },
            "plane-jet": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-jet.parm"],
            },
            "plane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane.parm",
            },
            "plane-dspoilers": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-dspoilers.parm"]
            },
            "plane-soaring": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-soaring.parm"]
            },
            "gazebo-zephyr": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/gazebo-zephyr.parm",
            },
            "last_letter": {
                "waf_target": "bin/arduplane",
            },
            "CRRCSim": {
                "waf_target": "bin/arduplane",
            },
            "jsbsim": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane-jsbsim.parm",
            },
            "scrimmage-plane" : {
                "make_target": "sitl",
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane.parm",
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
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
            "balancebot": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm",
                                            "default_params/balancebot.parm"],
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

        if "make_target" not in ret:
            ret["make_target"] = "sitl"

        if "waf_target" not in ret:
            ret["waf_target"] = self.default_waf_target(vehicle)

        if opts.build_target is not None:
            ret["make_target"] = opts.build_target
            ret["waf_target"] = opts.build_target

        return ret



