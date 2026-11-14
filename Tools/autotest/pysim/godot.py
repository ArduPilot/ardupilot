# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Run the Godot and Jolt physics backend in libraries/SITL_Godot,
which provides a vehicle's physics to SITL through SITL's JSON model.

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import atexit
import os
import shlex
import socket
import shutil
import subprocess
import sys
import time

from pysim import util

PROJECT = util.reltopdir("libraries/SITL_Godot")
# Godot advances 1/FIXED_FPS per main loop, so the window redraws
# speedup x FIXED_FPS times a second. Drawing is the largest avoidable
# cost, so hold a steady rate whatever the speedup.
RENDER_FPS = 40
# Seconds the backend waits for a SITL which has stopped sending before it
# quits.  Long enough to cover a reboot and a slow start, short enough
# that a killed run does not leave the port held for the next one.
SITL_TIMEOUT = 120

# the SITL model which talks to the backend
SITL_MODEL = "JSON:127.0.0.1"

# vehicleinfo.json vehicle -> {frame: model in the Godot project's models directory}
MODELS = {
    "ArduCopter": {"+": "quadx", "quad": "quadx", "X": "quadx", "heli": "heli"},
    "ArduPlane": {"plane": "plane", "plane-elevrev": "plane", "plane-3d": "plane3d",
                  "quadplane": "quadplane",
                  "plane-catapult": "plane", "plane-bungee": "plane",
                  "glider": "glider"},
    "Rover": {"rover": "buggy", "motorboat": "speedboat",
              "balancebot": "balancebot",
              "sailboat": "sailboat", "sailboat-motor": "sailboat",
              "rover-hexapod": "hexapod"},
    "ArduSub": {"vectored": "sub"},
    "AntennaTracker": {"tracker": "tracker"},
    "Helicopter": {"heli": "heli"},
    "Blimp": {"blimp": "blimp", "blimp-motor": "blimp"},
}

# servo outputs which a frame's SITL model reverses, and whose parameters
# reverse them back, so the backend must reverse them too
REVERSED_SERVOS = {
    "ArduPlane": {"plane-elevrev": (2, 4)},
}


# Frames launched off a stationary rail, as kind,acceleration,length,angle.
# A catapult (sled) carries the aeroplane on a shuttle; a bungee (ramp)
# hauls on the aeroplane itself. Accelerations are SITL's (SIM_Plane.cpp).
LAUNCHERS = {
    "ArduPlane": {
        "plane-catapult": "sled,15,5,15",
        "plane-bungee": "ramp,7,10,15",
        # no balloon here: the glider is held at 400 m and released on the
        # launcher servo at 25 m/s, between its 12 m/s stall and 38 m/s cruise
        "glider": "drop,400,25",
    },
}


# Water a vehicle needs under it, as depth,radius in metres. The default
# lake floats a boat but is too shallow for a submarine to dive in.
WATERS = {
    "ArduSub": {"vectored": (60.0, 600.0)},
    # a sailing boat tacks, and sails out of the default 45 m lake
    "Rover": {"sailboat": (3.0, 100.0), "sailboat-motor": (3.0, 100.0)},
}


# Wind a frame needs the backend to blow, as north,east,down in m/s.
# SIM_WIND_* only set SITL's wind model and do not reach the physics.
WINDS = {
    # Northward wind is on the beam of a boat that spawns heading west; over
    # the bow it could never get under way. 3 m/s because 5 knocks this
    # 4 kg boat flat.
    "Rover": {"sailboat": (3.0, 0.0, 0.0), "sailboat-motor": (3.0, 0.0, 0.0)},
}


def wind_for(vehicle, frame):
    '''the wind a frame needs blowing, or None'''
    return WINDS.get(vehicle, {}).get(frame)


def water_for(vehicle, frame):
    '''the depth and radius of water a frame needs, or None'''
    return WATERS.get(vehicle, {}).get(frame)


def launcher_for(vehicle, frame):
    '''the launcher a frame leaves the ground from, or None'''
    return LAUNCHERS.get(vehicle, {}).get(frame)


def model_for(vehicle, frame):
    '''the Godot model for a vehicle's frame, or None if there isn't one'''
    return MODELS.get(vehicle, {}).get(frame)


def reversed_servos_for(vehicle, frame):
    return REVERSED_SERVOS.get(vehicle, {}).get(frame, ())


def frames_for(vehicle):
    return sorted(MODELS.get(vehicle, {}).keys())


def check_supported(vehicle):
    if vehicle not in MODELS:
        raise ValueError("no Godot physics model for %s; there are models for %s" %
                         (vehicle, ", ".join(sorted(MODELS.keys()))))


def params_filepath(model):
    '''parameters for a Godot model, to apply after the frame's defaults'''
    return os.path.join(PROJECT, "models", model + ".parm")


def lua_filepath(model):
    '''a Godot model's lua companion, or None. It sets what a defaults
    file cannot, such as another script's parameters, once they exist'''
    path = os.path.join(PROJECT, "models", model + ".lua")
    return path if os.path.exists(path) else None


def find_godot():
    '''the Godot binary, from $GODOT or the PATH'''
    for candidate in [os.environ.get("GODOT"), "godot", "godot4", "Godot",
                      "/Applications/Godot.app/Contents/MacOS/Godot"]:
        if candidate:
            path = shutil.which(candidate)
            if path is not None:
                return path
    raise FileNotFoundError("Godot 4.4 or later not found: put it on the PATH as 'godot' or set GODOT")


def import_project():
    '''Write Godot's .godot/ class cache with one editor pass. A fresh checkout
    has none, so the backend fails to parse and SITL waits in lockstep.'''
    cache = os.path.join(PROJECT, ".godot", "global_script_class_cache.cfg")
    if os.path.exists(cache):
        return
    print("Godot: first run in this tree, importing the project (one moment)")
    subprocess.run([find_godot(), "--headless", "--editor", "--quit",
                    "--path", PROJECT],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not os.path.exists(cache):
        raise RuntimeError("Godot could not import %s" % PROJECT)
    print("Godot: project imported")


# the backend listening on each UDP port
running = {}


def backend_for(logfile, headless=True):
    '''The backend already serving this port, when it is alive and writing to
    the same log. autotest builds a new suite per test, so without this each
    test would start its own process and window.'''
    have = running.get(9002)
    if (have is not None and have.is_running()
            and have.logfile == logfile and have.headless == headless):
        return have
    # a different vehicle logs somewhere else, and a process cannot be
    # told to write to another file after it has started
    return Backend(logfile, headless=headless)


class Backend(object):
    '''the backend process. It keeps running across SITL restarts, and is
    told about a different vehicle rather than restarted for one, so a
    whole test run is served by one process and one window.'''

    def __init__(self, logfile, instance=0, headless=True):
        self.logfile = logfile
        # SITL's JSON model sends to port 9002 + 10 x instance
        self.port = 9002 + 10 * instance
        # the backend listens for a new simulation here; no vehicle uses
        # it, since vehicle i is on port + 10 x i
        self.control_port = self.port + 5
        self.headless = headless
        self.process = None
        self.config = None
        atexit.register(self.stop)

    def set_mode(self, name):
        '''tell the backend the flight mode, for its HUD. A servo packet
        carries no mode, so it has no other way to know.'''
        self._say("mode " + name)

    def set_wp(self, seq):
        '''tell the backend which mission item is being steered for'''
        self._say("wp %s" % seq)

    def set_msg(self, text):
        '''tell the backend the last status text ArduPilot sent'''
        self._say("msg " + text)

    def set_waypoints(self, radius, items):
        '''tell the backend the mission, so it can stand a marker at each
        waypoint: items is a list of (seq, lat, lon, alt above home)'''
        self._say("wps %.2f;" % radius +
                  ";".join("%u,%.7f,%.7f,%.2f" % item for item in items), limit=8000)

    def set_target(self, lat, lng, height, heading=0.0, bank=0.0):
        '''tell the backend where the aeroplane the vehicle is following
        is, so it can be drawn: height is above home, and bank is positive
        with the right wing down'''
        self._say("target %.7f,%.7f,%.2f,%.1f,%.1f" % (lat, lng, height, heading, bank))

    def _say(self, line, limit=500):
        '''a word for the backend's HUD; never worth failing a test over'''
        if not self.is_running():
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(line.encode("utf-8", "replace")[:limit], ("127.0.0.1", self.control_port))
            sock.close()
        except OSError:
            pass

    def set_test(self, name):
        '''tell the backend which autotest it is flying, for its HUD'''
        self._say("test " + name)

    def set_hidden(self, hidden):
        '''hide the vehicle while the harness is only tidying up, so the
        end of a run does not show it back at home as though it had
        jumped there'''
        self._say("hide 1" if hidden else "hide 0")

    def reconfigure(self, args):
        '''ask the running backend for a different simulation, as the
        --key=value it would have been started with. True if it took it.'''
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(20)
        try:
            sock.sendto(" ".join(args).encode(), ("127.0.0.1", self.control_port))
            return sock.recv(64).strip() == b"ok"
        except OSError:
            return False
        finally:
            sock.close()

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def fixed_fps(self, speedup):
        '''main loop rate which renders at about RENDER_FPS'''
        if not speedup or speedup <= 0:
            speedup = 1
        return str(max(2, min(50, int(round(RENDER_FPS / float(speedup))))))

    def start(self, model, reversed_servos=(), home=None, spacing=None,
              speedup=1, launcher=None, water=None, wind=None):
        '''model is a list of up to five Godot model names which share one
        world, one per SITL instance from this backend's up, spawned
        spacing="east,north" metres apart. home is "lat,lon" for the imagery'''
        models = tuple(model)
        # extra backend options (see the README), e.g. --wind=0,5,0
        extra = tuple(shlex.split(os.environ.get("GODOT_SIM_ARGS", "")))
        config = (models, tuple(reversed_servos), home, spacing, extra, launcher, water, wind)
        # every option which says what is being simulated, so the same
        # list serves a fresh start and a reconfigure of a running one
        sim_args = ["--vehicle=" + ",".join("res://models/%s.gltf" % m for m in models)]
        # cleared explicitly: a reconfigure which leaves one out would
        # inherit the last simulation's value
        sim_args.append("--reverse=%s" % ",".join(str(ch) for ch in reversed_servos))
        sim_args.append("--home=%s" % (home or ""))
        sim_args.append("--spacing=%s" % (spacing or "5,0"))
        sim_args.append("--launcher=%s" % (launcher or ""))
        if wind is not None:
            sim_args.append("--wind=%g,%g,%g" % wind)
        if water is not None:
            sim_args.append("--lake-depth=%g" % water[0])
            sim_args.append("--lake-radius=%g" % water[1])
        sim_args.append("--sitl-timeout=%u" % SITL_TIMEOUT)
        sim_args.extend(extra)

        why = None
        if self.is_running():
            if config == self.config:
                return
            # the window, the imported project and the loaded imagery are
            # worth keeping: ask for the new simulation in place, and only
            # start a process again if the backend will not take it
            if self.reconfigure(sim_args):
                with open(self.logfile, "a") as log:
                    log.write("=== reconfigured: %s\n" % " ".join(sim_args))
                self.config = config
                return
            why = "it would not take a new simulation: %s -> %s" % (self.config, config)
            self.stop()
        elif self.process is not None:
            why = "the backend is gone (exit %s)" % self.process.poll()
        # one left running by a test which did not finish would hold the port
        other = running.get(self.port)
        if other is not None and other is not self:
            other.stop()
        headless = self.headless
        if (not headless and sys.platform.startswith("linux") and
                not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY")):
            print("No display, so running the Godot backend without its window")
            headless = True
        import_project()
        # a fixed frame rate lets the physics run faster than real time
        # when SITL is sped up; SITL sets the pace, so V-Sync must not
        fps = os.environ.get("GODOT_FIXED_FPS") or self.fixed_fps(speedup)
        cmd = [find_godot(), "--path", PROJECT, "--fixed-fps", fps]
        if headless:
            cmd.append("--headless")
        else:
            cmd.append("--disable-vsync")
        cmd.append("--")
        cmd.extend(sim_args)
        cmd.append("--port=%u" % self.port)
        cmd.append("--control-port=%u" % self.control_port)
        # append, or each restart (a test's teardown boot changes the
        # model) destroys the log of the flight that just happened
        with open(self.logfile, "a") as log:
            if why:
                log.write("=== restarting because %s\n" % why)
            log.write("=== backend start: %s\n" % " ".join(cmd))
            log.flush()
            self.process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        self.config = config
        running[self.port] = self
        deadline = time.time() + 60
        while time.time() < deadline:
            with open(self.logfile) as f:
                if "waiting for SITL" in f.read():
                    return
            if self.process.poll() is not None:
                raise RuntimeError("the Godot backend exited; see %s" % self.logfile)
            time.sleep(0.1)
        self.stop()
        raise RuntimeError("the Godot backend did not start; see %s" % self.logfile)

    def stop(self):
        if running.get(self.port) is self:
            del running[self.port]
        if not self.is_running():
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
