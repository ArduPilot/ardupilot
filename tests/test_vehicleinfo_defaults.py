"""Tests for when sim_vehicle.py passes a frame's defaults with --defaults.

The SITL binary resolves its default parameters from --model through the
vehicleinfo.json embedded in it. sim_vehicle.py must add the -f frame's
files only when SITL can't find them itself.

AP_FLAKE8_CLEAN
"""

import pathlib
import sys
import types

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "Tools" / "autotest"))

from pysim import vehicleinfo  # noqa: E402


def needs_defaults(vehicle, frame, model):
    """Mirror sim_vehicle.py: frame is None when -f is not given."""
    vinfo = vehicleinfo.VehicleInfo()
    frame_given = frame is not None
    if frame is None:
        frame = vinfo.default_frame(vehicle)
    opts = types.SimpleNamespace(model=model, build_target=None)
    info = vinfo.options_for_frame(frame, vehicle, opts)
    # the --model sim_vehicle.py passes to SITL
    if model is None and "frame" in info:
        model = info["frame"]
    else:
        model = info["model"]
    return vinfo.needs_defaults(info, frame_given, model)


@pytest.mark.parametrize("model, expected", [
    ("quad", True),
    ("+", True),
    ("Callisto", True),
    ("octa-quad-cwx:@ROMFS/models/Callisto.json", True),
    ("quad:@ROMFS/models/freestyle.json", True),
    ("plane-catapult", True),
    ("heli-dual-foo", True),
    ("octaquad", False),
    ("JSON", False),
    ("JSON:127.0.0.1", False),
    ("webots-quad:127.0.0.1:5599", False),
])
def test_sitl_resolves(model, expected):
    """mirrors SITL_State::resolve_defaults_from_romfs()"""
    assert vehicleinfo.VehicleInfo().sitl_resolves(model) is expected


@pytest.mark.parametrize("vehicle, frame, model, expected", [
    # --model names a backend, not a frame: pass the -f frame's defaults
    ("ArduCopter", "gazebo-iris", "JSON", True),
    ("ArduCopter", "octaquad", "JSON:127.0.0.1", True),
    ("ArduCopter", "quad", "webots-quad:127.0.0.1:5599", True),
    ("Rover", "gazebo-rover", "JSON", True),
    # -f matched only by prefix and SITL can't resolve it (octaquad -> octa)
    ("ArduCopter", "octaquad", None, True),
    # SITL resolves it by itself, so its defaults must not be overridden
    ("ArduCopter", "quad", None, False),
    ("ArduCopter", "Callisto", None, False),
    ("ArduCopter", "calibration", None, False),
    ("ArduPlane", "plane-catapult", None, False),
    ("ArduCopter", "heli-dual-foo", None, False),
    ("ArduCopter", None, "X", False),
    ("ArduCopter", "quad", "X", False),
    ("ArduCopter", "quad", "quad:@ROMFS/models/freestyle.json", False),
    ("ArduCopter", None, "octa-quad-cwx:@ROMFS/models/Callisto.json", False),
    ("ArduPlane", None, "last_letter", False),
    # no -f: nothing to add, even if SITL finds no frame
    ("ArduCopter", None, "JSON", False),
    ("ArduCopter", None, "webots-quad:127.0.0.1:5599", False),
])
def test_needs_defaults(vehicle, frame, model, expected):
    assert needs_defaults(vehicle, frame, model) is expected


def test_model_override_leaves_frame_table_alone():
    vinfo = vehicleinfo.VehicleInfo()
    opts = types.SimpleNamespace(model="JSON", build_target=None)
    vinfo.options_for_frame("gazebo-iris", "ArduCopter", opts)
    assert "model" not in vinfo.options["ArduCopter"]["frames"]["gazebo-iris"]
    assert not vinfo.sitl_resolves("JSON")
