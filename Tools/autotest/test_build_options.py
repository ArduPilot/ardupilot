#!/usr/bin/env python

"""
Contains functions used to test the ArduPilot build_options.py structures

AP_FLAKE8_CLEAN
"""

from __future__ import print_function


import os

from pysim import util


# swiped from app.py:
def get_build_options_from_ardupilot_tree():
    '''return a list of build options'''
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "build_options.py",
        os.path.join(os.path.dirname(os.path.realpath(__file__)),
                     '..', 'scripts', 'build_options.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.BUILD_OPTIONS


def write_defines_to_file(defines, filepath):
    content = "\n".join(["define %s %s" % (a, b) for (a, b) in defines.items()])
    with open(filepath, "w") as f:
        f.write(content)


def get_defines(feature, options):
    '''returns a hash of (name, value) defines to turn feature off -
    recursively gets dependencies'''
    ret = {
        feature.define: 0,
    }
    if feature.dependency is None:
        return ret
    for depname in feature.dependency.split(','):
        dep = None
        for f in options:
            if f.label == depname:
                dep = f
        if dep is None:
            raise ValueError("Invalid dep (%s)" % dep)
        ret.update(get_defines(dep, options))
    return ret


def test_feature(feature, options):
    defines = get_defines(feature, options)
    test_compile_with_defines(defines)


def test_compile_with_defines(defines):
    extra_hwdef_filepath = "/tmp/extra.hwdef"
    write_defines_to_file(defines, extra_hwdef_filepath)
    util.waf_configure(
        "CubeOrange",
        extra_hwdef=extra_hwdef_filepath,
    )
    for t in 'copter', 'plane', 'rover', 'antennatracker', 'sub', 'blimp':
        try:
            util.waf_build(t)
        except Exception:
            print("Failed to build (%s) with everything disabled" %
                  (t,))
            raise


def run_disable_in_turn():
    options = get_build_options_from_ardupilot_tree()
    count = 1
    for feature in options:
        print("##### Disabling feature %s(%s) (%u/%u)" %
              (feature.label, feature.define, count, len(options)))
        test_feature(feature, options)
        count += 1


def run_disable_all():
    options = get_build_options_from_ardupilot_tree()
    defines = {}
    for feature in options:
        defines[feature.define] = 0
    test_compile_with_defines(defines)


def run():
    run_disable_all()
    run_disable_in_turn()


if __name__ == '__main__':
    run()
