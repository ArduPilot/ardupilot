#!/usr/bin/env python

"""
Contains functions used to test the ArduPilot build_options.py structures

AP_FLAKE8_CLEAN
"""

from __future__ import print_function

import fnmatch
import optparse
import os

from pysim import util


class TestBuildOptions(object):
    def __init__(self,
                 match_glob=None,
                 do_step_disable_all=True,
                 do_step_disable_none=False,
                 do_step_disable_defaults=True,
                 do_step_disable_in_turn=True,
                 build_targets=None,
                 board="DevEBoxH7v2",
                 extra_hwdef=None):
        self.extra_hwdef = extra_hwdef
        self.sizes_nothing_disabled = None
        self.match_glob = match_glob
        self.do_step_disable_all = do_step_disable_all
        self.do_step_disable_none = do_step_disable_none
        self.do_step_run_with_defaults = do_step_disable_defaults
        self.do_step_disable_in_turn = do_step_disable_in_turn
        self.build_targets = build_targets
        if self.build_targets is None:
            self.build_targets = self.all_targets()
        self._board = board

    @staticmethod
    def all_targets():
        return ['copter', 'plane', 'rover', 'antennatracker', 'sub', 'blimp']

    def progress(self, message):
        print("###### %s" % message)

    # swiped from app.py:
    def get_build_options_from_ardupilot_tree(self):
        '''return a list of build options'''
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "build_options.py",
            os.path.join(os.path.dirname(os.path.realpath(__file__)),
                         '..', 'scripts', 'build_options.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.BUILD_OPTIONS

    def write_defines_to_file(self, defines, filepath):
        lines = []
        lines.extend(["undef %s\n" % (a, ) for (a, b) in defines.items()])
        lines.extend(["define %s %s\n" % (a, b) for (a, b) in defines.items()])
        content = "".join(lines)
        with open(filepath, "w") as f:
            f.write(content)

    def get_defines(self, feature, options):
        '''returns a hash of (name, value) defines to turn feature off -
        recursively gets dependencies'''
        ret = {
            feature.define: 0,
        }
        added_one = True
        while added_one:
            added_one = False
            for option in options:
                if option.define in ret:
                    continue
                if option.dependency is None:
                    continue
                for dep in option.dependency.split(','):
                    f = self.get_option_by_label(dep, options)
                    if f.define not in ret:
                        continue

                    print("%s requires %s" % (option.define, f.define))
                    added_one = True
                    ret[option.define] = 0
                    break
        return ret

    def test_feature(self, feature, options):
        defines = self.get_defines(feature, options)

        if len(defines.keys()) > 1:
            self.progress("Disabling %s disables (%s)" % (
                feature.define,
                ",".join(defines.keys())))

        self.test_compile_with_defines(defines)

    def board(self):
        '''returns board to build for'''
        return self._board

    def test_compile_with_defines(self, defines):
        extra_hwdef_filepath = "/tmp/extra.hwdef"
        self.write_defines_to_file(defines, extra_hwdef_filepath)
        if self.extra_hwdef is not None:
            content = open(self.extra_hwdef, "r").read()
            with open(extra_hwdef_filepath, "a") as f:
                f.write(content)
        util.waf_configure(
            self.board(),
            extra_hwdef=extra_hwdef_filepath,
        )
        for t in self.build_targets:
            try:
                util.run_cmd([util.relwaf(), t])
            except Exception:
                print("Failed to build (%s) with things disabled" %
                      (t,))
                raise

    def find_build_sizes(self):
        '''returns a hash with size of all build targets'''
        ret = {}
        target_to_binpath = {
            "copter": "arducopter",
            "plane": "arduplane",
            "rover": "ardurover",
            "antennatracker": "antennatracker",
            "sub": "ardusub",
            "blimp": "blimp",
        }
        for target in self.build_targets:
            path = os.path.join("build", self.board(), "bin", "%s.bin" % target_to_binpath[target])
            ret[target] = os.path.getsize(path)
        return ret

    def disable_in_turn_check_sizes(self, feature, sizes_nothing_disabled):
        if not self.do_step_disable_none:
            self.progress("disable-none skipped, size comparison not available")
            return
        current_sizes = self.find_build_sizes()
        for (build, new_size) in current_sizes.items():
            old_size = sizes_nothing_disabled[build]
            self.progress("Disabling %s(%s) on %s saves %u bytes" %
                          (feature.label, feature.define, build, old_size - new_size))

    def run_disable_in_turn(self):
        options = self.get_build_options_from_ardupilot_tree()
        if self.match_glob is not None:
            options = list(filter(lambda x : fnmatch.fnmatch(x.define, self.match_glob), options))
        count = 1
        for feature in options:
            self.progress("Disabling feature %s(%s) (%u/%u)" %
                          (feature.label, feature.define, count, len(options)))
            self.test_feature(feature, options)
            count += 1
            self.disable_in_turn_check_sizes(feature, self.sizes_nothing_disabled)

    def get_option_by_label(self, label, options):
        for x in options:
            if x.label == label:
                return x
        raise ValueError("No such option (%s)" % label)

    def run_disable_all(self):
        options = self.get_build_options_from_ardupilot_tree()
        defines = {}
        for feature in options:
            if self.match_glob is not None:
                if not fnmatch.fnmatch(feature.define, self.match_glob):
                    continue
            defines[feature.define] = 0
        self.test_compile_with_defines(defines)

    def run_disable_none(self):
        self.test_compile_with_defines({})
        self.sizes_nothing_disabled = self.find_build_sizes()

    def run_with_defaults(self):
        options = self.get_build_options_from_ardupilot_tree()
        defines = {}
        for feature in options:
            defines[feature.define] = feature.default
        self.test_compile_with_defines(defines)

    def check_deps_consistency(self):
        # self.progress("Checking deps consistency")
        options = self.get_build_options_from_ardupilot_tree()
        for feature in options:
            self.get_defines(feature, options)

    def run(self):
        self.check_deps_consistency()
        if self.do_step_run_with_defaults:
            self.progress("Running run-with-defaults step")
            self.run_with_defaults()
        if self.do_step_disable_all:
            self.progress("Running disable-all step")
            self.run_disable_all()
        if self.do_step_disable_none:
            self.progress("Running disable-none step")
            self.run_disable_none()
        if self.do_step_disable_in_turn:
            self.progress("Running disable-in-turn step")
            self.run_disable_in_turn()


if __name__ == '__main__':

    parser = optparse.OptionParser()
    parser.add_option("--define-match-glob",
                      type='string',
                      default=None,
                      help='feature define must match this glob to be tested')
    parser.add_option("--no-run-with-defaults",
                      action='store_true',
                      help='Do not run the run-with-defaults step')
    parser.add_option("--no-disable-all",
                      action='store_true',
                      help='Do not run the disable-all step')
    parser.add_option("--no-disable-none",
                      action='store_true',
                      help='Do not run the disable-none step')
    parser.add_option("--no-disable-in-turn",
                      action='store_true',
                      help='Do not run the disable-in-turn step')
    parser.add_option("--build-targets",
                      type='choice',
                      choices=TestBuildOptions.all_targets(),
                      action='append',
                      help='vehicle targets to build')
    parser.add_option("--extra-hwdef",
                      type='string',
                      default=None,
                      help="file containing extra hwdef information")
    parser.add_option("--board",
                      type='string',
                      default="DevEBoxH7v2",
                      help='board to build for')

    opts, args = parser.parse_args()

    tbo = TestBuildOptions(
        match_glob=opts.define_match_glob,
        do_step_disable_all=not opts.no_disable_all,
        do_step_disable_none=not opts.no_disable_none,
        do_step_disable_defaults=not opts.no_run_with_defaults,
        do_step_disable_in_turn=not opts.no_disable_in_turn,
        build_targets=opts.build_targets,
        board=opts.board,
        extra_hwdef=opts.extra_hwdef,
    )
    tbo.run()
