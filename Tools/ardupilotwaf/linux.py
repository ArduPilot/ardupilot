# encoding: utf-8

"""
Waf tool for Linux build

AP_FLAKE8_CLEAN
"""

from waflib.TaskGen import after_method, before_method, feature

import os
import pickle
import sys
import traceback

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_Linux/hwdef/scripts'))
import linux_hwdef  # noqa:501


@feature('linux_ap_library', 'linux_ap_program')
@before_method('process_source')
def linux_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(exec_command, self).exec_command(cmd, **kw)


@feature('linux_ap_program')
@after_method('process_source')
def linux_firmware(self):
    pass


def load_env_vars(env):
    '''optionally load extra environment variables from env.py in the build directory'''
    print("Checking for env.py")
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for k in e.keys():
        v = e[k]
        if k in env:
            if isinstance(env[k], dict):
                a = v.split('=')
                env[k][a[0]] = '='.join(a[1:])
                print("env updated %s=%s" % (k, v))
            elif isinstance(env[k], list):
                env[k].append(v)
                print("env appended %s=%s" % (k, v))
            else:
                env[k] = v
                print("env added %s=%s" % (k, v))
        else:
            env[k] = v
            print("env set %s=%s" % (k, v))


def configure(cfg):

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()

    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    env.SRCROOT = srcpath('')
    env.BUILDROOT = bldpath('')

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['linux_ap_program']

    try:
        generate_hwdef_h(env)
    except Exception as e:
        print(get_exception_stacktrace(e))
        cfg.fatal("Failed to generate hwdef")
    load_env_vars(cfg.env)


def get_exception_stacktrace(e):
    ret = "%s\n" % e
    ret += ''.join(traceback.format_exception(type(e),
                                              e,
                                              tb=e.__traceback__))
    return ret


def generate_hwdef_h(env):
    '''run linux_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_Linux/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)
    lh = linux_hwdef.LinuxHWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
    )
    lh.run()


def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    load_env_vars(bld.env)
    if bld.env.HAL_NUM_CAN_IFACES:
        bld.get_board().with_can = True
    if bld.env.WITH_LITTLEFS:
        bld.get_board().with_littlefs = True
    hwdef_h = os.path.join(bld.env.BUILDROOT, 'hwdef.h')
    if not os.path.exists(hwdef_h):
        print("Generating hwdef.h")
        try:
            generate_hwdef_h(bld.env)
        except Exception:
            bld.fatal(f"Failed to process hwdef.dat {hwdef_h}")


def build(bld):
    bld(
        # build hwdef.h from hwdef.dat. This is needed after a waf clean
        source=bld.path.ant_glob(bld.env.HWDEF),
        rule="",
        group='dynamic_sources',
        target=[
            bld.bldnode.find_or_declare('hwdef.h'),
        ]
    )
