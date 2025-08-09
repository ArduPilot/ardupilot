"""
Waf tool for functions common to the esp32, Linux and ChibiOS builds.

AP_FLAKE8_CLEAN
"""

import os
import pickle


def load_env_vars(env, kv_handler=None):
    '''optionally load extra environment variables from env.py in the build directory'''
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for kv in e.items():
        if kv_handler is not None:
            kv_handler(env, kv)
            continue
        load_env_vars_handle_kv_pair(env, kv)


def load_env_vars_handle_kv_pair(env, kv_pair):
    '''handle a key/value pair out of the pickled environment dictionary'''
    (k, v) = kv_pair
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
