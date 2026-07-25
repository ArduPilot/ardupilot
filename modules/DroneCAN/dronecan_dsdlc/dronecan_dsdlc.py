#!/usr/bin/env python
# encoding: utf-8

'''
Copyright Jonathan Challinger, 2021
Copyright Siddharth Bharat Purohit, CubePilot Pty. Ltd. 2021
Released under GNU GPL version 3 or later
'''
import sys
import os
import argparse
import em
import shutil
import time

if __name__ == '__main__':
    import multiprocessing

    default_method = multiprocessing.get_all_start_methods()[0]
    # we are not compatible with forkserver because we trust argv to be set up
    # during module import, which using forkserver it is not in Python 3.13 and
    # later. as Python 3.14 and later make forkserver the default on Linux, we
    # need to switch back to the previous default of fork until we fix our code
    # to not run stuff on import.
    if default_method == "forkserver":
        default_method = "fork"
    multiprocessing.set_start_method(default_method)

try:
    import dronecan.dsdl
except Exception as ex:
    # for usage in CI try with a local pydronecan path
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "../pydronecan/"))
    try:
        import dronecan.dsdl
    except Exception as ex:
        print(ex)
        print("Failed to import dronecan.dsdl, please install dronecan with 'python3 -m pip install dronecan'")
        sys.exit(1)

from dronecan_dsdlc_helpers import *
from dronecan_dsdlc_tester import *

templates = [
    {'type': 'request_header',
     'output_file': 'include/@(msg_header_name_request(msg))'},
    {'type': 'response_header',
     'output_file': 'include/@(msg_header_name_response(msg))'},
    {'type': 'broadcast_header',
     'output_file': 'include/@(msg_header_name(msg))'},
    {'type': 'service_header',
     'output_file': 'include/@(msg_header_name(msg))'},
    {'type': 'request_src',
     'output_file': 'src/@(msg_c_file_name_request(msg))'},
    {'type': 'response_src',
     'output_file': 'src/@(msg_c_file_name_response(msg))'},
    {'type': 'broadcast_src',
     'output_file': 'src/@(msg_c_file_name(msg))'},
]

test_templates = [
    {'type': 'test_broadcast_src',
     'output_file': 'test/@(msg_test_file_name(msg))'},
    {'type': 'test_request_src',
     'output_file': 'test/@(msg_test_file_name_request(msg))'},
    {'type': 'test_response_src',
     'output_file': 'test/@(msg_test_file_name_response(msg))'},
    {'type': 'Makefile_broadcast',
     'output_file': 'test/@(msg_test_makefile_name(msg))'},
    {'type': 'Makefile_request',
     'output_file': 'test/@(msg_test_makefile_name_request(msg))'},
    {'type': 'Makefile_response',
     'output_file': 'test/@(msg_test_makefile_name_response(msg))'},
]

parser = argparse.ArgumentParser()
parser.add_argument('--output', '-O', action='store')
parser.add_argument('--build', action='append')
parser.add_argument('--run-tests', action='store_true')
parser.add_argument('-j', '--jobs', type=int)
parser.add_argument('namespace_dir', nargs='+')
args = parser.parse_args()

buildlist = None

if args.build:
    buildlist = set(args.build)

namespace_paths = [os.path.abspath(path) for path in args.namespace_dir]
build_dir = os.path.abspath(args.output)

templates_dir = os.path.join(os.path.dirname(__file__), 'templates')

messages = dronecan.dsdl.parse_namespaces(namespace_paths)
message_dict = {}
builtlist = set()

for msg in messages:
    message_dict[msg.full_name] = msg

# join test templates with standard templates if run_tests is True
if args.run_tests:
    # join the lists of templates
    templates += test_templates

# load standard templates
with open(os.path.join(templates_dir, 'msg.h.em'), 'rb') as f:
    msg_header_template = f.read().decode("utf-8")
with open(os.path.join(templates_dir, 'msg.c.em'), 'rb') as f:
    msg_source_template = f.read().decode("utf-8")
with open(os.path.join(templates_dir, 'service.h.em'), 'rb') as f:
    service_header_template = f.read().decode("utf-8")
with open(os.path.join(templates_dir, 'test_msg.cpp.em'), 'rb') as f:
    test_src_template = f.read().decode("utf-8")
with open(os.path.join(templates_dir, 'Makefile.em'), 'rb') as f:
    test_mk_template = f.read().decode("utf-8")


# expand source files and test files for messages
def expand_message(msg_name):
    print('expanding %s' % (msg_name,))
    msg = message_dict[msg_name]
    # expand templates
    for template in templates:
        output = ''
        if msg.kind == msg.KIND_SERVICE and template['type'] == 'service_header':
            output = em.expand(service_header_template, {"msg": msg})
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'response_header':
            output = em.expand(msg_header_template, get_empy_env_response(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'request_header':
            output = em.expand(msg_header_template, get_empy_env_request(msg))
        elif msg.kind == msg.KIND_MESSAGE and template['type'] == 'broadcast_header':
            output = em.expand(msg_header_template, get_empy_env_broadcast(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'response_src':
            output = em.expand(msg_source_template, get_empy_env_response(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'request_src':
            output = em.expand(msg_source_template, get_empy_env_request(msg))
        elif msg.kind == msg.KIND_MESSAGE and template['type'] == 'broadcast_src':
            output = em.expand(msg_source_template, get_empy_env_broadcast(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'test_response_src':
            output = em.expand(test_src_template, get_empy_env_response(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'test_request_src':
            output = em.expand(test_src_template, get_empy_env_request(msg))
        elif msg.kind == msg.KIND_MESSAGE and template['type'] == 'test_broadcast_src':
            output = em.expand(test_src_template, get_empy_env_broadcast(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'Makefile_response':
            output = em.expand(test_mk_template, get_empy_env_response(msg))
        elif msg.kind == msg.KIND_SERVICE and template['type'] == 'Makefile_request':
            output = em.expand(test_mk_template, get_empy_env_request(msg))
        elif msg.kind == msg.KIND_MESSAGE and template['type'] == 'Makefile_broadcast':
            output = em.expand(test_mk_template, get_empy_env_broadcast(msg))
        if not output.strip():
            continue

        output_file = os.path.join(build_dir, em.expand('@{from dronecan_dsdlc_helpers import *}'+template['output_file'], {"msg": msg}))
        mkdir_p(os.path.dirname(output_file))
        with open(output_file, 'wb') as f:
            f.write(output.encode("utf-8"))
    return msg_name

def process_test(msg_name, jobs):
    print(bcolors.HEADER + 'Starting Test for %s' % (msg_name,) + bcolors.ENDC)
    try:
        if message_dict[msg_name].kind == message_dict[msg_name].KIND_SERVICE:
            if len(message_dict[msg_name].request_fields):
                compile_test_app(msg_name+'_request', build_dir)
                run_test(message_dict[msg_name], 'request', build_dir)
            compile_test_app(msg_name+'_response', build_dir, jobs)
            run_test(message_dict[msg_name], 'response', build_dir)
        else:
            compile_test_app(msg_name, build_dir, jobs)
            run_test(message_dict[msg_name], None, build_dir)
    except Exception as e:
        raise Exception("Test for %s failed!" % (msg_name,)) from e

# callback for maintaining list of built messages
def append_builtlist(msg_name):
    global builtlist
    builtlist.add(msg_name)

if __name__ == '__main__':
    if buildlist is not None:
        while True:
            new_buildlist = set(buildlist)
            for msg_name in buildlist:
                msg = message_dict[msg_name]
                fields = getattr(msg, 'fields', []) + getattr(msg, 'request_fields', []) + getattr(msg, 'response_fields', [])
                for field in fields:
                    if field.type.category == field.type.CATEGORY_COMPOUND:
                        new_buildlist.add(field.type.full_name)
                    elif field.type.category == field.type.CATEGORY_ARRAY and field.type.value_type.category == field.type.CATEGORY_COMPOUND:
                        new_buildlist.add(field.type.value_type.full_name)

            if not new_buildlist-buildlist:
                break

            buildlist = new_buildlist

    from multiprocessing import Pool
    jobs = args.jobs if args.jobs is not None else os.cpu_count()

    pool = Pool(processes=jobs)

    results = []
    if buildlist is not None:
        for msg_name in buildlist:
            builtlist.add(msg_name)
            results.append(pool.apply_async(expand_message, (msg_name,), callback=append_builtlist))
    else:
        buildlist = set()
        for msg_name in [msg.full_name for msg in messages]:
            buildlist.add(msg_name)
            # print('expanding %s' % (msg_name,))
            # expand_message(msg_name)
            # append_builtlist(msg_name)
            results.append(pool.apply_async(expand_message, (msg_name,), callback=append_builtlist))

    pool.close()
    pool.join()

    if len(buildlist-builtlist):
        seen_no_attribute_error = False
        for result in results:
            try:
                x = result.get()
            except AttributeError as ex:
                print("Caught exception! %s" % str(ex))
                if "module 'em' has no attribute 'expand'" in str(ex):
                    seen_no_attribute_error = True
        if seen_no_attribute_error:
            print("############ try installing 'empy' rather than 'em' Python module")

    assert not buildlist-builtlist, "%s not built" % (buildlist-builtlist,)

    with open(os.path.join(build_dir+'/include/', 'dronecan_msgs.h'), 'w') as f:
        f.write('#pragma once\n')
        for msg_name in sorted(builtlist):
            include_line = '#include "%s.h"' % (msg_name,)
            f.write(include_line + '\n')

    if not args.run_tests:
        sys.exit(0)

    # Continue with building tests
    # copy test_helpers.h from template directory to build directory
    shutil.copy(os.path.join(templates_dir, 'test_helpers.h'), build_dir+'/test/')

    # start building test apps
    pool = Pool(processes=jobs)
    try:
        one_run = False
        results = []
        msg_list = [msg_name for msg_name in sorted(builtlist, reverse=True) if message_dict[msg_name].default_dtid is not None]
        if len(msg_list) > 0:
             # need to run one test not in parallel so all the dependent code gets built
            process_test(msg_list.pop(), jobs)
        # don't keep too many jobs pending so we can wait for them to finish quickly on error
        while len(results) > 0 or len(msg_list) > 0:
            while len(results) < jobs*1.25 and len(msg_list) > 0:
                results.append(pool.apply_async(process_test, (msg_list.pop(), 1)))
            time.sleep(0.1)
            pending = []
            for result in results:
                if result.ready():
                    result.get() # will raise exception if failed
                else:
                    pending.append(result)
            results = pending
    except KeyboardInterrupt:
        pool.terminate()
        raise
    finally:
        pool.close()
        pool.join()

    print("All tests completed successfully")
