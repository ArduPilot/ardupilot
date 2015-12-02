# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## ROS Message generatation
##
##

import sys
import os
import em
import genmsg.command_line
import genmsg.msgs
import genmsg.msg_loader
import genmsg.gentools

# generate msg or srv files from a template file
# template_map of the form { 'template_file':'output_file'} output_file can contain @NAME@ which will be replaced by the message/service name
def _generate_from_spec(input_file, output_dir, template_dir, msg_context, spec, template_map, search_path):

    md5sum = genmsg.gentools.compute_md5(msg_context, spec)

    # precompute msg definition once
    if isinstance(spec, genmsg.msgs.MsgSpec):
        msg_definition = genmsg.gentools.compute_full_text(msg_context, spec)

    # Loop over all files to generate
    for template_file_name, output_file_name in template_map.items():
        template_file = os.path.join(template_dir, template_file_name)
        output_file = os.path.join(output_dir, output_file_name.replace("@NAME@", spec.short_name))

        #print "generate_from_template %s %s %s" % (input_file, template_file, output_file)

        ofile = open(output_file, 'w') #todo try

        # Set dictionary for the generator interpreter
        g = {
            "file_name_in": input_file,
            "spec": spec,
            "md5sum": md5sum,
            "search_path": search_path,
            "msg_context": msg_context
        }
        if isinstance(spec, genmsg.msgs.MsgSpec):
            g['msg_definition'] = msg_definition

        # todo, reuse interpreter
        interpreter = em.Interpreter(output=ofile, globals=g, options={em.RAW_OPT:True,em.BUFFERED_OPT:True})
        if not os.path.isfile(template_file):
            ofile.close()
            os.remove(output_file)
            raise RuntimeError("Template file %s not found in template dir %s" % (template_file_name, template_dir))
        interpreter.file(open(template_file)) #todo try
        interpreter.shutdown()

def _generate_msg_from_file(input_file, output_dir, template_dir, search_path, package_name, msg_template_dict):
    # Read MsgSpec from .msg file
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(package_name, os.path.basename(input_file))
    spec = genmsg.msg_loader.load_msg_from_file(msg_context, input_file, full_type_name)
    # Load the dependencies
    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    # Generate the language dependent msg file
    _generate_from_spec(input_file,
                            output_dir,
                            template_dir,
                            msg_context,
                            spec,
                            msg_template_dict,
                            search_path)

def _generate_srv_from_file(input_file, output_dir, template_dir, search_path, package_name, srv_template_dict, msg_template_dict):
    # Read MsgSpec from .srv.file
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(package_name, os.path.basename(input_file))
    spec = genmsg.msg_loader.load_srv_from_file(msg_context, input_file, full_type_name)
    # Load the dependencies
    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    # Generate the language dependent srv file
    _generate_from_spec(input_file,
                        output_dir,
                        template_dir,
                        msg_context,
                        spec,
                        srv_template_dict,
                        search_path)
    # Generate the language dependent msg file for the srv request
    _generate_from_spec(input_file,
                        output_dir,
                        template_dir,
                        msg_context,
                        spec.request,
                        msg_template_dict,
                        search_path)
    # Generate the language dependent msg file for the srv response
    _generate_from_spec(input_file,
                        output_dir,
                        template_dir,
                        msg_context,
                        spec.response,
                        msg_template_dict,
                        search_path)

# uniform interface for genering either srv or msg files
def generate_from_file(input_file, package_name, output_dir, template_dir, include_path, msg_template_dict, srv_template_dict):
    # Normalize paths
    input_file = os.path.abspath(input_file)
    output_dir = os.path.abspath(output_dir)

    # Create output dir
    try:
        os.makedirs(output_dir)
    except OSError as e:
        if e.errno != 17: # ignore file exists error
            raise

    # Parse include path dictionary
    if( include_path ):
        search_path = genmsg.command_line.includepath_to_dict(include_path)
    else:
        search_path = {}

    # Generate the file(s)
    if input_file.endswith(".msg"):
        _generate_msg_from_file(input_file, output_dir, template_dir, search_path, package_name, msg_template_dict)
    elif input_file.endswith(".srv"):
        _generate_srv_from_file(input_file, output_dir, template_dir, search_path, package_name, srv_template_dict, msg_template_dict)
    else:
        assert False, "Uknown file extension for %s"%input_file

def generate_module(package_name, output_dir, template_dir, template_dict):
    # Locate generate msg files
    files = os.listdir(output_dir)

    # Loop over all files to generate
    for template_file_name, output_file_name in template_dict.items():
        template_file = os.path.join(template_dir, template_file_name)
        output_file = os.path.join(output_dir, output_file_name)

        ofile = open(output_file, 'w') #todo try

        # Set dictionary for the generator intepreter
        g = dict(files=files,
                 package=package_name)

        # todo, reuse interpreter
        interpreter = em.Interpreter(output=ofile, options={em.RAW_OPT:True,em.BUFFERED_OPT:True})
        interpreter.updateGlobals(g)
        if not os.path.isfile(template_file):
            ofile.close()
            os.remove(output_file)
            raise RuntimeError("Template file %s not found in template dir %s" % (template_file_name, template_dir))
        interpreter.file(open(template_file)) #todo try
        interpreter.shutdown()

# Uniform interface to support the standard command line options
def generate_from_command_line_options(argv, msg_template_dict, srv_template_dict, module_template_dict = {}):
    from optparse import OptionParser
    parser = OptionParser("[options] <srv file>")
    parser.add_option("-p", dest='package',
                      help="ros package the generated msg/srv files belongs to")
    parser.add_option("-o", dest='outdir',
                      help="directory in which to place output files")
    parser.add_option("-I", dest='includepath',
                      help="include path to search for messages",
                      action="append")
    parser.add_option("-m", dest='module',
                      help="write the module file",
                      action='store_true', default=False)
    parser.add_option("-e", dest='emdir',
                      help="directory containing template files",
                      default=sys.path[0])

    (options, argv) = parser.parse_args(argv)

    if( not options.package or not options.outdir or not options.emdir):
        parser.print_help()
        exit(-1)

    if( options.module ):
        generate_module(options.package, options.outdir, options.emdir, module_template_dict)
    else:
        if len(argv) > 1:
            generate_from_file(argv[1], options.package, options.outdir, options.emdir, options.includepath, msg_template_dict, srv_template_dict)
        else:
            parser.print_help()
            exit(-1)

