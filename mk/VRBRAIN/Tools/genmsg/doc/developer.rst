Developer documenation
======================

Project ``genmsg`` exists in order to decouple code generation from
.msg format files from the parsing of these files and from
impementation details of the build system (project directory layout,
existence or nonexistence of utilities like ``rospack``, values of
environment variables such as ``ROS_PACKAGE_PATH``): i.e. none of
these are required to be set in any particular way.

Code generators expose a compiler-like interface that make all inputs,
outputs and search paths explicit.  For instance, the invocation of
``gencpp`` for ros message ``nav_msgs/Odometry.msg`` looks like this::

  /ssd/catkin/test/src/gencpp/scripts/gen_cpp.py
  /ssd/catkin/test/src/common_msgs/nav_msgs/msg/Odometry.msg
  -Inav_msgs:/ssd/catkin/test/src/common_msgs/nav_msgs/msg
  -Igeometry_msgs:/ssd/catkin/test/src/common_msgs/geometry_msgs/msg
  -Istd_msgs:/ssd/catkin/test/src/std_msgs/msg
  -p nav_msgs
  -o /ssd/catkin/test/build/gen/cpp/nav_msgs
  -e /ssd/catkin/test/src/gencpp/scripts

where the code generator (the first argument), is a python script
``gen_cpp.py``.  The commandline arguments have the following
meanings:

``/path/to/Some.msg``
     The flagless argument is the path to the
     input ``.msg`` file.

``-I NAMESPACE:/some/path``
     find messages in NAMESPACE in directory /some/path

``-p THIS_NAMESPACE``
     put generated message into namespace THIS_NAMESPACE

``-o /output/dir``
     Generate code into directory :file:`/output/dir`

``-e /path/to/templates``
     Find empy templates in this directory


Code generators may not use any information other than what is
provided on the commandline.


.. rubric:: Writing the generator

Code generators depend on ``genmsg`` to parse the .msg file itself.
They then use the parse tree to generate code in whatever language or
format they prefer.

A separate project must exists for each language you wish to generate for.
Such a project contains:

* A message_generator tag in the stack.xml file
* Executable scripts for generating the code based on .msg/.srv files
* Definitions of certain CMake macros to make the generator accessible by the build system.

Generator Scripts
~~~~~~~~~~~~~~~~~

The recommended way of implementing the generator is by using empy
template files. See: http://www.alcyone.com/software/empy A empy
template is a text file part of which can contain python code that is
evaluated during code generation.  ``genmsg`` includes python methods
for parsing the command line arguments and performing the code
generation very easily if the template model is used.

The script for generating cpp files looks as::

  import sys
  import genmsg.template_tools

  msg_template_map = { 'msg.h.template':'@NAME@.h' }
  srv_template_map = { 'srv.h.template':'@NAME@.h' }

  if __name__ == "__main__":
    genmsg.template_tools.generate_from_command_line_options(sys.argv, msg_template_map, srv_template_map)

``msg_template_map`` and ``srv_template_map`` defines the template
files used for generating from .msg and .srv files respectively.  The
format is ``<type>_template_map = {
'<template_filename>':'<output_file_name>' }``.  The entry ``@NAME@``
will be replaced by the short name of the message such as ``String``
for ``String.msg`` etc.  The call to
``generate_from_command_line_options`` will use the correct map
depending on the file gives as command line argument.  When a service
is generated, two messages are also generated, namely the
``<SrvName>Request`` and ``<SrvName>Response``.

``genmsg`` will parse the respective .msg and .srv file and expose the
information in three python variables awailable in the empy template.
These are:

* ``file_name_in`` (String) Filename of the source .msg /.srv file
* ``spec`` (msggen.MsgSpec) Parsed specification of the .msg/.srv file
* ``md5sum`` (String) MD5Sum of the msg/srv

See https://github.com/ros/gencpp/blob/master/scripts/msg.h.template
and https://github.com/ros/gencpp/blob/master/scripts/srv.h.template
for example template files.

If the language requires a common file to exists for all the generated
source code files (Such as __init__.py for python) it is possible to
specify a ``module_template_map``.  See
https://github.com/ros/genpybindings/blob/master/scripts/module.cpp.template
for example of this.


Catkin (fuerte)
~~~~~~~~~~~~~~~
Each language is identified by a name which must be specified in the stack.xml file.

In catkin fuerte, message generators declared their contribution in the stack.xml file.

The example entry for the generator for C++ is::

  <message_generator>cpp</message_generator>

The project name for the generator with identifier ``X`` should be ``genX``.

Catkin (groovy)
~~~~~~~~~~~~~~~
In catkin groovy, message generators declared their contribution in the package.xml file::

  <message_generator>cpp</message_generator>

Providing cmake code to catkin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Generator packages define several macros (below), and use catkin
mechanisms to make the definitions of these macros available, see
``catkin_package``.  catkin will generate calls to them for

* each message
* each service
* once for the overall package

For a generator called ``X``, in a package called ``genX``:

.. cmake:macro:: _generate_msg_X(PACKAGE MESSAGE IFLAGS MSG_DEPS OUTDIR)

   :param PACKAGE: name of package that the generated message MESSAGE
                   is found in.
   :param MESSAGE: full path to ``.msg`` file
   :param IFLAGS: a list of flags in ``-I<package>:/path`` format
   :param MSG_DEPS: a list of ``.msg`` files on which this message depends
   :param OUTDIR: destination directory for generated files

There are two other macros, ``_generate_srv_X``,

.. cmake:macro:: _generate_srv_X(PACKAGE SERVICE IFLAGS MSG_DEPS OUTDIR)

   :param PACKAGE: name of package that the generated message MESSAGE
                   is found in.

   :param SERVICE: full path to ``.srv`` file

   :param IFLAGS: a list of flags in ``-I<package>:/path`` format

   :param MSG_DEPS: a list of ``.msg`` files on which this message
          depends

   :param OUTDIR: destination directory for generated files

and

.. cmake:macro:: _generate_module_X(PACKAGE OUTDIR GENERATED_FILES)

   :param PACKAGE:  name of package

   :param OUTDIR:  destination directory

   :param GENERATED_FILES: Files that were generated (from messages
                           and services) for this package.  Usually
                           used to pass to the ``DEPENDS`` option of
                           cmake's ``add_custom_command()``

   Generate any "module" code necessary, e.g. ``__init__.py`` for
   python or ``module.cpp`` for boost.python bindings.



Examples
~~~~~~~~

Example projects that use this infrastructure are ``gencpp``,
``genpy``, and ``genpybindings``, all found in the github repositories
at http://github.com/ros.

