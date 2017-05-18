.. _index:

genmsg:  generating code from ros .msg format
=============================================

Project ``genmsg`` exists in order to decouple code generation from
``.msg`` & ``.srv`` format files from the parsing of these files and
from impementation details of the build system (project directory
layout, existence or nonexistence of utilities like ``rospack``,
values of environment variables such as ``ROS_PACKAGE_PATH``):
i.e. none of these are required to be set in any particular way.

.. toctree::

   usermacros
   python_api
   developer

Code generators may not use any information other than what is
provided on the commandline.

