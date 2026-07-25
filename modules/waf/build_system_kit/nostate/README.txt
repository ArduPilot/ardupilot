Note from the author: using signatures is of course much better
this example is for research purposes only


In this example, no configuration will be written or read,
and the build will only use timestamps (no cache files
and no signatures). There is no build directory either.

To build, use "./ebd"


Although the wscript file only declares a build function,
the system performs a configuration internally to check for
a C compiler. The configuration and build context classes are
overridden to hide output messages and to avoid creating cache
files.

The task class is monkey-patched so that all existing build tasks
will execute without using signatures (only file timestamps are considered).
Implicit dependencies such as headers are still computed automatically.

