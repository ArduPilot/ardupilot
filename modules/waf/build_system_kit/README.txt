The new "concurrent.futures" module from Python 3.2 will make it
even easier to execute tasks concurrently:
http://www.python.org/dev/peps/pep-3148/

It may be tempting to try to create a new build system from it,
but that's only a small part of a build system:

* reinventing a system for handling commands and command-line options
* adding a system of (task) order and dependencies
* creating an extension system for new programming languages
* handling exceptions and errors
* adding support for python versions < 3.2

All this represents a lot of work, and there are of course lots of design
mistakes to avoid. It is so easy to create a system with poor usability,
poor extensibility, and poor performance.

These pitfalls and many others are already solved in the Waf build system, which
also enables the re-use of its components into new build tools. By using these
tested and maintained components, much more time will be left to work
on the interesting problems such as creating an intuitive XML/YAML/JSON schema
or creating a domain-specific programming language (make-like, cmake-like, ...),
or extracting commands and dependencies to create derivated files (Makefiles, Visual studio, ..)

A few examples are provided to illustrate the range of possibilities:
* overview:        how to create a custom file using the waf framework to perform a simple build
* parser:          how to add a parser for a domain-specific language
* noscript:        infer what to build from given files, use no script file
* makefile_dumper: create a makefile corresponding to the current build, extracting as many dependencies as possible
* nostate:         use timestamps only, and no build directory (very make-like)
* extpy:           a custom waf file able to read wscript files having the extension ".py"

Thomas Nagy, 2010-2016
