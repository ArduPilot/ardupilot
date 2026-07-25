#########
Example 1
#########

:purpose: demonstrate basic rst processing

Static SVG Picture
##################

.. figure:: test.svg
   :align: center
   :width: 50%

   This is a static svg file

Generated PNG Picture
#####################

.. figure:: test.png
   :align: center
   :width: 50%

   This figure is a png file generated
   (in the build folder of course).

   As of 2013-08-25, rst2pdf and docutils do not have
   some way of specifying include paths, in which includes
   are searched.
   So the figure is broken.
   We could generate the figure in the source directory,
   but I prefer not having a figure at all for the sake
   of cleanleness.
   A docutils patch is available here though:
   http://zougloub.eu/git/overlay-zougloub/plain/dev-python/docutils/files/docutils-0.11-search-path.patch


Generated rst Content
#####################

.. include:: generated.rst

Generated CSV Content
#####################

As of 2013-08-25, waf doesn't scan the csv-table directive
including an external file. You'll have to specify
the dependency manually, use build groups, or count on your luck.

.. csv-table::
   :file: generated.csv




Generated Content using raw Directive
#####################################

The following line will be empty if not using the HTML
output.

.. raw:: html
   :file: generated.html

Did you see something?

As of 2013-08-25, waf doesn't scan the raw directive
including an external file. You'll have to specify
the dependency manually, use build groups, or count on your luck.

