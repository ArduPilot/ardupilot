#########
Example 2
#########

:purpose: demonstrate rst processing through rst2latex

Generated PDF Picture
#####################

.. figure:: test.pdf
   :align: center
   :width: 50%

   This figure shows a pdf file that was auto-generated
   (test.pdf).

Generated PNG Picture
#####################

.. figure:: test.png
   :align: center
   :width: 50%

   This figure shows a pdf file that was auto-generated
   (test.png).

Generated rst Content
#####################


.. include:: generated.rst

Generated CSV Content
#####################

.. csv-table::
   :file: generated.csv

Generated LaTeX Content (raw Directive)
#######################################

.. raw:: latex
   :file: generated.tex


