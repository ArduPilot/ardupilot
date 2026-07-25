%{
#include "a.h"
%}


%include "a.h"

%module test_swig_waf
%pythoncode "python/include.py"
