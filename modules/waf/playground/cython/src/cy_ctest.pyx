from cpython.version cimport PY_VERSION
cimport cy_ctest
#cimport commented_import

def pyhello():
    cy_ctest.hello()
    print("Compiled with python version %s" % PY_VERSION)
