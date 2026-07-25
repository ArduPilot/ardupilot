from cpython.version cimport PY_VERSION
cimport cy_cxxtest

def pyhello():
    cy_cxxtest.hello()
    print("Compiled with python version %s" % PY_VERSION)

cdef public api void cy_hello():
    print("hello cython-world!")
