#include <Python.h>

static PyObject *ping() {
   return PyUnicode_FromString("pong");
}

static PyMethodDef methods[] = {
    {"ping", ping, METH_VARARGS, "Ping function"},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "ping",
    NULL, /* no docs */
    -1,
    methods
};

PyMODINIT_FUNC
PyInit_baz_ext(void)
{
    return PyModule_Create(&module);
}

