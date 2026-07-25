#include <Python.h>

static int numargs=0;

static PyObject* emb_numargs(PyObject *self, PyObject *args)
{
    if(!PyArg_ParseTuple(args, ":numargs"))
        return NULL;
    return Py_BuildValue("i", numargs);
}

static PyMethodDef EmbMethods[] = {
    {"numargs", emb_numargs, METH_VARARGS,
     "Return the number of arguments received by the process."},
    {NULL, NULL, 0, NULL}
};


#if PY_VERSION_HEX >= 0x03000000

/* Python 3.x code */

static struct PyModuleDef embmodule = {
   PyModuleDef_HEAD_INIT,
   "emb",   /* name of module */
   "emb_doc", /* module documentation, may be NULL */
   -1,       /* size of per-interpreter state of the module,
                or -1 if the module keeps state in global variables. */
   EmbMethods
};

PyMODINIT_FUNC
PyInit_emb(void)
{
    (void) PyModule_Create(&embmodule);
}

#endif


int main(int argc, char *argv[])
{
#if PY_VERSION_HEX >= 0x03000000
    PyImport_AppendInittab("emb", PyInit_emb);
#endif

    Py_Initialize();
    numargs = argc;

#if PY_VERSION_HEX < 0x03000000
    Py_InitModule("emb", EmbMethods);
#endif

    PyRun_SimpleString("import emb; print('Number of arguments', emb.numargs())");
    Py_Finalize();
    return 0;
}
