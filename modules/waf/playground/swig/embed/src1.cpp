#include "Python.h"
#include "src1.h"

extern "C"
{
#if PY_VERSION_HEX >= 0x03000000
	void PyInit__swigdemo(void);
#else
	void init_swigdemo(void);
#endif
}

TestClass* TestClass::_instance = 0;

int main()
{
	Py_Initialize();
#if PY_VERSION_HEX >= 0x03000000
	PyInit__swigdemo();
#else
	init_swigdemo();
#endif

	/*FILE* file_py;
	  file_py = fopen(i_oFile.toLocal8Bit(), "r");
	  PyRun_SimpleFile(file_py, i_oFile.toLocal8Bit());
	  fclose(file_py);
	 */
	PyRun_SimpleString("import swigdemo, sys\nsys.stderr.write(str(swigdemo.TestClass.instance().test()))");
	Py_Finalize();
}

