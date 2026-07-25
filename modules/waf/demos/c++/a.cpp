
#ifdef _MSC_VER
#	define testshlib_EXPORT __declspec(dllexport)
#else
#	define testshlib_EXPORT
#endif

extern testshlib_EXPORT void foo();

static const int truc=5;

void foo() { }

