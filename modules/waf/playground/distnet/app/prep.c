
#ifdef _MSC_VER
#define testEXPORT __declspec(dllexport)
#else
#define testEXPORT
#endif

testEXPORT int prepouet() {
	return 0;
}
