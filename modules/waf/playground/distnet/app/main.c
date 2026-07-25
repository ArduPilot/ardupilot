
#ifdef _MSC_VER
#define testEXPORT __declspec(dllexport)
#else
#define testEXPORT
#endif

testEXPORT int pouet() {
	return prepouet();
}
