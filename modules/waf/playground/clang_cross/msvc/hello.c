#include <Windows.h>

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	WriteConsole(GetStdHandle(STD_OUTPUT_HANDLE), "Hello world!\n", 13, NULL, NULL);

	return 0;
}
