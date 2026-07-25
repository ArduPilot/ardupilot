#include <stdio.h>
#include <stdlib.h>

#include <pkg1/exLibC/exLibC.hpp>

int main(int argc, char *argv[]) {
	printf("Hello world!\n");
	if (argc < 2) {
		printf("Too few parameters passed!\n");
	} else {
		int val=atoi(argv[1]);
		printf("Result is: %d\n",check_smaller(val));
	}

	return 0;
}
