#include <stdio.h>

int mult10(int);

int main()
{
  int asm_val = mult10(2);
  printf("From ASM: %d\n", asm_val);
  return 0;
}
