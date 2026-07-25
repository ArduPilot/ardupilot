#include "header.h"

.data

val:
 .long SOME_VALUE

.text
.align 2

// Multiply input value by 421...
.global mult10
.type mult10, @function
mult10:
	pushq    %rbp
	movq     %rsp,%rbp

	movabs val,%eax
	imull %edi,%eax

	popq     %rbp
	ret
