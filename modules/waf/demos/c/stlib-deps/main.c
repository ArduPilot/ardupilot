
#include <stdio.h>

#include "external_vars.h"
#include "sum.h"
#include "diff.h"

int main()
{
    /* This should return to whatever the default value is. */
    print_value_of_k();
    sum(6);
    print_value_of_k();
    diff(8);
    print_value_of_k();
    sum(8);
    print_value_of_k();
    diff(6);
    print_value_of_k();

    return 0;
}
