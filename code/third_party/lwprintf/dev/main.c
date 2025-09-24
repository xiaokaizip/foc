#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lwprintf/lwprintf.h"
#include "windows.h"

extern int test_printf(void);

int
main(void) {
    return test_printf();
}
