/* Hello World Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stddef.h>
#include <stdio.h>
#include "pico/stdlib.h"

int main(int argc, char *argv[]);

void app_main()
{
    stdio_init_all();
    printf("Hello, world!\n");
    sleep_ms(1000);
    printf("Hello, world again!\n");
    // while (true) {
    //     printf("Hello, world!\n");
    //     sleep_ms(1000);
    // }
    main(0, NULL);
}
