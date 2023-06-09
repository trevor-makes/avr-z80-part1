#include "bios.h"

char name[32];

void main() {
    putstr("Enter your name: ");
    char *name_ptr = name;
    while (name_ptr != name + sizeof(name) - 1) {
        char c = getchar();
        if (c == 0xFF) {
            continue;
        } else if (c == '\n') {
            break;
        }

        putchar(c);
        *name_ptr++ = c;
    }
    putchar('\n');
    *name_ptr = '\0';

    uint16_t last_t = millis();
    for (uint8_t i = 5; i > 0; --i) {
        // Wait one second
        uint16_t t;
        do {
            t = millis();
        } while (t - last_t < 500);
        last_t = t;

        putstr("Hello, ");
        putstr(name);
        putchar('\n');
    }
}