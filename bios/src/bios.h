#pragma once

#include <stdint.h>

// TODO maybe replace exit/flush/break with a single sync call that takes code in A
void exit();

// Flush output buffer
void flush();

// Return elapsed time in milliseconds
uint16_t millis() __sdcccall(1);

// Read character from input, returning -1 (0xFF) if none
// Does not block as standard getchar does by default
// NOTE standard expects `int getchar()`
char getchar() __sdcccall(1);

// Write single character to output buffer
// NOTE standard expects `int putchar(int)`
void putchar(char out) __sdcccall(1);

// Write null-terminated string to output buffer
// Doesn't append '\n' or return int like puts
void putstr(const char* out) __sdcccall(1);

// Write 2-digit packed BCD
void putbcd(uint8_t out) __sdcccall(1);
