#pragma once

#include <stdint.h>

// TODO make this an inline xor A/rst 8?
void exit();

// Yield to Arduino: 0=exit, 1=flush, 2=break
void yield(uint8_t code) __sdcccall(1);

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

// Set seed for rand()
void srand(uint16_t seed) __sdcccall(1);

// 16-bit xorshift pseudorandom number generator
// NOTE using z88dk_fastcall to return in HL instead of DE
uint16_t rand() __z88dk_fastcall;
