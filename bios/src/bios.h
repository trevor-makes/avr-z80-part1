#pragma once

#include <stdint.h>

// TODO make this an inline xor A/rst 8?
void exit();

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

// Send VT220 DECTCEM sequence
void hide_cursor();

// Send VT220 DECTCEM sequence
void show_cursor();

// Send ANSI ED sequence
void clear_screen();

// Send ANSI CUP sequence with row and col as BCD
void set_cursor(uint8_t row, uint8_t col) __sdcccall(1);

// Send ANSI ECH sequence with count as BCD
void erase_chars(uint8_t count) __sdcccall(1);

// Send ANSI SGR sequence with attr as BCD
void set_rendition(uint8_t attr) __sdcccall(1);

// Set seed for rand()
void srand(uint16_t seed) __sdcccall(1);

// 16-bit xorshift pseudorandom number generator
// NOTE using z88dk_fastcall to return in HL instead of DE
uint16_t rand() __z88dk_fastcall;

// Attributes for set_rendition
enum SGR {
  SGR_RESET  = 0x00,
  SGR_BOLD   = 0x01,
  FG_BLACK   = 0x30,
  FG_RED     = 0x31,
  FG_GREEN   = 0x32,
  FG_YELLOW  = 0x33,
  FG_BLUE    = 0x34,
  FG_MAGENTA = 0x35,
  FG_CYAN    = 0x36,
  FG_WHITE   = 0x37,
  FG_DEFAULT = 0x39,
  BG_BLACK   = 0x40,
  BG_RED     = 0x41,
  BG_GREEN   = 0x42,
  BG_YELLOW  = 0x43,
  BG_BLUE    = 0x44,
  BG_MAGENTA = 0x45,
  BG_CYAN    = 0x46,
  BG_WHITE   = 0x47,
  BG_DEFAULT = 0x49,
};
