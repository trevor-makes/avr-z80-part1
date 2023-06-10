#include "bios.h"

uint8_t rand80() __naked {
__asm
// Receive random bits in HL
  call _rand

// Map 8-bits in L to (0-9) in lower 4 bits
// L = L * 5 / 128 = (L + (L >> 2)) >> 5
// A = L >> 2
  ld A, L
  srl A
  srl A
// A = (A + L) >> 1
  add L
  rra // rotate carry into high bit
// L = A >> 4
  and #0xF0
  rrca
  rrca
  rrca
  rrca
  ld L, A

// Mask-off (0-7) in upper 4 random bits from H
  ld A, H
  and #0x70

// Join and return upper and lower digits in A
  or L
  ret
__endasm;
}

uint8_t rand100() __naked {
__asm
// Receive random bits in HL
  call _rand

// Map 8-bits in L to (0-9) in lower 4 bits
// L = L * 5 / 128 = (L + (L >> 2)) >> 5
// A = L >> 2
  ld A, L
  srl A
  srl A
// A = (A + L) >> 1
  add L
  rra // rotate carry into high bit
// L = A >> 4
  and #0xF0
  rrca
  rrca
  rrca
  rrca
  ld L, A

// Map 8-bits in H to (0-9) in upper 4 bits
// H = H * 5 / 8 = (H + (H >> 2)) >> 1
// A = H >> 2
  ld A, H
  srl A
  srl A
// A = (A + H) >> 1
  add H
  rra // rotate carry into high bit
  and #0xF0

// Join and return upper and lower digits in A
  or L
  ret
__endasm;
}

void main() {
  srand(millis());
  for (uint16_t i = 0; i < 2000; ++i) {
    putbcd(rand100());
    putchar(' ');
  }
  putchar('\n');
}
