#include "bios.h"

void game_loop();

void main() {
  hide_cursor();
  clear_screen();

  game_loop();

  set_rendition(SGR_RESET);
  clear_screen();
  show_cursor();
  set_cursor(1, 1);
}

uint8_t rand_row() __sdcccall(1);
uint8_t rand_col() __sdcccall(1);

uint8_t bcd_add(uint8_t a, uint8_t l) __sdcccall(1);
uint8_t bcd_sub(uint8_t a, uint8_t l) __sdcccall(1);

#define TAIL_MAX_LEN 80
uint8_t tail_rows[TAIL_MAX_LEN];
uint8_t tail_cols[TAIL_MAX_LEN];
uint8_t tail_index = 0;
uint8_t tail_length = 0;
uint8_t target_length = 6;

void shrink_tail() {
  // Erase tail graphic
  set_cursor(tail_rows[tail_index], tail_cols[tail_index]);
  set_rendition(BG_DEFAULT);
  putstr("  ");

  // Pop tail from ring buffer
  ++tail_index;
  if (tail_index == TAIL_MAX_LEN)
    tail_index = 0;
  --tail_length;
}

void grow_tail(uint8_t row, uint8_t col) {
  // Push head into ring buffer
  uint8_t head_index = tail_index + tail_length;
  if (head_index >= TAIL_MAX_LEN)
    head_index -= TAIL_MAX_LEN;
  tail_rows[head_index] = row;
  tail_cols[head_index] = col;
  ++tail_length;

  // Draw head graphic
  set_cursor(row, col);
  set_rendition(BG_GREEN);
  putstr("  ");
}

bool is_tail(uint8_t row, uint8_t col) {
  for (uint8_t i = 0; i < tail_length; ++i) {
    uint8_t index = tail_index + i;
    // Wrap index into circular buffer
    if (index >= TAIL_MAX_LEN) {
      index -= TAIL_MAX_LEN;
    }
    if (tail_rows[index] == row && tail_cols[index] == col) {
      return true;
    }
  }
  return false;
}

uint8_t apple_row; // BCD
uint8_t apple_col; // BCD

void new_apple() {
  // Get random position not on tail
  do {
    apple_row = rand_row();
    apple_col = rand_col();
  } while (is_tail(apple_row, apple_col));

  // Draw apple at new position
  set_cursor(apple_row, apple_col);
  set_rendition(BG_RED);
  putstr("  ");
}

uint8_t head_dir = KEY_RIGHT;
uint8_t head_row = 0x10; // BCD
uint8_t head_col = 0x01; // BCD

#define MIN_DELAY 60
uint8_t move_delay = 150; // millis per move

void game_loop() {
  uint16_t t_prev = millis();
  srand(t_prev);

  new_apple();

  for (;;) {
    uint8_t prev_dir = head_dir;
    char input;
    do {
      input = getchar();
      switch (input) {
        case KEY_UP:
          if (prev_dir != KEY_DOWN) {
            head_dir = KEY_UP;
          }
          break;
        case KEY_DOWN:
          if (prev_dir != KEY_UP) {
            head_dir = KEY_DOWN;
          }
          break;
        case KEY_RIGHT:
          if (prev_dir != KEY_LEFT) {
            head_dir = KEY_RIGHT;
          }
          break;
        case KEY_LEFT:
          if (prev_dir != KEY_RIGHT) {
            head_dir = KEY_LEFT;
          }
          break;
        case '\x08': // backspace
        case '\x7F': // delete
          __asm__("rst 0x10");
          break;
        case 'q':
          return;
      }
    } while (input != KEY_NONE);

    switch (head_dir) {
      case KEY_UP:
        head_row = bcd_sub(head_row, 0x01);
        break;
      case KEY_DOWN:
        head_row = bcd_add(head_row, 0x01);
        break;
      case KEY_RIGHT:
        head_col = bcd_add(head_col, 0x02);
        break;
      case KEY_LEFT:
        head_col = bcd_sub(head_col, 0x02);
        break;
    }

    // Game over if head eats tail
    if (is_tail(head_row, head_col)) {
      return;
    }

    grow_tail(head_row, head_col);
  
    // If an apple was eaten...
    if (head_row == apple_row && head_col == apple_col) {
      if (target_length < TAIL_MAX_LEN) {
        ++target_length;
      }
      if (move_delay > MIN_DELAY) {
        // roughly multiply by ~0.96
        move_delay = (move_delay >> 6) + (move_delay >> 5) + (move_delay >> 4)
          + (move_delay >> 3) + (move_delay >> 2) + (move_delay >> 1);
      }
      new_apple();
    }

    if (tail_length == target_length) {
      shrink_tail();
    }

    // Wait for next move
    uint16_t t_now;
    do {
      t_now = millis();
    } while ((t_now - t_prev) < move_delay);
    t_prev = t_now;
  }
}

uint8_t bcd_add(uint8_t a, uint8_t l) __sdcccall(1) __naked {
  a; l;
__asm
  add A, L
  daa // adjust result for BCD
  ret
__endasm;
}

uint8_t bcd_sub(uint8_t a, uint8_t l) __sdcccall(1) __naked {
  a; l;
__asm
  sub A, L
  daa // adjust result for BCD
  ret
__endasm;
}

uint8_t rand80() __sdcccall(1) __naked {
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

// return random BCD in [1...20]
uint8_t rand_row() __sdcccall(1) __naked {
__asm
  call _rand80
  and #0x1F // constraint to [0...19]
  inc A
  daa // adjust result for BCD
  ret
__endasm;
}

// return random odd BCD in [1...79]
uint8_t rand_col() __sdcccall(1) __naked {
__asm
  call _rand80
  or #0x01 // round to next largest odd number [1...79]
  ret
__endasm;
}
