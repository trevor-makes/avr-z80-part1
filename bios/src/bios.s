	.module	bios
	.globl	_yield
	.globl	_millis
	.globl	_getchar
	.globl	_putchar
	.globl	_putstr
	.globl	_putbcd
	.globl	_hide_cursor
	.globl	_show_cursor
	.globl	_clear_screen
	.globl	_set_cursor
	.globl	_erase_chars
	.globl	_set_rendition
	.globl	_srand
	.globl	_rand

_init = 0x100 ; defined in crt0

	.area	_HEADER (ABS)

	.org	0xD8
; memory locations shared between Arduino and Z80
_stack_ptr:	.ds 2	; 0xD8
_yield_flg:	.ds 1	; 0xDA
_read_reg:	.ds 1	; 0xDB
_clock_reg:	.ds 2	; 0xDC
_write_ptr:	.ds 2	; 0xDE
_write_buf:	.ds 32	; 0xE0
_write_end = .	; 0x100

YIELD_EXIT = 0
YIELD_FLUSH = 1
YIELD_BREAK = 2

	.org 	0x00
	; if yield flag is not 0, resume from previous halt
	ld	A, (_yield_flg)
	or	A
	ret	NZ
	; otherwise jump to init vector
	jp _init

	.org	0x08
flush:
	ld	A, #YIELD_FLUSH
; void yield(uint8_t) __sdcccall(1)
; takes code (0=exit, 1=flush, 2=break) in A
_yield::
	ld	(_yield_flg), A
	halt

	; TODO 2 unused bytes here

	.org	0x10
; void break()
_break::
	push	AF
	ld	(_stack_ptr), SP
	push	HL
	jr	break_ext

	.org	0x18

	; TODO unused reset vector

	.org	0x20
; print ANSI CSI "\e["
	ld	A, #0x1B
	rst	0x38
	ld	A, #'['
	jr	_putchar

	.org	0x28
; void putbcd(uint8_t out) __sdcccall(1)
; uint8 out in A
_putbcd::
	; tmp = out
	ld	HL, #bcdtmp
	ld	(HL), A
	; putchar('0' | tmp >> 4)
	ld	A, #'0' ; 0x30
	jr	putbcd_ext

	.org	0x30
; void putstr(char* out) __sdcccall(1)
; char* out in HL
_putstr::
	; while (*out != 0)
	ld	A, (HL)
	or	A
	ret	Z
	; putchar(*out++)
	rst 0x38 ; call	_putchar
	inc HL
	jr	_putstr

	.org	0x38
; void putchar(char out) __sdcccall(1)
; char out in A
_putchar::
	; *write_ptr++ = out
	push	HL
	ld	HL, (_write_ptr)
	ld	(HL), A
	inc	HL
	ld	(_write_ptr), HL
	; if (write_ptr != write_end) return
	; NOTE just comparing low byte since buf size is less than 256
	; TODO assert write_end-write_buf <= 256
	ld	A, #<_write_end
	cp	L
	pop	HL
	ret	NZ
	; else
	jr	flush

; continued from _break reset vector
break_ext:
	push	BC
	push	DE
	exx
	ex	AF, AF'
	push	AF
	push	HL
	push	BC
	push	DE
	exx
	ex	AF, AF'
	push	IX
	push	IY
	ld	A, #YIELD_BREAK
	call	_yield
	ld	SP, (_stack_ptr)
	pop	AF
	ret

; continued from _putbcd reset vector
putbcd_ext:
	; putchar('0' | tmp >> 4)
	rld
	rst 0x38 ; call _putchar
	; putchar('0' | tmp & 0xf)
	ld	A, #'0' ; 0x30
	rld
	jr	_putchar
bcdtmp: .ds 1

; char getchar() __sdcccall(1)
; return char in A
_getchar::
	ld	HL, #_read_reg
	; if (read_reg == -1) flush()
	ld	A, (HL)
	inc	A
	jr	NZ, skip_flush
	rst	0x08	; flush
skip_flush:
	ld	A, (HL)
	ld	(HL), #0xFF	; read_reg = -1
	ret

; void hide_cursor()
_hide_cursor::
	rst	0x20	; print "\e["
	ld	HL, #dectcem_l
	jr	_putstr	; putstr("?25l")
dectcem_l:
	.asciz	"?25l"

; void show_cursor()
_show_cursor::
	rst	0x20	; print "\e["
	ld	HL, #dectcem_h
	jr	_putstr	; putstr("?25h")
dectcem_h:
	.asciz	"?25h"

; void clear_screen()
_clear_screen::
	rst	0x20	; print "\e["
	ld	A, #'2'
	rst	0x38	; putchar('2')
	ld	A, #'J'
	jr	_putchar	; putchar('J')

; void set_cursor(uint8 row, uint8_t col) __sdcccall(1)
; take row (BCD) in A, col (BCD) in L
_set_cursor::
	ld	B, A
	ld	C, L
	rst	0x20	; print "\e["
	ld	A, B
	rst	0x28	; putbcd(row)
	ld	A, #';'
	rst	0x38	; putchar(';')
	ld	A, C
	rst	0x28	; putbcd(col)
	ld	A, #'H'
	jr	_putchar	; putchar('H')

; void erase_chars(uint8 count) __sdcccall(1)
; take count (BCD) in A
_erase_chars::
	ld	B, A
	rst	0x20	; print "\e["
	ld	A, B
	rst	0x28	; putbcd(count)
	ld	A, #'X'
	jr	_putchar	; putchar('X')

; void set_rendition(uint8 attr) __sdcccall(1)
; take attr (BCD) in A
_set_rendition::
	ld	B, A
	rst	0x20	; print "\e["
	ld	A, B
	rst	0x28	; putbcd(attr)
	ld	A, #'m'
	jr	_putchar	; putchar('m')

; uint16_t millis() __sdcccall(1)
; return uint16 in DE
_millis::
	; flush to get fresh millis from Arduino
	rst	0x08	; flush
	ld	DE, (_clock_reg)
	ret

; void srand(uint16_t seed) __z88dk_fastcall
; take seed in HL
_srand::
	set 7, H	; seed |= 0x8000
	ld (_rand+1), HL
	ret

; uint16_t rand() __z88dk_fastcall
; http://www.retroprogramming.com/2017/07/xorshift-pseudorandom-numbers-in-z80.html
; return in HL
_rand::
	; NOTE rand and srand keep state in the following instruction
	ld hl,#1

	ld a,h
	rra
	ld a,l
	rra
	xor h
	ld h,a
	ld a,l
	rra
	ld a,h
	rra
	xor l
	ld l,a
	xor h
	ld h,a

	ld (_rand+1),hl
	ret
