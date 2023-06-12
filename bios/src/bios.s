	.module	bios
	.globl	_yield
	.globl	_millis
	.globl	_getchar
	.globl	_putchar
	.globl	_putstr
	.globl	_putbcd
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
; void yield(uint8_t) __sdcccall(1)
; takes code (0=exit, 1=flush, 2=break) in A
_yield::
	ld	(_yield_flg), A
	halt

	; TODO 4 unused bytes here

	.org	0x10
; void break()
_break::
	push	AF
	ld	(_stack_ptr), SP
	push	HL
	jr	break_ext

	.org	0x18

	.org	0x20

	.org	0x28

	.org	0x30

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
	; else yield(1)
	ld	A, #YIELD_FLUSH
	jr	_yield

; char getchar() __sdcccall(1)
; return char in A
_getchar::
	ld	HL, #_read_reg

	; if (read_reg == -1)
	ld	A, (HL)
	inc	A
	jr	NZ, skip_flush
	; { yield(1) }
	ld	A, #YIELD_FLUSH
	rst	0x08

skip_flush:
	ld	A, (HL)
	ld	(HL), #0xFF	; read_reg = -1
	ret

; continued from _break reset vector above
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
	rst	0x08
	ld	SP, (_stack_ptr)
	pop	AF
	ret

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

; void putbcd(uint8_t out) __sdcccall(1)
; uint8 out in A
_putbcd::
	; tmp = out
	ld	HL, #bcdtmp
	ld	(HL), A

	; putchar('0' | tmp >> 4)
	ld	A, #'0' ; 0x30
	rld
	rst 0x38 ; call _putchar

	; putchar('0' | tmp & 0xf)
	ld	A, #'0' ; 0x30
	rld
	rst 0x38 ; call _putchar

	ret
bcdtmp: .ds 1

; uint16_t millis() __sdcccall(1)
; return uint16 in DE
_millis::
	; call yield(1) to get fresh millis from Arduino
	ld	A, #YIELD_FLUSH
	rst	0x08
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

; TODO how much other crap can we cram in page 0? priority bcd and rand, maybe draw?
; TODO what about debug break vector?
; TODO is it worth wasted padding to align to reset vectors?
