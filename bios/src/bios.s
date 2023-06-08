	.module	bios
	.globl	_exit
	.globl	_flush
	.globl	_millis
	.globl	_getchar
	.globl	_putchar
	.globl	_putstr

_init = 0x100 ; defined in crt0

; TODO use end of header segment (up to 0x100) for BIOS registers?
; TODO use .org/.blkb/.blkw instead of =?
;.org 0x747A
;_yield_flg: .blkb 1
;_read_reg: .blkb 1
;_clock_reg: .blkw 1
;.assume _write_buf - 0x7F80
;.assume _write_end - 0x7F00
_yield_flg = 0x7F7A
_read_reg  = 0x7F7B
_clock_reg = 0x7F7C
_write_ptr = 0x7F7E
_write_buf = 0x7F80
_write_end = 0x7F00

	.area	_HEADER (ABS)

	.org 	0
	; if yield flag is not 0, resume from previous halt
	ld	A, (_yield_flg)
	or	A
	; TODO should this restore AF from flush? if so, have to branch across 0x08 vector
	ret	NZ
	jp _init

	.org	0x08
; void exit()
; TODO take int param?
_exit::
	; clear yield flag and halt to terminate
	ld	A, #0 ;xor	A
	ld	(_yield_flg), A
	halt

	.org	0x10
; void flush()
_flush::
	; set yield flag and halt to yield
	; TODO need to push AF?
	ld	A, #1
	ld	(_yield_flg), A
	halt

	.org	0x18
; uint16_t millis() __sdcccall(1)
; return uint16 in DE
_millis::
	; get fresh millis from Arduino and return it
	;call	_flush
	rst	0x10
	;ld	HL, (_clock_reg) ;__z88dk_fastcall
	ld	DE, (_clock_reg)
	ret

	.org	0x20
; char getchar() __sdcccall(1)
; return char in A
_getchar::
	ld	HL, #_read_reg

	; if (read_reg == -1) flush()
	ld	A, (HL)
	inc	A
	call	Z, _flush

	ld	A, (HL)
	ld	(HL), #0xFF	; read_reg = -1
	;ld	L, A ;__z88dk_fastcall
	ret

	.org	0x30
; void putchar(char out) __sdcccall(1)
; char out in A
_putchar::
	; *write_ptr++ = out
	;ld	A, L ;__z88dk_fastcall
	ld	HL, (_write_ptr)
	ld	(HL), A
	inc	HL
	ld	(_write_ptr), HL

	; if (write_ptr != write_end) return else flush()
	; NOTE just comparing low byte since buf size is less than 256
	; TODO assert write_end-write_buf <= 256
	ld	A, #<_write_end
	cp	L
	ret	NZ
	jr	_flush

; void print(char* out) __sdcccall(1)
; char* out in HL
_putstr::
	; while (*out != 0)
	ld	A, (HL)
	or	A
	ret	Z

	; putchar(*out++)
	push	HL
	ld	L, (HL)
	;call	_putchar
	rst 0x30
	pop HL
	inc HL

	jr	_putstr

; TODO how much other crap can we cram in page 0? priority bcd and rand, maybe draw?
; TODO what about debug break vector?
; TODO is it worth wasted padding to align to reset vectors?
