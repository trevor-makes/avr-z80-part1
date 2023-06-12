	.module break_test
	.globl	_main

	.area	_CODE

_main::
	ld	A, #0xA1
	scf	; F = 01100101
	ex	AF, AF'
	ld	A, #0xA2
	sub	#0	; F = 10100010
	ex	AF, AF'
	ld	HL, #0xACC1
	ld	BC, #0xBC01
	ld	DE, #0xDE01
	exx
	ld	HL, #0xACC2
	ld	BC, #0xBC02
	ld	DE, #0xDE02
	exx
	ld	IX, #0xDEAD
	ld	IY, #0xBEEF
	rst	0x10	; break
	rst	0x10	; break
	ret
