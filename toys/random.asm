; Draw pseudo-random pattern on 12 user-controlled LEDs
; source: http://map.grauw.nl/sources/external/z80bits.html
;   -- "taken from an old ZX Spectrum game and slightly optimised"
        ld  de, 0x0000
again:  ld  a,d
        ld  h,e
        ld  l,253
        or  a
        sbc hl,de
        sbc a,0
        sbc hl,de
        ld  d,0
        sbc a,d
        ld  e,a
        sbc hl,de
        jr  nc,rand
        inc hl
rand:   ex  de, hl
		ld  c, 0x10
		out (c), e
		inc c
		out (c), d
		jr again
