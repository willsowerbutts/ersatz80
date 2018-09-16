; Larson Scanner
; Bounce a pattern side-to-side on the 12 user-controlled LEDs
; Useful to demonstrate speeding the clock up and down

        ld hl, 1        ; try 1, 3, 0x55 -- any non-zero value
        ld b, 0         ; b=0 -> shift left;  b=1 -> shift right
again:  ld a, l         ; first we output the pattern
        out (0x11), a   ; all 8 bits are used
        ld a, h
        out (0x12), a   ; low 8 bits used, top 8 bits ignored
        bit 0, b        ; test B bit 0: going left (0) or right (1)?
        jr nz, sright   ; jump if shifting right
        add hl, hl      ; else shift left 1 bit
        bit 3, h        ; is the bit at leftmost edge lit?
        jr dotest       ; jump to shared test code
sright: srl h           ; shift HL right 1 bit (high byte)
        rr l            ; ... (low byte)
        bit 0, l        ; is the bit at rightmode edge lit?
dotest: jr z, again     ; not yet -- shift again
        inc b           ; yes - switch directions (flip B bit 0)
        jr again        ; and continue shifting
