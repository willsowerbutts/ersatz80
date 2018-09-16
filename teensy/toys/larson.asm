; Larson Scanner
; Bounce a pattern side-to-side on the 12 user-controlled LEDs
; Useful to demonstrate speeding the clock up and down

    ld hl, 1        ; try 1, 3, 0x55 -- any non-zero value
    ld b, 0         ; b=0 -> shift left;  b=1 -> shift right
again:
    ; first we output the pattern
    ld a, l
    out (0x11), a   ; all 8 bits used
    ld a, h
    out (0x12), a   ; low 8 bits used, top 8 bits ignored
    ; are we going left or right?
    bit 0, b        ; test if B is 0 or 1
    jr nz, sright   ; 1? shift right
    add hl, hl      ; shift HL left 1 bit
    bit 3, h        ; bit at leftmost edge lit?
    jr z, again     ; not yet -- shift again
    inc b           ; yes - switch directions
    jr again        ; and shift again
sright:
    srl h           ; shift HL right
    rr l            ; by 1 bit
    bit 0, l        ; bit at rightmode edge lit?
    jr z, again     ; not yet -- shift again
    dec b           ; yes - switch directions
    jr again        ; and shift again
