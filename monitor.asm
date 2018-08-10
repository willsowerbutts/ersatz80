; 2018-08-10: WRS - based on monitor from socz80

; hardware register addresses
UART0_STATUS:  equ 0x00 ; [7: RX READY] [6: TX BUSY] [6 unused bits]
UART0_DATA:    equ 0x01
STACK_INIT:    equ 0x1000 ; stack (grows downwards, runtime movable)
INPUT_BUFFER:  equ 0x0F80 ; input buffer (grows upwards, runtime movable)

        ; ROM is mapped in at 32KB+
        ; a JMP 0x8000 is placed in early RAM before the system starts
        org 0x8000 ; early ROM location (also Z80 reset vector)
        di ; disable interrupts (for those arriving via RST 0 rather than CPU reset)
boot:
warmboot:
        ld sp, STACK_INIT   ; load stack pointer to point to 1 byte past top of memory
        ld iy, INPUT_BUFFER ; set default input buffer location 256 bytes below top of stack

        ; put a reset vector in place to jump back into us
        ld a, 0xc3 ; jmp instruction
        ld (0), a
        ld hl, boot
        ld (1), hl

        ; print our greeting
        ld hl, greeting
        call outstring

        ; flush UART FIFO
fifoflush:
        in a, (UART0_DATA)     ; read byte from UART
        in a, (UART0_STATUS)   ; check status
        bit 7, a               ; test if data waiting
        jr nz, fifoflush       ; keep flushing if there is

monitor_loop:
        ld hl, monitor_prompt
        call outstring

        push iy ; HL=IY
        pop hl
        call instring
        call outnewline

        ld ix, cmd_table
testnextcmd:
        push iy ; HL=IY
        pop hl
        ; test for end of string
        ld a, (hl)
        cp 0
        jr z, monitor_loop
        ; load string address
        ld e, (ix+0)
        ld d, (ix+1)
        call cmdcompare
        jp z, runcmd
        ; advance ix to point to next command
        ld bc, 4
        add ix, bc
        ; list is terminated with a zero, check for that
        ld a, (ix+0)
        or (ix+1)
        cp 0
        jr nz, testnextcmd
        ; end of list but no command found
        ld hl, what_msg
        call outstring
        jr monitor_loop
runcmd: ; run command at ix+2, ix+3
        ld e, (ix+2)
        ld d, (ix+3)
        push de
        pop ix
        jp (ix)

do_help:
        ld hl, help_msg
        call outstring
        jp monitor_loop

; parse command line to display I/O register contents. call with (HL) pointing into input buffer after "in "
do_in:
        call skipspaces
        call parsehexorfail
        ; DE now contains the I/O address to read
        push de
        call outde
        ld a, 0x3d
        call outchar
        ld a, 0x20
        call outchar
        pop bc
        in a, (c) ; asserts full 16-bit address bus
        call outcharhex
        call outnewline
        call skipspaces
        ld a, (hl)
        cp 0
        jp z, monitor_loop  ; don't use ret, we get here by a jump and not a call.
        jr do_in  ; next address

; write to I/O register
do_out:
        call skipspaces
        call parsehexorfail
        ; DE now contains the I/O address
        push de
nextio:
        call skipspaces
        call parsehexorfail
        ; DE now contains the value to write
        ld a, d ; check D=0
        cp 0
        jr nz, invalid
        pop bc ; I/O address
        out (c), e
        inc bc ; next I/O address
        call skipspaces
        ld a, (hl)
        cp 0
        jp z, monitor_loop
        push bc
        jr nextio

; write to memory command
do_wm:
        call skipspaces
        call parsehexorfail
        push de
        pop ix    ; put the memory address in ix
wm_nextbyte:
        call skipspaces
        ld a, (hl)
        cp 0
        jp z, monitor_loop ; end of string; go back to the monitor
        call parsehex
        ; data value to write to now in DE
        ; confirm D byte is zero ie we're dealing with a single byte
        ld a, d
        cp 0
        jr nz, invalid
        ; do the write
        ld (ix+0), e
        ; move to next memory address for the next byte
        inc ix
        jr wm_nextbyte

invalid:
        ld hl, invalid_msg
        call outstring
        jp monitor_loop ; don't use ret

; parse command line to display memory contents. call with (HL) pointing into input buffer after "dm "
do_dm:
        call skipspaces
        call parsehexorfail
        push de ; start address
        call skipspaces
        call parsehex
        ld a, e
        or d
        cp 0
        jr nz, dmgo
        ; ok DE=0, which is a problem. we'll dump 64K -- takes a while!
        ld e, 1 ; force DE=0x0001
dmgo:
        pop  hl ; start address
        call dumpmem
        jp monitor_loop ; don't use ret, we get here by a jump and not a call.

; display memory starting at (HL) length DE
dumpmem:
        ; start by writing the memory address
        call outhl
        ld a, 0x3a
        call outchar

        ; now print the bytes from memory separated by spaces
dumpnb: ld a, 0x20
        call outchar
        ld a, (hl)
        call outcharhex

        ; next byte
        inc hl
        dec de

        ; got to the end?
        ld a, e
        or d
        jr z, dumpdone

        ; newline?
        ld a, l
        and 0x0f
        cp 0
        jr nz, dumpnb

dumpnl: call outnewline
        jr dumpmem

dumpdone:
        call outnewline
        ret

; jump to given address
do_run:
        call skipspaces
        call parsehexorfail
        ; address to jump to is now in (DE)
        ex de, hl ; put it in HL where we can use it
        ld bc, warmboot
        push bc  ; push return address (monitor entry point)
        jp (hl)
        ; we never get here.

do_cp:
        ; copy memory
        call skipspaces
        call parsehexorfail
        ; src is now in DE
        push de
        call skipspaces
        call parsehexorfail
        ; dst is now in DE
        push de
        call skipspaces
        call parsehexorfail
        ; len is now in DE
        push de
        ; set up for LDIR:
        ; DE dst
        ; HL src
        ; BC count
        pop bc
        pop de
        pop hl
        ldir ; copy copy copy!
        jp monitor_loop

do_sp:
        call skipspaces
        ld a, (hl) ; check if we're at the end of string?
        cp 0
        jr z, showsp
        ; if not, load SP first
        call parsehex
        ; new SP is now in DE
        push de
        pop hl
        ld sp, hl
showsp:
        ; you can't read SP directly but you can add it to HL (go figure)
        ld hl, sp_msg
        call outstring
        ld hl, 0
        add hl, sp
        call outhl
        call outnewline
        jp monitor_loop

do_buf:
        call skipspaces
        ld a, (hl) ; check if we're at the end of string?
        cp 0
        jr z, showbuf
        ; if not, load SP first
        call parsehex
        ; new SP is now in DE
        push de
        pop iy
showbuf:
        ld hl, buf_msg
        call outstring
        push iy
        pop hl
        call outhl
        call outnewline
        jp monitor_loop

; compare strings at (HL) and (DE), input buffer in (HL) and command in (DE).
; return with flags NZ -> inequality
; return with flags  Z -> equal ie string at (DE) is a prefix of (HL)
cmdcompare:
        ld a, (de)
        cp 0
        ret z    ; end of string at (DE) -> return zero flag
        cp (hl)
        ret nz   ; non-matching -> return nonzero flag
        inc de
        inc hl
        jr cmdcompare

; outstring: Print the string at (HL) until 0 byte is found
; destroys: AF HL
outstring:
        ld a, (hl)     ; load next character
        and a          ; test if zero
        ret z          ; return when we find a 0 byte
        call outchar
        inc hl         ; next char please
        jr outstring

; print the string at (HL) in hex (continues until 0 byte seen)
outstringhex:
        ld a, (hl)     ; load next character
        and a          ; test if zero
        ret z          ; return when we find a 0 byte
        call outcharhex
        ld a, 0x20 ; space
        call outchar
        inc hl         ; next char please
        jr outstringhex

; output a newline
outnewline:
        ld a, 0x0d   ; output newline
        call outchar
        ld a, 0x0a
        call outchar
        ret

outhl:  ; prints HL in hex. Destroys AF.
        ld a, h
        call outcharhex
        ld a, l
        call outcharhex
        ret

outbc:  ; prints BC in hex. Destroys AF.
        ld a, b
        call outcharhex
        ld a, c
        call outcharhex
        ret

outde:  ; prints DE in hex. Destroys AF.
        ld a, d
        call outcharhex
        ld a, e
        call outcharhex
        ret

; print the byte in A as a two-character hex value
outcharhex:
        push bc
        ld c, a  ; copy value
        ; print the top nibble
        rra
        rra
        rra
        rra
        call outnibble
        ; print the bottom nibble
        ld a, c
        call outnibble
        pop bc
        ret

; print the nibble in the low four bits of A
outnibble:
        and 0x0f ; mask off low four bits
        cp 10
        jr c, numeral ; less than 10?
        add 0x07 ; start at 'A' (10+7+0x30=0x41='A')
numeral:add 0x30 ; start at '0' (0x30='0')
        call outchar
        ret

; outchar: Wait for UART TX idle, then print the char in A
; destroys: AF
outchar:
        push bc
        ld b, a
        ; wait for transmitter to be idle
ocloop: in a, (UART0_STATUS)
        bit 6, a
        jr nz, ocloop   ; loop while busy

        ; now output the char to serial port
        ld a, b
        out (UART0_DATA), a
        pop bc
        ret

; instring: Read a typed string into (HL) terminate with 0 byte when CR seen (leaves (HL) pointing at the 0 byte)
;           Returns length of string (excluding 0 terminator) in C
;           Special handling for newline, backspace and non-printing characters.
;           If first character received is '@' it is not inserted into the buffer and the buffer is not echoed back (for bulk data load etc)
; DOES NOT PERFORM BUFFER LENGTH CHECKING!
instring:
        ld c, 0        ; we use c to remember our string length
        call incharwait
        cp '@'
        jr nz, gotchar
        ; we've got an @ so we should do a bulk load
        call outchar
        jr instringbulk
instringloop:
        call incharwait ; reads single byte into A
gotchar:; test for cr/lf
        cp 0x0d
        jr z, cr
        cp 0x0a
        jr z, cr
        ; test for backspace
        cp 0x08
        jr z, backspace
        cp 0x7f
        jr z, backspace
        ; test for non-printing characters
        cp 0x20 ; < 0x20?
        jp c, instringloop 
        cp 0x7f ; > 0x7f?
        jp nc, instringloop
        ; store the character in the buffer
        ld (hl), a
        inc hl
        inc c
        call outchar ; echo back the character typed
        jr instringloop
backspace:
        ld a, c
        cp 0
        jr z, instringloop ; cannot backspace past the start
        dec hl
        dec c
        ld a, 0x08 ; move back
        call outchar
        ld a, 0x20 ; print space
        call outchar
        ld a, 0x08 ; move back again
        call outchar
        jr instringloop
cr:     
        ld a, 0
        ld (hl), a
        ret

; a fast bulk version of instring
; does not echo back characters received after the first space, in order to avoid overflowing the rx fifo
; does not perform full terminal handling
instringbulk:
        call incharwait
        cp 0x0d
        jr z, cr
        cp 0x0a
        jr z, cr
        ld (hl), a
        inc hl
        cp 0x20
        jr z, isbspace
        call outchar
        jr instringbulk
isbspace:
        call incharwait
        cp 0x0d
        jr z, cr
        cp 0x0a
        jr z, cr
        ld (hl), a
        inc hl
        jr isbspace



; incharwait: Wait for UART RX, return character read in A
; destroys: A
incharwait:
        in a, (UART0_STATUS)
        bit 7, a
        jr z, incharwait   ; loop while no character received
        in a, (UART0_DATA)
        ret

; advance HL until it no longer points at a space
skipspaces:
        ld a, (hl)
        cp 0x20
        ret nz
        inc hl
        jr skipspaces

; convert character in A to upper case
toupper:
        cp 0x61    ; < "a" ?
        ret c
        cp 0x7b
        ret nc
        and 0x5f
        ret

; if at end of string, jump to invalid, else continue
; and run parsehex
parsehexorfail:
        ld a, (hl)
        cp 0
        jp z, invalid
        ; fall through to parsehex
; read hex digits pointed to by hl, load into DE
; does not work when fed with non-hex digits!!
parsehex:
        ld d, 0
        ld e, 0
parsemorehex:
        ld a, (hl)
        cp 0x30 ; not a hex character?
        ret c
        ; convert to nibble
        cp 0x40 ; <= '9'?
        jr c, phnumeral
        sub 7
phnumeral: sub 0x30
        and 0xf
        ; A now contains the nibble at (hl) ie bits are 0000nnnn
        ; now we do:
        ; D = D << 4 | E << 4
        ; E = E << 4 | A
        ; remember rl shifts 9 bits total; 8 bits in the register plus the carry bit from flags register
        push bc ; don't clobber B
        ld b, 4 ; prepare to do this four times
phshift:
        and a   ; clear carry flag (so we shift zeros into the low four bits of E)
        rl e    ; rotate C register left one bit, top bit moves into carry flag
        rl d    ; rotate B register left one bit, carry flag moves into lower bit
        djnz phshift ; repeat four times please
        pop bc
        or e    ; combine C and A
        ld e, a ; move A into C
        inc hl
        jr parsemorehex

greeting:           db "\r"
                    db "                    ___   ___  \r\n"
                    db " ___  ___   ___ ___( _ ) / _ \\ \r\n"
                    db "/ __|/ _ \\ / __|_  / _ \\| | | |\r\n"
                    db "\\__ \\ (_) | (__ / / (_) | |_| |\r\n"
                    db "|___/\\___/ \\___/___\\___/ \\___/ \r\n"
                    db "Z80 ROM Monitor (Will Sowerbutts, 2013-12-12)\r\n", 0
monitor_prompt:     db "Z80> ", 0
what_msg:           db "Error reduces\r\nYour expensive computer\r\nTo a simple stone.\r\n", 0
invalid_msg:        db "Errors have occurred.\r\nWe won't tell you where or why.\r\nLazy programmers.\r\n", 0 
mmu_header_msg:     db "Virtual (F8)\tPhysical (FC FD)\tFlags (FB)\r\n", 0
mmu_read_msg:       db "READ ", 0
mmu_write_msg:      db "WRITE ", 0
mmu_ptr_msg:        db "17th Page Pointer (FA) = ",0
rboot_msg:          db "Loading stage 2 bootstrap from RAM disk to ", 0
rboot_fail_msg:     db "Bad magic number. Gentlemen, please check your RAM disks.\r\n", 0
sp_msg:             db "SP=", 0
buf_msg:            db "BUF=", 0
type_check_msg:     db "Checking SPI flash type: ", 0
type_check_ok_msg:   db " (OK)\r\n", 0
type_check_fail_msg: db " FAIL! :(\r\n", 0
erasewarn_msg:      db "Erase RAM disk starting at page ", 0
readwarn1_msg:      db "Read RAM disk starting at page ", 0
readwarn2_msg:      db " from flash page ", 0
writewarn1_msg:     db "Write RAM disk starting at page ", 0
writewarn2_msg:     db " to flash page ", 0
genwarn_msg:        db " (y/n)?", 0
verifybad_msg:      db "Flash write verify failed :(\r\n", 0
help_msg:           db "Commands:\r\n"
                    db "\tdm addr [len]\t\t\tdisplay memory contents from addr for len (default 1) bytes\r\n"
                    db "\twm addr val [val...]\t\twrite bytes to memory starting at addr\r\n"
                    db "\tcp src dst len\t\t\tcopy len bytes from src to dst\r\n"
                    db "\trun addr\t\t\trun code at addr\r\n"
                    db "\tin addr\t\t\t\tread I/O port at addr, display result\r\n"
                    db "\tout addr val [val...]\t\twrite I/O port at addr with val\r\n"
                    db "\tsp [addr]\t\t\tshow stack pointer (and set to addr)\r\n"
                    db "\tbuf [addr]\t\t\tshow input buffer (and set to addr)\r\n"
                    db "\t@[cmd]\t\t\t\tPerform command without echo or terminal handling (bulk operations)\r\n"
                    db 0
cmd_buf:            db "buf", 0   ; no space at end (arg is optional)
cmd_cp:             db "cp ", 0
cmd_dm:             db "dm ", 0
cmd_help:           db "help", 0
cmd_help2:          db "?", 0
cmd_in:             db "in ", 0
cmd_out:            db "out ", 0
cmd_run:            db "run ", 0
cmd_sp:             db "sp",0     ; no space at end (arg is optional)
cmd_wm:             db "wm ", 0
cmd_table:          
                    dw cmd_cp, do_cp
                    dw cmd_dm, do_dm
                    dw cmd_help, do_help
                    dw cmd_help2, do_help
                    dw cmd_in, do_in
                    dw cmd_out, do_out
                    dw cmd_run, do_run
                    dw cmd_wm, do_wm
                    dw cmd_sp, do_sp
                    dw cmd_buf, do_buf
                    dw 0 ; terminate command table

; pad to size
;                    ds 0x10000 - $, 0xfe  ; this will be negative when the ROM exceeds 4K so the assembler will alert us to our excess.
