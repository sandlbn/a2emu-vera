; boot.s -- Disk II boot sector (track 0, sector 0): loads the program that
; follows it on the disk and jumps to it.  No DOS: it reuses the P5A PROM's
; sector reader at $Cn5C, which re-enters us at $0801 after every sector ($27
; and $3D already advanced).  The program is stored in physical-sector order:
; track 0 sectors 1-15, then track 1 sectors 0-15, and so on.
;
; Built once (cl65 -t none --start-addr 0x800); a2fs mkimage fills in the
; parameter block at $0804 for each program, so no assembler is needed then.

        .setcpu "6502"

BUFLO   := $26          ; PROM: destination buffer
BUFHI   := $27
SLOT16  := $2B          ; PROM: boot slot * 16
SECTOR  := $3D          ; PROM: physical sector wanted
TRACK   := $41          ; PROM: track expected in the address field
BASHIMEM:= $73          ; Applesoft HIMEM
WAIT    := $FCA8        ; monitor delay (the ROM is in at boot)

        .byte   1               ; the PROM loads just this sector, then JMP $0801
        jmp     entry

; parameter block at $0804, patched by a2fs mkimage
nsect:  .byte   0               ; sectors to load
load:   .word   $0900           ; load address (page aligned) = entry point
himem:  .word   $9400           ; Applesoft HIMEM: cc65's crt0 puts the C stack there
                                ;   when there is no ProDOS

entry:  lda     started
        bne     next
        inc     started
        lda     SLOT16          ; read vector = $Cn5C
        lsr     a
        lsr     a
        lsr     a
        lsr     a
        ora     #$C0
        sta     readv+1
        lda     #0
        sta     BUFLO
        sta     TRACK
        lda     load+1
        sta     BUFHI
        lda     #1              ; first program sector: track 0, sector 1
        sta     SECTOR
        lda     nsect
        bne     read
        beq     done            ; nothing to load

next:   dec     nsect
        beq     done
        lda     SECTOR
        cmp     #16
        bcc     read
        lda     #0              ; past sector 15: next track
        sta     SECTOR
        jsr     step
        jsr     step
        inc     TRACK
read:   ldx     SLOT16          ; the PROM reader indexes the slot with X
        jmp     (readv)

done:   ldx     SLOT16
        lda     $C088,x         ; motor off
        lda     himem
        sta     BASHIMEM
        lda     himem+1
        sta     BASHIMEM+1
        jmp     (load)

; one half-track outwards: next phase on, settle, previous phase off
step:   inc     halft
        lda     halft
        jsr     phase
        lda     $C081,x
        lda     #$56            ; ~20 ms
        jsr     WAIT
        lda     halft
        sec
        sbc     #1
        jsr     phase
        lda     $C080,x
        rts
phase:  and     #3              ; X = slot*16 + phase*2
        asl     a
        ora     SLOT16
        tax
        rts

readv:  .word   $C05C
started:.byte   0
halft:  .byte   0               ; current half-track (track * 2)
