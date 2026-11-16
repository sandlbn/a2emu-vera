; rwts.s -- Disk II raw sector I/O for cc65 programs (slot 6, drive 1).
; Only the timing-critical parts are here: head stepping, finding a sector's
; address field, reading the 343 data nibbles and writing a data field with
; the real drive's timing (a nibble every 32 cycles, 40 for 10-bit sync).
; The 6-and-2 encoding is done in C (a2fs.c) in NIBBUF.
;
;   rw_track, rw_sector       the sector to access (physical sector number)
;   rw_init()                 motor on, read mode, recalibrate to track 0
;   rw_rawread()  -> 0 = ok   NIBBUF[0..342] = the data field's nibbles
;   rw_rawwrite() -> 0 = ok   writes NIBBUF[0..349]: D5 AA AD, 343 nibbles, DE AA EB, FF
;   rw_off()                  motor off
; Errors: 1 = sector not found, 2 = no data field, 3 = write protected.

        .setcpu "6502"
        .export _rw_init, _rw_off, _rw_rawread, _rw_rawwrite
        .export _rw_track, _rw_sector

SLOT16  = $60           ; slot 6
PHASE0  = $C080         ; + phase*2 (off) / +1 (on), indexed by X = SLOT16
MOTOFF  = $C088
MOTON   = $C089
DRIVE1  = $C08A
Q6L     = $C08C         ; read data / shift
Q6H     = $C08D         ; load latch / sense write protect
Q7L     = $C08E         ; read mode
Q7H     = $C08F         ; write mode
NIBBUF  = $9400         ; 2 pages, page aligned: the write loops must not cross pages

.segment "BSS"
_rw_track:  .res 1
_rw_sector: .res 1
curhalf:    .res 1      ; head position in half-tracks
target:     .res 1
addr:       .res 4      ; checksum, sector, track, volume (read backwards)
tmp:        .res 1
count:      .res 2

.segment "CODE"

_rw_init:
        ldx     #SLOT16
        lda     MOTON,x
        lda     DRIVE1,x
        lda     Q7L,x           ; read mode
        lda     Q6L,x
        lda     #80             ; unknown position: assume the far end and step back
        sta     curhalf
        lda     #0
        jsr     seek
        lda     #0
        tax
        rts

_rw_off:
        ldx     #SLOT16
        lda     MOTOFF,x
        rts

; seek to the track in A, one half-track at a time: next phase on, settle,
; previous phase off
seek:   asl     a
        sta     target
@loop:  lda     curhalf
        cmp     target
        beq     @done
        bcc     @out
        dec     curhalf         ; towards track 0
        lda     curhalf
        jsr     phon
        lda     curhalf
        clc
        adc     #1
        jsr     phoff
        jmp     @loop
@out:   inc     curhalf         ; towards the hub
        lda     curhalf
        jsr     phon
        lda     curhalf
        sec
        sbc     #1
        jsr     phoff
        jmp     @loop
@done:  ldx     #SLOT16
        rts

phon:   jsr     phx
        lda     PHASE0+1,x
        lda     #$56            ; ~20 ms
        jmp     wait
phoff:  jsr     phx
        lda     PHASE0,x
        rts
phx:    and     #3              ; X = SLOT16 + phase*2
        asl     a
        ora     #SLOT16
        tax
        rts

; the monitor's WAIT ($FCA8), (26 + 27A + 5A^2) / 2 us: a copy, because cc65
; programs run with the language card RAM, not the ROM, at $D000-$FFFF
wait:   sec
@outer: pha
@inner: sbc     #1
        bne     @inner
        pla
        sbc     #1
        bne     @outer
        rts

; seek to rw_track and find rw_sector's address field.  C = 0: found, the head is
; just past its "DE AA" epilogue and X = SLOT16; C = 1: not found.
locate: lda     _rw_track
        jsr     seek
        lda     #0
        sta     count
        lda     #$60            ; ~24K nibbles: almost 4 revolutions
        sta     count+1
@next:  dec     count
        bne     @d5
        dec     count+1
        beq     @fail
@d5:    lda     Q6L,x
        bpl     @d5
@isd5:  cmp     #$D5
        bne     @next
@aa:    lda     Q6L,x
        bpl     @aa
        cmp     #$AA
        bne     @isd5
@96:    lda     Q6L,x
        bpl     @96
        cmp     #$96
        bne     @isd5
        ldy     #3              ; volume, track, sector, checksum in 4-and-4
@odd:   lda     Q6L,x
        bpl     @odd
        sec
        rol     a
        sta     tmp
@even:  lda     Q6L,x
        bpl     @even
        and     tmp
        sta     addr,y
        dey
        bpl     @odd
@de:    lda     Q6L,x           ; epilogue first, checks after: the data field
        bpl     @de             ; follows in a few nibbles
        cmp     #$DE
        bne     @next
@eaa:   lda     Q6L,x
        bpl     @eaa
        cmp     #$AA
        bne     @next
        lda     addr+3          ; volume ^ track ^ sector = checksum
        eor     addr+2
        eor     addr+1
        cmp     addr
        bne     @next
        lda     addr+2
        cmp     _rw_track
        bne     @next
        lda     addr+1
        cmp     _rw_sector
        bne     @next
        clc
        rts
@fail:  sec
        rts

_rw_rawread:
        jsr     locate
        bcc     @prolog
        lda     #1
        ldx     #0
        rts
@prolog:
        ldy     #32             ; the data prologue is within a few nibbles
@next:  dey
        beq     @nodata
@d5:    lda     Q6L,x
        bpl     @d5
@isd5:  cmp     #$D5
        bne     @next
@aa:    lda     Q6L,x
        bpl     @aa
        cmp     #$AA
        bne     @isd5
@ad:    lda     Q6L,x
        bpl     @ad
        cmp     #$AD
        bne     @isd5
        ldy     #0              ; 343 nibbles: 16 cycles each, well within 32
@r1:    lda     Q6L,x
        bpl     @r1
        sta     NIBBUF,y
        iny
        bne     @r1
@r2:    lda     Q6L,x
        bpl     @r2
        sta     NIBBUF+256,y
        iny
        cpy     #87
        bne     @r2
        lda     #0
        tax
        rts
@nodata:
        lda     #2
        ldx     #0
        rts

_rw_rawwrite:
        jsr     locate
        bcc     @wp
        lda     #1
        ldx     #0
        rts
@wp:    lda     Q6H,x
        lda     Q7L,x           ; bit 7 = write protected
        bpl     @go
        lda     Q6L,x
        lda     #3
        ldx     #0
        rts
@go:    jmp     wfield

; The data field writer.  Cycle counts: every STA Q6H (the latch load) is 32
; cycles after the previous one, 40 after a sync byte.  Its own segment is page
; aligned, so no branch below crosses a page (that would add a cycle).
.segment "RWTSW"
wfield: lda     #$FF
        sta     Q6H,x           ; (no effect in read mode on a real drive)
        sta     Q7H,x           ; write mode, latch = $FF: sync 1
        ora     Q6L,x           ; 4
        ldy     #4              ; 2
        bit     $00             ; 3 -> 40 with the loop's 26 + STA 5
@sync:  nop                     ; 26 cycles
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        sta     Q6H,x           ; 5  sync 2..5
        ora     Q6L,x           ; 4
        dey                     ; 2
        bne     @sync           ; 3: 4+2+3+26+5 = 40
        nop                     ; 2 (BNE not taken) + 23 + LDA 4 + STA 5 = 40
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        bit     $00
@w1:    lda     NIBBUF,y        ; 4  first page: Y = 0..255
        sta     Q6H,x           ; 5
        ora     Q6L,x           ; 4
        iny                     ; 2
        beq     @w1x            ; 2
        nop                     ; 12
        nop
        nop
        nop
        nop
        nop
        jmp     @w1             ; 3: 4+2+2+12+3+4+5 = 32
@w1x:   nop                     ; 14: 4+2+3 (BEQ taken)+14+4+5 = 32
        nop
        nop
        nop
        nop
        nop
        nop
@w2:    lda     NIBBUF+256,y    ; 4  second page: Y = 0..93
        sta     Q6H,x           ; 5
        ora     Q6L,x           ; 4
        iny                     ; 2
        cpy     #94             ; 2
        beq     @w2x            ; 2
        nop                     ; 10
        nop
        nop
        nop
        nop
        jmp     @w2             ; 3: 4+2+2+2+10+3+4+5 = 32
@w2x:   nop                     ; let the last $FF go out
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        lda     Q7L,x           ; back to read mode
        lda     Q6L,x
        lda     #0
        tax
        rts
        .assert >wfield = >*, error, "rwts.s: the timed writer crosses a page"
