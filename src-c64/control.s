
.segment "EXEHDR"

transfer = $8000
 
     .word coldstart            ; coldstart vector
     .word warmstart            ; warmstart vector
     .byte $C3,$C2,$CD,$38,$30  ; "CBM8O". Autostart string

;* = $8009

.SEGMENT "CODE"

     ; Do not modify this loop
     ; This poked from IC directly
docopy:
     ; x = *800a
     ldx #$01
copyloop:
     dex
     ; a = *800d, *800e
     lda $8100,x
     ; a = *8010,*8011
     sta $0400,x
     txa
     bne copyloop
    rts

    ; * = $8016
command:
.byte 0

jump_address:
.byte $4c
.byte 0
.byte 0

; FIXME cannot use constants of form 0xffff
;#include "../transfer.h"

transfer_buffer = $8100

transfer_command = $dff0
TRANSFER_CMD_TRANSFER = 1
TRANSFER_CMD_TRANSFER_2 = 2
TRANSFER_CMD_RUN = 8
TRANSFER_CMD_JUMP = 9
TRANSFER_CMD_PRINT = 10
TRANSFER_CMD_READY = $c0

transfer_status = $dff1
TRANSFER_STATUS_READY = $c0
TRANSFER_STATUS_IDLE = $c1


warmstart:
     SEI
;     pha
;     txa
;     pha
;     tya
;     pha
     lda $01
     pha
     lda #$37
     sta $01

cmdloop:
     lda transfer_command
     and #$7f
    cmp #TRANSFER_CMD_TRANSFER
    bne :+
    ; docopy - Copy to RAM
    inc $d020
    lda #$33 ; hide I/O area at $d000
    sta $01
    jsr docopy
     lda #$37
     sta $01
    dec $d020
    jmp nmiexit

:
    cmp #TRANSFER_CMD_TRANSFER_2
    bne skipcopy2
    ; docopy2 - Copy to I/O
    inc $d020
    jsr docopy
    dec $d020
    jmp nmiexit

skipcopy2:
    cmp #TRANSFER_CMD_RUN
    bne skipbasic
    ; run basic
    ; FIXME copy this code to RAM
    lda #$ff
    sta $cc
    jsr $a659
    jmp $a7ae

skipbasic:
    cmp #TRANSFER_CMD_JUMP
    bne skipjump
    ; FIXME copy this code to RAM
    jmp jump_address

skipjump:
    cmp #TRANSFER_CMD_PRINT
    bne skipprint
    ldx #$0
l1:
    lda transfer_buffer,x
    beq l2
    jsr $ffd2
    inx
    bne l1
l2:

skipprint:
    cmp #TRANSFER_CMD_READY
    bne nmiexit
    lda #TRANSFER_STATUS_IDLE
    jmp nmiexit2

nmiexit:
    lda #TRANSFER_STATUS_READY
nmiexit2:
    sta transfer_command

     lda transfer_command
     bmi cmdloop

    pla
    sta $1
     pla
     tay
     pla
     tax
     pla
     rti
;    CLI
;    jmp $FE5E





 
coldstart:
     inc $d020
;     jmp $fcef

;     sei
     stx $d016
     jsr $fda3 ;Prepare IRQ
;     jsr $fd50 ;Init memory. Rewrite this routine to speed up boot process.

     LDA #$00
     TAY
coldstart2:
     STA $0002,Y
     STA $0200,Y
     STA $0300,Y
     INY
     BNE coldstart2
     LDX #$3C
     LDY #$03
     STX $B2
     STY $B3
     LDX #$00
     LDY #$a0
     STX $C1
     STY $C2
     JSR $FD8C
    ; continue normal boot
     jmp $fcf8
;     jsr $fd15 ;Init I/O
;     jsr $ff5b ;Init video
;     cli
;     inc $d020
;     jmp *-3
 

 

;* = $83ff                     ; fill up to -$9fff (or $bfff if 16K)
     .byte 0
 
