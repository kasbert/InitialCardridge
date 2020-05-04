; fastMemory.s by deathshadow
; some simple routines to move things around quicker
; wee bit faster than memcpy or memset since it doesn't
; include support for >255 byte copies.

; the 'or/and' functions being an essential additon to allow faster
; blitting of transparent fonts and non-hardware sprites.

   .export       _fastOr, _fastAnd, _fastCopy, _fastSet, _fastSetOr, _fastSetAnd
   .importzp     sp, ptr1, ptr2

; --------------------------------

; void* __fastcall__ fastOr (void* dest, const void* src, unsigned char count);

_fastOr:
   jsr  fastWriteGetParams
fastOrloop:
   lda  (ptr1),y
   ora  (ptr2),y
   sta  (ptr2),y
   dey
   bne  fastOrloop
   rts

; --------------------------------

; void* __fastcall__ fastAnd (void* dest, const void* src, unsigned char count);

_fastAnd:
   jsr  fastWriteGetParams
fastAndLoop:
   lda  (ptr1),y
   and  (ptr2),y
   sta  (ptr2),y
   dey
   bne  fastAndLoop
   rts

; --------------------------------

; void* __fastcall__ fastCopy (void* dest, const void* src, unsigned char count);

_fastCopy:
   jsr  fastWriteGetParams
fastCopyLoop:
   lda  (ptr1),y
   sta  (ptr2),y
   dey
   bne  fastCopyLoop
   rts

; --------------------------------

; void* __fastcall__ fastSet (void* dest, unsigned char value, unsigned char count);
_fastSet:
   jsr fastSetGetParams
fastSetLoop:
   sta (ptr1),y
   dey
   bne fastSetLoop
   rts

; --------------------------------

; void* __fastcall__ fastSetOr (void* dest, unsigned char value, unsigned char count);

_fastSetOr:
   jsr fastSetGetParams
   tax
fastSetOrLoop:
   ora (ptr1),y
   sta (ptr1),y
   txa
   dey
   bne fastSetOrLoop
   rts

; ----------------

; void* __fastcall__ fastSetAnd (void* dest, unsigned char value, unsigned char count);

_fastSetAnd:
   jsr fastSetGetParams
   tax
fastSetAndLoop:
   and (ptr1),y
   sta (ptr1),y
   txa
   dey
   bne fastSetAndLoop
   rts

; --------------------------------

; CC65 passes __FASTCALL__ parameters thus:
;   count  A
;   src    SP   : SP+1  { low byte, high byte }
;   dest   SP+2 : SP+3  { low byte, high byte }

fastWriteGetParams:
; remember, X is off limits in fastWriteGetParams since we
; stored A, our count, in it while testing to see if we even
; need to run any of this.


   tax
   beq  fastWriteSkipPopRTS
   ldy  #0
   lda  (sp),y
   sta  ptr1
   iny
   lda  (sp),y
   sta  ptr1+1
   iny
   lda  (sp),y
   sta  ptr2
   iny
   lda  (sp),y
   sta  ptr2+1
   iny
   tya
   clc
   adc  sp
   sta  sp ; thankfully store leaves carry flag alone
   bne  fastWriteAdjustCounts
   inc  sp+1

fastWriteAdjustCounts:
   ; subtract 1 to account for 'count' being +1
   lda  ptr1
   bne  fastWriteSkipDec1
   dec  ptr1+1
fastWriteSkipDec1:
   dec  ptr1

   ; subtract 1 to account for 'count' being +1
   lda  ptr2
   bne  fastWriteSkipDec2
   dec  ptr2+1
fastWriteSkipDec2:
   dec  ptr2

; move X into Y for loop
   txa
   tay
   rts

; --------------------------------

fastWriteSkipPopRTS:
; pop 2 bytes so we basically can skip one RTS
   pla
   pla
fastWriteSkip:
   ; clean up the stack
   clc
   lda #4
   adc sp
   bne fastWriteSkipReturn
   inc sp+1
fastWriteSkipReturn:
   rts

; --------------------------------

fastSetGetParams:
   tax
   beq  fastGetSkipPopRTS
   ldy  #0
   lda  (sp),y
   iny
   pha
   lda  (sp),y
   iny
   sta  ptr1
   lda  (sp),y
   iny
   sta  ptr1+1
   tya
   clc
   adc  sp
   sta  sp
   bne  fastSetAdjustCounts
   inc  sp+1
fastSetAdjustCounts:
   ; subtract 1 to account for 'count' being +1
   lda  ptr1
   bne  fastSetSkipDec
   dec  ptr1+1
fastSetSkipDec:
   dec  ptr1
   txa
   tay
   pla
   rts

; --------------------------------

fastGetSkipPopRTS:
; pop 2 bytes so we basically can skip one RTS
   pla
   pla
fastGetSkip:
   ; clean up the stack
   clc
   lda #3
   adc sp
   bne fastGetSkipReturn
   inc sp+1
fastGetSkipReturn:
   rts 