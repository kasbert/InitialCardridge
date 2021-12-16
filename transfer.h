
#ifndef TRANSFER_H
#define TRANSFER_H

#define TRANSFER_MAX 256

// Addresses in transfer.asm
#define TRANSFER_COUNT_LOCATION 0x0a // $800a
#define TRANSFER_SOURCE_LOCATION 0x0d // $800d
#define TRANSFER_TARGET_LOCATION 0x10 // $8010
//#define TRANSFER_CMD_ADDR 0x16
#define TRANSFER_JMP_LOCATION 0x18 // $8018

#define TRANSFER_CMD_ADDR 0xdff0 // Visible in $dffd and $8ffd
#define TRANSFER_STATUS_ADDR 0xdff1 // Visible in $dffc and $8ffc

#define TRANSFER_CMD_TRANSFER 1
#define TRANSFER_CMD_TRANSFER_2 2
#define TRANSFER_CMD_RUN 8
#define TRANSFER_CMD_JUMP 9
#define TRANSFER_CMD_PRINT 10


// FIXME use status
#define TRANSFER_CMD_READY 0xc0
#define TRANSFER_CMD_IDLE 0xc1

#define TO_C64_BUFFER1_ADDR  (0x8100)
#define TO_C64_BUFFER1  ((unsigned char *) TO_C64_BUFFER1_ADDR)
#define TO_C64_BUFFER2_ADDR  (0x8200)
#define TO_C64_BUFFER2  ((unsigned char *) TO_C64_BUFFER2_ADDR)

#define FROM_C64_BUFFER_ADDR  (0xdf00)
#define FROM_C64_BUFFER  ((unsigned char *) FROM_C64_BUFFER_ADDR)

#endif
