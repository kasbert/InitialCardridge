/* tapecart - a tape port storage pod for the C64

   Copyright (C) 2013-2017  Ingo Korb <ingo@akana.de>
   All rights reserved.
   Idea by enthusi

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
   SUCH DAMAGE.


   commands.h: command interface from C64 to InitialCardridge.

*/

#ifndef COMMAND_H
#define COMMAND_H

typedef enum {
/*
  CMD_EXIT = 0,
  CMD_READ_DEVICEINFO,
  CMD_READ_DEVICESIZES,
  CMD_READ_CAPABILITIES,

  CMD_READ_FLASH  = 0x10,
  CMD_READ_FLASH_FAST,
  CMD_WRITE_FLASH,
  CMD_WRITE_FLASH_FAST, // FIXME: Not Yet Implemented
  CMD_ERASE_FLASH_64K,
  CMD_ERASE_FLASH_BLOCK,
  CMD_CRC32_FLASH,

  CMD_READ_LOADER = 0x20,
  CMD_READ_LOADINFO,
  CMD_WRITE_LOADER,
  CMD_WRITE_LOADINFO,
  */

  CMD_LED_OFF = 0x30,
  CMD_LED_ON,
  CMD_READ_DEBUGFLAGS,
  CMD_WRITE_DEBUGFLAGS,
  /*

  CMD_DIR_SETPARAMS = 0x40,
  CMD_DIR_LOOKUP,
  */
  
  /* SD commands */
  CMD_SD_OPEN_DIR = 0x80,
  CMD_SD_READ_DIR_FAST,
  CMD_SD_SELECT_FILE,

  CMD_READY = 0xc0,
  CMD_IDLE = 0xc1,
  
  /* internal use only */
  //CMD_RAMEXEC = 0xf0,
} command_t;

// FIXME use these for status
#define STATUS_READY 0xc0
#define STATUS_IDLE 0xc1
#define STATUS_ERR 0xc2
#define STATUS_READY_MORE_DATA 0xc3

#define IO_BUFFER_ADDR  (0xdf00)
#define IO_BUFFER  ((unsigned char *) IO_BUFFER_ADDR)

#define CMD_TO_IC_ADDR (0xdff2)
#define CMD_IC_STATUS_ADDR (0xdff3)
#define CMD_TO_IC (*(volatile unsigned char *) CMD_TO_IC_ADDR)
#define CMD_IC_STATUS (*(volatile unsigned char *) CMD_IC_STATUS_ADDR)


#define FILENAME_LENGTH 20
//#define LOADER_LENGTH   171

typedef enum {
  FILE_NONE = 0x00,
  FILE_DIR,

  FILE_PRG,
  FILE_TCRT,

  FILE_TAP,
  FILE_D64,

  FILE_UNKNOWN = 0xFF
} file_t;

typedef struct direlement {
    uint32_t size;
    char name[FILENAME_LENGTH+1];
    uint8_t type;
} DirElement;

// 8 * (4 + 20 + 1 + 1) = 208
#define ELEMS_PER_CMD 8

#endif
