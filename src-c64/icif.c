/* 
   icif (InitialCardridge Interface)

   Based on tapecart - a tape port storage pod for the C64

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


   tapecartif.c: Interface to the tapecart

*/

#include <6502.h>
#include <c64.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <peekpoke.h>
#include <string.h>
#include "minidelay.h"
#include "icif.h"

//#define TAPECART_CMDMODE_MAGIC 0xfce2

#define SENSE_LOOP_TIMEOUT 3000

static bool ic_waitcmdresult(uint8_t cmd)
{
  unsigned int timeout;

  /* wait until IC responds by setting cmd = c0 */
  timeout = SENSE_LOOP_TIMEOUT;
  while (timeout > 0 && CMD_IC_STATUS != cmd)
  {
    --timeout;
    tinydelay();
    __asm__("inc $d020");
    __asm__("dec $d020");
  }
  if (timeout == 0)
  {
    __asm__("inc $d020");
    return false;
  }
  return true;
}

bool ic_cmd(uint8_t cmd)
{
  while (CMD_TO_IC != CMD_READY)
    CMD_TO_IC = CMD_READY;
  ic_waitcmdresult(CMD_IDLE);
  while (CMD_TO_IC != cmd)
    CMD_TO_IC = cmd;
  return ic_waitcmdresult(CMD_READY);
}

void ic_param_u16(uint16_t value)
{
  IO_BUFFER[0] = value;
  IO_BUFFER[1] = value >> 8;
}

bool ic_cmdmode(void)
{
  bool result;

  IO_BUFFER[0] = 0xa5;
  if (IO_BUFFER[0] != 0xa5)
  {
    // Shared memory is not present
    return false;
  }

  SEI(); // TODO not needed

  result = ic_cmd(CMD_LED_ON);
  if (!result)
  {
    goto end;
  }
  result = ic_cmd(CMD_LED_OFF);
  if (!result)
  {
    goto end;
  }

end:
  CLI();
  return result;
}

// TODO optimise
static void filenamecpy(char *target, const char *source)
{
  uint8_t i;
  for (i = 0; i < FILENAME_LENGTH; ++i)
  {
    target[i] = source[i];
  }
  target[i] = 0;
}

static int elemCount; // Elements in buffer

bool ic_open_dir(char *path, char *new_path)
{
  bool result;
  filenamecpy(IO_BUFFER, path);
  result = ic_cmd(CMD_SD_OPEN_DIR);
  if (result)
  {
    filenamecpy(new_path, IO_BUFFER);
  }
  elemCount = ELEMS_PER_CMD;
  return result;
}

bool ic_next_dir_entry(DirElement *element)
{
  bool result;
  if (elemCount >= ELEMS_PER_CMD) {
    result = ic_cmd(CMD_SD_READ_DIR_FAST);
    if (!result) {
      return result;
    }
    elemCount = 0;
  }
  memcpy(element, IO_BUFFER + elemCount * sizeof(DirElement), sizeof(DirElement));
  elemCount++;
  return result;
}

bool ic_load_and_run_file(DirElement *element)
{
  memcpy(IO_BUFFER, element, sizeof(DirElement));
  return ic_cmd(CMD_SD_SELECT_FILE);
}
