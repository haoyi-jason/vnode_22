#ifndef _BINCMD_SHELL_
#define _BINCMD_SHELL_

#include "hal.h"

#define CMD_STRUCT_SZ   8
#define MAGIC1          0xAB
#define MAGIC2          0xBA

#define MASK_CMD                0xa0
#define MASK_DATA               0x30
#define MASK_CMD_RET_OK         0x50
#define MASK_CMD_RET_ERR        0x51
#define MASK_CMD_RET_BUSY       0x52
#define MASK_CMD_RET_CFG        0x5F

typedef struct{
  uint8_t magic1;
  uint8_t magic2;
  uint8_t type;
  uint8_t pid;
  uint16_t len;
  uint16_t chksum;
}BinCommandHeader;

typedef void (*bincmd_t)(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);

typedef struct{
  BinCommandHeader header;
  bincmd_t func;
}BinShellCommand;

typedef struct{
  BaseSequentialStream *sc_channel;
  const BinShellCommand *sc_commands;
}BinShellConfig;

uint16_t checksum(uint8_t *data, uint16_t length);


THD_FUNCTION(binshellProc,p);

#endif