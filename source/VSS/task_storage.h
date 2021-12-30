#ifndef _TASK_STORAGE_
#define _TASK_STORAGE_

#include "hal.h"
#include "ff.h"

#define EV_SD_INS       EVENT_MASK(0)
#define EV_SD_WRITE       EVENT_MASK(1)
#define EV_SD_READ       EVENT_MASK(2)


#define SD_BUF_SIZE     1024


typedef struct FSDriver FSDriver;


struct SDConfig{
  SDCDriver *dev;
  ioportid_t ins_port;
  ioportmask_t ins_pad;
};

struct FSDriver{
  const struct SDConfig *config;
  FATFS FileSystem;
  DIR   currentDir;
  char  FileName;
  uint32_t capacity;
  uint8_t buffer[SD_BUF_SIZE];
  uint16_t bufferSize;
  uint8_t buf_in_use;
  bool cardReady;
};

#endif