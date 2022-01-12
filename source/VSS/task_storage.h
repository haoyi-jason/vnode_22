#ifndef _TASK_STORAGE_
#define _TASK_STORAGE_

#include "hal.h"
#include "ff.h"

#define EV_SD_INS       EVENT_MASK(0)
#define EV_SD_WRITE       EVENT_MASK(1)
#define EV_SD_READ       EVENT_MASK(2)
#define EV_SD_LS        EVENT_MASK(3)        

enum fs_actions{
  FS_OP_MOUNT,
  FS_OP_OPENDIR,
  FS_OP_LISTDIR,
  FS_OP_READFILE,
  FS_OP_WRITEFILE
};

#define FILE_OP_OK         0x10
#define FILE_OP_ERR        0x20

#define FILE_OP_CHDIR       0x10
#define FILE_LIST          0x01
#define FILE_OPEN          0x02
#define FILE_READ          0x03
#define FILE_WRITE         0x04
#define FILE_REMOVE        0x05
//#define FILE_CHDIR         0x06
#define FILE_CLOSE         0x07

#define SD_BUF_SIZE     1024


typedef struct FSDriver FSDriver;


typedef struct {
  SDCDriver *dev;
  ioportid_t ins_port;
  ioportmask_t ins_pad;
}_SDCConfig;

#define _sdc_data \
  _base_asynchronous_channel_data \
  input_buffers_queue_t iqueue; \
  output_buffers_queue_t oqueue; \
  uint8_t ib[SD_BUF_SIZE]; \
  uint8_t ob[SD_BUF_SIZE]; \
  const _SDCConfig *config; \
  uint32_t capacity; \
  bool cardReady; \
  FATFS FileSystem; \
  event_source_t evs_insertion; \
  uint8_t fileName[32]; \
  uint32_t readOffset;
    
#define _fs_driver_methods     \
  _base_asynchronous_channel_methods

struct SDFSDriverVMT{
  _fs_driver_methods
};

struct FSDriver{
  const struct SDFSDriverVMT *vmt;
  _sdc_data
};
      
//* External declarations **/
extern FSDriver SDFS1;

void sdfsInit(void);
void sdfsObjectInit(FSDriver *sdfsp);
void sdfsStart(FSDriver *sdfsp, const _SDCConfig *config);
void sdfsStop(void);
void sdfs_loadCard(FSDriver *dev);
size_t sdfs_write(FSDriver *dev, char *fileName, uint8_t *buff, size_t sz);


#endif