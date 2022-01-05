#ifndef _TASK_STORAGE_
#define _TASK_STORAGE_

#include "hal.h"
#include "ff.h"

#define EV_SD_INS       EVENT_MASK(0)
#define EV_SD_WRITE       EVENT_MASK(1)
#define EV_SD_READ       EVENT_MASK(2)


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
  event_source_t evs_insertion;
    
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