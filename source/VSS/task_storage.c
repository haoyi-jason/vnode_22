#include "ch.h"
#include "hal.h"
#include "task_storage.h"
#include "ff.h"




struct _runTime{
  uint8_t state;
  thread_reference_t trp;
  thread_t *self;
  bool cardReady;
  FIL f;
  char readFileName[32];
  uint32_t readSize;
};

struct _nvmParam{
  uint8_t flag;
};

static struct _runTime runTime, *storage_runTime;
static struct _nvmParam nvmParam, *storage_nvmParam;

static struct SDConfig sdConfig = {
  &SDCD1,
  GPIOB,10
};
static FSDriver fsDriver;

static void read_capacity(FSDriver *dev)
{
  if(palReadPad(dev->config->ins_port,dev->config->ins_pad) == PAL_HIGH){
    sdcDisconnect(dev->config->dev);
    dev->cardReady = false;
  }else{
    sdcConnect(dev->config->dev);
    FRESULT err = f_mount(&dev->FileSystem,"0",1);
    if(err != FR_OK){
      sdcDisconnect(dev->config->dev);
      dev->cardReady = false;
    }
    else{
      dev->cardReady = true;
    }
  }
}

static void sd_ins_handler(void *arg)
{
  (void*)arg;
  
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_SD_INS);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waFS,2048);
static THD_FUNCTION(procFS,p)
{
  // start gpio
  fsDriver.config = &sdConfig;
  palSetPadMode(sdConfig.ins_port, sdConfig.ins_pad, PAL_MODE_INPUT_PULLUP);
  // set int handler
  palSetLineCallback(PAL_LINE(sdConfig.ins_port, sdConfig.ins_pad), sd_ins_handler,NULL);
  palEnableLineEvent(PAL_LINE(sdConfig.ins_port, sdConfig.ins_pad),PAL_EVENT_MODE_FALLING_EDGE);
  
  // start fs
  sdcStart(fsDriver.config->dev,NULL);
  // check sd capacity if ins port is low
  read_capacity(&fsDriver);
  
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_SD_INS){
      read_capacity(&fsDriver);
    }
    if(evt & EV_SD_WRITE){
      
    }
    
    if(evt & EV_SD_READ){
      
    }
  }
  
  
}


/* Exported functions */

bool fsReady(){
  return runTime.cardReady;
}

uint32_t capacity()
{
  return fsDriver.capacity;
}

int8_t fs_write(uint8_t *d, uint16_t sz)
{
  if(fsDriver.buf_in_use) return -1;
  
  if(sz > SD_BUF_SIZE) return -1;
  
  memcpy(fsDriver.buffer, d,sz);
  fsDriver.bufferSize = sz;
  fsDriver.buf_in_use = 1;
  chEvtSignal(runTime.self, EV_SD_WRITE);
  return 0; 
}

void fs_set_read_file(char *fileName)
{
  memcpy(runTime.readFileName,fileName, strlen(fileName));
  
}

uint32_t fs_read(uint8_t *d, uint32_t offset, uint16_t sz)
{
  chEvtSignal(runTime.self,EV_SD_READ);
  chSysLock();
  chThdSuspend(runTime.trp);
  chSysUnlock();  
}

int8_t fs_init()
{
  runTime.self = chThdCreateStatic(waFS,sizeof(waFS),NORMALPRIO,procFS,NULL);
  chSysLock();
  msg_t ret = chThdSuspend(&runTime.trp);
  chSysUnlodk();
  
  if(ret != MSG_OK) return -1;
  
  return 0;
}



