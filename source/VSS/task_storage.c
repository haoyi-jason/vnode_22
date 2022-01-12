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

static  _SDCConfig sdConfig = {
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

//static THD_WORKING_AREA(waFS,2048);
//static THD_FUNCTION(procFS,p)
//{
//  // start gpio
//  fsDriver.config = &sdConfig;
//  palSetPadMode(sdConfig.ins_port, sdConfig.ins_pad, PAL_MODE_INPUT_PULLUP);
//  // set int handler
//  palSetLineCallback(PAL_LINE(sdConfig.ins_port, sdConfig.ins_pad), sd_ins_handler,NULL);
//  palEnableLineEvent(PAL_LINE(sdConfig.ins_port, sdConfig.ins_pad),PAL_EVENT_MODE_FALLING_EDGE);
//  
//  // start fs
//  sdcStart(fsDriver.config->dev,NULL);
//  // check sd capacity if ins port is low
//  read_capacity(&fsDriver);
//  
//  while(!chThdShouldTerminateX()){
//    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
//    if(evt & EV_SD_INS){
//      read_capacity(&fsDriver);
//    }
//    if(evt & EV_SD_WRITE){
//      
//    }
//    
//    if(evt & EV_SD_READ){
//      
//    }
//  }
//  
//  
//}


/* Exported functions */

bool fsReady(){
  return runTime.cardReady;
}

uint32_t capacity()
{
  return SDFS1.config->dev->capacity;
}

int8_t fs_write(uint8_t *d, uint16_t sz)
{
//  if(fsDriver.buf_in_use) return -1;
//  
//  if(sz > SD_BUF_SIZE) return -1;
//  
//  memcpy(fsDriver.buffer, d,sz);
//  fsDriver.bufferSize = sz;
//  fsDriver.buf_in_use = 1;
//  chEvtSignal(runTime.self, EV_SD_WRITE);
  return 0; 
}

void fs_set_read_file(char *fileName)
{
  memcpy(runTime.readFileName,fileName, strlen(fileName));
  
}

uint32_t fs_read(uint8_t *d, uint32_t offset, uint16_t sz)
{
  chEvtSignal(runTime.self,EV_SD_READ);
  //chSysLock();
  //chThdSuspendS(runTime.trp);
  //chSysUnlock();  
}

int8_t fs_init()
{
//  runTime.self = chThdCreateStatic(waFS,sizeof(waFS),NORMALPRIO,procFS,NULL);
//  chSysLock();
//  msg_t ret = chThdSuspendS(&runTime.trp);
//  chSysUnlock();
  
  sdfsObjectInit(&SDFS1);
  sdfsStart(&SDFS1, &sdConfig);
  
  //if(ret != MSG_OK) return -1;
  
  return 0;
}


/** Interface implementation **/

FSDriver SDFS1;

static size_t _write(void *ip, const uint8_t *bp, size_t n)
{
  FSDriver *sdfsp = (FSDriver*)ip;
  size_t sz = obqWriteTimeout(&((FSDriver*)ip)->oqueue,bp,n,TIME_INFINITE);
  obqPostFullBuffer(&sdfsp->oqueue, sz);
  return sz;
}

static size_t _read(void *ip, uint8_t *bp, size_t n)
{
  return ibqReadTimeout(&((FSDriver*)ip)->iqueue,bp,n,TIME_INFINITE);
}

static msg_t _put(void *ip, uint8_t b)
{
  return obqPutTimeout(&((FSDriver*)ip)->oqueue,b,TIME_INFINITE);
}

static msg_t _get(void *ip)
{
  return ibqGetTimeout(&((FSDriver*)ip)->iqueue,TIME_INFINITE);
}

static msg_t _putt(void *ip, uint8_t b, sysinterval_t timeout)
{
  return obqPutTimeout(&((FSDriver*)ip)->oqueue,b,timeout);
}

static msg_t _gett(void *ip, sysinterval_t timeout)
{
  return ibqGetTimeout(&((FSDriver*)ip)->iqueue,timeout);
}

static size_t _writet(void *ip, const uint8_t *bp, size_t n,sysinterval_t timeout)
{
  return obqWriteTimeout(&((FSDriver*)ip)->oqueue,bp,n,timeout);
}

static size_t _readt(void *ip, uint8_t *bp, size_t n,sysinterval_t timeout)
{
  return ibqReadTimeout(&((FSDriver*)ip)->iqueue,bp,n,timeout);
}

static msg_t _ctl(void *ip, unsigned int operation, void *arg)
{
  
  return MSG_OK;
}

static const struct SDFSDriverVMT vmt = {
  (size_t)0,
  _write,_read,_put,_get,
  _putt,_gett,_writet,_readt,
  _ctl
};

static void ibnotify(io_buffers_queue_t *bqp)
{
  
}

static void obnotify(io_buffers_queue_t *bqp)
{
  FSDriver *sdfsp = bqGetLinkX(bqp);
  //chSysLock();
  chEvtBroadcastFlagsI(&sdfsp->evs_insertion,EVENT_MASK(1));
 // chEvtSignalI(runTime.self,RSI_APP_EVENT_SPP_TX);
 // chSysUnlock();
}

static void sd_insert_handler(void *arg)
{
  FSDriver *dev = (FSDriver*)arg;
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&dev->evs_insertion,EVENT_MASK(0));
  chSysUnlockFromISR();
}

void sdfsListFile(FSDriver *sdfsp)
{
  //chSysLockFromISR();
  chEvtBroadcastFlags(&sdfsp->evs_insertion,EV_SD_LS);
  //chSysUnlockFromISR();
}

void sdReadFile(FSDriver *sdfsp)
{
  chEvtBroadcastFlags(&sdfsp->evs_insertion,EV_SD_READ);
}

void sdfsInit(void){}

void sdfsObjectInit(FSDriver *sdfsp)
{
  sdfsp->vmt = &vmt;
  osalEventObjectInit(&sdfsp->event);
  ibqObjectInit(&sdfsp->iqueue, true, sdfsp->ib, SD_BUF_SIZE, 1, ibnotify, sdfsp);
  obqObjectInit(&sdfsp->oqueue, true, sdfsp->ob, SD_BUF_SIZE, 1, obnotify, sdfsp);
}
void sdfsStart(FSDriver *sdfsp, const _SDCConfig *config)
{
  sdfsp->config = config;
  if(config->ins_port != NULL){
    palSetPadMode(config->ins_port,config->ins_pad, PAL_MODE_INPUT);
    
    // enable interrupt
    palSetLineCallback(PAL_LINE(config->ins_port, config->ins_pad), sd_insert_handler, sdfsp);
    palEnableLineEvent(PAL_LINE(config->ins_port, config->ins_pad), PAL_EVENT_MODE_BOTH_EDGES);             
  }
  if(config->dev != NULL){
    sdcStart(config->dev,NULL);
  }
  
  chEvtObjectInit(&sdfsp->evs_insertion);
  
  sdfs_loadCard(sdfsp);
}
void sdfsStop(void)
{
  
}

void sdfs_loadCard(FSDriver *dev)
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

size_t sdfs_write(FSDriver *dev, char *fileName, uint8_t *buff, size_t sz)
{
  if(!dev->cardReady) return 0;
  
  UINT SZ;
  FIL f;
  FRESULT res = f_open(&f,fileName,FA_WRITE | FA_OPEN_APPEND);
  if(res == FR_OK){
    res = f_write(&f,buff,sz,&SZ);
    SZ = f.obj.objsize;
    f_close(&f);
  }
  return SZ;
}

size_t sdfs_read(FSDriver *dev, char *fileName, uint32_t offset, size_t readCount)
{
  if(!dev->cardReady) return 0;
  
}




