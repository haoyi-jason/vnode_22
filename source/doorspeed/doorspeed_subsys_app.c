#include "ch.h"
#include "hal.h"
#include "vnode_app.h"
//#include "adxl355_dev.h"
//#include "adxl355_defs.h"
#include "bincmd_shell.h"
#include "nvm_config.h"
//#include "usbcfg.h"
#include "bootConfig.h"
#include "ylib/sensor/bmi160/bmi160_defs.h"
#include "ylib/sensor/bmi160/bmi160_dev.h"
#include "task_analog_input.h"



static struct{
  uint8_t flag;
  node_param_t nodeParam;
  adxl355_cfg_t adxlParam;
  module_setting_t moduleParam;
  imu_config_t imuParam;
  struct{
    uint16_t resistance;
    uint16_t beta;
    uint16_t beta_temp;
    uint16_t shunt_resistance;
  }ntc_config;
}nvmParam;

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

#define EV_ADXL_FIFO_FULL EVENT_MASK(0)
#define EV_IMU_FIFO_FULL EVENT_MASK(1)
#define EV_START_TRIGGER EVENT_MASK(2)

void cmd_config(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_start(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_stop(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_read(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_boot(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_idn(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);

BinShellCommand commands[] ={
  {{0xab,0xba,0xA2,0x00,0,0},cmd_config}, // node param
  {{0xab,0xba,0xA2,0x01,0,0},cmd_config}, // adxl param
  {{0xab,0xba,0xA2,0x02,0,0},cmd_config}, // imu param
  {{0xab,0xba,0xA2,0x40,0,0},cmd_config}, // module param
  {{0xab,0xba,0xA2,0xFF,0,0},cmd_config}, // sample interval
  {{0xab,0xba,0xA1,0x01,0,0},cmd_start}, // start
  {{0xab,0xba,0xA1,0x00,0,0},cmd_stop}, // stop
  {{0xab,0xba,0x01,0x00,0,0},cmd_read},
  {{0xab,0xba,0xAF,0x00,0,0},cmd_boot},
  {{0xab,0xba,0xA4,0x00,0,0},cmd_idn},
  {{0xab,0xba,0x01,0x00,0,0},NULL},
};

static const BinShellConfig shell_cfg = {
  (BaseSequentialStream *)&SD1,
  commands
};

typedef struct {
  uint8_t state;
  uint8_t *rxPtr;
  uint8_t *txPtr;
  uint8_t *bufEnd;
  uint8_t buffer[300];
  uint8_t buf2[100];
  uint8_t imuData[12];
  uint8_t imuInUse;
  mutex_t mutex;
  uint16_t rxSz;
  event_source_t es_sensor;
  event_listener_t el_sensor;
  struct {
    uint16_t ms_on;
    uint16_t ms_off;
  }ledBlink;
  thread_t *opThread;
  uint8_t activatedSensor;
  uint8_t sensorReady;
  thread_t *shelltp;
  virtual_timer_t vt;
  uint32_t timeout;
  thread_t *monitorThread;
  uint16_t pollIntervalMs;
  thread_t *self;
  thread_t *bmiThread;
  semaphore_t sem;
  systime_t tm_start,tm_stop;
  systime_t elapsed;
  uint8_t pat_offset;
  uint8_t usePattern;
}_runTime;

static _runTime runTime, *appRuntime;

static SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  0,
  SPI_CR1_BR_2
};

static SerialConfig serialCfg={
  234000
};

_imu_interface_t bmiInterface = {
  &SPID1,
  &spicfg
};

static _bmi160_config_t bmiConfig = {
  {0x0,BMI160_ACCEL_NORMAL_MODE,BMI160_ACCEL_RANGE_2G,0x0},
  {0x0,BMI160_GYRO_NORMAL_MODE,BMI160_GYRO_RANGE_2000_DPS,0x0},
  GPIOA,4,
  GPIOA,2,
  GPIOA,3
};

static BMI160Driver bmi160 = {
  &bmiInterface,
  &bmiConfig
};

const module_setting_t module_default = {
  0xBB,
  "DOORSPEED",
  0x12345678, 
  0x00000001,
  "Grididea-AT32",
  "DOORSPEED" // supported config
};
static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
  if(nvmSz > SZ_NVM_CONFIG){
    // error
    while(1);
  }
  nvm_flash_read(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
  if(nvmParam.flag != 0xAB){
    nvmParam.flag = 0xAB;
    nvmParam.nodeParam.activeSensor = SENSOR_ADXL355;
    nvmParam.nodeParam.commType = COM_IF_USB;
    nvmParam.nodeParam.opMode = OP_STREAM;
    
//    nvmParam.adxlParam.fs = 0x1;
//    nvmParam.adxlParam.odr = ADXL355_ODR_500;
//    nvmParam.adxlParam.hpf = 0;
    
    memcpy((uint8_t*)&nvmParam.moduleParam,(uint8_t*)&module_default, sizeof(module_setting_t));
    
    nvmParam.imuParam.accel.power = 0;
    nvmParam.imuParam.accel.odr = BMI160_ACCEL_ODR_1600HZ;
    nvmParam.imuParam.accel.range = BMI160_ACCEL_RANGE_2G;
    nvmParam.imuParam.accel.lpf = 0;
    
    nvmParam.imuParam.gyro.power = 0;
    nvmParam.imuParam.gyro.odr = BMI160_GYRO_ODR_1600HZ;
    nvmParam.imuParam.gyro.range = BMI160_GYRO_RANGE_2000_DPS;
    nvmParam.imuParam.gyro.lpf = 0;    
    nvm_flash_write(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
  }
}

void save_settings(uint8_t option)
{
  //chSysLock();
//  chSysDisable();
    nvm_flash_write(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,sizeof(nvmParam));
    //chSysUnlock();
  //  chSysEnable();
}

//void adxl_int_handler(void *arg)
//{
//  chSysLockFromISR();
//  chEvtBroadcastFlagsI(&runTime.es_sensor,EV_ADXL_FIFO_FULL);
//  chSysUnlockFromISR();
//}

static void conversion_cb(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.opThread, EVENT_MASK(1));
  chVTSetI(&runTime.vt,TIME_MS2I(runTime.pollIntervalMs),conversion_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waBMIReader,512);
static THD_FUNCTION(procBMIReader ,p)
{  
  if(bmi160_dev_init(&bmi160) == MSG_OK){
    runTime.sensorReady = SENSOR_BMI160;
  }
  memcpy((uint8_t*)&bmi160.imu.accel_cfg,(uint8_t*)&nvmParam.imuParam.accel,sizeof(nvmParam.imuParam.accel));
  memcpy((uint8_t*)&bmi160.imu.gyro_cfg,(uint8_t*)&nvmParam.imuParam.gyro,sizeof(nvmParam.imuParam.gyro));
  
  
  bmi160.imu.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  bmi160.imu.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
  
  bmi160_poweron(&bmi160);
  uint16_t READSZ;
  bool bStop = false;
  uint8_t buffer[16];
  while(!bStop)
  {
    if(chThdShouldTerminateX()){
      bStop = true;
    }
//    chSemWait(&runTime.sem);
    // read from gyro.z, accel.x/y/z
//    bmi160_single_read(&bmi160,runTime.imuData,8,&READSZ); 
    bmi160_single_read(&bmi160,buffer,12,&READSZ); 
    memcpy(runTime.imuData,&buffer[4],8);
//    chSemSignal(&runTime.sem);
    //chThdSleepMicroseconds(100);
  }
  chThdExit((msg_t)0);
}

uint16_t sin_x[15] = {
  4067,7431,9510,9945,8660,
  5877,2079,-2079,-5877,-8660,
  -9945,-9510,-7431,-4067
};

static THD_WORKING_AREA(waOperation,1200);
static THD_FUNCTION(procOperation ,p)
{
  BinCommandHeader *header;
  BaseSequentialStream *stream = p;
  size_t sz;
  uint8_t buf[320];
  uint16_t bufSz = 0;
  int32_t data[96];
  uint8_t *p_src,*p_dst;
  uint16_t bsz;
  systime_t t_start;
  uint8_t pktCount = 0;
  static uint8_t adxl_sta;
 
  uint8_t opMode = nvmParam.nodeParam.opMode;
  uint8_t activeSensor = nvmParam.nodeParam.activeSensor;
    
  bool bStop = false;

  runTime.ledBlink.ms_off = 500;
  runTime.ledBlink.ms_on = 500;
  uint32_t fifo_size;
  eventmask_t evt;
  runTime.rxSz = 0;
  
//  chEvtRegisterMask(&bmi160.evsource,&runTime.el_sensor,EV_IMU_FIFO_FULL );
//  runTime.time.scale.x = bmi160.lsb_accel;
//  runTime.time.scale.y = bmi160.lsb_accel;
//  runTime.time.scale.z = bmi160.lsb_accel;
  
  // todo: start a timer to periodically
  chVTObjectInit(&runTime.vt);
  chVTSet(&runTime.vt,TIME_MS2I(runTime.pollIntervalMs),conversion_cb,NULL);

  runTime.rxPtr = runTime.buffer;
  
  runTime.tm_start = chVTGetSystemTime();
  
  runTime.pat_offset = 0;
  // enable vout
  palSetPad(GPIOC,13);
  while(!bStop){
    if(chThdShouldTerminateX()){
      bStop = true;
    }
    evt = chEvtWaitAny(ALL_EVENTS);
    if(runTime.usePattern == 0){
      analog_input_read_packed(0xff, runTime.rxPtr);
      runTime.rxPtr += 8;
    }
    else{
      memcpy(runTime.rxPtr,&sin_x[runTime.pat_offset],2);
      runTime.rxPtr+=2;
      memcpy(runTime.rxPtr,&sin_x[runTime.pat_offset],2);
      runTime.rxPtr+=2;
      memcpy(runTime.rxPtr,&sin_x[runTime.pat_offset],2);
      runTime.rxPtr+=2;
      memcpy(runTime.rxPtr,&sin_x[runTime.pat_offset],2);
      runTime.rxPtr+=2;
      runTime.pat_offset++;
      if(runTime.pat_offset == 15) runTime.pat_offset = 0;
    }
    //chSemWait(&runTime.sem);
    memcpy(runTime.rxPtr, runTime.imuData,8);
    //chSemSignal(&runTime.sem);
    runTime.rxPtr += 8;
    runTime.rxSz += 16;
    

    palClearPad(GPIOC,14);
    if(runTime.rxSz >= 80){
      palSetPad(GPIOC,14);
      if(stream != NULL){
        memcpy(runTime.buf2,runTime.buffer,runTime.rxSz);
//        streamWrite(stream,runTime.buffer, runTime.rxSz);
        streamWrite(stream,runTime.buf2, runTime.rxSz);
//        runTime.tm_stop = chVTGetSystemTime();
      }
      runTime.rxPtr = runTime.buffer;
      runTime.rxSz = 0;
    }
    runTime.elapsed = TIME_I2US(chVTTimeElapsedSinceX(runTime.tm_start));
    runTime.tm_start = chVTGetSystemTime();
   // flags = chEvtGetAndClearFlags(&elSensor);
  }
  palClearPad(GPIOC,13);
  chThdExit((msg_t)0);
 // chEvtUnregister(&bmi160.evsource,&runTime.el_sensor);
}
static void keep_live(void *arg);

static void startTransfer(void)
{
  if(runTime.pollIntervalMs == 0)
    runTime.pollIntervalMs = 10;
  if(!runTime.opThread){
    chSemReset(&runTime.sem,1);
    analog_input_set_sample_interval(runTime.pollIntervalMs);
    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO-1,procOperation,&SD1);
    runTime.bmiThread = chThdCreateStatic(waBMIReader,sizeof(waBMIReader),NORMALPRIO-1,procBMIReader,NULL);
    //chVTSet(&runTime.vt,TIME_MS2I(200),keep_live,NULL);
    runTime.timeout = 0;
  }
}

static void stopTransfer(void)
{
  if(runTime.opThread){
    chThdTerminate(runTime.opThread);
    chEvtSignal(runTime.opThread, EVENT_MASK(10));
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    chVTReset(&runTime.vt);
    chThdTerminate(runTime.bmiThread);
    chThdWait(runTime.bmiThread);
    runTime.bmiThread = NULL;
  }
}

static void handshake_io_handler(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_START_TRIGGER);
  chSysUnlockFromISR();
}

static void keep_live(void *arg)
{
  chSysLockFromISR();
  chVTSetI(&runTime.vt, TIME_MS2I(200),keep_live,NULL);
  runTime.timeout++;
  chSysUnlockFromISR();
}
static THD_WORKING_AREA(waMonitor,512);
static THD_FUNCTION(procMonitor ,p)
{
  while(1){
    if(runTime.opThread != NULL){
      runTime.timeout ++;
      if(runTime.timeout > 10){
        stopTransfer();
      }
    }
    
    chThdSleepMilliseconds(200);
  }
  
}
//static THD_WORKING_AREA(waShell,2048);
#define SHELL_WA_SIZE   1024
static THD_WORKING_AREA(waShell, 1024);
void doorspeed_app_init()
{
  load_settings();
  bincmd_shellInit();
  analog_input_task_init();
  appRuntime = &runTime;
//  memcpy((uint8_t*)adxl.config,(uint8_t*)&nvmParam.adxlParam,sizeof(nvmParam.adxlParam));
//  if(adxl355_init(&adxl) == ADXL355_OK){
//    runTime.sensorReady = 1;
//    adxl355_powerup(&adxl);
//    adxl355_powerdown(&adxl);
//  }
  sdStart(&SD1,&serialCfg);
  runTime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binshellProc,(void*)&shell_cfg);
//  chVTObjectInit(&runTime.vt);
//  runTime.monitorThread = chThdCreateStatic(waMonitor,sizeof(waMonitor),NORMALPRIO-1,procMonitor,NULL);

  // register handshake io
  palSetLineCallback(PAL_LINE(GPIOA,12), handshake_io_handler,NULL);
  palEnableLineEvent(PAL_LINE(GPIOA,12),PAL_EVENT_MODE_BOTH_EDGES);
  
  runTime.self = chThdGetSelfX();
  chSemObjectInit(&runTime.sem,0);
  
  runTime.usePattern = 0;
  palClearPad(GPIOC,13);
  //startTransfer();
  while(1){
    {
      eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
      if(evt & EV_START_TRIGGER){
        if(palReadPad(GPIOA,12) == PAL_LOW){
          startTransfer();
        }
        else{
          stopTransfer();
        }
      }
      chThdSleepMilliseconds(50);
    }
  }
}

int main()
{
  halInit();
  chSysInit();
  AFIO->MAPR |= AFIO_MAP_SWJTAG_CONF_JTAGDISABLE;
  doorspeed_app_init();
}

void cmd_config(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  BinCommandHeader *header = (BinCommandHeader*)data;
  uint8_t buffer[256];
  BinCommandHeader *resp = (BinCommandHeader*)buffer;
  memcpy(buffer,data,8);
  if(header->len == CMD_STRUCT_SZ){ // read config
    // crc check
    uint16_t crc = checksum(data,CMD_STRUCT_SZ);
    if(crc == header->chksum){
      bool valid = false;
      switch(header->pid){
      case 0x0: // NODE
        memcpy(&buffer[8],(uint8_t*)&nvmParam.nodeParam, sizeof(nvmParam.nodeParam));
        resp->len = sizeof(nvmParam.nodeParam);
        valid = true;
        break;
      case 0x1: // ADXL
        memcpy(&buffer[8],(uint8_t*)&nvmParam.adxlParam, sizeof(nvmParam.adxlParam));
        resp->len = sizeof(nvmParam.adxlParam);
        valid = true;
        break;
      case 0x2: // IMU
        memcpy(&buffer[8],(uint8_t*)&nvmParam.imuParam, sizeof(nvmParam.imuParam));
        resp->len = sizeof(nvmParam.imuParam);
        valid = true;
        break;
      case 0x3: // TIME
        break;
      case 0x4: // FREQ
        break;
      case 0x5: // OLED
        break;
      case 0x40: // MODULE
        memcpy(&buffer[8],(uint8_t*)&nvmParam.moduleParam, sizeof(nvmParam.moduleParam));
        resp->len = sizeof(nvmParam.moduleParam);
        valid = true;
        break;
      case 0x41: // SERIAL
        break;
      case 0x42: // LAN
        break;
      case 0x43: // WLAN
        break;
      case 0x4F: // USER PARAM
        break;
      }
      if(valid){
        resp->len += CMD_STRUCT_SZ;
        resp->chksum = checksum(buffer,resp->len);
        // write response
        streamWrite(chp,buffer,resp->len);
      }
    }
    else{
      resp->len = 8;
      resp->type = MASK_CMD_RET_ERR;
      resp->pid = 0;
      resp->chksum = checksum(buffer,resp->len);
      // write response
      streamWrite(chp,buffer,resp->len);
    }
  }
  else{ // write config
    // read remaining data
    streamRead(chp,&buffer[CMD_STRUCT_SZ],header->len - CMD_STRUCT_SZ);
    uint16_t crc = checksum(buffer,header->len);
    if(crc == header->chksum){
      bool valid = false;
      switch(header->pid){
      case 0x0: // NODE
        memcpy((uint8_t*)&nvmParam.nodeParam,&buffer[8], sizeof(nvmParam.nodeParam));
        valid = true;
        break;
      case 0x1: // ADXL
        memcpy((uint8_t*)&nvmParam.adxlParam,&buffer[8], sizeof(nvmParam.adxlParam));
//        memcpy((uint8_t*)adxl.config,&buffer[8], sizeof(nvmParam.adxlParam));
        valid = true;
        break;
      case 0x2: // IMU
        memcpy((uint8_t*)&nvmParam.imuParam,&buffer[8], sizeof(nvmParam.imuParam));
        break;
      case 0x3: // TIME
        break;
      case 0x4: // FREQ
        break;
      case 0x5: // OLED
        break;
      case 0x40: // MODULE
        memcpy((uint8_t*)&nvmParam.moduleParam,&buffer[8], sizeof(nvmParam.moduleParam));
        valid = true;
        break;
      case 0x41: // SERIAL
        break;
      case 0x42: // LAN
        break;
      case 0x43: // WLAN
        break;
      case 0x4F: // USER PARAM
        break;
      case 0xFF: // rate config
        runTime.pollIntervalMs = buffer[8] & 0x7F;
        if(buffer[8] & 0x80){
          runTime.usePattern = 1;
        }
        break;
      }
      if(valid){
        resp->len = 8;
        resp->type = MASK_CMD_RET_OK;
        resp->pid = 0;
        resp->chksum = checksum(buffer,resp->len);
        // write response
        streamWrite(chp,buffer,resp->len);
        save_settings(0);
      }
      else{
        
      }
    }
    else{
      
    }    
  }
  
}
void cmd_start(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  if(!runTime.opThread){
    BinCommandHeader header;
    header.magic1 = MAGIC1;
    header.magic2 = MAGIC2;
    header.type = MASK_CMD_RET_OK;
    header.pid = 0;
    header.len = CMD_STRUCT_SZ;
    header.chksum = checksum((uint8_t*)&header,header.len);
//    streamWrite(chp,(uint8_t*)&header,8);
//    startTransfer();
    
  }
  else{
    BinCommandHeader header;
    header.magic1 = MAGIC1;
    header.magic2 = MAGIC2;
    header.type = MASK_CMD_RET_OK;
    header.pid = 0;
    header.len = CMD_STRUCT_SZ;
    header.chksum = checksum((uint8_t*)&header,header.len);
    streamWrite(chp,(uint8_t*)&header,8);
    stopTransfer();
  }
}
void cmd_stop(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  if(runTime.opThread){
    BinCommandHeader header;
    header.magic1 = MAGIC1;
    header.magic2 = MAGIC2;
    header.type = MASK_CMD_RET_OK;
    header.pid = 0;
    header.len = CMD_STRUCT_SZ;
    header.chksum = checksum((uint8_t*)&header,header.len);
    streamWrite(chp,(uint8_t*)&header,8);
    stopTransfer();
  }
}
void cmd_read(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data)
{
  
}

void cmd_boot(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  if(runTime.opThread != NULL) return;
  
  BinCommandHeader *header = (BinCommandHeader*)data;
  uint8_t buffer[96];
  BinCommandHeader *resp = (BinCommandHeader*)buffer;
  memcpy(buffer,header,8);
  // read remaining data
  if(header->len == (CMD_STRUCT_SZ + 4)){
    boot_info bootInfo;
//    bootInfo.loaderVersion = 0x0;
    streamRead(chp,&buffer[CMD_STRUCT_SZ],header->len - CMD_STRUCT_SZ);
    uint16_t crc = checksum(buffer,header->len);
    resp->len = CMD_STRUCT_SZ;
    if(crc == header->chksum){
      flash_Read(BL_CONFIG_ADDRESS,(uint16_t*)&bootInfo, sizeof(bootInfo)/2);
      memcpy((uint8_t*)&bootInfo.bootOption,&buffer[CMD_STRUCT_SZ],4);
//      bootInfo.bootOption = BOOT_KEY;
      flash_Write(BL_CONFIG_ADDRESS,(uint16_t*)&bootInfo, sizeof(bootInfo)/2);
      resp->type = MASK_CMD_RET_OK;
    }
    else{
      resp->type = MASK_CMD_RET_ERR;
    }
    resp->pid = 0;
    resp->chksum = checksum(buffer,resp->len);
    chSysDisable();
    NVIC_SystemReset();
  }

}

void cmd_idn(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data)
{
  uint8_t buffer[96];
  BinCommandHeader *hin = (BinCommandHeader*)data;
  BinCommandHeader *resp = (BinCommandHeader*)buffer;
  memcpy(buffer, (uint8_t*)header,8);

  if(header->len == CMD_STRUCT_SZ){
    boot_info bootInfo;
//    bootInfo.loaderVersion = 0x0;
    uint16_t crc = checksum(buffer,header->len);
    //resp->len = CMD_STRUCT_SZ + sizeof(bootInfo);
    if(crc == header->chksum){
      resp->len = CMD_STRUCT_SZ + sizeof(bootInfo);
      flash_Read(BL_CONFIG_ADDRESS,(uint16_t*)&bootInfo, sizeof(bootInfo)/2);
      memcpy(&buffer[CMD_STRUCT_SZ], (uint8_t*)&bootInfo.bootOption,sizeof(bootInfo));
      resp->type = MASK_CMD_RET_OK;
    }
    else{
      resp->len = CMD_STRUCT_SZ;
      resp->type = MASK_CMD_RET_ERR;
    }
    resp->pid = 0;
    resp->chksum = checksum(buffer,resp->len);
    // write response
    streamWrite(chp,buffer,resp->len);
  }  
}



