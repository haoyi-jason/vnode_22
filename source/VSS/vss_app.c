#include "ch.h"
#include "hal.h"
#include "vnode_app.h"
#include "adxl355_dev.h"
#include "adxl355_defs.h"
#include "bincmd_shell.h"
#include "nvm_config.h"
//#include "usbcfg.h"
#include "ylib/eeprom/at24_eep.h"
#include "task_wireless.h"
#include "task_storage.h"
#include "ylib/sensor/htud/htu2x.h"
#include "ylib/sensor/bmi160/bmi160_defs.h"
#include "ylib/sensor/bmi160/bmi160_dev.h"

#define ADC_GRP1_NUM_CHANNELS   3
#define ADC_GRP1_BUF_DEPTH      8
static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  NULL,
  0,    //cr1
  ADC_CR2_SWSTART, //cr2
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_480)| ADC_SMPR1_SMP_AN12(ADC_SAMPLE_480),// SMPR1
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS), // HTR
  0,    // LTR
  0,    // SQR1
  0,    // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12), // SQR3  
};


#define NVM_FLAG        0xAD
struct _nvmParam{
  uint8_t flag;
  node_param_t nodeParam;
  adxl355_cfg_t adxlParam;
  module_setting_t moduleParam;
  imu_config_t imuParam;
};



static void startTransfer(BaseSequentialStream*);

static struct _nvmParam nvmParam, *app_nvmParam;

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

#define EV_ADXL_FIFO_FULL EVENT_MASK(0)
#define EV_ISM_FIFO_FULL EVENT_MASK(1)
#define EV_FILE_LIST            EVENT_MASK(2)
#define EV_FILE_READ            EVENT_MASK(3)
#define EV_FILE_WRITE           EVENT_MASK(4)
#define EV_USER_BUTTON          EVENT_MASK(11)

void cmd_config(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_start(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_start_log(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_stop(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_read(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);

void cmd_list_file(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_read_file(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_write_file(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_remove_file(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
BinShellCommand commands[] ={
  {{0xab,0xba,0xA2,0x00,0,0},cmd_config}, // node param
  {{0xab,0xba,0xA2,0x01,0,0},cmd_config}, // adxl param
  {{0xab,0xba,0xA2,0x02,0,0},cmd_config}, // imu param
  {{0xab,0xba,0xA2,0x40,0,0},cmd_config}, // module param
  {{0xab,0xba,0xA2,0x43,0,0},cmd_config}, // wlan param
  {{0xab,0xba,0xA2,0x4E,0,0},cmd_config}, // user param
  {{0xab,0xba,0xA2,0x4F,0,0},cmd_config}, // user param
  {{0xab,0xba,0xA2,0x0E,0,0},cmd_config}, // RTC
  {{0xab,0xba,0xA1,0x01,0,0},cmd_start}, // start
  {{0xab,0xba,0xA1,0x02,0,0},cmd_start}, // start log to sd
  {{0xab,0xba,0xA1,0x00,0,0},cmd_stop}, // stop
  {{0xab,0xba,0x01,0x00,0,0},cmd_read},
  {{0xab,0xba,0xA3,0x11,0,0},cmd_list_file},
  {{0xab,0xba,0xA3,0x03,0,0},cmd_read_file},
  {{0xab,0xba,0xA3,0x04,0,0},cmd_write_file},
  {{0xab,0xba,0xA3,0x05,0,0},cmd_remove_file},
  {{0xab,0xba,0x01,0x00,0,0},NULL},
};

static const BinShellConfig shell_cfg = {
  (BaseSequentialStream *)&SDW1,
  commands
};

struct _runTime{
  thread_t *self;
  uint8_t state;
  uint8_t *rxPtr;
  uint8_t *txPtr;
  uint8_t *bufEnd;
  uint8_t buffer[512];
  //uint8_t tmp[512];
  mutex_t mutex;
  uint16_t rxSz;
  uint16_t rxWrSz;
  event_source_t es_sensor;
  event_listener_t el_sensor;
  event_listener_t el_sdfs;
  event_listener_t el_wireless;
  struct {
    uint16_t ms_on;
    uint16_t ms_off;
  }ledBlink;
  thread_t *opThread;
  uint8_t activatedSensor;
  uint8_t sensorReady;
  thread_t *shelltp;
  struct{
    char writeFileName[64];
    char readFileName[64];
    uint32_t read_offset;
    uint16_t read_count;
  }logFile;
  virtual_timer_t vt;
  BaseSequentialStream *activeStream;
  adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];
  int16_t battery_mv[2];
  uint8_t activeWlan;
  uint16_t blinkPeriod;
  systime_t elapsed;
  systime_t last;
  uint16_t htu_data[2];
};

static struct _runTime runTime, *app_runTime;
static FSDriver SDFS1;;
static SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  0,
  SPI_CR1_BR_1
};

_adxl_interface_t adxlInterface = {
  &SPID3,
  &spicfg
};

 
static adxl355_config_t adxlConfig = {
  0x0,0x0,0x0,0x0,
  48,
  GPIOC,4,
  GPIOB,9,
  GPIOB,8,
};


static ADXLDriver adxl = {
  &adxlInterface,
  &adxlConfig
};

_imu_interface_t bmiInterface = {
  &SPID3,
  &spicfg
};

static _bmi160_config_t bmiConfig = {
  {0x0,BMI160_ACCEL_NORMAL_MODE,BMI160_ACCEL_RANGE_2G,0x0},
  {0x0,BMI160_GYRO_NORMAL_MODE,BMI160_GYRO_RANGE_2000_DPS,0x0},
  GPIOA,8,
  GPIOA,11,
  GPIOA,12
};

static BMI160Driver bmi160 = {
  &bmiInterface,
  &bmiConfig
};

const module_setting_t module_default = {
  NVM_FLAG,
  "VSS",
  0x12345678, 
  0x00000001,
  "Grididea-ST32",
  "VSS-II" // supported config
};

/*
battery monitor was divide by 2 resistor, the vref = 2.72V, 12-bit ADC

*/

static void update_adc()
{
  int16_t chSum[3] = {0,0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    chSum[0] += runTime.samples[i * 3];
    chSum[1] += runTime.samples[i * 3 +1];
    chSum[2] += runTime.samples[i * 3 +2];
  }
  chSum[0] >>= 3;
  chSum[1] >>=3;
  chSum[2] >>=3;
  
  float ratio = (float)chSum[0]/4096.F;
  runTime.battery_mv[0] = ratio *5440;
  
  ratio = (float)chSum[1]/4096.F;
  runTime.battery_mv[1] = ratio *5440;

  runTime.buffer[CMD_STRUCT_SZ+4] = (uint8_t)((runTime.battery_mv[0] - 2500)/10);
  runTime.buffer[CMD_STRUCT_SZ+5] = (uint8_t)((runTime.battery_mv[1] - 2500)/10);
}

static void blink_cb(void *arg)
{
  chSysLockFromISR();
  palTogglePad(GPIOC,3);
  chVTSetI(&runTime.vt, TIME_MS2I(runTime.blinkPeriod),blink_cb,NULL);
  chSysUnlockFromISR();
}

static void user_button_isr(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_USER_BUTTON);
  chSysUnlockFromISR();
}

static void load_default()
{
  uint16_t nvmSz = sizeof(nvmParam);
    nvmParam.flag = NVM_FLAG;
    nvmParam.nodeParam.activeSensor = SENSOR_ADXL355;
    nvmParam.nodeParam.commType = COM_IF_BT;
    nvmParam.nodeParam.opMode = OP_STREAM;
    memcpy(nvmParam.nodeParam.log_file_prefix,"SensorNode\0",11);
    nvmParam.nodeParam.logFileSize = 1024*1024*10;
    
    nvmParam.adxlParam.fs = 0x1;
    nvmParam.adxlParam.odr = ADXL355_ODR_500;
    nvmParam.adxlParam.hpf = 0;
    
    nvmParam.imuParam.accel.lpf = BMI160_ACCEL_BW_NORMAL_AVG4;
    nvmParam.imuParam.accel.odr = BMI160_ACCEL_ODR_800HZ;
    nvmParam.imuParam.accel.power = BMI160_ACCEL_NORMAL_MODE;
    nvmParam.imuParam.accel.range = BMI160_ACCEL_RANGE_2G;

    nvmParam.imuParam.gyro.lpf = BMI160_GYRO_BW_NORMAL_MODE;
    nvmParam.imuParam.gyro.odr = BMI160_GYRO_ODR_800HZ;
    nvmParam.imuParam.gyro.power = BMI160_GYRO_NORMAL_MODE;
    nvmParam.imuParam.gyro.range = BMI160_GYRO_RANGE_2000_DPS;
    
    memcpy((uint8_t*)&nvmParam.moduleParam,(uint8_t*)&module_default, sizeof(module_setting_t));
    //nvm_flash_write(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
    eepromWrite(OFFSET_NVM_CONFIG,nvmSz,(uint8_t*)&nvmParam);
}

static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
  if(nvmSz > SZ_NVM_CONFIG){
    // error
    while(1);
  }
//  nvm_flash_read(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
  eepromRead(OFFSET_NVM_CONFIG,nvmSz,(uint8_t*)&nvmParam);
  if(nvmParam.flag != NVM_FLAG){
    load_default();  
  }
  
}

static void save_settings(uint8_t option)
{
  eepromWrite(OFFSET_NVM_CONFIG,sizeof(nvmParam),(uint8_t*)&nvmParam);
}
static void writeLogHeader()
{
  char *ptr = runTime.buffer;
  size_t sz;
  memset(ptr,0,512);
  if(nvmParam.nodeParam.activeSensor == SENSOR_ADXL355){
    ptr += chsnprintf(ptr, 512, "VSS LOG FILE REV 1.0\n");
  }
  else if(nvmParam.nodeParam.activeSensor == SENSOR_BMI160){
    ptr += chsnprintf(ptr, 512, "VSS LOG FILE REV 1.0\n");
  }
  
  if(nvmParam.nodeParam.activeSensor == SENSOR_ADXL355){
    ptr += chsnprintf(ptr,512,"SENSOR=ADXL355\n");
    switch(nvmParam.adxlParam.fs){
    case 1:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=2 G\n");break;
    case 2:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=4 G\n");break;
    case 3:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=8 G\n");break;
    }
    uint16_t odr = 4000/(1<<nvmParam.adxlParam.odr);
    ptr += chsnprintf(ptr,512,"DATA RATE=%d SPS\n",odr);
    ptr += chsnprintf(ptr,512,"HPF=0x%x\n",nvmParam.adxlParam.hpf);
  }
  else if(nvmParam.nodeParam.activeSensor == SENSOR_BMI160){
    ptr += chsnprintf(ptr,512,"SENSOR=BMI160\n");
    switch(nvmParam.imuParam.accel.range){
    case 0x03:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=2 G\n");break;
    case 0x05:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=4 G\n");break;
    case 0x08:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=8 G\n");break;
    case 0x0c:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=16 G\n");break;
    default: ptr += chsnprintf(ptr,512,"ACCEL_RANGE = WRONG\n");break;
    }

    uint16_t odr = 1600/((0x0C - nvmParam.imuParam.accel.odr) << 1);
    ptr += chsnprintf(ptr,512,"DATA RATE=%d SPS\n",odr);
    ptr += chsnprintf(ptr,512,"LPF=0x%x\n",nvmParam.imuParam.accel.lpf);

    switch(nvmParam.imuParam.gyro.range){
    case 0x00:ptr += chsnprintf(ptr,512,"GYRO_RANGE=2000 DPS\n");break;
    case 0x01:ptr += chsnprintf(ptr,512,"GYRO_RANGE=1000 DPS\n");break;
    case 0x02:ptr += chsnprintf(ptr,512,"GYRO_RANGE=500 DPS\n");break;
    case 0x03:ptr += chsnprintf(ptr,512,"GYRO_RANGE=250 DPS\n");break;
    case 0x04:ptr += chsnprintf(ptr,512,"GYRO_RANGE=125 DPS\n");break;
    default: ptr += chsnprintf(ptr,512,"GYRO_RANGE = WRONG\n");break;
    }

    odr = 1600/((0x0C - nvmParam.imuParam.gyro.odr) << 1);
    ptr += chsnprintf(ptr,512,"DATA RATE=%d SPS\n",odr);
    ptr += chsnprintf(ptr,512,"LPF=0x%x\n",nvmParam.imuParam.gyro.lpf);
  }
  
  FRESULT fres;
  FIL f;
  fres = f_open(&f,runTime.logFile.writeFileName,FA_WRITE | FA_OPEN_APPEND);
  if(fres == FR_OK){
    f_lseek(&f,0);
    f_write(&f,runTime.buffer,512,&sz);
    f_close(&f);
  }
  
}

static void valid_log_fileName()
{
  RTCDateTime timespec;
  struct tm now;
  rtcGetTime(&RTCD1,&timespec);
  rtcConvertDateTimeToStructTm(&timespec,&now,NULL);
  char fileName[64];
  chsnprintf(fileName,64,"%s_%04d%02d%02d-%02d%02d%02d.bin\0",
             nvmParam.nodeParam.log_file_prefix,
             now.tm_year+1900,
             now.tm_mon+1,
             now.tm_mday,
             now.tm_hour,
             now.tm_min,
             now.tm_sec);
  
  if(strncmp(fileName, runTime.logFile.writeFileName, strlen(fileName)) != 0){
    FRESULT fres;
    FIL f;
    memcpy(runTime.logFile.writeFileName,fileName, strlen(fileName));
    fres = f_open(&f,runTime.logFile.writeFileName,FA_READ | FA_WRITE | FA_CREATE_NEW);
    if(fres != FR_OK){
      while(1);
    }
    
    fres = f_lseek(&f,512);
    f_close(&f);
  }
  
}

static void finish_log_file()
{
  char *ptr = runTime.buffer;
  size_t sz;
  memset(runTime.buffer,0,512);
  ptr += chsnprintf(ptr,512,"SensorNode LOG FILE REV 1.1\n");
  if(nvmParam.nodeParam.activeSensor == SENSOR_ADXL355){
    ptr += chsnprintf(ptr,512,"SENSOR=ADXL355\n");
    switch(nvmParam.adxlParam.fs){
    case 1:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=2 G\n");break;
    case 2:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=4 G\n");break;
    case 3:ptr += chsnprintf(ptr,512,"ACCEL_RANGE=8 G\n");break;
    }
    uint16_t odr = 4000/(1<<nvmParam.adxlParam.odr);
    ptr += chsnprintf(ptr,512,"DATA RATE=%d SPS\n",odr);
    ptr += chsnprintf(ptr,512,"HPF=0x%x\n",nvmParam.adxlParam.hpf);
  }
  else if(nvmParam.nodeParam.activeSensor == SENSOR_BMI160){
    ptr += chsnprintf(ptr,512,"SENSOR=BMI160\n");
    switch(nvmParam.imuParam.accel.range){
    case BMI160_ACCEL_RANGE_2G:
      ptr += chsnprintf(ptr,512,"ACCEL_RANGE=2 G\n");
      break;
    case BMI160_ACCEL_RANGE_4G:
      ptr += chsnprintf(ptr,512,"ACCEL_RANGE=4 G\n");
      break;
    case BMI160_ACCEL_RANGE_8G:
      ptr += chsnprintf(ptr,512,"ACCEL_RANGE=8 G\n");
      break;
    case BMI160_ACCEL_RANGE_16G:
      ptr += chsnprintf(ptr,512,"ACCEL_RANGE=16 G\n");
      break;
    }
    switch(nvmParam.imuParam.gyro.range){
    case BMI160_GYRO_RANGE_2000_DPS:
      ptr += chsnprintf(ptr,512,"GYRO_RANGE=2000 DPS\n");
      break;
    case BMI160_GYRO_RANGE_1000_DPS:
      ptr += chsnprintf(ptr,512,"GYRO_RANGE=1000 DPS\n");
      break;
    case BMI160_GYRO_RANGE_500_DPS:
      ptr += chsnprintf(ptr,512,"GYRO_RANGE=500 DPS\n");
      break;
    case BMI160_GYRO_RANGE_250_DPS:
      ptr += chsnprintf(ptr,512,"GYRO_RANGE=250 DPS\n");
      break;
    case BMI160_GYRO_RANGE_125_DPS:
      ptr += chsnprintf(ptr,512,"GYRO_RANGE=125 DPS\n");
      break;
    }
    switch(nvmParam.imuParam.accel.odr){
    case BMI160_ACCEL_ODR_25HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=25 SPS\n");
      break;
    case BMI160_ACCEL_ODR_50HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=50 SPS\n");
      break;
    case BMI160_ACCEL_ODR_100HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=100 SPS\n");
      break;
    case BMI160_ACCEL_ODR_200HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=200 SPS\n");
      break;
    case BMI160_ACCEL_ODR_400HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=400 SPS\n");
      break;
    case BMI160_ACCEL_ODR_800HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=800 SPS\n");
      break;
    case BMI160_ACCEL_ODR_1600HZ:
      ptr += chsnprintf(ptr,512,"DATA_RATE=1600 SPS\n");
      break;
    }
  }  
  FIL f;
  FRESULT fres;
  fres = f_open(&f, runTime.logFile.writeFileName, FA_OPEN_APPEND | FA_WRITE);
  if(fres == FR_OK){
    UINT SZ;
    f_lseek(&f,0);
    f_write(&f,runTime.buffer,512,&SZ);
    f_close(&f);
  }
}

static int8_t adxl355_cmd_start(ADXLDriver *dev)
{
  int8_t ret = ADXL355_OK;
  float rate = 4000.0F;
  uint8_t buffer[64];
  adxl355_powerdown(dev);
  dev->config->intmask = ADXL355_INT_FULL_EN1;
  adxl355_set_filter(dev);
  adxl355_set_full_scale(dev);
  //adxl355_set_interrupt(dev);
  
  switch(dev->config->fullscale){
  case 1:
    dev->sensitivity = 0.0000039;
    break;
  case 2:
    dev->sensitivity = 0.0000078;
    break;
  case 3:
  default:
    dev->sensitivity = 0.0000156;
    break;
  }
  rate = rate /(1 << dev->config->outputrate);
//  time_domain_param_t *time = (time_domain_param_t*)buffer;
//  m_timeDomain.nofSamples = (uint16_t)(rate * time->samplePeriodMs/1000);
//  m_timeDomain.sampleTimesec = time->samplePeriodMs/1000.;
//  m_timeDomain.scale.x = m_timeDomain.scale.y = m_timeDomain.scale.z = dev->sensitivity;

  adxl355_powerup(dev);    

  return ret;
}

static int8_t adxl355_cmd_stop(ADXLDriver *dev)
{
  dev->config->intmask = 0x0;
  //adxl355_set_interrupt(dev);  
  return adxl355_powerdown(dev);
}
static THD_WORKING_AREA(waOperation,2048);
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
  uint16_t room_size;
 
  uint8_t opMode = nvmParam.nodeParam.opMode;
  uint8_t activeSensor = nvmParam.nodeParam.activeSensor;
    
  bool bStop = false;

//  event_listener_t elSensor;
  eventflags_t flags;
  //activeSensor = SENSOR_ISM330;
  //activeSensor = SENSOR_ADXL355;
  if(activeSensor == SENSOR_ADXL355){
//    chEvtRegisterMask(&adxl.evsource,&runTime.el_sensor,EV_ADXL_FIFO_FULL);
//    runTime.ledBlink.ms_on = 500;
//    runTime.ledBlink.ms_off = 500;
//    adxl355_get_fifo_size(&adxl,&sz);
//    if(sz > 0){
//      bStop = false;
//    }
    room_size = 288;
    SDFS1.packet_size = room_size + 14;
    adxl355_cmd_start(&adxl);  
  }
  else if(activeSensor == SENSOR_BMI160){
    room_size = 300;
    SDFS1.packet_size = room_size + 14;
    bmi160_poweron(&bmi160);
    bmi160_start(&bmi160);
//    chEvtRegisterMask(&bmi160.evsource,&runTime.el_sensor,EV_ISM_FIFO_FULL);
  }
  else if(activeSensor == SENSOR_ISM330){
//    chEvtRegisterMask(&runTime.es_sensor,&elSensor,EV_ISM_FIFO_FULL );
//    palSetLineCallback(LINE_ISM_INT,gpioa3_int_handler,NULL);
//    palEnableLineEvent(LINE_ISM_INT,PAL_EVENT_MODE_RISING_EDGE);
//    runTime.ledBlink.ms_on = 500;
//    runTime.ledBlink.ms_off = 500;
//    ism330_cmd_start_config();
  }
  
//  uint8_t packetToIgnore = 5;
//  if(runTime.resetBuffer != NULL){
//    runTime.resetBuffer();
//  }
  runTime.ledBlink.ms_off = 500;
  runTime.ledBlink.ms_on = 500;
  uint32_t fifo_size;
  eventmask_t evt;
  runTime.rxSz = CMD_STRUCT_SZ + 6;
  runTime.rxWrSz = room_size;
  uint8_t packetIgnore = 2;
  while(!bStop){
    //evt = chEvtWaitAny(ALL_EVENTS);
   // flags = chEvtGetAndClearFlags(&elSensor);
    if(activeSensor == SENSOR_ADXL355){ // adxl 
      adxl355_get_status(&adxl,&adxl_sta);
      // read fifo data, sz indicate the number of records, not bytes
      adxl355_get_fifo_size(&adxl,&sz);
      fifo_size = 9*(sz/9);
      
      if(fifo_size){ // read if > 10 samples
        uint16_t readSz = (fifo_size < runTime.rxWrSz)?fifo_size:runTime.rxWrSz;
        adxl355_read_fifo(&adxl,&runTime.buffer[runTime.rxSz],readSz);
        runTime.rxSz += readSz;
        runTime.rxWrSz -= readSz;
        if(runTime.rxWrSz == 0){     
          if(packetIgnore){
            packetIgnore--;
          }
          else{
            header = (BinCommandHeader*)runTime.buffer;
            header->magic1 = MAGIC1;
            header->magic2 = MAGIC2;
            header->type = MASK_DATA ;//| runTime.lbt;
            header->len = 288 + CMD_STRUCT_SZ + 6;
            header->pid = pktCount++;
            header->chksum = checksum(runTime.buffer,header->len);
            if(stream != NULL && packetIgnore==0){
              if(stream == (BaseSequentialStream*)&SDFS1){
                sdfs_insertData((FSDriver*)stream,runTime.buffer, header->len);
              }
              else if(stream == (BaseSequentialStream*)&SDW1){
                //chSysLock();
                streamWrite(stream,runTime.buffer, header->len);
                //chSysUnlock();
              }
            }
          }
          runTime.rxSz = CMD_STRUCT_SZ + 6;
          runTime.rxWrSz = room_size;

//          if(sz > readSz){
//            memcpy(&runTime.buffer[runTime.rxSz],&buf[readSz], sz - readSz);
//            runTime.rxSz += (sz - readSz);
//            runTime.rxWrSz -= (sz - readSz);
//          }
        }
      }
      //A0_LO;
    }
    else if(activeSensor == SENSOR_BMI160){
      uint16_t sz;
      bmi160_fifo_read(&bmi160,buf,320,&sz);
      if(sz){
        uint16_t readSz = (sz < runTime.rxWrSz)? sz:runTime.rxWrSz;
        memcpy(&runTime.buffer[runTime.rxSz], buf,readSz);
        runTime.rxSz += readSz;
        runTime.rxWrSz -= readSz;
        if(runTime.rxWrSz == 0){
          if(packetIgnore){
            packetIgnore--;
          }
          else{
            header = (BinCommandHeader*)runTime.buffer;
            header->magic1 = MAGIC1;
            header->magic2 = MAGIC2;
            header->type = MASK_DATA;
            header->len = 300 + CMD_STRUCT_SZ + 6;
            header->pid = pktCount++;
            header->chksum = checksum(runTime.buffer,header->len);
            if(stream != NULL && packetIgnore==0){
              if(stream == (BaseSequentialStream*)&SDFS1){
                sdfs_insertData((FSDriver*)stream,runTime.buffer, header->len);
              }
              else if(stream == (BaseSequentialStream*)&SDW1){
                //chSysLock();
                streamWrite(stream,runTime.buffer, header->len);
                //chSysUnlock();
              }
            }
          }
          runTime.rxSz = CMD_STRUCT_SZ + 6;
          runTime.rxWrSz = room_size;
          
          if(sz > readSz){
            memcpy(&runTime.buffer[runTime.rxSz],&buf[readSz], sz - readSz);
            runTime.rxSz += (sz - readSz);
            runTime.rxWrSz -= (sz - readSz);
          }
        }
      }
    }
    bStop = chThdShouldTerminateX();
    if(bStop){
      if(activeSensor == SENSOR_ADXL355){
        adxl355_cmd_stop(&adxl);  
        adxl355_get_fifo_size(&adxl,&sz);
        if(sz){
          adxl355_read_fifo(&adxl,&runTime.buffer[runTime.rxSz],sz);
        }
        // disable interrupt
        runTime.ledBlink.ms_on = 500;
        runTime.ledBlink.ms_off = 500;
        //chEvtUnregister(&runTime.es_sensor,&elSensor);
      }
      else if(activeSensor == SENSOR_ISM330){
        bmi160_stop(&bmi160);
      }
      else if(activeSensor == SENSOR_ISM330){
        //palDisableLineEvent(LINE_ISM_INT);
        //ism330_cmd_stop_config();
        //chEvtUnregister(&runTime.es_sensor,&elSensor);
      }
      runTime.ledBlink.ms_on = 1000;
      runTime.ledBlink.ms_off = 1000;
    }
    chThdSleepMilliseconds(5);
  }
//  chThdExit((msg_t)0);
   // chEvtUnregister(&adxl.evsource,&runTime.el_sensor);
  // turn off led
  palClearPad(GPIOC,3);
}


static void stopTransfer(void)
{
  if(runTime.opThread){
    chThdTerminate(runTime.opThread);
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    if(runTime.activeWlan == 0){
      chVTReset(&runTime.vt);
      palSetPad(GPIOC,3);
    }
    else{
      runTime.blinkPeriod = 1000;
    }
    if(runTime.activeStream == (BaseSequentialStream*)&SDFS1){
//      size_t bsz = SDFS1.iqueue.q_counter;
//      if(bsz >= 0) {
//        size_t n;
//        n = streamRead(&SDFS1,runTime.tmp,512);
//        sdfs_write(&SDFS1, runTime.logFile.writeFileName, runTime.tmp, n);
//      }      
      writeLogHeader();
    }
  }
  if(runTime.activeWlan == 0){
    chVTReset(&runTime.vt);
    palSetPad(GPIOC,3);
  }
  else{
    runTime.blinkPeriod = 1000;
  }
}

static void startTransfer(BaseSequentialStream *stream)
{
  if(!runTime.opThread){
    runTime.activeStream = stream;
    if(stream == (BaseSequentialStream*)&SDFS1){
      valid_log_fileName();
    }
    runTime.blinkPeriod = 500;
    chVTSet(&runTime.vt,TIME_MS2I(runTime.blinkPeriod),blink_cb,NULL);
    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO,procOperation,stream);
  }
  else{
    stopTransfer();
//    if(stream == (BaseSequentialStream*)&SDFS1){
//      runTime.blinkPeriod = 1000;
//    }
//    else{
//    }
  }
}


void list_file()
{
  BinCommandHeader *header = (BinCommandHeader*)runTime.buffer;
  header->magic1 = MAGIC1;
  header->magic2 = MAGIC2;
  FRESULT res;
  DIR dir;
  FILINFO finfo;
  uint32_t sz;
  res = f_opendir(&dir, "/");
  if(res == FR_OK){
    for(;;){
      res = f_readdir(&dir,&finfo);
      if(res != FR_OK || finfo.fname[0] == 0) break;
      if(finfo.fattrib & AM_ARC){
        header->type = FILE_OP_OK;
        header->type = 0xA3;
        header->pid = 0x11;
        sz = finfo.fsize;
        memcpy(&runTime.buffer[8],"/\0",2);
        memcpy(&runTime.buffer[40],finfo.fname,strlen(finfo.fname));
        runTime.buffer[40+strlen(finfo.fname)] = 0x0;
        memcpy(&runTime.buffer[72],(uint8_t*)&sz,4);
        header->len = 76;
        header->chksum = checksum(runTime.buffer,header->len);
        streamWrite((BaseSequentialStream*)&SDW1,runTime.buffer,header->len);
        chThdSleepMilliseconds(50);
      }
    }
    header->len = CMD_STRUCT_SZ;
    header->chksum = checksum((uint8_t*)&header,header->len);
    streamWrite((BaseSequentialStream*)&SDW1,runTime.buffer,header->len);
    chThdYield();
  }
}

void read_file()
{
  BinCommandHeader *header = (BinCommandHeader*)runTime.buffer;
  header->magic1 = MAGIC1;
  header->magic2 = MAGIC2;
  FRESULT fres;
  DIR dir;
  FIL f;
  uint32_t offset;
  UINT readSz = 300;
  UINT szRead;
  if(SDFS1.readOffset == 0xFFFFFFFF){ // read all file
    fres = f_open(&f,SDFS1.fileName, FA_READ);
    offset = 0;
    if(fres == FR_OK){
      header->type = 0xA3;
      header->pid = 0x13;
      for(;;){
        memcpy(&runTime.buffer[8],(uint8_t*)&offset,4);
        f_read(&f, &runTime.buffer[12],readSz,&szRead);
        header->len = szRead + CMD_STRUCT_SZ + 4;
        header->chksum = checksum(runTime.buffer,header->len);
        streamWrite((BaseSequentialStream *)&SDW1,runTime.buffer,header->len);
        offset += szRead;
        chThdSleepMilliseconds(25);
        if(offset == f.obj.objsize){ // read at the end of file
          f_close(&f);
          break;
        }
      }
      // transfer last packet with 0-byte data
      header->len = CMD_STRUCT_SZ;
      header->chksum = checksum(runTime.buffer,header->len);
      streamWrite((BaseSequentialStream *)&SDW1,runTime.buffer,header->len);
      chThdYield();
      chThdSleepMilliseconds(200);
    }
  }
  else{
    fres = f_open(&f,SDFS1.fileName, FA_READ);
    if(fres == FR_OK){
      offset = SDFS1.readOffset;
      if(offset < f.obj.objsize){
        readSz = ((f.obj.objsize - offset)>300)?300:(f.obj.objsize - offset + 1);
        f_lseek(&f,offset);
        memcpy(&runTime.buffer[8],(uint8_t*)&offset,4);
        f_read(&f, &runTime.buffer[12],readSz,&szRead);
        header->len = szRead + CMD_STRUCT_SZ + 4;
        header->chksum = checksum(runTime.buffer,header->len);
        streamWrite((BaseSequentialStream *)&SDW1,runTime.buffer,header->len);
        chThdYield();
      }
    }
    f_close(&f);
  }                            
}

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};

#define PCA9548_BASE_ADDR       0x70
void pca_set_channel(uint8_t ch)
{
  static msg_t ret;
  i2cAcquireBus(&I2CD1);
  i2cStart(&I2CD1,&i2ccfg);
  uint8_t ucTx = (1 << ch);
  uint8_t adr = 0;
  ret = i2cMasterTransmitTimeout(&I2CD1,PCA9548_BASE_ADDR + adr,&ucTx,1,NULL,0,TIME_MS2I(50));  
  i2cStop(&I2CD1);
  i2cReleaseBus(&I2CD1);
}


#define SHELL_WA_SIZE   1024
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);
void vnode_app_init()
{
  app_nvmParam = &nvmParam;
  app_runTime = &runTime;
  at24eep_init(&I2CD1,32,1024,0x50,2);
  load_settings();
  
  uint8_t cc = 0;
  // check if user press button
  while(palReadPad(GPIOC,13) == PAL_LOW){
    cc++;
    if(cc > 10){
      load_default();
      break;
    }
    chThdSleepMilliseconds(100);
  }
  
  // start wireless
  //nvmParam.nodeParam.commType = 0x80;
  task_wireless_init(nvmParam.nodeParam.commType);
//  task_wireless_init(COMM_USE_WIFI);
  
  
  
  bincmd_shellInit();
  chVTObjectInit(&runTime.vt);
  uint8_t activeSensor = nvmParam.nodeParam.activeSensor;
  if(activeSensor == SENSOR_ADXL355){
    memcpy((uint8_t*)adxl.config,(uint8_t*)&nvmParam.adxlParam,sizeof(nvmParam.adxlParam));
    if(adxl355_init(&adxl) == ADXL355_OK){
      runTime.sensorReady = SENSOR_ADXL355;
      adxl355_powerup(&adxl);
      adxl355_powerdown(&adxl);
    }
  }
  else if(activeSensor == SENSOR_BMI160){
    bmi160.config->accel.power = nvmParam.imuParam.accel.power;
    bmi160.config->accel.odr = nvmParam.imuParam.accel.odr;
    bmi160.config->accel.fs = nvmParam.imuParam.accel.range;
    bmi160.config->accel.lpf = nvmParam.imuParam.accel.lpf;
    bmi160.config->gyro.power = nvmParam.imuParam.gyro.power;
    bmi160.config->gyro.odr = nvmParam.imuParam.gyro.odr;
    bmi160.config->gyro.fs = nvmParam.imuParam.gyro.range;
    bmi160.config->gyro.lpf = nvmParam.imuParam.gyro.lpf;
    if(bmi160_dev_init(&bmi160) == MSG_OK){
      runTime.sensorReady = SENSOR_BMI160;
      uint8_t imudata[12];
      bmi160_single_read(&bmi160,imudata,12,NULL);
    }
  }
  runTime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binshellProc,(void*)&shell_cfg);

  // start SD Logger
  fs_init(&SDFS1);
  //uint8_t tmp[320];
  chEvtRegisterMask(&SDFS1.evs_insertion, &runTime.el_sdfs, EV_SD_INS | EV_SD_WRITE);
  chEvtRegisterMask(&SDW1.es, &runTime.el_wireless,RSI_APP_EVENT_SPP_CONN | RSI_APP_EVENT_SPP_DISCONN );
  // enable user button interrupt
  palSetLineCallback(PAL_LINE(GPIOC,13),user_button_isr,NULL);
  palEnableLineEvent(PAL_LINE(GPIOC,13),PAL_EVENT_MODE_RISING_EDGE);
  runTime.self = chThdGetSelfX();
//  chEvtRegisterMask(&SDFS1.evs_insertion, &runTime.el_sdfs, EVENT_MASK(1));
  
  // start ADC for battery voltage measurment
  adcStart(&ADCD1,NULL);
  adcStartConversion(&ADCD1,&adcgrpcfg,runTime.samples,ADC_GRP1_BUF_DEPTH);
  pca_set_channel(2);
  htu2xinit(&I2CD1,(I2CConfig*)&i2ccfg);
  static uint16_t cntr = 20;
  runTime.activeWlan = 0;
  uint16_t htu_data[2];
  
  //startTransfer(NULL);
  
//  uint8_t test_log = 1;
//  uint8_t test_started = 0;
//  uint32_t test_second = 200;
//  systime_t test_start_time;
  
  while(1){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    eventflags_t flags = chEvtGetAndClearFlags(&runTime.el_sdfs);
//    if(test_log == 1){
//      if(test_started == 0){
//        //valid_log_fileName();
//        startTransfer((BaseSequentialStream*)&SDFS1);      
//        test_started = 1;
//        test_start_time = chVTGetSystemTimeX();
//      }
//      else{
//        uint32_t interval = TIME_I2S(chVTTimeElapsedSinceX(test_start_time));
//        if(interval >= test_second){
//          stopTransfer();
//          test_log = 0;
//        }
//      }
//      
//    }
    if(flags & EV_SD_INS){// insert
      chThdSleepMilliseconds(200);
      sdfs_loadCard(&SDFS1);
    }
    if(flags & EV_SD_WRITE){// write
//      size_t n;
//      n = streamRead(&SDFS1,runTime.tmp,512);
//      if(sdfs_write(&SDFS1, runTime.logFile.writeFileName, runTime.tmp, n)> nvmParam.nodeParam.logFileSize){
//        writeLogHeader();
//        valid_log_fileName();
//      }
    }
    
    if(runTime.opThread != NULL){
//      size_t bsz = SDFS1.iqueue.q_counter;
//      if(bsz >= 512) {
//        size_t n;
//        n = streamRead(&SDFS1,runTime.tmp,512);
        if(sdfs_write(&SDFS1, runTime.logFile.writeFileName, 0, 0)> nvmParam.nodeParam.logFileSize){
          writeLogHeader();
          valid_log_fileName();
        }
//      }
    }

    
    if(flags & EV_SD_LS){
      list_file();
    }
    
    if(flags & EV_SD_READ){
      read_file();
    }
    
    if(flags & EV_SD_REMOVE){
      BinCommandHeader *header = (BinCommandHeader*)runTime.buffer;
      header->magic1 = MAGIC1;
      header->magic2 = MAGIC2;
      if(f_unlink(SDFS1.fileName) == FR_OK){
        header->type = MASK_CMD_RET_OK;
        header->pid = 0x01;
      }
      else{
        header->type = MASK_CMD_RET_ERR;
        header->pid = 0x00;
      }
      header->len = CMD_STRUCT_SZ;
      header->chksum = checksum(runTime.buffer,header->len);
      streamWrite((BaseSequentialStream *)&SDW1,runTime.buffer,header->len);
    }

    if(evt & EV_USER_BUTTON){
      chThdSleepMilliseconds(200);
      startTransfer((BaseSequentialStream*)&SDFS1);
    }

    flags = chEvtGetAndClearFlags(&runTime.el_wireless);
    if(flags & RSI_APP_EVENT_SPP_CONN){
      runTime.activeWlan = 1;
      runTime.blinkPeriod = 1000;
      chVTSet(&runTime.vt,TIME_MS2I(runTime.blinkPeriod),blink_cb,NULL);
    }
    if(flags & RSI_APP_EVENT_SPP_DISCONN){
      runTime.activeWlan = 0;
      stopTransfer();
    }
    cntr--;
    if(cntr == 0){
      cntr = 100;
    }
    else if(cntr == 90){
      update_adc();
    }
    else if(cntr == 80){
      runTime.htu_data[0] = sen_htu2xx_read_temp_raw();
    }
    else if(cntr == 70){
      runTime.htu_data[1] = sen_htu2xx_read_humidity_raw();  
      memcpy(&runTime.buffer[CMD_STRUCT_SZ], (uint8_t*)&runTime.htu_data[0],4);
    }
    chThdSleepMilliseconds(10);
  }
}

void cmd_config(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  BinCommandHeader *header = (BinCommandHeader*)data;
  uint8_t buffer[270];
  BinCommandHeader *resp = (BinCommandHeader*)buffer;
  memcpy(buffer,data,CMD_STRUCT_SZ);
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
        wireless_read_lan_param(&buffer[8],&resp->len,256);
        valid = true;
        break;
      case 0x43: // WLAN
        wireless_read_wlan_param(&buffer[8],&resp->len,256);
        valid = true;
        break;
      case 0x4E: // USER PARAM
        eepromRead(OFFSET_NVM_USER,256,&buffer[8]);
        resp->len = 264;
        valid = 1;
        break;
      case 0x4F: // USER PARAM
        eepromRead(OFFSET_NVM_USER2,256,&buffer[8]);
        resp->len = 264;
        valid = 1;
        break;
      case 0x0E: // read RTC
        {
          RTCDateTime timespec;
          rtc_config_t *rtc_config = (rtc_config_t*)&buffer[CMD_STRUCT_SZ];
          struct tm tm;
          rtcGetTime(&RTCD1,&timespec);
          rtcConvertDateTimeToStructTm(&timespec,&tm,NULL);
          rtc_config->yy = tm.tm_year;
          rtc_config->mm = tm.tm_mon + 1;
          rtc_config->dd = tm.tm_mday;
          rtc_config->hh = tm.tm_hour;
          rtc_config->nn = tm.tm_min;
          rtc_config->ss = tm.tm_sec;
          resp->len = 6;
          valid = 1;
        }
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
      chThdYield();
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
        memcpy((uint8_t*)adxl.config,&buffer[8], sizeof(nvmParam.adxlParam));
        valid = true;
        break;
      case 0x2: // IMU
        memcpy((uint8_t*)&nvmParam.imuParam,&buffer[8], sizeof(nvmParam.imuParam));
        memcpy((uint8_t*)bmi160.config,&buffer[8], sizeof(nvmParam.imuParam));
        valid = true;
        break;
      case 0x3: // TIME
        break;
      case 0x4: // FREQ
        break;
      case 0x5: // OLED
        break;
      case 0xC0: // MODULE Write use 0xC0
        // check for key to enable write module param
        // module param only avaliable for private use
        memcpy((uint8_t*)&nvmParam.moduleParam,&buffer[8], sizeof(nvmParam.moduleParam));
        valid = true;
        break;
      case 0x41: // SERIAL
        break;
      case 0x42: // LAN
        wireless_write_lan_param(&buffer[8],header->len - CMD_STRUCT_SZ);
        valid = true;
        break;
      case 0x43: // WLAN
        wireless_write_wlan_param(&buffer[8],header->len - CMD_STRUCT_SZ);
        valid = true;
        break;
      case 0x4E: // USER PARAM
        eepromWrite(OFFSET_NVM_USER,header->len - CMD_STRUCT_SZ,&buffer[8]);
        break;
      case 0x4F: // USER PARAM
        eepromWrite(OFFSET_NVM_USER2,header->len - CMD_STRUCT_SZ,&buffer[8]);
        break;
      case 0x0E: // set rtc
        {
          rtc_config_t *cfg = (rtc_config_t*)&buffer[8];
          RTCDateTime timespec;
          struct tm tim;
          tim.tm_year = cfg->yy;
          tim.tm_mon = cfg->mm-1;
          tim.tm_mday = cfg->dd;
          tim.tm_hour = cfg->hh;
          tim.tm_min = cfg->nn;
          tim.tm_sec = cfg->ss;
          rtcConvertStructTmToDateTime(&tim,0,&timespec);
          rtcSetTime(&RTCD1,&timespec);
          valid = true;          
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
        chThdYield();
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
    streamWrite(chp,(uint8_t*)&header,8);
    chThdYield();
    if(hin->pid == 0x01){
      startTransfer(chp);
    }
    else{
      valid_log_fileName();
      startTransfer((BaseSequentialStream*)&SDFS1);
    }
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
    chThdYield();
    stopTransfer();
  }
}
void cmd_stop(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  BinCommandHeader header;
  header.magic1 = MAGIC1;
  header.magic2 = MAGIC2;
  header.type = MASK_CMD_RET_ERR;

  if(runTime.opThread){
    header.type = MASK_CMD_RET_OK;
    stopTransfer();
  }
  header.pid = 0;
  header.len = CMD_STRUCT_SZ;
  header.chksum = checksum((uint8_t*)&header,header.len);
  streamWrite(chp,(uint8_t*)&header,8);
  chThdYield();
}
void cmd_read(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  BinCommandHeader header;
  header.magic1 = MAGIC1;
  header.magic2 = MAGIC2;
  header.type = MASK_CMD_RET_ERR;

  if(runTime.opThread){
    header.type = MASK_CMD_RET_OK;
    stopTransfer();
  }
  header.pid = 0;
  header.len = CMD_STRUCT_SZ;
  header.chksum = checksum((uint8_t*)&header,header.len);
  streamWrite(chp,(uint8_t*)&header,8);
}

void cmd_list_file(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
//  BinCommandHeader *header = (BinCommandHeader*)runTime.buffer;
//  header->magic1 = MAGIC1;
//  header->magic2 = MAGIC2;
//  header->type = FILE_OP_ERR;
//  header->pid = hin->pid;
  
  sdfsListFile(&SDFS1);
  
}
void cmd_read_file(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
//  BinCommandHeader *header = (BinCommandHeader*)runTime.buffer;
//  header->magic1 = MAGIC1;
//  header->magic2 = MAGIC2;
//  header->type = 0xA3;
//  header->pid = hin->pid;
  //uint8_t buffer[256];
  char fileName[32];
  uint32_t offset;
  UINT readSz = 256;
  UINT szRead;
  if(hin->len == 48){ // name[32], offset/size 4/4
    // read remaining data
    streamRead(chp,fileName,32);
    streamRead(chp,(uint8_t*)&offset,4);
    streamRead(chp,(uint8_t*)&szRead,4);
    memcpy(SDFS1.fileName,fileName,32);
    SDFS1.readOffset = offset;
    sdReadFile(&SDFS1);
    
  }
}
void cmd_write_file(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  
}

void cmd_remove_file(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  char fileName[32];
  uint32_t offset;
  UINT readSz = 256;
  UINT szRead;
  if(hin->len == 40){ // name[32]
    // read remaining data
    streamRead(chp,fileName,32);
    memcpy(SDFS1.fileName,fileName,32);
    sdReMoveFile(&SDFS1);
    
  }
}


int main()
{
  thread_t *shelltp1 = NULL;
  halInit();
  chSysInit();
  
  vnode_app_init();
}




