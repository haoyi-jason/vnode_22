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
//#include "task_storage.h"
#include "ylib/sensor/htud/htu2x.h"
#include "ylib/protocol/hs_uart/hs_uart.h"
#include "ylib/sensor/bmi160/bmi160_defs.h"

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
  imu_config_t imuParam;;
};

static void startTransfer(BaseSequentialStream*);

static struct _nvmParam nvmParam, *app_nvmParam;

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

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
BinShellCommand commands[] ={
  {{0xab,0xba,0xA2,0x00,0,0},cmd_config}, // node param
  {{0xab,0xba,0xA2,0x01,0,0},cmd_config}, // adxl param
  {{0xab,0xba,0xA2,0x02,0,0},cmd_config}, // imu param
  {{0xab,0xba,0xA2,0x40,0,0},cmd_config}, // module param
  {{0xab,0xba,0xA2,0x43,0,0},cmd_config}, // wlan param
  {{0xab,0xba,0xA2,0x4F,0,0},cmd_config}, // user param
  {{0xab,0xba,0xA2,0xFF,0,0},cmd_config}, // sample interval
  {{0xab,0xba,0xA1,0x01,0,0},cmd_start}, // start
  {{0xab,0xba,0xA1,0x02,0,0},cmd_start}, // start log to sd
  {{0xab,0xba,0xA1,0x00,0,0},cmd_stop}, // stop
  {{0xab,0xba,0x01,0x00,0,0},cmd_read},
  {{0xab,0xba,0xA3,0x11,0,0},cmd_list_file},
  {{0xab,0xba,0xA3,0x03,0,0},cmd_read_file},
  {{0xab,0xba,0xA3,0x04,0,0},cmd_write_file},
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
  uint8_t buffer[320];
  mutex_t mutex;
  uint16_t rxSz;
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
  uint8_t pollIntervalMs;
  uint32_t readCount;
  systime_t start;
  float rate;
};

static struct _runTime runTime, *app_runTime;

static SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  0,
  SPI_CR1_BR_2
};

static SerialConfig serialCfg={
  230400
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

const module_setting_t module_default = {
  NVM_FLAG,
  "DoorSpeed",
  0x12345678, 
  0x00000001,
  "Grididea-ST32",
  "DoorSpeed" // supported config
};

static void update_adc()
{
  int16_t chSum[3] = {0,0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    chSum[0] += runTime.samples[i * 3];
    chSum[1] += runTime.samples[i * 3 +1];
    chSum[2] += runTime.samples[i * 3 +2];
  }
  chSum[0] >>= 2;
  chSum[1] >>=2;
  chSum[2] >>=2;
  
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
  chVTSetI(&runTime.vt, TIME_MS2I(500),blink_cb,NULL);
  chSysUnlockFromISR();
}

static void user_button_isr(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_USER_BUTTON);
  chSysUnlockFromISR();
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
    nvmParam.flag = NVM_FLAG;
    nvmParam.nodeParam.activeSensor = SENSOR_BMI160;
    nvmParam.nodeParam.commType = COM_IF_BT;
    nvmParam.nodeParam.opMode = OP_STREAM;
    memcpy(nvmParam.nodeParam.log_file_prefix,"DoorSpeed\0",11);
    nvmParam.nodeParam.logFileSize = 1024*1024*10;
    
//    nvmParam.adxlParam.fs = 0x1;
//    nvmParam.adxlParam.odr = ADXL355_ODR_500;
//    nvmParam.adxlParam.hpf = 0;
    
    memcpy((uint8_t*)&nvmParam.moduleParam,(uint8_t*)&module_default, sizeof(module_setting_t));
    
    nvmParam.imuParam.accel.power = BMI160_ACCEL_NORMAL_MODE;
    nvmParam.imuParam.accel.odr = BMI160_ACCEL_ODR_1600HZ;
    nvmParam.imuParam.accel.range = BMI160_ACCEL_RANGE_2G;
    nvmParam.imuParam.accel.lpf = 0;
    
    nvmParam.imuParam.gyro.power = BMI160_GYRO_NORMAL_MODE;
    nvmParam.imuParam.gyro.odr = BMI160_GYRO_ODR_1600HZ;
    nvmParam.imuParam.gyro.range = BMI160_GYRO_RANGE_2000_DPS;
    nvmParam.imuParam.gyro.lpf = 0; 
    //nvm_flash_write(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
    eepromWrite(OFFSET_NVM_CONFIG,nvmSz,(uint8_t*)&nvmParam);
  }
  
}

static void save_settings(uint8_t option)
{
  eepromWrite(OFFSET_NVM_CONFIG,sizeof(nvmParam),(uint8_t*)&nvmParam);
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
  adxl355_set_interrupt(dev);
  
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
  adxl355_set_interrupt(dev);  
  return adxl355_powerdown(dev);
}

static uint32_t leastSetBit(uint32_t mask)
{
  uint32_t ret = 0;
  for(uint8_t i = 0; i < 32; i++){
    ret = ( 1 << i);
    if((mask & ret) == ret){
      break;
    }
  }
  return ret;
}

static THD_WORKING_AREA(waOperation,1200);
static THD_FUNCTION(procOperation ,p)
{
  BinCommandHeader *header;
  BaseSequentialStream *stream = p;
  SerialDriver *dev = p;
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
  uint32_t eid = EVENT_MASK(0) | EVENT_MASK(1);
  //eventflags_t flags = ~(CHN_OUTPUT_EMPTY | CHN_TRANSMISSION_END);
  //chEvtRegisterMaskWithFlags(&SD1.event,&runTime.el_sensor, eid,flags);
  //sdStart(&SD1,&serialCfg);
  hs_uart_init(&UARTD1,230400);
  // config sub mcu
  header = (BinCommandHeader*)runTime.buffer;
  header->magic1 = MAGIC1;
  header->magic2 = MAGIC2;
  header->type = 0xA2;
  header->len = CMD_STRUCT_SZ + 1;
  header->pid = 0xFF;
  runTime.buffer[8] = runTime.pollIntervalMs;
  header->chksum = checksum(runTime.buffer,header->len);
  //streamWrite(&SD1,runTime.buffer,header->len);
  hs_uart_send(runTime.buffer, header->len);
  chThdSleepMilliseconds(10); // short wait  
  header = (BinCommandHeader*)runTime.buffer;
  header->magic1 = MAGIC1;
  header->magic2 = MAGIC2;
  header->type = 0xA2;
  header->pid = 0x02; // imu param
  memcpy(&runTime.buffer[8],(uint8_t*)&nvmParam.imuParam, sizeof(nvmParam.imuParam));
  header->len = CMD_STRUCT_SZ + sizeof(nvmParam.imuParam);
  header->chksum = checksum(runTime.buffer,header->len);
  //streamWrite(&SD1,runTime.buffer,header->len);
  hs_uart_send(runTime.buffer, header->len);
  chThdSleepMilliseconds(10); // short wait  
    
  
    hs_uart_set_wm(80);
  bool bStop = false;

  runTime.ledBlink.ms_off = 500;
  runTime.ledBlink.ms_on = 500;
  uint32_t fifo_size;
  eventmask_t evt;
  runTime.rxSz = CMD_STRUCT_SZ + 6;
  eventflags_t pendFlags;
  uint32_t READSZ;
  palClearPad(GPIOA,8);
  
  header->magic1 = MAGIC1;
  header->magic2 = MAGIC2;
  header->type = MASK_DATA;
  //header->len = CMD_STRUCT_SZ + 6 + 192;
  
  runTime.readCount = 0;
  runTime.start = chVTGetSystemTime();
  //hs_uart_receive(&runTime.buffer[runTime.rxSz],80);
  while(!bStop){
    //chThdSleepMilliseconds(5);
        palClearPad(GPIOA,15);
      if((READSZ = hs_uart_receive(&runTime.buffer[runTime.rxSz],80)) == 80){
          //hs_uart_receive(&runTime.buffer[runTime.rxSz],80);
//        READSZ = streamRead(&SD1,&runTime.buffer[runTime.rxSz],80);
        palSetPad(GPIOA,15);
        runTime.readCount++;
        runTime.rxSz+=READSZ;
        if(runTime.rxSz >= 200){
          header->pid = pktCount++;
          header->len = runTime.rxSz;
          header->chksum = checksum(runTime.buffer,header->len);
          if(stream != NULL){
            streamWrite(stream,runTime.buffer,header->len);
          }
          runTime.rxSz = CMD_STRUCT_SZ + 6;
        }
      }
      systime_t elapsed = TIME_I2S(chVTTimeElapsedSinceX(runTime.start));
      runTime.rate = (float)(runTime.readCount / elapsed);
      
//    chEvtWaitAny(eid);
//    pendFlags = chEvtGetAndClearFlags(&runTime.el_sensor) & flags;
//    if(pendFlags != 0x00){
//      do{
//        eventflags_t reason = leastSetBit(pendFlags);
//        pendFlags &= ~reason;
//        switch(reason){
//        case CHN_INPUT_AVAILABLE:
//          if(SD1.iqueue.q_counter >=80){
//            READSZ = streamRead(&SD1,&runTime.buffer[runTime.rxSz],80);
//            palSetPad(GPIOA,15);
//            runTime.readCount++;
//            runTime.rxSz+=READSZ;
//            if(runTime.rxSz >= 200){
//              header->pid = pktCount++;
//              header->chksum = checksum(runTime.buffer,header->len);
//              if(stream != NULL){
//                streamWrite(stream,runTime.buffer,header->len);
//              }
//              runTime.rxSz = CMD_STRUCT_SZ + 6;
//            }
//            palClearPad(GPIOA,15);
//          }
//          
//          break;
//        case SD_OVERRUN_ERROR:
//          break;
//        case SD_BREAK_DETECTED:
//          break;
//        case SD_FRAMING_ERROR:
//          break;
//        case SD_PARITY_ERROR:
//          break;
//        case SD_NOISE_ERROR:
//          break;
//        case CHN_OUTPUT_EMPTY:
//          break;
//        case CHN_TRANSMISSION_END:
//          break;
//        default:
//          break;
//        }
//      }while(pendFlags);
//    }
    bStop = chThdShouldTerminateX();
    if(bStop){
      runTime.ledBlink.ms_on = 1000;
      runTime.ledBlink.ms_off = 1000;
    }
  }
//    chEvtUnregister(&adxl.evsource,&runTime.el_sensor);
  // turn off led
  palClearPad(GPIOC,3);
  palSetPad(GPIOA,8);
  //chEvtUnregister(&SD1.event,&runTime.el_sensor);
  //sdStop(&SD1);
  chThdExit((msg_t)0);
}

static void stopTransfer(void)
{
  if(runTime.opThread){
    chEvtSignal(runTime.opThread, EVENT_MASK(1));
    chThdTerminate(runTime.opThread);
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    chVTReset(&runTime.vt);
    palSetPad(GPIOC,3);
  }
}

static void startTransfer(BaseSequentialStream *stream)
{
  if(!runTime.opThread){
    runTime.activeStream = stream;
    chVTSet(&runTime.vt,TIME_MS2I(500),blink_cb,NULL);
    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO+1,procOperation,stream);
  }
  else{
    stopTransfer();
  }
}

static THD_WORKING_AREA(waShell,1024);
#define SHELL_WA_SIZE   1024
void vnode_app_init()
{
  app_nvmParam = &nvmParam;
  app_runTime = &runTime;
  at24eep_init(&I2CD1,32,1024,0x50,2);
  load_settings();
  
  nvmParam.nodeParam.activeSensor = 0x80 | SENSOR_BMI160; // doorspeed type
  // start wireless
  task_wireless_init(nvmParam.nodeParam.commType);
//  task_wireless_init(COMM_USE_WIFI);
  
  
  bincmd_shellInit();
  chVTObjectInit(&runTime.vt);
 
  runTime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binshellProc,(void*)&shell_cfg);

//  uint8_t tmp[320];
//  chEvtRegisterMask(&SDFS1.evs_insertion, &runTime.el_sdfs, EVENT_MASK(0) | EVENT_MASK(1));
  chEvtRegisterMask(&SDW1.es, &runTime.el_wireless,RSI_APP_EVENT_SPP_CONN | RSI_APP_EVENT_SPP_DISCONN );
  // enable user button interrupt
  palSetLineCallback(PAL_LINE(GPIOC,13),user_button_isr,NULL);
  palEnableLineEvent(PAL_LINE(GPIOC,13),PAL_EVENT_MODE_RISING_EDGE);
  runTime.self = chThdGetSelfX();
//  chEvtRegisterMask(&SDFS1.evs_insertion, &runTime.el_sdfs, EVENT_MASK(1));
  
  // start ADC for battery voltage measurment
  adcStart(&ADCD1,NULL);
  adcStartConversion(&ADCD1,&adcgrpcfg,runTime.samples,ADC_GRP1_BUF_DEPTH);
//  htu2xinit(&I2CD1,(I2CConfig*)&i2ccfg);
  static uint16_t cntr = 20;
  while(1){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    eventflags_t flags = chEvtGetAndClearFlags(&runTime.el_sdfs);
   
    if(evt & EV_USER_BUTTON){
//      startTransfer((BaseSequentialStream*)&SDW1);
      startTransfer(NULL);
    }

    flags = chEvtGetAndClearFlags(&runTime.el_wireless);
    if(flags & RSI_APP_EVENT_SPP_CONN){
      palClearPad(GPIOC,3);
    }
    if(flags & RSI_APP_EVENT_SPP_DISCONN){
      palSetPad(GPIOC,3);
      stopTransfer();
    }
    cntr--;
    if(cntr == 0){
      if(runTime.opThread != NULL){
        uint16_t htu_data[2];
        update_adc();
        memcpy(&runTime.buffer[CMD_STRUCT_SZ], (uint8_t*)&htu_data[0],4);
      }
      cntr = 20;
    }
    chThdSleepMilliseconds(50);
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
        break;
      case 0x43: // WLAN
        resp->len = get_wlan_config(&buffer[8],256);
        valid = true;
        break;
      case 0x4F: // USER PARAM
        eepromRead(OFFSET_NVM_USER,256,&buffer[8]);
        resp->len = 264;
        valid = 1;
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
        break;
      case 0x43: // WLAN
        set_wlan_config(&buffer[8],header->len - CMD_STRUCT_SZ);
        break;
      case 0x4F: // USER PARAM
        eepromWrite(OFFSET_NVM_USER,header->len - CMD_STRUCT_SZ,&buffer[8]);
        break;
      case 0xFF: // rate config
        runTime.pollIntervalMs = buffer[8];
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
      //valid_log_fileName();
      //startTransfer((BaseSequentialStream*)&SDFS1);
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
  
  //sdfsListFile(&SDFS1);
  
}
void cmd_read_file(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
//  BinCommandHeader *header = (BinCommandHeader*)runTime.buffer;
//  header->magic1 = MAGIC1;
//  header->magic2 = MAGIC2;
//  header->type = 0xA3;
//  header->pid = hin->pid;
  //uint8_t buffer[256];


}
void cmd_write_file(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  
}



int main()
{
  thread_t *shelltp1 = NULL;
  halInit();
  chSysInit();
  
  vnode_app_init();
}




