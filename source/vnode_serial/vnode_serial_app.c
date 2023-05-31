#include "ch.h"
#include "hal.h"
#include "vnode_app.h"
#include "adxl355_dev.h"
#include "adxl355_defs.h"
#include "nvm_config.h"
#include "ylib/eeprom/at24_eep.h"
#include "ylib/numeric/time/time_domain.h"
#include "bincmd_shell.h"
#include "ylib/sensor/ntc/ntc.h"
#include "ylib/sensor/ism330dlc/ism330_dev.h"

#include "ylib/sensor/bmi160/bmi160_defs.h"
#include "ylib/sensor/bmi160/bmi160_dev.h"

#include "ylib/numeric/filters/recursive.h"

#include "ylib/numeric/frequency/cmsis_fft.h"

#define NVM_FLAG        0xAC

#define EV_ADXL_FIFO_FULL       EVENT_MASK(0)
#define EV_IMU_FIFO_FULL        EVENT_MASK(1)
#define EV_EXEC_FFT             EVENT_MASK(2)

struct _nvmParam{
  uint8_t flag;
  node_param_t nodeParam;
  adxl355_cfg_t adxlParam;
  module_setting_t moduleParam;
  imu_config_t imuParam;
  time_domain_param_t time;
  mbRTUOpts_t mbParam;
  struct{
    uint16_t resistance;
    uint16_t beta;
    uint16_t beta_temp;
    uint16_t shunt_resistance;
  }ntc_config;
};


struct _runTime{
  thread_t *self;
  thread_t *opThread;
  virtual_timer_t vt,blinker;
  uint8_t state;
  time_domain_t time;
  _float_3d rms,peak,crest,velocity;
  _float_3d hisRMS,hisPEAK;
  event_listener_t el_sensor;
  uint8_t buffer[320];
  uint16_t rxSz;
  struct {
    uint16_t ms_on;
    uint16_t ms_off;
    uint16_t on_time,off_time;
    uint16_t countDown;
    ioportid_t port;
    ioportmask_t pad;
  }ledBlink;
  uint8_t sensorReady;
  float temperature[2];
  _hpf_3d hpf;
  _fft_data_t fft_data;
  uint16_t max_bin_index;
  float bin_freq;
  float freq[5];
  float fft_bins[FFT_SAMPLE_NUMBER>>1];
  bool fillFFT;
};

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      8

static adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];

//static void adccallback(ADCDriver *adcp, adcsample_t *buffer,size_t n);
static void adccallback(ADCDriver *adcp);

static void adcerror(ADCDriver *adcp, adcerror_t err);

static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  NULL,
  0,
  ADC_CR2_SWSTART,
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480),
  0,
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)
};

static struct _nvmParam nvmParam, *app_nvmParam;
static struct _runTime runTime, *app_runTime;
static void blink_cb(void *arg);


static SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  0,
  SPI_CR1_BR_2
};

_adxl_interface_t adxlInterface = {
  &SPID1,
  &spicfg
};

static adxl355_config_t adxlConfig = {
  0x0,0x0,0x0,0x0,
  48,
  GPIOA,4,
  GPIOB,2,
  GPIOB,1,
};


static ADXLDriver adxl = {
  &adxlInterface,
  &adxlConfig
};

_ism330_interface_t ismInterface = {
  &SPID1,
  &spicfg
};

static ism330_config_t ismConfig = {
  {0x0,ISM330DLC_XL_ODR_104Hz,ISM330DLC_2g,0x0},
  {0x0,ISM330DLC_GY_ODR_104Hz,ISM330DLC_2000dps,0x0},
  GPIOA,4,
  GPIOA,2,
  GPIOA,3
};

static ISM330Driver ism330 = {
  &ismInterface,
  &ismConfig
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
  NVM_FLAG,
  "VNODE",
  0x12345678, 
  0x00000001,
  "Grididea-ST32",
  "VNODE_SERIAL" // supported config
};

/* Start Implementation */

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
    nvmParam.nodeParam.commType = COM_IF_SERIAL;
    nvmParam.nodeParam.opMode = OP_VNODE;
    memcpy(nvmParam.nodeParam.log_file_prefix,"SensorNode\0",11);
    nvmParam.nodeParam.logFileSize = 1024*1024*10;
    
    nvmParam.adxlParam.fs = 0x1;
    nvmParam.adxlParam.odr = ADXL355_ODR_2000;
    nvmParam.adxlParam.hpf = 0;
    
    memcpy((uint8_t*)&nvmParam.moduleParam,(uint8_t*)&module_default, sizeof(module_setting_t));
    
    
    nvmParam.time.sampleNumber = 1000;
    nvmParam.time.samplePeriodMs = 1000;
    
    nvmParam.mbParam.sla = 0x01;
    nvmParam.mbParam.baudrate = 192;
    nvmParam.mbParam.dataBits = 8;
    nvmParam.mbParam.parity = 2; // no parity
    nvmParam.mbParam.stopBits = 0;
    
    nvmParam.ntc_config.resistance = 100; // 0.1K
    nvmParam.ntc_config.beta = 3435; 
    nvmParam.ntc_config.beta_temp = 25;
    nvmParam.ntc_config.shunt_resistance = 100; // 0.1K
    
    // config for ISM-330 default
//    nvmParam.imuParam.accel.power = 0;
//    nvmParam.imuParam.accel.odr = ISM330DLC_XL_ODR_104Hz;
//    nvmParam.imuParam.accel.range = ISM330DLC_16g;
//    nvmParam.imuParam.accel.lpf = 0;
//    
//    nvmParam.imuParam.gyro.power = 0;
//    nvmParam.imuParam.gyro.odr = ISM330DLC_GY_ODR_104Hz;
//    nvmParam.imuParam.gyro.range = ISM330DLC_2000dps;
//    nvmParam.imuParam.gyro.lpf = 0;
    
    // config for BMI160 default
    nvmParam.imuParam.accel.power = BMI160_ACCEL_NORMAL_MODE;
    nvmParam.imuParam.accel.odr = BMI160_ACCEL_ODR_1600HZ;
    nvmParam.imuParam.accel.range = BMI160_ACCEL_RANGE_16G;
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
  //uint8_t buffer[64];
  adxl355_powerdown(dev);
  dev->config->intmask = ADXL355_INT_FULL_EN1;
  adxl355_set_filter(dev);
  adxl355_set_full_scale(dev);
  adxl355_set_interrupt(dev);
  
  switch(dev->config->fullscale){
  case 1:
    dev->sensitivity = 0.0000039*9.81;
    break;
  case 2:
    dev->sensitivity = 0.0000078*9.81;
    break;
  case 3:
  default:
    dev->sensitivity = 0.0000156*9.81;
    break;
  }
  rate = rate /(1 << dev->config->outputrate);
  adxl355_powerup(dev);    

  return ret;
}

static int8_t adxl355_cmd_stop(ADXLDriver *dev)
{
  dev->config->intmask = 0x0;
  adxl355_set_interrupt(dev);  
  return adxl355_powerdown(dev);
}

static void blink_cb(void *arg)
{
  chSysLockFromISR();
  
  if(runTime.ledBlink.countDown == 0){
    palSetPad(runTime.ledBlink.port, runTime.ledBlink.pad);
    runTime.ledBlink.off_time = 0;
    runTime.ledBlink.on_time =  0;
  }
  else{
    if(palReadPad(runTime.ledBlink.port, runTime.ledBlink.pad) == PAL_HIGH){
      runTime.ledBlink.off_time +=100;
      if(runTime.ledBlink.off_time >= runTime.ledBlink.ms_off){
        palClearPad(runTime.ledBlink.port,runTime.ledBlink.pad);
        runTime.ledBlink.on_time = 0;
      }
    }
    else{
      runTime.ledBlink.on_time +=100;
      if(runTime.ledBlink.on_time >= runTime.ledBlink.ms_on){
        palSetPad(runTime.ledBlink.port,runTime.ledBlink.pad);
        runTime.ledBlink.off_time = 0;
      }
    }
  }
  if(runTime.ledBlink.countDown > 0)
    runTime.ledBlink.countDown -= 100;
  chVTSetI(&runTime.blinker,TIME_MS2I(100),blink_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waOperation,8192);
static THD_FUNCTION(procOperation ,p)
{
  BinCommandHeader *header;
  BaseSequentialStream *stream = p;
  size_t sz;
  //uint8_t buf[320];
  //uint16_t bufSz = 0;
  int32_t data[96];
  //uint8_t *p_src,*p_dst;
  uint16_t bsz;
  //systime_t t_start;
  uint8_t pktCount = 0;
  static uint8_t adxl_sta;
 
  uint8_t opMode = nvmParam.nodeParam.opMode;
  uint8_t activeSensor = nvmParam.nodeParam.activeSensor;
    
  bool bStop = false;
//  time_domain_param_t *time = (time_domain_param_t*)buffer;
//  runTime.timedomain.nofSamples = (uint16_t)(rate * time->samplePeriodMs/1000);
//  runTime.timedomain.sampleTimesec = time->samplePeriodMs/1000.;
//  runTime.timedomain.scale.x = runTime.timedomain.scale.y = runTime.timedomain.scale.z = dev->sensitivity;



//  event_listener_t elSensor;
  //eventflags_t flags;
//  activeSensor = SENSOR_ISM330;
  //activeSensor = SENSOR_ADXL355;
  float scale = 1.0;
  runTime.bin_freq = 1.0;
  if(activeSensor == SENSOR_ADXL355){
    chEvtRegisterMask(&adxl.evsource,&runTime.el_sensor,EV_ADXL_FIFO_FULL);
//    runTime.ledBlink.ms_on = 500;
//    runTime.ledBlink.ms_off = 500;
    adxl355_get_fifo_size(&adxl,&sz);
    if(sz > 0){
      bStop = false;
    }
    adxl355_cmd_start(&adxl);  
    runTime.time.scale.x = adxl.sensitivity;
    runTime.time.scale.y = adxl.sensitivity;
    runTime.time.scale.z = adxl.sensitivity;
    scale = adxl.sensitivity;
    float rate = 4000./(1 << (adxl.config->outputrate));
    runTime.bin_freq = rate/(float)FFT_SAMPLE_NUMBER;
    
  }
  else if(activeSensor == SENSOR_BMI160){
    chEvtRegisterMask(&bmi160.evsource,&runTime.el_sensor,EV_IMU_FIFO_FULL );
//    runTime.ledBlink.ms_on = 500;
//    runTime.ledBlink.ms_off = 500;
    //bmi160_dev_init(&bmi160);
    bmi160_start(&bmi160);
    runTime.time.scale.x = bmi160.lsb_accel;
    runTime.time.scale.y = bmi160.lsb_accel;
    runTime.time.scale.z = bmi160.lsb_accel;
    scale = bmi160.lsb_accel;

    float rate = 1600./(1 << (0x0c - bmi160.imu.accel_cfg.odr + 1));
    runTime.bin_freq = rate/(float)FFT_SAMPLE_NUMBER;
  }
  else if(activeSensor == SENSOR_ISM330){
    chEvtRegisterMask(&ism330.evsource,&runTime.el_sensor,EV_IMU_FIFO_FULL );
//    runTime.ledBlink.ms_on = 500;
//    runTime.ledBlink.ms_off = 500;
    ism330_start(&ism330);
    
    runTime.time.scale.x = ism330.lsb_accel;
    runTime.time.scale.y = ism330.lsb_accel;
    runTime.time.scale.z = ism330.lsb_accel;
  }
  
  runTime.hpf.x.type = HPF;
  runTime.hpf.y.type = HPF;
  runTime.hpf.z.type = HPF;
  recursive_set_single_pole(&runTime.hpf.x,0.95);
  recursive_set_single_pole(&runTime.hpf.y,0.95);
  recursive_set_single_pole(&runTime.hpf.z,0.95);
  
//  uint8_t packetToIgnore = 5;
//  if(runTime.resetBuffer != NULL){
//    runTime.resetBuffer();
//  }
  //runTime.ledBlink.ms_off = 500;
  //runTime.ledBlink.ms_on = 500;
  uint32_t fifo_size;
  eventmask_t evt;
  uint8_t ignorePacket = 1;
  //runTime.fillFFT = true;
  //runTime.rxSz = CMD_STRUCT_SZ + 4;
  while(!bStop){
    evt = chEvtWaitAny(ALL_EVENTS);
   // flags = chEvtGetAndClearFlags(&elSensor);
    uint8_t *p_src, *p_dst;
    if(evt & EV_ADXL_FIFO_FULL){ // adxl int1, fifo full
      adxl355_get_status(&adxl,&adxl_sta);
      if((adxl_sta & 0x02) == 0x00) continue;
      //A0_HI;
      // read fifo data, sz indicate the number of records, not bytes
      adxl355_get_fifo_size(&adxl,&sz);
      fifo_size = sz;
      sz /= 3;
      if(sz){
        switch(opMode){
        case OP_STREAM:
          bsz = sz*9; // read x/y/z combo
          if(bsz >= 144){
            adxl355_read_fifo(&adxl,&runTime.buffer[runTime.rxSz],144); // each record has 9-bytes (x/y/z)*3
            runTime.rxSz += 144;
            if(runTime.rxSz > 270){                
                header = (BinCommandHeader*)runTime.buffer;
                header->magic1 = MAGIC1;
                header->magic2 = MAGIC2;
                header->type = MASK_DATA ;//| runTime.lbt;
                header->len = 288 + CMD_STRUCT_SZ + 4;
                header->pid = pktCount++;
                header->chksum = checksum(runTime.buffer,header->len);
                if(stream != NULL){
                  streamWrite(stream,runTime.buffer, header->len);
                }
                runTime.rxSz = CMD_STRUCT_SZ + 4;
              }
          }
          break;
        case OP_VNODE:
        case OP_OLED:
          bsz = sz*9;
          adxl355_read_fifo(&adxl,runTime.buffer,bsz); // each record has 9-bytes (x/y/z)*3
          bsz = sz*4*3;
          p_src = runTime.buffer;
          p_dst = (uint8_t*)data;
          p_dst += 3;
          for(uint16_t j=0;j<bsz;j++){
            if((j%4)==3){
              *(p_dst--)=0;
              p_dst = (uint8_t*)data + j + 4;
            }else{
              *(p_dst--)=*(p_src++);
            }
          }
          // time domain
          if(feed_fifo32(&runTime.time,(uint8_t*)data,sz)==1){
            // update modbus data
            memcpy((void*)&runTime.rms,(void*)&runTime.time.rms,12);
            memcpy((void*)&runTime.crest,(void*)&runTime.time.crest,12);
            memcpy((void*)&runTime.velocity,(void*)&runTime.time.velocity,12);
            runTime.peak.x = (runTime.time.peak.x - runTime.time.peakn.x);
            runTime.peak.y = (runTime.time.peak.y - runTime.time.peakn.y);
            runTime.peak.z = (runTime.time.peak.z - runTime.time.peakn.z);
            resetObject(&runTime.time);
          }
          
//          // FFT
//          if(runTime.fillFFT){
//            fft_feed_fifo_adxl(&runTime.fft_data,(uint8_t*)data,sz,scale);
//            if(runTime.fft_data.newData == 1){
//              memcpy(runTime.fft_bins,runTime.fft_data.zf, sizeof(float)*(FFT_SAMPLE_NUMBER>>1));
//              runTime.max_bin_index = runTime.fft_data.maxIndex+1; // +1 is for calculate right frequency
//              runTime.freq[0] = runTime.max_bin_index*runTime.bin_freq;
//              runTime.fft_data.newData = 0;
//              runTime.fillFFT = false;
//            }
//          }
          break;
        case OP_FNODE:
          bsz = sz*9;
          adxl355_read_fifo(&adxl,runTime.buffer,bsz); // each record has 9-bytes (x/y/z)*3
          bsz = sz*4*3;
          p_src = runTime.buffer;
          p_dst = (uint8_t*)data;
          p_dst += 3;
          for(uint16_t j=0;j<bsz;j++){
            if((j%4)==3){
              *(p_dst--)=0;
              p_dst = (uint8_t*)data + j + 4;
            }else{
              *(p_dst--)=*(p_src++);
            }
          }
          if(runTime.fillFFT){
            fft_feed_fifo_adxl(&runTime.fft_data,(uint8_t*)data,sz,scale);
            if(runTime.fft_data.newData == 1){
              memcpy(runTime.fft_bins,runTime.fft_data.zf, sizeof(float)*(FFT_SAMPLE_NUMBER>>1));
              runTime.max_bin_index = runTime.fft_data.maxIndex+1; // +1 is for calculate right frequency
              runTime.freq[0] = runTime.max_bin_index*runTime.bin_freq;
              runTime.fft_data.newData = 0;
              runTime.fillFFT = false;
            }
          }
          break;
        }
      }
      //A0_LO;
    }
    if(evt & EV_IMU_FIFO_FULL){
      uint16_t sz;
      uint16_t readSz;
      // check if fifo overrun first
      
      switch(opMode){
      case OP_STREAM:
        if(ism330_cmd_fifo_read(&ism330,&runTime.buffer[CMD_STRUCT_SZ],300,&readSz)){
          //runTime.RxSz = readSz+CMD_STRUCT_SZ;
          header = (BinCommandHeader*)runTime.buffer;
          header->magic1 = MAGIC1;
          header->magic2 = MAGIC2;
          header->type = MASK_DATA;// | runTime.lbt;
          header->len = 300 + CMD_STRUCT_SZ;
          header->pid = pktCount++;
          header->chksum = checksum(runTime.buffer,header->len);
          if(stream != NULL){
            streamWrite(stream,runTime.buffer, header->len);
          }
        }
        break;
      case OP_VNODE:
      case OP_OLED:
        if(activeSensor == SENSOR_ISM330){
          if(ism330_cmd_fifo_read(&ism330,runTime.buffer,300,&readSz)){
            sz = readSz/12; // nof records
            if(ignorePacket > 0){
              ignorePacket--;
              continue;
            }
            if(feed_fifo16_imu(&runTime.time,(uint8_t*)runTime.buffer,sz)==1){
              memcpy((void*)&runTime.rms,(void*)&runTime.time.rms,12);
              memcpy((void*)&runTime.crest,(void*)&runTime.time.crest,12);
              memcpy((void*)&runTime.velocity,(void*)&runTime.time.velocity,12);
              runTime.peak.x = (int32_t)((runTime.time.peak.x - runTime.time.peakn.x)*1000);
              runTime.peak.y = (int32_t)((runTime.time.peak.y - runTime.time.peakn.y)*1000);
              runTime.peak.z = (int32_t)((runTime.time.peak.z - runTime.time.peakn.z)*1000);
              runTime.peak.x /= 1000;
              runTime.peak.y /= 1000;
              runTime.peak.z /= 1000;
              
              runTime.rms.x = (int32_t)(runTime.rms.x * 1000)/1000;
              runTime.rms.y = (int32_t)(runTime.rms.y * 1000)/1000;
              runTime.rms.z = (int32_t)(runTime.rms.z * 1000)/1000;
              runTime.crest.x = (int32_t)(runTime.crest.x * 1000)/1000;
              runTime.crest.y = (int32_t)(runTime.crest.y * 1000)/1000;
              runTime.crest.z = (int32_t)(runTime.crest.z * 1000)/1000;
              runTime.velocity.x = (int32_t)(runTime.velocity.x * 1000)/1000;
              runTime.velocity.y = (int32_t)(runTime.velocity.y * 1000)/1000;
              runTime.velocity.z = (int32_t)(runTime.velocity.z * 1000)/1000;
              
              resetObject(&runTime.time);
              // send data via WIFI/BT
  //            header = (cmd_header_t*)buf;
  //            header->magic1 = MAGIC1;
  //            header->magic2 = MAGIC2;
  //            header->type = MASK_DATA;
  //            header->pid = pktCount++;
  //            uint8_t *ptr = &buf[CMD_STRUCT_SZ];
  //            memcpy(ptr,&runTime.peak,12);
  //            ptr += 12;
  //            memcpy(ptr,&runTime.rms,12);
  //            ptr += 12;
  //            memcpy(ptr,&runTime.crest,12);
  //            ptr += 12;
  //            memcpy(ptr,&runTime.velocity,12);
  //            ptr += 12;
  //            header->len = 48 + CMD_STRUCT_SZ;
  //            header->chksum = cmd_checksum(buf,header->len);
  //            if(stream != NULL){
  //              streamWrite(stream,runTime.buffer,header->len);
  //            }
            }
          }
        }
        else if(activeSensor == SENSOR_BMI160){
          //chSysLock();
          if(bmi160_fifo_read(&bmi160,runTime.buffer,300,&readSz) == MSG_OK){
            sz = readSz/12; // nof records
            if(ignorePacket > 0){
              ignorePacket--;
              continue;
            }
            if(feed_fifo16_imu_hpf(&runTime.time,(uint8_t*)runTime.buffer,&runTime.hpf,sz)==1){
              memcpy((void*)&runTime.rms,(void*)&runTime.time.rms,12);
              memcpy((void*)&runTime.crest,(void*)&runTime.time.crest,12);
              memcpy((void*)&runTime.velocity,(void*)&runTime.time.velocity,12);
              runTime.peak.x = (int32_t)((runTime.time.peak.x - runTime.time.peakn.x)*1000);
              runTime.peak.y = (int32_t)((runTime.time.peak.y - runTime.time.peakn.y)*1000);
              runTime.peak.z = (int32_t)((runTime.time.peak.z - runTime.time.peakn.z)*1000);
              runTime.peak.x /= 1000;
              runTime.peak.y /= 1000;
              runTime.peak.z /= 1000;
              
              runTime.rms.x = ((int32_t)(runTime.rms.x * 1000));
              runTime.rms.y = ((int32_t)(runTime.rms.y * 1000));
              runTime.rms.z = ((int32_t)(runTime.rms.z * 1000));
              runTime.crest.x = ((int32_t)(runTime.crest.x * 1000));
              runTime.crest.y = ((int32_t)(runTime.crest.y * 1000));
              runTime.crest.z = ((int32_t)(runTime.crest.z * 1000));
              runTime.velocity.x = ((int32_t)(runTime.velocity.x * 1000));
              runTime.velocity.y = ((int32_t)(runTime.velocity.y * 1000));
              runTime.velocity.z = ((int32_t)(runTime.velocity.z * 1000));
              runTime.rms.x /= 1000;
              runTime.rms.y /= 1000;
              runTime.rms.z /= 1000;
              runTime.crest.x /= 1000;
              runTime.crest.y /= 1000;
              runTime.crest.z /= 1000;
              runTime.velocity.x /= 1000;
              runTime.velocity.y /= 1000;
              runTime.velocity.z /= 1000;


              resetObject(&runTime.time);
            }
//            fft_feed_fifo_imu(&runTime.fft_data,(uint8_t*)runTime.buffer,sz,scale);
//            if(runTime.fft_data.newData == 1){
//              memcpy(runTime.fft_bins,runTime.fft_data.zf, sizeof(float)*(FFT_SAMPLE_NUMBER>>1));
//              runTime.max_bin_index = runTime.fft_data.maxIndex+1; // +1 is for calculate right frequency
//              runTime.freq[0] = runTime.max_bin_index*runTime.bin_freq;
//              runTime.fft_data.newData = 0;
//            }
          }
          //chSysUnlock();
        }
        break; 
      case OP_FNODE:
        if(activeSensor == SENSOR_ISM330){
          if(ism330_cmd_fifo_read(&ism330,runTime.buffer,300,&readSz)){
            sz = readSz/12; // nof records
            if(ignorePacket > 0){
              ignorePacket--;
              continue;
            }
            if(runTime.fillFFT){
              fft_feed_fifo_imu(&runTime.fft_data,(uint8_t*)runTime.buffer,sz,scale);
              if(runTime.fft_data.newData == 1){
                memcpy(runTime.fft_bins,runTime.fft_data.zf, sizeof(float)*(FFT_SAMPLE_NUMBER>>1));
                runTime.max_bin_index = runTime.fft_data.maxIndex+1; // +1 is for calculate right frequency
                runTime.freq[0] = runTime.max_bin_index*runTime.bin_freq;
                runTime.fft_data.newData = 0;
                runTime.fillFFT = false;
              }
            }
          }
        }
        else if(activeSensor == SENSOR_BMI160){
          //chSysLock();
          if(bmi160_fifo_read(&bmi160,runTime.buffer,300,&readSz) == MSG_OK){
            sz = readSz/12; // nof records
            if(ignorePacket > 0){
              ignorePacket--;
              continue;
            }
            if(runTime.fillFFT){
              fft_feed_fifo_imu(&runTime.fft_data,(uint8_t*)runTime.buffer,sz,scale);
              if(runTime.fft_data.newData == 1){
                memcpy(runTime.fft_bins,runTime.fft_data.zf, sizeof(float)*(FFT_SAMPLE_NUMBER>>1));
                runTime.max_bin_index = runTime.fft_data.maxIndex+1; // +1 is for calculate right frequency
                runTime.freq[0] = runTime.max_bin_index*runTime.bin_freq;
                runTime.fft_data.newData = 0;
                runTime.fillFFT = false;
              }
            }
          }
        }
        break;
      }
    }
    bStop = chThdShouldTerminateX();
    if(bStop){
      if(activeSensor == SENSOR_ADXL355){
        adxl355_cmd_stop(&adxl);  
        // disable interrupt
        runTime.ledBlink.ms_on = 500;
        runTime.ledBlink.ms_off = 500;
        //chEvtUnregister(&runTime.es_sensor,&elSensor);
      }
      else if(activeSensor == SENSOR_ISM330){
        //palDisableLineEvent(LINE_ISM_INT);
        //ism330_cmd_stop_config();
        //chEvtUnregister(&runTime.es_sensor,&elSensor);
      }
      runTime.ledBlink.ms_on = 1000;
      runTime.ledBlink.ms_off = 1000;
    }
  }
//  chThdExit((msg_t)0);
    chEvtUnregister(&adxl.evsource,&runTime.el_sensor);
  // turn off led
  palClearPad(GPIOC,3);
}

static void stopTransfer(void)
{
  if(runTime.opThread){
    chThdTerminate(runTime.opThread);
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    chVTReset(&runTime.vt);
  }
}

static void startTransfer(BaseSequentialStream *stream)
{
  if(!runTime.opThread){

    runTime.ledBlink.ms_off = 200;
    runTime.ledBlink.ms_on = 500;
    //chVTSet(&runTime.vt,TIME_MS2I(500),blink_cb,NULL);
    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO,procOperation,stream);
  }
  else{
    stopTransfer();
  }
}

// LIR = 40000/4 = 10,000 Hz, reload 1000 -> 100ms
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_4,
  STM32_IWDG_RL(1000)
};

void vnode_app_init()
{
  app_nvmParam = &nvmParam;
  app_runTime = &runTime;
  at24eep_init(&I2CD1,32,1024,0x50,2);
  load_settings();
  
  // start modbus slave
  mbPortInit(&nvmParam.mbParam);
  chVTObjectInit(&runTime.vt);
  
  runTime.ledBlink.countDown = 0;
  runTime.ledBlink.port = GPIOA;
  runTime.ledBlink.pad = 8;
  chVTObjectInit(&runTime.blinker);;
  chVTSet(&runTime.blinker,TIME_MS2I(100),blink_cb,NULL);
  //nvmParam.nodeParam.activeSensor = SENSOR_ISM330;
  
  //nvmParam.nodeParam.activeSensor = SENSOR_BMI160;

  //nvmParam.nodeParam.activeSensor = SENSOR_ADXL355;
  if(nvmParam.nodeParam.activeSensor == SENSOR_ADXL355){
    memcpy((uint8_t*)adxl.config,(uint8_t*)&nvmParam.adxlParam,sizeof(nvmParam.adxlParam));
    
    if(adxl355_init(&adxl) == ADXL355_OK){
      runTime.sensorReady = 1;
      adxl355_powerup(&adxl);
      adxl355_powerdown(&adxl);
    }
  }
  else if(nvmParam.nodeParam.activeSensor == SENSOR_ISM330){
    memcpy((uint8_t*)&ism330.config->accel,(uint8_t*)&nvmParam.imuParam.accel,sizeof(nvmParam.imuParam.accel));
    memcpy((uint8_t*)&ism330.config->gyro,(uint8_t*)&nvmParam.imuParam.gyro,sizeof(nvmParam.imuParam.gyro));
    
    if(ism330_init(&ism330) == MSG_OK){
      runTime.sensorReady = SENSOR_ISM330;
    }
  }
  
  else if(nvmParam.nodeParam.activeSensor == SENSOR_BMI160){   
    if(bmi160_dev_init(&bmi160) == MSG_OK){
      runTime.sensorReady = SENSOR_BMI160;
    }
    memcpy((uint8_t*)&bmi160.imu.accel_cfg,(uint8_t*)&nvmParam.imuParam.accel,sizeof(nvmParam.imuParam.accel));
    memcpy((uint8_t*)&bmi160.imu.gyro_cfg,(uint8_t*)&nvmParam.imuParam.gyro,sizeof(nvmParam.imuParam.gyro));
  }
  
  ntcInit(2.72,(float)nvmParam.ntc_config.shunt_resistance/10.,(float)nvmParam.ntc_config.resistance/10.,nvmParam.ntc_config.beta,nvmParam.ntc_config.beta_temp);

  // start ADC
  adcStart(&ADCD1,NULL);
  adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);
  

  //uint8_t tmp[320];
  runTime.self = chThdGetSelfX();
  
  timeDomainInit(&runTime.time);
  runTime.time.nofSamples = nvmParam.time.sampleNumber;
  cmsis_fft_init(&runTime.fft_data);
  
//  chEvtRegisterMask(&SDFS1.evs_insertion, &runTime.el_sdfs, EVENT_MASK(1));
  startTransfer(NULL);
  wdgStart(&WDGD1, &wdgcfg);
  while(1){
    //eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    wdgReset(&WDGD1);
    chThdSleepMilliseconds(50);
  }
}

int main()
{
  thread_t *shelltp1 = NULL;
  RCC->CSR |= RCC_CSR_RMVF;
  halInit();
  chSysInit();
  
 
  vnode_app_init();
}

/* modbus register map
  40000 ~005: X/Y/Z Peak value, float
  40006 ~ 40011: X/Y/Z RMS value, float

  41000 : ID
  41001 :Baud, 0/1/2/3/4: 9600/19200/38400/57600/115200
  41002 : Parity, 0/1/2 ODD/EVEN/NONE
  41005 : Number of sample to run calculation

  40200 : ADXL Range, 1/2/3 2/4/8G
  40201 : ODR, 0: 4000, etc
  40202 : HPF
*/

static void mapMBWord(uint8_t *dptr, uint8_t *val)
{
  *dptr = *(val+1);
  *(dptr+1) = *val;
}

static void mapMBFloat(uint8_t *dptr, uint8_t *val)
{
  *(dptr+0) = *(val+1);
  *(dptr+1) = *(val+0);
  *(dptr+2) = *(val+3);
  *(dptr+3) = *(val+2);
}


#define RANGE_CHECK(a,b,c)      ((a >= b) && (a <= c))
int8_t vnode_modbus_handler(uint16_t address, uint8_t *dptr, uint8_t rw)
{
  
  int8_t ret = 0;
  int16_t iv16;
  if(rw == 0){ // read
    if(RANGE_CHECK(address,0,5)){
      uint8_t *sptr = (uint8_t*)&runTime.peak;
      sptr += address *2;
      memcpy(dptr,sptr,4);
      ret = 2;
    }
    else if(RANGE_CHECK(address,6,11)){
      uint8_t *sptr = (uint8_t*)&runTime.rms;
      sptr += (address - 6) *2;
      memcpy(dptr,sptr,4);
      ret = 2;
    }
    else if(RANGE_CHECK(address,12,17)){
      uint8_t *sptr = (uint8_t*)&runTime.crest;
      sptr += (address - 12) *2;
      memcpy(dptr,sptr,4);
      ret = 2;
    }
    else if(RANGE_CHECK(address,18,21)){
      uint8_t *sptr = (uint8_t*)&runTime.temperature;
      sptr += (address - 18) *2;
      memcpy(dptr,sptr,4);
      ret = 2;
    }
    else if(address == 198){
      iv16 = nvmParam.nodeParam.opMode;
      memcpy(dptr,(uint8_t *)&iv16,2);
      ret = 1;
    }
    else if(address == 199){
      iv16 = nvmParam.nodeParam.activeSensor;
      memcpy(dptr,(uint8_t *)&iv16,2);
      ret = 1;
    }
    else if(RANGE_CHECK(address,200,202)){
      switch(address - 200){
      case 0:
        switch(nvmParam.nodeParam.activeSensor){
        case SENSOR_ADXL355:
          iv16 = nvmParam.adxlParam.fs;
          break;
        case SENSOR_BMI160:
        case SENSOR_ISM330:
          iv16 = nvmParam.imuParam.accel.range;
          break;
        default:break;
        }
        ret = 1;
        break;
      case 1:
        switch(nvmParam.nodeParam.activeSensor){
        case SENSOR_ADXL355:
          iv16 = nvmParam.adxlParam.odr;
          break;
        case SENSOR_BMI160:
        case SENSOR_ISM330:
          iv16 = nvmParam.imuParam.accel.odr;
          break;
        default:break;
        }
        ret = 1;
        break;
      case 2:
        switch(nvmParam.nodeParam.activeSensor){
        case SENSOR_ADXL355:
          iv16 = nvmParam.adxlParam.hpf;
          break;
        case SENSOR_BMI160:
        case SENSOR_ISM330:
          iv16 = nvmParam.imuParam.accel.lpf;
          break;
        default:break;
        }
        ret = 1;
        break;
      default:break;
      }
      memcpy(dptr,(uint8_t *)&iv16,2);
    }
    else if(RANGE_CHECK(address,1000,1020)){
      switch(address - 1000){
      case 0:
        iv16 = nvmParam.mbParam.sla;
//        memcpy(dptr,(uint8_t *)&nvmParam.mbParam.sla,1);
        ret = 1;
        break;
      case 1:
        iv16 = nvmParam.mbParam.baudrate;
        ret = 1;
        break;
      case 2:
        iv16 = nvmParam.mbParam.parity;
        ret = 1;
        break;
      case 5:
        iv16 = nvmParam.time.sampleNumber;
        ret = 1;
        break;
      case 10:
        iv16 = nvmParam.ntc_config.resistance;
        ret = 1;
        break;
      case 11:
        iv16 = nvmParam.ntc_config.shunt_resistance;
        ret = 1;
        break;
      case 12:
        iv16 = nvmParam.ntc_config.beta;
        ret = 1;
        break;
      case 13:
        iv16 = nvmParam.ntc_config.beta_temp;
        ret = 1;
        break;
      default:
        ret = 1;
        break;
      }
      memcpy(dptr,(uint8_t *)&iv16,2);
    }
    else if(RANGE_CHECK(address,1021,1023)){
      if(address == 1021){
        iv16 = runTime.max_bin_index + 1024;
        memcpy(dptr,(uint8_t *)&iv16,2);
        ret = 1;
      }
      else if(address == 1022){
        mapMBFloat(dptr,(uint8_t*)&runTime.freq[0]);
        ret = 2;
      }
      
    }
    else if(RANGE_CHECK(address,1024,3072)){
      mapMBFloat(dptr,(uint8_t*)&runTime.fft_bins[(address - 1024)>>1]);
      ret = 2;
    }
    else{
      iv16 = 0;
      memcpy(dptr,(uint8_t *)&iv16,2);
      ret = 1;
    }
  }
  else{
    uint8_t save = 0;
    if(address == 198){
      mapMBWord((uint8_t*)&iv16,dptr);
      if((iv16 == OP_VNODE) || (iv16 == OP_FNODE)){
        nvmParam.nodeParam.opMode = (uint8_t)iv16;
        save = 1;
      }
    }
    else if(address == 199){
      mapMBWord((uint8_t*)&iv16,dptr);
      if((iv16 & SENSOR_ADXL355) == SENSOR_ADXL355){
        nvmParam.nodeParam.activeSensor = SENSOR_ADXL355;
        save =1;
      }
      else if((iv16 & SENSOR_BMI160) == SENSOR_BMI160){
        nvmParam.nodeParam.activeSensor = SENSOR_BMI160;
        save =1;
      }
      else if((iv16 & SENSOR_ISM330) == SENSOR_ISM330){
        nvmParam.nodeParam.activeSensor = SENSOR_ISM330;
        save =1;
      }
      ret = 1;
    }
    else if(RANGE_CHECK(address,200,202)){
      mapMBWord((uint8_t*)&iv16,dptr);
      switch(address - 200){
      case 0:
        switch(nvmParam.nodeParam.activeSensor){
        case SENSOR_ADXL355:
          if(RANGE_CHECK(iv16,1,3)){
            nvmParam.adxlParam.fs = iv16;
            save = 1;
          }
          break;
        case SENSOR_BMI160:
          if((iv16 == 0x03) || (iv16 == 0x05) || (iv16 == 0x08) || (iv16 == 0x0c)){
            nvmParam.imuParam.accel.range = iv16;
            save = 1;
          }
          break;
        case SENSOR_ISM330:
          if(RANGE_CHECK(iv16,0,3)){
            nvmParam.imuParam.accel.range = iv16;
            save = 1;
          }
          break;
        default:break;
        }
        ret = 1;
        break;
      case 1:
        switch(nvmParam.nodeParam.activeSensor){
        case SENSOR_ADXL355:
          if(RANGE_CHECK(iv16,0,10)){
            nvmParam.adxlParam.odr = iv16;
            save = 1;
          }
          break;
        case SENSOR_BMI160:
          if(RANGE_CHECK(iv16,1,12)){
            nvmParam.imuParam.accel.odr = iv16;
            save = 1;
          }
          break;
        case SENSOR_ISM330:
          if(RANGE_CHECK(iv16,0,10)){
            nvmParam.imuParam.accel.odr = iv16;
            save = 1;
          }
          break;
        default:break;
        }
        ret = 1;
        break;
      case 2:
        switch(nvmParam.nodeParam.activeSensor){
        case SENSOR_ADXL355:
          nvmParam.adxlParam.hpf = iv16;
          save = 1;
          break;
        case SENSOR_BMI160:
          if((iv16 >=0) && (iv16 <=2)){
            nvmParam.imuParam.accel.lpf = iv16;
            save = 1;
          }
          break;
        case SENSOR_ISM330:
          break;
        default:break;
        }
        ret = 1;
        break;
      default:break;
      }
    }
    else if(RANGE_CHECK(address,1000,1020)){
      mapMBWord((uint8_t*)&iv16,dptr);
      switch(address - 1000){
      case 0:
        if(RANGE_CHECK(iv16,1,240)){
          nvmParam.mbParam.sla = iv16;
          save = 1;
        }
        ret = 1;
        break;
      case 1:
        nvmParam.mbParam.baudrate = iv16;
        ret = 1;
        save = 1;
        break;
      case 2:
        if(RANGE_CHECK(iv16,0,2)){
          nvmParam.mbParam.parity = iv16;
          save = 1;
        }
        ret = 1;
        break;
      case 5:
        nvmParam.time.sampleNumber = iv16;
        ret = 1;
        save = 1;
        break;
      case 10: // ntc section
        nvmParam.ntc_config.resistance = iv16;
        ret = 1;
        break;
      case 11:
        nvmParam.ntc_config.shunt_resistance = iv16;
        ret = 1;
        break;
      case 12:
        nvmParam.ntc_config.beta = iv16;
        ret = 1;
        break;
      case 13:
        nvmParam.ntc_config.beta_temp = iv16;
        ret = 1;
        break;
      default:
        ret = 1;
        break;
      }
    }
    else if(address == 1021){
      mapMBWord((uint8_t*)&iv16,dptr);
      if(iv16 == 0xAA){
        if(!runTime.fillFFT){
          runTime.fillFFT = true;
        }
      }
    }
    else if(address == 299){
      mapMBWord((uint8_t*)&iv16,dptr);
      if((iv16 == 1290) || (iv16 == 0x1290)){
        chSysDisable();
        NVIC_SystemReset();
      }
    }
    else{
      ret = 1;
    }
    if(save == 1){
      save_settings(0);
    }
  }
  runTime.ledBlink.countDown = 1000;
  return ret;
}


//static void adccallback(ADCDriver *adcp, adcsample_t *buffer,size_t n)
static void adccallback(ADCDriver *adcp)
{
  (void)adcp;
  uint16_t chSum[3] = {0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    for(uint8_t j=0;j<ADC_GRP1_NUM_CHANNELS;j++){
      chSum[j] += samples[i*ADC_GRP1_NUM_CHANNELS+j];
    }
  }
  for(uint8_t j=0;j<ADC_GRP1_NUM_CHANNELS;j++){
    chSum[j] >>= 3;
  }
  
  for(uint8_t j=0;j<ADC_GRP1_NUM_CHANNELS;j++){
    if((chSum[j] < 100) || (chSum[j] > 4000)){
      runTime.temperature[j] = 0;
    }
    else{
      float r = (float)(chSum[j])/(4096.);
  //    runTime.temperature[j] = ntcGetTempFromRatioF_RTOP(r);
      runTime.temperature[j] = ntcGetTempFromRatioF_RBOT(r);
    }
  }
  
}

static void adcerror(ADCDriver *adcp, adcerror_t err)
{
  (void)adcp;
  while(1){}
  
}
