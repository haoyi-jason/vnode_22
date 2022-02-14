#include "ch.h"
#include "hal.h"
#include "vnode_app.h"
#include "adxl355_dev.h"
#include "adxl355_defs.h"
#include "bincmd_shell.h"
#include "nvm_config.h"
#include "usbcfg.h"
#include "bootConfig.h"

static struct{
  uint8_t flag;
  node_param_t nodeParam;
  adxl355_cfg_t adxlParam;
  module_setting_t moduleParam;
}nvmParam;

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

#define EV_ADXL_FIFO_FULL EVENT_MASK(0)
#define EV_ISM_FIFO_FULL EVENT_MASK(1)

void cmd_config(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_start(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_stop(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_read(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_boot(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);
void cmd_idn(BaseSequentialStream *chp, BinCommandHeader *header, uint8_t *data);

BinShellCommand commands[] ={
  {{0xab,0xba,0xA2,0x00,0,0},cmd_config}, // node param
  {{0xab,0xba,0xA2,0x01,0,0},cmd_config}, // adxl param
  {{0xab,0xba,0xA2,0x40,0,0},cmd_config}, // module param
  {{0xab,0xba,0xA1,0x01,0,0},cmd_start}, // start
  {{0xab,0xba,0xA1,0x00,0,0},cmd_stop}, // stop
  {{0xab,0xba,0x01,0x00,0,0},cmd_read},
  {{0xab,0xba,0xAF,0x00,0,0},cmd_boot},
  {{0xab,0xba,0xA4,0x00,0,0},cmd_idn},
  {{0xab,0xba,0x01,0x00,0,0},NULL},
};

static const BinShellConfig shell_cfg = {
  (BaseSequentialStream *)&SDU1,
  commands
};

struct {
  uint8_t state;
  uint8_t *rxPtr;
  uint8_t *txPtr;
  uint8_t *bufEnd;
  uint8_t buffer[1200];
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
}runTime;

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
  GPIOA,1,
  GPIOA,2,
};


static ADXLDriver adxl = {
  &adxlInterface,
  &adxlConfig
};

const module_setting_t module_default = {
  0xBB,
  "VNODE",
  0x12345678, 
  0x00000001,
  "Grididea-AT32",
  "VSS-II" // supported config
};
static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
  if(nvmSz > SZ_NVM_CONFIG){
    // error
    while(1);
  }
  nvm_flash_read(OFFSET_NVM_CONFIG,(uint8_t*)&nvmParam,nvmSz);
  if(nvmParam.flag != 0xAA){
    nvmParam.flag = 0xAA;
    nvmParam.nodeParam.activeSensor = SENSOR_ADXL355;
    nvmParam.nodeParam.commType = COM_IF_USB;
    nvmParam.nodeParam.opMode = OP_STREAM;
    
    nvmParam.adxlParam.fs = 0x1;
    nvmParam.adxlParam.odr = ADXL355_ODR_500;
    nvmParam.adxlParam.hpf = 0;
    
    memcpy((uint8_t*)&nvmParam.moduleParam,(uint8_t*)&module_default, sizeof(module_setting_t));
    
    
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

//  event_listener_t elSensor;
  eventflags_t flags;
  //activeSensor = SENSOR_ISM330;
  if(activeSensor == SENSOR_ADXL355){
    chEvtRegisterMask(&adxl.evsource,&runTime.el_sensor,EV_ADXL_FIFO_FULL);
//    runTime.ledBlink.ms_on = 500;
//    runTime.ledBlink.ms_off = 500;
    adxl355_get_fifo_size(&adxl,&sz);
    if(sz > 0){
      bStop = false;
    }
    adxl355_cmd_start(&adxl);  
  }
  else if(activeSensor == SENSOR_BMI160){
    
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
  runTime.rxSz = CMD_STRUCT_SZ + 4;
  while(!bStop){
    evt = chEvtWaitAny(ALL_EVENTS);
   // flags = chEvtGetAndClearFlags(&elSensor);
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
          //if(bsz > 144) 
            //bsz = 144;
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
                  runTime.timeout = 0;
                }
                runTime.rxSz = CMD_STRUCT_SZ + 4;
              }
          }
          break;
        case OP_VNODE:
        case OP_OLED:
//          bsz = sz*9;
//          adxl355_read_fifo(&adxl,buf,bsz); // each record has 9-bytes (x/y/z)*3
//          bsz = sz*4*3;
//          p_src = buf;
//          p_dst = (uint8_t*)data;
//          p_dst += 3;
//          for(uint16_t j=0;j<bsz;j++){
//            if((j%4)==3){
//              *(p_dst--)=0;
//              p_dst = (uint8_t*)data + j + 4;
//            }else{
//              *(p_dst--)=*(p_src++);
//            }
//          }
//          if(packetToIgnore){
//            packetToIgnore--;
//          }
//          else if(feed_fifo32(&m_timeDomain,(uint8_t*)data,sz)==1){
//            // update modbus data
//            memcpy((void*)&runTime.rms,(void*)&m_timeDomain.rms,12);
//            //memcpy((void*)&runTime.peak,(void*)&m_timeDomain.peak,12);
//            memcpy((void*)&runTime.crest,(void*)&m_timeDomain.crest,12);
//            memcpy((void*)&runTime.velocity,(void*)&m_timeDomain.velocity,12);
//            runTime.peak.x = (m_timeDomain.peak.x - m_timeDomain.peakn.x);
//            runTime.peak.y = (m_timeDomain.peak.y - m_timeDomain.peakn.y);
//            runTime.peak.z = (m_timeDomain.peak.z - m_timeDomain.peakn.z);
//            resetObject(&m_timeDomain);
//            
//            // send data via WIFI/BT
//            // prepare data transmit
//            if(opMode == OP_VNODE){
//              header = (cmd_header_t*)buf;
//              header->magic1 = MAGIC1;
//              header->magic2 = MAGIC2;
//              header->type = MASK_DATA;
//              header->pid = pktCount++;
//              uint8_t *ptr = &buf[CMD_STRUCT_SZ];
//              memcpy(ptr,&runTime.peak,12);
//              ptr += 12;
//              memcpy(ptr,&runTime.rms,12);
//              ptr += 12;
//              memcpy(ptr,&runTime.crest,12);
//              ptr += 12;
//              memcpy(ptr,&runTime.velocity,12);
//              ptr += 12;
//              header->len = 48 + CMD_STRUCT_SZ;
//              header->chksum = cmd_checksum(buf,header->len);
//              if(runTime.writefcn)
//                runTime.writefcn(buf,header->len);
//            }
//            if(opMode == OP_OLED){
//              if(nofSampleToIgnore) nofSampleToIgnore--;
//              else{
//                runTime.hisPEAK.x = (runTime.peak.x > runTime.hisPEAK.x)?runTime.peak.x:runTime.hisPEAK.x;
//                runTime.hisPEAK.y = (runTime.peak.y > runTime.hisPEAK.y)?runTime.peak.y:runTime.hisPEAK.y;
//                runTime.hisPEAK.z = (runTime.peak.z > runTime.hisPEAK.z)?runTime.peak.z:runTime.hisPEAK.z;
//                runTime.hisRMS.x = (runTime.rms.x > runTime.hisRMS.x)?runTime.rms.x:runTime.hisRMS.x;
//                runTime.hisRMS.y = (runTime.rms.y > runTime.hisRMS.y)?runTime.rms.y:runTime.hisRMS.y;
//                runTime.hisRMS.z = (runTime.rms.z > runTime.hisRMS.z)?runTime.rms.z:runTime.hisRMS.z;
//                chEvtSignal(runTime.dispThread,0x1);
//              }
//            }
//          }
          break;
        }
      }
      //A0_LO;
    }
    if(flags & EV_ISM_FIFO_FULL){
//      uint16_t sz;
//        uint16_t readSz;
//        switch(opMode){
//        case OP_STREAM:
//          if(ism330_cmd_fifo_read(&runTime.txBuf.buffer[CMD_STRUCT_SZ],300,&readSz)){
//            if(packetToIgnore){
//              packetToIgnore--;
//            }
//            else{
//              runTime.txBuf.sz = readSz+CMD_STRUCT_SZ;
//              header = (cmd_header_t*)runTime.txBuf.buffer;
//              header->magic1 = MAGIC1;
//              header->magic2 = MAGIC2;
//              header->type = MASK_DATA | runTime.lbt;
//              header->len = runTime.txBuf.sz;
//              header->pid = pktCount++;
//              header->chksum = cmd_checksum(runTime.txBuf.buffer,header->len);
//              if(runTime.writefcn){
//                runTime.writefcn(runTime.txBuf.buffer, header->len);
//              }
//            }
//          }
//          break;
//        case OP_VNODE:
//        case OP_OLED:
//          if(ism330_cmd_fifo_read(buf,300,&readSz)){
//            sz = readSz/12; // nof records
//            if(packetToIgnore){
//              packetToIgnore--;
//            }
//            else if(feed_fifo16_imu(&m_timeDomain,(uint8_t*)buf,sz)==1){
//              memcpy((void*)&runTime.rms,(void*)&m_timeDomain.rms,12);
//              memcpy((void*)&runTime.crest,(void*)&m_timeDomain.crest,12);
//              memcpy((void*)&runTime.velocity,(void*)&m_timeDomain.velocity,12);
//              runTime.peak.x = (m_timeDomain.peak.x - m_timeDomain.peakn.x);
//              runTime.peak.y = (m_timeDomain.peak.y - m_timeDomain.peakn.y);
//              runTime.peak.z = (m_timeDomain.peak.z - m_timeDomain.peakn.z);
//              resetObject(&m_timeDomain);
//              // send data via WIFI/BT
//              header = (cmd_header_t*)buf;
//              header->magic1 = MAGIC1;
//              header->magic2 = MAGIC2;
//              header->type = MASK_DATA;
//              header->pid = pktCount++;
//              uint8_t *ptr = &buf[CMD_STRUCT_SZ];
//              memcpy(ptr,&runTime.peak,12);
//              ptr += 12;
//              memcpy(ptr,&runTime.rms,12);
//              ptr += 12;
//              memcpy(ptr,&runTime.crest,12);
//              ptr += 12;
//              memcpy(ptr,&runTime.velocity,12);
//              ptr += 12;
//              header->len = 48 + CMD_STRUCT_SZ;
//              header->chksum = cmd_checksum(buf,header->len);
//              if(runTime.writefcn){
//                runTime.writefcn(buf,header->len);
//              }
//            }
//          break;          
//        }
      }
      
    //}
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
}
static void keep_live(void *arg);

static void startTransfer(void)
{

  if(!runTime.opThread){
    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO-1,procOperation,&SDU1);
    //chVTSet(&runTime.vt,TIME_MS2I(200),keep_live,NULL);
    runTime.timeout = 0;
  }
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
void vnode_app_init()
{
  load_settings();
  bincmd_shellInit();
  memcpy((uint8_t*)adxl.config,(uint8_t*)&nvmParam.adxlParam,sizeof(nvmParam.adxlParam));
  if(adxl355_init(&adxl) == ADXL355_OK){
    runTime.sensorReady = 1;
    adxl355_powerup(&adxl);
    adxl355_powerdown(&adxl);
  }
  //runTime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binshellProc,(void*)&shell_cfg);
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbDisconnectBus(serusbcfg.usbp);
//  palClearPad(GPIOA,15);
//  chThdSleepMilliseconds(1000);
//  palSetPad(GPIOA,15);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);  
  
//  chVTObjectInit(&runTime.vt);
  runTime.monitorThread = chThdCreateStatic(waMonitor,sizeof(waMonitor),NORMALPRIO-1,procMonitor,NULL);
  

  while(1){
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      /* Starting shells.*/
      if (runTime.shelltp == NULL) {
        runTime.shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                       "shell1", NORMALPRIO + 1,
                                       binshellProc, (void *)&shell_cfg);
      }

      /* Waiting for an exit event then freeing terminated shells.*/
      chEvtWaitAny(EVENT_MASK(0));
      if (chThdTerminatedX(runTime.shelltp)) {
        chThdRelease(runTime.shelltp);
        runTime.shelltp = NULL;
      }
    }
    else {
      chThdSleepMilliseconds(50);
    }
  }
}

int main()
{
  halInit();
  chSysInit();
  vnode_app_init();
}

void cmd_config(BaseSequentialStream *chp, BinCommandHeader *hin, uint8_t *data)
{
  BinCommandHeader *header = (BinCommandHeader*)data;
  uint8_t buffer[256];
  BinCommandHeader *resp = (BinCommandHeader*)buffer;
  memcpy(buffer,data,4);
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
        memcpy((uint8_t*)adxl.config,&buffer[8], sizeof(nvmParam.adxlParam));
        valid = true;
        break;
      case 0x2: // IMU
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
//    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO-1,procOperation,NULL);
    header.magic1 = MAGIC1;
    header.magic2 = MAGIC2;
    header.type = MASK_CMD_RET_OK;
    header.pid = 0;
    header.len = CMD_STRUCT_SZ;
    header.chksum = checksum((uint8_t*)&header,header.len);
    streamWrite(chp,(uint8_t*)&header,8);
    startTransfer();
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
    bootInfo.loaderVersion = 0x0;
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
    bootInfo.loaderVersion = 0x0;
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



