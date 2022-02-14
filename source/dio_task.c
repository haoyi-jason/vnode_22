#include "hal.h"
#include "ch.h"
#include "nvm_config.h"

#define NOF_DIN_CHANNEL         2
#define NOF_DOUT_CHANNEL         2

#define EV_PARAM_SAVE           EVENT_MASK(4)

enum _dio_dir_e{
  DIO_DIR_INPUT,
  DIO_DIR_OUTPUT,
  DIO_DIR_BYDIREC
};

enum _dio_type_e{
  DIO_TYPE_GPIO,
  DIO_TYPE_FREQ,    // output: frequency, input: counter
  DIO_TYPE_PWM,     // output: pwm, input: capture
};

enum digital_in_type_e{
  DIN_GPIO,
  DIN_CNTR,
  DIN_FIN,
  DIN_MAPOUT,
};

enum digital_out_type_e{
  DOUT_COMM, 
  DOUT_ALARM,
  DOUT_NALARM,
  DOUT_MAPDI,
  DOUT_NMAPDI,
  DOUT_PWM,
  DOUT_FOUT,
};

typedef struct{
  uint16_t type;
  uint16_t config;
  uint32_t value;
  ioportid_t port;
  uint16_t pad;
}userConfig;

struct _nvmParam{
  uint8_t flag;
  userConfig input[NOF_DIN_CHANNEL];
  userConfig output[NOF_DOUT_CHANNEL];
};

static struct _nvmParam nvmParam, *dioNvmParam;

struct _runTime{
  uint8_t state;
  thread_t *self;
  thread_reference_t ref;
  uint8_t cycleTime;
  uint16_t digital_input[NOF_DIN_CHANNEL];
  uint16_t digital_output[NOF_DOUT_CHANNEL];
//  struct {
//    uint16_t valid;
//    icucnt_t period;
//    icucnt_t width;
//  }icuState[2];
};


//static void icuwidthcb(ICUDriver *icup);
//static void icuperiodcb(ICUDriver *icup);
//
//static ICUConfig icucfg_t2 = {
//  ICU_INPUT_ACTIVE_HIGH,
//  10000,
//  icuwidthcb,
//  icuperiodcb,
//  NULL,
//  ICU_CHANNEL_2,
//  0
//};
//
//static ICUConfig icucfg_t3 = {
//  ICU_INPUT_ACTIVE_LOW,
//  100000,
//  icuwidthcb,
//  icuperiodcb,
//  NULL,
//  ICU_CHANNEL_1,
//  0
//};
//
//
//static void icuwidthcb(ICUDriver *icup)
//{
//  if(icup == &ICUD2){
//    runTime.icuState[0].width = icuGetWidthX(icup);
//  }
//  else if(icup == &ICUD3){
//    runTime.icuState[1].width = icuGetWidthX(icup);
//  }
//}
//static void icuperiodcb(ICUDriver *icup)
//{
//  if(icup == &ICUD2){
//    runTime.icuState[0].period = icuGetPeriodX(icup);
//    runTime.icuState[0].valid = 1;
//  }
//  else if(icup == &ICUD3){
//    runTime.icuState[1].period = icuGetPeriodX(icup);
//    runTime.icuState[1].valid = 1;
//  }
//}

static struct _runTime runTime, *dioRuntime;

static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
  if(nvmSz > SZ_NVM_CONFIG){
    // error
    while(1);
  }
  nvm_flash_read(OFFSET_NVM_DIGCFG,(uint8_t*)&nvmParam,nvmSz);
  if(nvmParam.flag != 0xAA){
    nvmParam.flag = 0xAA;
    
    nvmParam.input[0].type = DIN_GPIO;
    nvmParam.input[0].value = 0;
    nvmParam.input[0].port = GPIOB;
    nvmParam.input[0].pad = 4;
    nvmParam.input[1].type = DIN_GPIO;
    nvmParam.input[1].value = 0;
    nvmParam.input[1].port = GPIOB;
    nvmParam.input[1].pad = 3;
    
    nvmParam.output[0].type = DOUT_COMM;
    nvmParam.output[0].value = 0;
    nvmParam.output[0].port = GPIOA;
    nvmParam.output[0].pad = 12;

    nvmParam.output[1].type = DOUT_COMM;
    nvmParam.output[1].value = 0;
    nvmParam.output[1].port = GPIOA;
    nvmParam.output[1].pad = 11;
    
    
    // todo:check if write properlly
    nvm_flash_write(OFFSET_NVM_DIGCFG,(uint8_t*)&nvmParam,nvmSz);
  }
}

static void init_config()
{
//  for(uint8_t i=0;i<NOF_DOUT_CHANNEL;i++){
//    userConfig *u = &nvmParam.output[i];
//  }  

  for(uint8_t i=0;i<NOF_DIN_CHANNEL;i++){
    userConfig *u = &nvmParam.input[i];
    switch(u->type){
    case DIN_GPIO:
      break;
    case DIN_CNTR:
      break;
    case DIN_FIN:
      break;
    default:break;
      
    }
  }  
}

static THD_WORKING_AREA(waDIO,2048);
static THD_FUNCTION(procDIO ,p)
{

  load_settings();
  runTime.state = 0;
  runTime.cycleTime = 100;
  eventmask_t evt;

  while(1){
    if((evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE)) != 0){
      if(evt & EV_PARAM_SAVE){
        chSysLock();
        nvm_flash_write(OFFSET_NVM_DIGCFG,(uint8_t*)&nvmParam,sizeof(nvmParam));
        chSysUnlock();
      }
    }
      
    
    switch(runTime.state){
    case 0:
      for(uint8_t i=0;i<NOF_DOUT_CHANNEL;i++){
        userConfig *u = &nvmParam.output[i];
        switch(u->type){
        case DOUT_COMM:
          if(runtime_get_do_from_com(i) == 1){
            palSetPad(u->port,u->pad);
            runTime.digital_output[i] = 1;
          }
          else{
            palClearPad(u->port,u->pad);
            runTime.digital_output[i] = 0;
          }
          break;
        case DOUT_ALARM:
          if(runtime_get_do_from_ana(i) == 1){
            palSetPad(u->port,u->pad);
            runTime.digital_output[i] = 1;
          }
          else{
            palClearPad(u->port,u->pad);
            runTime.digital_output[i] = 0;
          }
          break;
        case DOUT_NALARM:
          if(runtime_get_do_from_ana(i) == 1){
            palClearPad(u->port,u->pad);
            runTime.digital_output[i] = 0;
          }
          else{
            palSetPad(u->port,u->pad);
            runTime.digital_output[i] = 1;
          }
          break;
        case DOUT_MAPDI:
          if(nvmParam.input[i].value == 1){
            palSetPad(u->port,u->pad);
            runTime.digital_output[i] = 1;
          }
          else{
            palClearPad(u->port,u->pad);
            runTime.digital_output[i] = 0;
          }
          break;
        case DOUT_NMAPDI:
          if(nvmParam.input[i].value == 1){
            palClearPad(u->port,u->pad);
            runTime.digital_output[i] = 0;
          }
          else{
            palSetPad(u->port,u->pad);
            runTime.digital_output[i] = 1;
          }
          break;
        }
        //runtime_set_do_state(i,runTime.digital_output[i]));
      }
      runTime.state++;
      break;
    case 1:
      for(uint8_t i=0;i<NOF_DIN_CHANNEL;i++){
        userConfig *u = &nvmParam.input[i];
        switch(u->type){
        case DIN_GPIO:
            if(palReadPad(u->port,u->pad) == PAL_HIGH){
              runTime.digital_input[i] = 0;
            }
            else{
              runTime.digital_input[i] = 1;
            }
          break;
        case DIN_CNTR:
          break;
        case DIN_FIN:
          break;
        default:break;
        }
      }
      runTime.state = 0;
      break;
    default:break;
    }
    chThdSleepMilliseconds(runTime.cycleTime); // cycle time
  }
  
}


void dio_task_init()
{
  dioRuntime = &runTime;
  dioNvmParam = &nvmParam;
  runTime.self = chThdCreateStatic(waDIO,sizeof(waDIO),NORMALPRIO,procDIO,NULL);
  chSysLock();
  chThdSuspendS(&runTime.ref);
  chSysUnlock();
}

static void byteReorder(uint8_t *dptr, uint8_t n)
{
  uint8_t tmp;
  for(uint8_t i=0;i<(n>>1);i++){
    tmp = dptr[i*2];
    dptr[i*2] = dptr[i*2+1];
    dptr[i*2+1] = tmp;
  }
}

int8_t do_set_state(int16_t ch, uint16_t *dptr)
{
  int8_t ret = 0;
  if(ch < NOF_DIN_CHANNEL){
    int16_t value;
    byteReorder((int8_t*)dptr,2);
    value = *dptr;
    runtime_set_do_from_com(ch,value);
    ret = 1;
  }
  return ret;
}

int8_t do_get_state(int16_t ch, uint16_t *dptr)
{
  int8_t ret = 0;
  if(ch < NOF_DIN_CHANNEL){
    int16_t value;
    *dptr = runTime.digital_output[ch];
    ret = 1;
  }
  return ret;
}

int8_t di_get_state(int16_t ch, uint16_t *dptr)
{
  int8_t ret = 0;
  if(ch < NOF_DIN_CHANNEL){
    int16_t value;
    value = runTime.digital_input[ch];
    *dptr = value;
    ret = 1;
  }
  return ret;
}
/*42100 do type */
int8_t do_set_type(int16_t ch, uint16_t *dptr)
{
  int8_t ret = 0;
  if(ch < NOF_DIN_CHANNEL){
    userConfig *u = &nvmParam.output[ch];
    byteReorder((int8_t*)dptr,2);
    u->type = *dptr;
    ret = 1;
  }
  return ret;
}

int8_t do_get_type(int16_t ch, uint16_t *dptr)
{
  int8_t ret = 0;
  if(ch < NOF_DIN_CHANNEL){
    userConfig *u = &nvmParam.output[ch];
    *dptr = u->type;
    ret = 1;
  }
  return ret;
}



#define RANGE_CHECK(a,b,c)      ((a >= b) && (a < c))
int8_t dio_modbus_handler(uint16_t address, uint8_t *dptr, uint8_t rw)
{
  int8_t ret = 0;
  
  if(rw == 0){
    switch(address){
    case 10:
    case 11:
      di_get_state(address -10, (uint16_t*)dptr);
      ret = 1;
      break;
    case 16:
    case 17:
      do_get_state(address -16, (uint16_t*)dptr);
      ret = 1;
      break;
    case 2100:
    case 2101:
      ret = do_get_type(address - 2100,(uint16_t*)dptr);
      break;
    default:
      break;
    }
  }
  else if(rw == 1){
    switch(address){
    case 16:
    case 17:
      do_set_state(address -16, (uint16_t*)dptr);
      ret = 1;
      break;
    case 2100:
    case 2101:
      ret = do_set_type(address - 2100,(uint16_t*)dptr);
      break;
    case 299:
      {
      int16_t value;
      memcpy((uint8_t*)&value,dptr,2);
      byteReorder((int8_t*)&value,2);
      if((value == 5329) || (value == 0x5329)){
        chEvtSignal(runTime.self, EV_PARAM_SAVE);
      }
      ret = 0;
    }
    default:
      break;
    }
  }
  
  
  return ret;
}

