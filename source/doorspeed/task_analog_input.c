#include "ch.h"
#include "hal.h"
#include "task_analog_input.h"
#include "ylib/adda/ad76xx/ad76_drv.h"

#define EV_AD7606_RDY   EVENT_MASK(10)
#define EV_AD7606_ACQ   EVENT_MASK(11)

struct runTime{
  thread_t *thd_ad7606;
  thread_t *thd_ad2s;
  AD7606Driver *ad7606;
  event_listener_t el_ad7606;
  virtual_timer_t vt7606;
  uint16_t sampleIntervalms;
};

static struct runTime runTime, *analogRuntime;

static const SPIConfig spicfg_ad76 = {
  FALSE,
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static AD7606Config ad7606config = {
  &SPID2,
  &spicfg_ad76,
  GPIOB,
  12,
  GPIOA,
  8,
  GPIOB,
  15,
  GPIOB,
  3,
  GPIOB,
  4,
  GPIOA,
  15,
  GPIOB,
  8,
  GPIOB,
  7,
  GPIOB,
  6,
  GPIOB,
  5
};

static void ad7606_to(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.thd_ad7606,EV_AD7606_ACQ);
  chVTSetI(&runTime.vt7606, TIME_MS2I(runTime.sampleIntervalms),ad7606_to,NULL);
  chSysUnlockFromISR();
}

static AD7606Driver ad7606;
//= {
//  &ad7606config,
//  AD7608
//};

static THD_WORKING_AREA(waAD7606,1024);
static THD_FUNCTION(procAD7606 ,p)
{
  // ref sel connect to B.10
  palSetPadMode(GPIOB,10,PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOB,10);
  // range on PA.11
  palSetPadMode(GPIOA,11,PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOA,11);
  // start ad7606
  ad7606_init(runTime.ad7606,&ad7606config);
  
  runTime.ad7606->chipID = ADS8586;
  
  ad7606_start(runTime.ad7606);
  chEvtRegisterMask(&runTime.ad7606->ev_drdy,&runTime.el_ad7606,EV_AD7606_RDY);
  
  chVTObjectInit(&runTime.vt7606);
  chVTSet(&runTime.vt7606,TIME_MS2I(1),ad7606_to,NULL);
  ad7606_reset(runTime.ad7606);
  eventmask_t evt;
//  ad7606_start_conv(runTime.ad7606,1);
  while(!chThdShouldTerminateX()){
    evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_AD7606_RDY){
      ad7606_read_conversion(runTime.ad7606,2);
      palClearPad(GPIOC,15);
    }
    if(evt & EV_AD7606_ACQ){
      palSetPad(GPIOC,15);
      ad7606_start_conv(runTime.ad7606,3);
    }
    
  }
  
  
}

void analog_input_task_init()
{
  runTime.ad7606 = &ad7606;
  runTime.sampleIntervalms = 1;
  analogRuntime = &runTime;
  runTime.thd_ad7606 = chThdCreateStatic(waAD7606,sizeof(waAD7606),NORMALPRIO,procAD7606,NULL);
}

int8_t analog_input_read(uint8_t channel, int32_t *data)
{
  int8_t ret = 0;
  if(channel == 0xff){
    memcpy(data,ad7606.data,32);
  }
  else if(channel < 8){
    *data = ad7606.data[channel];
  }
  else{
    ret = -1;
  }
  return ret;
}

int8_t analog_input_read_packed(uint8_t channel, uint8_t *data)
{
  int8_t ret = 0;
  
  if(channel == 0xff){ // pack 4-channel only
    uint8_t *ptr = (uint8_t *)ad7606.data;
//    ptr += 2;
    memcpy(data,ptr,2);
    ptr +=4;
    memcpy(&data[2],ptr,2);
    ptr +=4;
    memcpy(&data[4],ptr,2);
    ptr +=4;
    memcpy(&data[6],ptr,2);
  }
  else if(channel < 6){
    *data = ad7606.data[channel];
  }
  else{
    ret = -1;
  }
  return ret;
}

void analog_input_set_sample_interval(uint16_t ms)
{
  //runTime.sampleIntervalms = ms;
}