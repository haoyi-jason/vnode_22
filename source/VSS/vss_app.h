#include "ch.h"
#include "hal.h"
#include "vss_app.h"
#include "adxl355_dev.h"
#include "adxl355_defs.h"
#include "bincmd_shell.h"
#include "nvm_config.h"
#include "usbcfg.h"


#define SENSOR_ADXL355  0x01
#define SENSOR_BMI160   0x02
#define SENSOR_ISM330   0x04

#define COM_IF_USB      0x10
#define COM_IF_SERIAL   0x20
#define COM_IF_BT       0x40
#define COM_IF_WIFI     0x80

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

BinShellCommand commands[] ={
  {{0xab,0xba,0xA2,0x00,0,0},cmd_config}, // node param
  {{0xab,0xba,0xA2,0x01,0,0},cmd_config}, // adxl param
  {{0xab,0xba,0xA2,0x40,0,0},cmd_config}, // module param
  {{0xab,0xba,0xA1,0x01,0,0},cmd_start}, // start
  {{0xab,0xba,0xA1,0x00,0,0},cmd_stop}, // stop
  {{0xab,0xba,0x01,0x00,0,0},cmd_read},
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
  "VSS",
  0x12345678, 
  0x00000001,
  "Grididea-AT32",
  "VSS-II" // supported config
};





