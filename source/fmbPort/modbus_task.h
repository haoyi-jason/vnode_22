#ifndef _MODBUS_TASK_
#define _MODBUS_TASK_

#include "port.h"

typedef int8_t (*handler)(uint16_t address, uint8_t *dptr, uint8_t rw);

typedef struct{
  handler mapFunction;
  uint16_t addrStart;
  uint16_t addrEnd;
}_modbus_map_handler;


int mbPortInit(mbRTUOpts_t *opts);

#endif