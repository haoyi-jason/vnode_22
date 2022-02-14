#include "ch.h"
#include "hal.h"
#include "mbs.h"
#include "modbus_task.h"
#include "port.h"
#include "nvm.h"
#include "adc_task.h"
#include "dio_task.h"


#define MB_TCP_UID      6
#define MB_TCP_LEN      4
#define MB_TCP_FUNC      7

#define MB_TCP_DEFAULT_PORT     502
#define MB_TCP_BUF_SIZE (256+7)

#define EV_MODBUS_RW EVENT_MASK(0)

static struct
{
  thread_t *self;
  thread_reference_t ref;
}runTime;

#define MB_RTU_EN   1
#define MB_TCP_EN   1

_modbus_map_handler modbusHandlers[] = {
  {dio_modbus_handler,0,0},
  {adc_modbus_handler,0,0},
  {NULL,0,0},
};

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

static void byteReorder(uint8_t *dptr, uint8_t n)
{
  uint8_t tmp;
  for(uint8_t i=0;i<(n>>1);i++){
    tmp = dptr[i*2];
    dptr[i*2] = dptr[i*2+1];
    dptr[i*2+1] = tmp;
  }
}

int8_t ReadMBMap(uint16_t usAddr, uint8_t *dptr)
{
  int8_t ret;
  _modbus_map_handler *handler = modbusHandlers;
  uint8_t val[] = {0,0,0,0};
  while(handler->mapFunction != NULL){
    if((ret = handler->mapFunction(usAddr,dptr, 0/*0:R, 1:W*/)) != 0){
      //dptr += ret;
      break;
    }
    handler++;
  }
  if(ret == 1){
    //mapMBWord(dptr,(void*)&val);
    byteReorder(dptr,2);
  }
  else if(ret == 2){
//    mapMBFloat(dptr,(void*)&val);
    byteReorder(dptr,4);
  }
  return ret;
}
int8_t WriteMBMap(uint16_t usAddr, uint8_t *dptr)
{
  int8_t ret;
  _modbus_map_handler *handler = modbusHandlers;
  uint8_t val[] = {0,0,0,0};
  while(handler->mapFunction != NULL){
    if((ret = handler->mapFunction(usAddr,dptr, 1/*0:R, 1:W*/)) != 0){
      //dptr += ret;
      break;
    }
    handler++;
  }
  if(usAddr == 299) ret = 1;
//  if(ret == 1){
//    mapMBWord((void*)&val,dptr);
//  }
//  else if(ret == 2){
//    mapMBFloat((void*)&val,dptr);
//  }
  return ret;
}

uint16_t byteSwap(uint16_t v)
{
  uint16_t vret;
  vret = ((v&0xff)<<8) | (v >> 8);

  return vret;
}


eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer,USHORT usAddress,USHORT usNRegs)
{
  return MB_ENOREG;
}

eMBException eMBRegHoldingCB(UBYTE *pucRegBuffer, USHORT usAddress,USHORT usNRegs, eMBSRegisterMode eMode)
{
  eMBErrorCode eStatus = MB_ENOERR;
//  LED_G_ON();
  uint8_t i;
  uint16_t regVal;
  USHORT usAddr;// = usAddress--;
  uint8_t ch;
  uint8_t index;
  uint8_t szRet;
  usAddr = usAddress ;
  double dv;
  if(eMode == MBS_REGISTER_READ){
    for(i=0;i<usNRegs;i++){
      szRet=ReadMBMap(usAddr,pucRegBuffer);
      usAddr+=szRet;
      //pucRegBuffer+=2;
      if(szRet == 2){
        i++;
        //pucRegBuffer+=2;
      }
      pucRegBuffer += (szRet<<1);
    }
  }else if(eMode == MBS_REGISTER_WRITE){
    for(i=0;i<usNRegs;i++){
      szRet =WriteMBMap(usAddr,pucRegBuffer);
      usAddr+=szRet;
      //pucRegBuffer+=2;
      if(szRet == 2){
        i++;
        //pucRegBuffer+=2;
      }
      pucRegBuffer += (szRet<<1);
    }
    // issue write operation
    //sysSaveParam();
  }
  
  chEvtSignal(runTime.self,EV_MODBUS_RW);
  return eStatus;
}

static THD_WORKING_AREA(waMBTask,2048);
static THD_FUNCTION(thMBTask ,p)
{
  mbRTUOpts_t *opt = (mbRTUOpts_t*)p;
  chRegSetThreadName("MODBUS_Thread");
  xMBSHandle xMBSHdl;
  eMBErrorCode eStatus;
  uint8_t address = 0;
  
  if(palReadPad(GPIOB,0) == PAL_LOW){
    address |= 0x8;
  }
  if(palReadPad(GPIOB,1) == PAL_LOW){
    address |= 0x4;
  }
  if(palReadPad(GPIOB,2) == PAL_LOW){
    address |= 0x2;
  }
  if(palReadPad(GPIOB,10) == PAL_LOW){
    address |= 0x1;
  }
  
  if(address == 0) address = 1;
  
  
  if(MB_RTU_EN){
    if(!opt){
      if(eMBSSerialInit(&xMBSHdl,MB_RTU,address,0,9600,MB_PAR_NONE) != MB_ENOERR){
        
      }
      else if(MB_ENOERR != (eStatus = eMBSRegisterHoldingCB(xMBSHdl,eMBRegHoldingCB))){
        
      }
    }
//    else{
//      if(eMBSInit(MB_RTU,opt->sla,0,opt->baudrate,opt->parity) == MB_ENOERR){
//        eMBEnable(0);
//      }
//    }
  }
  
//  if(MB_TCP_EN){
//    if(eMBTCPInit(502) == MB_ENOERR){
//      eMBEnable(1);
//    }
//  }
  /* Resumes the caller and goes to the final priority.*/
  chThdResume(&runTime.ref, MSG_OK);
  while(true){
    eMBSPoll(xMBSHdl);
    chThdSleepMilliseconds(50);
  }
  
}

int mbPortInit(mbRTUOpts_t *opts)
{
  
  runTime.self = chThdCreateStatic(waMBTask,sizeof(waMBTask),NORMALPRIO,thMBTask,opts);
  // lock until thread finishing initial task
  chSysLock();
  chThdSuspendS(&runTime.ref);
  chSysUnlock();

  return 0;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBSRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
