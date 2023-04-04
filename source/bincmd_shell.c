#include "ch.h"
#include "hal.h"
#include "bincmd_shell.h"
#include "string.h"

event_source_t shell_terminated;

uint16_t checksum(uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint16_t sum = 0;

    for (i = 0; i < length; i++){
      if((i==6) || (i==7)) continue;
      sum += data[i];
    }
    return sum;
}

static bool cmdexec(const BinShellCommand *scp, BaseSequentialStream *chp, uint8_t *header)
{
  //uint8_t state;
  BinCommandHeader *h = (BinCommandHeader*)header;
  bool done = false;
  uint8_t buf[512];
  while(scp->func != NULL){
    if(memcmp((uint8_t*)&scp->header, (uint8_t*)h,4) == 0){
      memcpy(buf,header,8);
//      if(h->len > CMD_STRUCT_SZ){
//        streamRead(chp,&buf[8],h->len - CMD_STRUCT_SZ);
//      }
      // do checksum
      //if(h->chksum == checksum(buf,h->len)){
        scp->func(chp,h,buf);
        return true;
      //}
      //return false;
    }
    scp++;
  }
  
  return done;
}

static bool shellGetCommand(BinShellConfig *scfg, uint8_t *buf, uint16_t n)
{
  BaseSequentialStream *chp = scfg->sc_channel;
  uint8_t *p = buf;
  uint16_t rcved = 0;
  uint16_t szLen;
  char c;
  bool head_in = false;
  while(true){
    if(streamRead(chp,(uint8_t *)&c,1) == 0)
      return true;
    if(head_in){
      *p++ = c;
      rcved++;
      if(rcved == 8){
        return false;
      }
    }
    else{
      if(c == 0xAB){
        head_in = true;
        rcved = 1;
        *p++ = c;
      }
    }
  }
}

THD_FUNCTION(binshellProc,p)
{
  BinShellConfig *scfg = p;
  BaseSequentialStream *chp = scfg->sc_channel;
  const BinShellCommand *scp = scfg->sc_commands;
  uint8_t header[8];
  while(!chThdShouldTerminateX()){
    if(shellGetCommand(scfg,header,8)){
      chThdSleepMilliseconds(10);
    }
    else{
      cmdexec(scp,chp,header);
    }
  }
  
  chSysLock();
  chEvtBroadcastI(&shell_terminated);
  chSysUnlock();
  chThdExitS(MSG_OK);
}

void bincmd_shellInit()
{
  chEvtObjectInit(&shell_terminated);
 // chThdCreateStatic(waAD7124,sizeof(waAD7124),NORMALPRIO,procAD7124,NULL);
}