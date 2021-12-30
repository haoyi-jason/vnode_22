#include "ch.h"
#include "hal.h"
//#include "usbcfg.h"
#include "shell.h"
#include "chprintf.h"
#include "bincmd_shell.h"


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) 
{

  chprintf(chp,"\r\nWrite Command");
}

static void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  chprintf(chp,"\r\nRead Command");
}

static void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  chprintf(chp,"\r\nRun Command");
}

static const ShellCommand commands[] = {
  {"write", cmd_write},
  {"read", cmd_read},
  {"run", cmd_run},
  {NULL, NULL}
};

//static const ShellConfig shell_cfg1 = {
//  (BaseSequentialStream *)&SDU1,
//  commands
//};

static SPIConfig spicfg = {
  false,
  NULL,
  GPIOB,
  12,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static THD_WORKING_AREA(waBlink, 1024);
static THD_FUNCTION(procBlink, arg) 
{
  (void)arg;
  
  while(1){
    palSetPad(GPIOC,2);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOC,2);
    chThdSleepMilliseconds(500);
  }
}

int main()
{
  thread_t *shelltp1 = NULL;
  halInit();
  chSysInit();
  
//  sduObjectInit(&SDU1);
//  sduStart(&SDU1, &serusbcfg);
//  
//  
//  usbDisconnectBus(serusbcfg.usbp);
//  palClearPad(GPIOA,15);
//  chThdSleepMilliseconds(1000);
//  palSetPad(GPIOA,15);
//  usbStart(serusbcfg.usbp, &usbcfg);
//  usbConnectBus(serusbcfg.usbp);  

//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
  //spiTest();
//  serialTest();
  //canTest();
//  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, procBlink, NULL);
// shellInit();
  
  //vnode_app_init();
//  while(1){
//    if (SDU1.config->usbp->state == USB_ACTIVE) {
//      /* Starting shells.*/
//      if (shelltp1 == NULL) {
////        shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
////                                       "shell1", NORMALPRIO + 1,
////                                       binshellProc, (void *)&shell_cfg1);
////        shelltp1 = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,shellThread,(void*)&shell_cfg1);
//      }
//
//      /* Waiting for an exit event then freeing terminated shells.*/
//      chEvtWaitAny(EVENT_MASK(0));
//      if (chThdTerminatedX(shelltp1)) {
//        chThdRelease(shelltp1);
//        shelltp1 = NULL;
//      }
//    }
//    else {
//      chThdSleepMilliseconds(50);
//    }
//  }
  return 0;
}