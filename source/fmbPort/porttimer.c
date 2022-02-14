/*
 * FreeModbus Libary: Atmel STM32 Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: porttimer.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "ch.h"
#include "hal.h"
#include <stdlib.h>

#include "mbport.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"
/* ----------------------- Defines ------------------------------------------*/
#define MBP_DEBUG_TIMER_PERFORMANCE     ( 1 )

#define MAX_TIMER_HDLS                  ( 5 )
#define IDX_INVALID                     ( NULL )
#define EV_NONE                         ( 0 )

#define TIMER_TIMEOUT_INVALID           ( 65535U )
#define TIMER_PRESCALER                 ( 128U )
#define TIMER_XCLK                      ( 72000000U )

#define TIMER_MS2TICKS( xTimeOut )      ( ( TIMER_XCLK * ( xTimeOut ) ) / ( TIMER_PRESCALER * 1000U ) )

#define RESET_HDL( x ) do { \
    ( x )->dev = IDX_INVALID; \
	( x )->usNTimeOutMS = 0; \
	( x )->usNTimeLeft = TIMER_TIMEOUT_INVALID; \
    ( x )->xMBMHdl = MB_HDL_INVALID; \
    ( x )->pbMBPTimerExpiredFN = NULL; \
     (x)->gptConfig = NULL; \
} while( 0 );

#define GPT_BASE_CLOCK 1000000

#ifdef DEBUG_MB
#include "tm.h"
static TimeMeasurement tm;
#endif

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    GPTDriver*      dev;
    USHORT          usNTimeOutMS;
    USHORT          usNTimeLeft;
    xMBHandle       xMBMHdl;
    pbMBPTimerExpiredCB pbMBPTimerExpiredFN;
    GPTConfig           *gptConfig;
} xTimerInternalHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC xTimerInternalHandle arxTimerHdls[MAX_TIMER_HDLS];
STATIC BOOL     bIsInitalized = FALSE;

static void gpt_callback(GPTDriver *gptp);     //timerHandler
/* ----------------------- Static variables ---------------------------------*/
 
static GPTConfig gptcfg={
  GPT_BASE_CLOCK,                // GPT_BASE_CLOCK 18000000
  gpt_callback
};

uint32_t gptInterval;   //timerout


/* ----------------------- Start implementation -----------------------------*/
static void gpt_callback(GPTDriver *gptp)          
{
  (void)gptp;
  
  chSysLockFromISR(); 
  if(gptp == arxTimerHdls[0].dev){
    arxTimerHdls[0].usNTimeLeft--;
    if(0 == arxTimerHdls[0].usNTimeLeft){
      arxTimerHdls[0].usNTimeLeft = TIMER_TIMEOUT_INVALID;
      (void)arxTimerHdls[0].pbMBPTimerExpiredFN(arxTimerHdls[0].xMBMHdl);
    }
  }
  
  chSysUnlockFromISR();
}



eMBErrorCode
eMBPTimerInit( xMBPTimerHandle * xTimerHdl, USHORT usTimeOut1ms,
               pbMBPTimerExpiredCB pbMBPTimerExpiredFN, xMBHandle xHdl )
{
  xTimerInternalHandle *pxTimerHdl = (xTimerInternalHandle*)&arxTimerHdls[0];
  eMBErrorCode    eStatus = MB_EPORTERR;
  
  if((NULL != xTimerHdl) && (NULL != pbMBPTimerExpiredFN) && (MB_HDL_INVALID != xHdl)){
    if(!bIsInitalized){      
      arxTimerHdls[0].dev = &GPTD3;
      arxTimerHdls[0].gptConfig = &gptcfg;
      arxTimerHdls[0].xMBMHdl = xHdl;
      arxTimerHdls[0].usNTimeOutMS = usTimeOut1ms +1;
      arxTimerHdls[0].usNTimeLeft= TIMER_TIMEOUT_INVALID;
      arxTimerHdls[0].pbMBPTimerExpiredFN = pbMBPTimerExpiredFN;
      *xTimerHdl = &arxTimerHdls[0];

      gptInterval = usTimeOut1ms+1;
      gptInit();
      gptStart(arxTimerHdls[0].dev,arxTimerHdls[0].gptConfig);                      //General Purpose Timer,GPTD3:TIM1 
      bIsInitalized = TRUE;
      eStatus = MB_ENOERR;
    }
  }
  
  return eStatus;
}

void
vMBPTimerClose( xMBPTimerHandle xTimerHdl )
{
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;
    
    if(pxTimerIntHdl->dev != NULL){
      gptStop(pxTimerIntHdl->dev);
    }
}

eMBErrorCode
eMBPTimerSetTimeout( xMBPTimerHandle xTimerHdl, USHORT usTimeOut1ms )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( (pxTimerIntHdl->dev != NULL) &&
        ( usTimeOut1ms > 0 ) && ( usTimeOut1ms != TIMER_TIMEOUT_INVALID ) )
    {
        pxTimerIntHdl->usNTimeOutMS = usTimeOut1ms + 1;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPTimerStart( xMBPTimerHandle xTimerHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

//    MBP_ENTER_CRITICAL_SECTION(  );
    if( (pxTimerIntHdl->dev != NULL) )
    {
        pxTimerIntHdl->usNTimeLeft = pxTimerIntHdl->usNTimeOutMS;
        eStatus = MB_ENOERR;
        if(bMBPortIsWithinException()){
          gptStartContinuousI(arxTimerHdls[0].dev,1000);
        }else{
          gptStartContinuous(arxTimerHdls[0].dev,1000);
        }
    }
//    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPTimerStop( xMBPTimerHandle xTimerHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( (pxTimerIntHdl->dev != NULL) )
    {
        pxTimerIntHdl->usNTimeLeft = TIMER_TIMEOUT_INVALID;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

