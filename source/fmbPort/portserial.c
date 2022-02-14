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
 * File: $Id: portserial.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include <stdlib.h>
#include "ch.h"
#include "hal.h"

#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/

#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"

/* ----------------------- Defines ------------------------------------------*/
#define IDX_INVALID             ( NULL )
#define UART_BAUDRATE_MIN       ( 300 )
#define UART_BAUDRATE_MAX		( 115200 )


#define UART_1_ENABLED          ( 1 )   /*!< Set this to 1 to enable USART1 */
#define UART_2_ENABLED          ( 0 )   /*!< Set this to 1 to enable USART2 */

#if ( UART_1_ENABLED == 1 ) && ( UART_2_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_2_PORT             ( MB_UART_2 )
#define UART_1_IDX              ( 0 )
#define UART_2_IDX              ( 1 )
#define NUARTS                  ( 2 )
#elif ( UART_1_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_1_IDX              ( 0 )
#define NUARTS                  ( 1 )
#elif ( UART_2_ENABLED == 1 )
#define UART_2_PORT             ( MB_UART_2 )
#define UART_2_IDX              ( 0 )
#define NUARTS                  ( 1 )
#else
#define NUARTS                  ( 0 )
#endif

#define RS_485_UART_1_INIT(  )	\
do { \
} while( 0 )

#define RS_485_UART_1_ENABLE_TX(  )	\
do {\
   GPIO_WriteBit( GPIOA, GPIO_Pin_8, Bit_SET ); \
   GPIO_WriteBit( GPIOA, GPIO_Pin_11, Bit_SET ); \
} while( 0 )

#define RS_485_UART_1_DISABLE_TX(  ) \
do { \
   GPIO_WriteBit( GPIOA, GPIO_Pin_8, Bit_RESET ); \
   GPIO_WriteBit( GPIOA, GPIO_Pin_11, Bit_RESET ); \
} while( 0 )

#define RS_485_UART_2_INIT(  )\
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_ENABLE_TX(  )	\
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_DISABLE_TX(  ) \
do { \
    /* not implemented yet */ \
} while( 0 )

/* ----------------------- Defines (Internal - Don't change) ----------------*/
#define HDL_RESET( x ) do { \
	( x )->dev = IDX_INVALID; \
	( x )->pbMBMTransmitterEmptyFN = NULL; \
	( x )->pvMBMReceiveFN = NULL; \
	( x )->xMBMHdl = MB_HDL_INVALID; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    UARTDriver *dev;
    pbMBPSerialTransmitterEmptyAPIV1CB pbMBMTransmitterEmptyFN;
    pvMBPSerialReceiverAPIV1CB pvMBMReceiveFN;
    xMBHandle       xMBMHdl;
    UARTConfig *uartConfig;
} xSerialHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC xSerialHandle xSerialHdls[NUARTS];
STATIC BOOL     bIsInitalized = FALSE;

static void txDriverHasRead(UARTDriver *uartp) ;
static void txBufferEmpty(UARTDriver *uartp) ;
static void rxErr(UARTDriver *uartp, uartflags_t e);
static void rxChar(UARTDriver *uartp, uint16_t c);
static void rxEnd(UARTDriver *uartp);

static UCHAR    oneByteAccum = 0; // should we use a circular buffer ?

#define UARTDRIVER  UARTD1

#ifdef DEBUG_MB
static CHAR lastCharReceived;
CHAR getLastCharReceived (void){
  return lastCharReceived;
}
#endif

UARTConfig uartCfg = {
  txDriverHasRead,
  txBufferEmpty,
  rxEnd,
  rxChar,
  rxErr,
  NULL,
  9600,
  0,
  0,
  0
};


eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBMHdl )
{
  eMBErrorCode   eStatus;
    xSerialHandle  *pxSerialIntHdl = &xSerialHdls[0];
  
  if((MB_HDL_INVALID == xMBMHdl) || (NULL == pxSerialHdl)){
    eStatus = MB_EINVAL;
  }
  else{
    eStatus = MB_ENORES;
    
    if((ulBaudRate > UART_BAUDRATE_MIN) && (ulBaudRate < UART_BAUDRATE_MAX)){
      uartCfg.speed = ulBaudRate;
    }
    else{
      eStatus = MB_EINVAL;
    }
    
    switch(ucStopBits){
    case 1:
      uartCfg.cr2 &= ~USART_CR2_STOP_Msk;
      break;
    case 2:
      uartCfg.cr2 &= ~USART_CR2_STOP_Msk;
      uartCfg.cr2 |= USART_CR2_STOP_1;
      break;
    default:
      eStatus = MB_EINVAL;
      break;
    }
    switch(ucDataBits){
    case 8:
      if(eParity != MB_PAR_NONE){
        uartCfg.cr1 |= USART_CR1_M;
      }
      break;
    case 7:
      break;
    default:
      eStatus = MB_EINVAL;
    }
    switch(eParity){
    case MB_PAR_NONE:
      break;
    case MB_PAR_ODD:
      //uartCfg.cr1 |= (USART_CR1_PCE | USART_CR1_PS);
      break;
    case MB_PAR_EVEN:
      uartCfg.cr1 |= USART_CR1_PCE;
      break;
    default:
      eStatus = MB_EINVAL;
    }
  }

  if(eStatus != MB_EINVAL){
    xSerialHdls[0].uartConfig = &uartCfg;
    xSerialHdls[0].dev = &UARTD1;
    xSerialHdls[0].xMBMHdl = xMBMHdl;
    *pxSerialHdl = &xSerialHdls[0];
    
    uartStart(pxSerialIntHdl->dev,pxSerialIntHdl->uartConfig);
    eStatus = MB_ENOERR;
  }
  return eStatus;
}

eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
  eMBErrorCode    eStatus = MB_EINVAL;
  xSerialHandle  *pxSerialIntHdl = xSerialHdl;

  if(pxSerialIntHdl->dev != NULL){
    eStatus = MB_ENOERR;
    uartStop(pxSerialIntHdl->dev);
  }
  
  return eStatus;
}

eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl, pbMBPSerialTransmitterEmptyCB pbMBMTransmitterEmptyFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;
    
    //uartStop(pxSerialIntHdl->dev);
    if(pxSerialIntHdl->dev != NULL){
      if(NULL != pbMBMTransmitterEmptyFN){
        pxSerialIntHdl->pbMBMTransmitterEmptyFN = pbMBMTransmitterEmptyFN;
        //pxSerialIntHdl->uartConfig->cr1 |= USART_CR1_TCIE;
        // enable TX End interrupt
        pxSerialIntHdl->dev->usart->CR1 |= (USART_CR1_TCIE);
        BYTE ubTxByte;
        pxSerialIntHdl->pbMBMTransmitterEmptyFN(pxSerialIntHdl->xMBMHdl,&ubTxByte);
        uartStartSend(pxSerialIntHdl->dev, 1, &ubTxByte);      
      }
      else{
        pxSerialIntHdl->pbMBMTransmitterEmptyFN = NULL;
        //pxSerialIntHdl->uartConfig->cr1 &= ~USART_CR1_TCIE;
        // disable TX End interrupt
        pxSerialIntHdl->dev->usart->CR1 &= ~USART_CR1_TCIE;
        //pxSerialIntHdl->dev->usart->CR1 |= (USART_CR1_TCIE);
      }
      eStatus = MB_ENOERR;
    }
    //uartStart(pxSerialIntHdl->dev,pxSerialIntHdl->uartConfig);
    
    return eStatus;
}

eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBMReceiveFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;
    
    if(pxSerialIntHdl->dev != NULL){
      if(NULL != pvMBMReceiveFN){
        pxSerialIntHdl->pvMBMReceiveFN = pvMBMReceiveFN;
        pxSerialIntHdl->uartConfig->cr1 |= USART_CR1_RXNEIE;
        pxSerialIntHdl->dev->usart->CR1 |= USART_CR1_RXNEIE;
      }
      else{
        pxSerialIntHdl->pvMBMReceiveFN = NULL;
        pxSerialIntHdl->uartConfig->cr1 &= ~USART_CR1_RXNEIE;
        pxSerialIntHdl->dev->usart->CR1 &= ~USART_CR1_RXNEIE;
      }
      eStatus = MB_ENOERR;
    }
    
    return eStatus;
}
/* End of transmission buffer callback. TC  */
static void txDriverHasRead(UARTDriver *uartp){
  (void) uartp;
  chSysLockFromISR();
  //uartp->usart->CR1 &= ~USART_CR1_TCIE;
  chSysUnlockFromISR();
}

/* Physical end of transmission callback. TE */
void txBufferEmpty(UARTDriver *uartp){
  //(void) uartp;
  BOOL bHasMoreData = TRUE;
  UBYTE ubTxByte;
  chSysLockFromISR();
  for(uint8_t i=0;i<NUARTS;i++){
    xSerialHandle  *pxSerialIntHdl = &xSerialHdls[i];
    if((NULL != pxSerialIntHdl->pbMBMTransmitterEmptyFN) && (uartp == pxSerialIntHdl->dev)){
      bHasMoreData = pxSerialIntHdl->pbMBMTransmitterEmptyFN(pxSerialIntHdl->xMBMHdl,&ubTxByte);
    }
    if(!bHasMoreData){
      pxSerialIntHdl->pbMBMTransmitterEmptyFN = NULL;
      pxSerialIntHdl->uartConfig->cr1 &= ~USART_CR1_TCIE;
      pxSerialIntHdl->dev->usart->CR1 &= ~USART_CR1_TCIE;
    }
    else{
      uartStartSendI (pxSerialIntHdl->dev, 1, &ubTxByte);      
    }
  }

  chSysUnlockFromISR();
}


/* Receive buffer filled callback.  */
void rxEnd(UARTDriver *uartp){
  (void) uartp;
  
}


/* Character received while out if the UART_RECEIVE state.  */
void rxChar(UARTDriver *uartp, uint16_t c){
  
  (void) uartp;
  (void) c;

  oneByteAccum = (UCHAR) c;

  vMBPortSetWithinException (TRUE);
  chSysLockFromISR();
  vMBPEnterCritical();

  for(uint8_t i=0;i<NUARTS;i++){
    xSerialHandle  *pxSerialIntHdl = &xSerialHdls[i];
    if((NULL != pxSerialIntHdl->pvMBMReceiveFN) && (uartp == pxSerialIntHdl->dev)){
      pxSerialIntHdl->pvMBMReceiveFN(pxSerialIntHdl->xMBMHdl,(UBYTE)c);
    }
  }
    
  chSysUnlockFromISR();  
  vMBPExitCritical();
}

/* Receive error callback.  */
void rxErr(UARTDriver *uartp, uartflags_t e){
  (void) uartp;
  (void) e;
  
  chSysLockFromISR(); 
  if (e & USART_SR_PE) {
    //syslogErrorFromISR ("parity err");
  } else if (e & USART_SR_FE) {
    //syslogErrorFromISR ("framing err");
  } if (e & USART_SR_NE) {
    //syslogErrorFromISR ("noise err");
  } if (e & USART_SR_ORE) {
    //syslogErrorFromISR ("overrun err");
  }  if (e & USART_SR_IDLE) {
    //syslogErrorFromISR ("idle line err");
  } else {
    //syslogErrorFromISR ("uart rx err");
  }
  chSysUnlockFromISR();
  
  
}



