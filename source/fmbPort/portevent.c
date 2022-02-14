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
 * File: $Id: portevent.c,v 1.1 2010/06/05 09:57:48 wolti Exp $
 */
/* ----------------------- System includes ----------------------------------*/
#include "ch.h"
#include <stdlib.h>


#include "mbport.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"


/* ----------------------- Defines ------------------------------------------*/
#define MAX_EVENT_HDLS          ( 2 )
#define IDX_INVALID             ( 255 )
#define EV_NONE                 ( 0 )

#define HDL_RESET( x ) do { \
    ( x )->ubIdx = IDX_INVALID; \
    ( x )->xType = EV_NONE; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    UBYTE           ubIdx;
    xMBPEventType   xType;
} xEventInternalHandle;


/* ----------------------- Variables ----------------------------------------*/
STATIC BOOL     bIsInitialized = FALSE;
STATIC xEventInternalHandle arxEventHdls[MAX_EVENT_HDLS];


/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPEventCreate( xMBPEventHandle * pxEventHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    UBYTE           i;

    if( NULL != pxEventHdl )
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        if( !bIsInitialized )
        {
            for( i = 0; i < MAX_EVENT_HDLS; i++ )
            {
                HDL_RESET( &arxEventHdls[i] );
            }
            bIsInitialized = TRUE;
        }
        for( i = 0; i < MAX_EVENT_HDLS; i++ )
        {
            if( IDX_INVALID == arxEventHdls[i].ubIdx )
            {
                arxEventHdls[i].ubIdx = i;
                arxEventHdls[i].xType = EV_NONE;
                *pxEventHdl = &arxEventHdls[i];
                eStatus = MB_ENOERR;
                break;
            }
        }
        MBP_EXIT_CRITICAL_SECTION(  );
    }
    return eStatus;
}

eMBErrorCode
eMBPEventPost( const xMBPEventHandle xEventHdl, xMBPEventType xEvent )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xEventInternalHandle *pxEventHdl = xEventHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxEventHdl, arxEventHdls ) )
    {
        pxEventHdl->xType = xEvent;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
bMBPEventGet( const xMBPEventHandle xEventHdl, xMBPEventType * pxEvent )
{
    BOOL            bEventInQueue = FALSE;
    xEventInternalHandle *pxEventHdl = xEventHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxEventHdl, arxEventHdls ) )
    {
        if( EV_NONE != pxEventHdl->xType )
        {
            bEventInQueue = TRUE;
            *pxEvent = pxEventHdl->xType;
            pxEventHdl->xType = EV_NONE;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return bEventInQueue;
}

void
vMBPEventDelete( xMBPEventHandle xEventHdl )
{
    xEventInternalHandle *pxEventIntHdl = xEventHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxEventIntHdl, arxEventHdls ) )
    {
        HDL_RESET( pxEventIntHdl );
    }
    MBP_EXIT_CRITICAL_SECTION(  );
}

