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
 * File: $Id: portother.c,v 1.1 2010/06/05 09:57:48 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "ch.h"
#include <stdlib.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbport.h"
#include "common/mbtypes.h"
#include "common/mbframe.h"
#include "common/mbutils.h"

typedef BYTE u8;
typedef uint32_t u32;
/* ----------------------- Variables ----------------------------------------*/
static UBYTE    ubNesting = 0;
static unsigned long ulOldBasePRI;
BOOL bIsWithinException;
BOOL bIsIncriticalSection;
/* ----------------------- Start implementation -----------------------------*/
void
assert_failed( u8 * file, u32 line )
{
    ( void )file;
    ( void )line;
    vMBPAssert(  );
}

void
vMBPAssert( void )
{
    volatile BOOL   bBreakOut = FALSE;

    vMBPEnterCritical(  );
    while( !bBreakOut );
}


void
vMBPortSetWithinException( BOOL bInException )
{
    bIsWithinException = bInException;
}

BOOL
bMBPortIsWithinException( void )
{
    return bIsWithinException;
}


void
vMBPEnterCritical( void )
{
 if ((!bIsWithinException) && (!bIsIncriticalSection)) {
    bIsIncriticalSection=TRUE;
  }
}

void
vMBPExitCritical( void )
{
  if ((!bIsWithinException) && bIsIncriticalSection) {
    bIsIncriticalSection=FALSE;
  }
}



