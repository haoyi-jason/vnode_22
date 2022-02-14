/* 
 * MODBUS Library: AVR port
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbsconfig.h,v 1.1 2011-08-06 20:17:01 embedded-solutions.cwalter Exp $
 */

#ifndef _MBS_CONFIG_H
#define _MBS_CONFIG_H

#include "mbport.h"

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/
#define MBS_ASCII_ENABLED                       ( 0 )
#define MBS_RTU_ENABLED                         ( 1 )
#define MBS_TCP_ENABLED                         ( 0 )
#define MBS_SERIAL_RTU_MAX_INSTANCES            ( 1 )
#define MBS_SERIAL_ASCII_MAX_INSTANCES          ( 1 )
#define MBS_TCP_MAX_INSTANCES                   ( 0 )
#define MBS_SERIAL_API_VERSION                  ( 1 )

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
