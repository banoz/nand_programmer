/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef _LOG_H_
#define _LOG_H_

#include <stdio.h>
#include <inttypes.h>

/* This repo targeted a 2015-era arm-none-eabi toolchain; newer newlib
 * builds omit the C99 printf format macros from <inttypes.h> unless newlib
 * itself was configured with IO_C99_FORMATS, so PRIx64 may be undefined. */
#ifndef PRIx64
#define PRIx64 "llx"
#endif

#ifdef DEBUG
    #define DEBUG_PRINT printf
#else
    #define DEBUG_PRINT(...)
#endif
#define ERROR_PRINT(fmt, args...) printf("ERROR: "fmt, ## args)

#endif
