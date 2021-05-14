/******************************************************************************
 * Copyright (c) 2020-2029 Atel Technologies LLC.  All rights reserved.
 *
 * Atel Technologies is supplying this software for use solely and
 * exclusively with Atel Technologies tracker products.
 *
 * This software or documentation or copies thereof are not to be
 * distributed, rented, sub-licensed or otherwise made available to others,
 * except as expressly agreed upon in writing by Atel Technologies.
 *
 * You may not remove this copyright notice from this software.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS,
 * IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE.
 * ATEL TECHNOLOGIES SHALL NOT, UNDER ANY CIRCUMSTANCES, BE LIABLE FOR
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * ----------------------------------------------------------------------------
 *
 * File:    mnt_ble_sfile.h
 * Purpose: Definitions for ble simple file
 *
 * Author:  Mike Bay
 *
 ******************************************************************************/

#ifndef _MNT_BLE_SFILE_H_
#define _MNT_BLE_SFILE_H_
// for debug 
#define BLE_SFILE
#define PRODUCT_AT15
//
#include "monet.h"
#include "mnt_util.h"
#include "mnt_file.h"
#include "mnt_platform.h"
#include "mnt_picif.h"

#if defined BLE_SFILE || defined PRODUCT_AT15
typedef enum sf_type {
    AT_TINY_E = 0x1,
    SIMPLE_FILE_E,
    NR_SUPPORT_MAX,
} sFile_e;
void mnt_parseSFile(sFile_e type, char* pBuff);

#endif /* #if defined SUPPORT_AC61_CAMERA || defined SUPPORT_ZAZU_CAMERA */

#endif  /* #define _MNT_BLE_SFILE_H_ */

