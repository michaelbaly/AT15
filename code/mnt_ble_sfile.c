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
 * File:    mnt_ble_sfile.c
 * Purpose: Process simple file transfer via ble
 *
 * Author:  Mike B
 * Tips: currently only support at command from ble, generic simple file can 
 * extend further
 ******************************************************************************/

#if (defined PRODUCT_AT15) || (defined BLE_SFILE) // MNT-2997

#include "ars_queue.h"
#include "prot.h"
#if defined SUPPORT_AC61_CAMERA
#include "wct_hw_ftpclient.h"
#endif
#include "mnt_ble_sfile.h"
#include "mnt_tftp.h"
#include "mnt_report.h"

DEFINE_MODULE(MODULE_IO);


/* 
 * parse entry for simple file from ble
 * 
*/
void mnt_parseSFile(eFile_e ft, char* pBuff)
{
    // file type
    swtich (ft) {
        case AT_TINY_E:
            mnt_debug_printf(gThisModule, "%s: receive at via BLE \r\n", __func__);
            //TODO: parse tiny at command
            break;
        case SIMPLE_FILE_E:
            mnt_debug_printf(gThisModule, "%s: receive simple file via BLE \r\n", __func__);
            //TODO: not support now
            break;
        default:
            mnt_debug_printf(gThisModule, "%s: unknow file type \r\n", __func__);
            break;
    }
}
#endif

