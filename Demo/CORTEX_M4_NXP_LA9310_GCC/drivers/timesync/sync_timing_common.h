/*!***************************************************************************************
 *  \addtogroup api
 *  @{
 *
 *  \file          sync_timing_common.h
 *
 *  \details       Header file for Sync Timing Driver Common Definitions
 *
 *  \date          Created: 06/27/2018
 *
 ****************************************************************************************/

/****************************************************************************************/
/**                  Copyright (c) 2018, 2021 Skyworks Solution Inc.                   **/
/****************************************************************************************/
/** This software is provided 'as-is', without any express or implied warranty.        **/
/** In no event will the authors be held liable for any damages arising from the use   **/
/** of this software.                                                                  **/
/** Permission is granted to anyone to use this software for any purpose, including    **/
/** commercial applications, and to alter it and redistribute it freely, subject to    **/
/** the following restrictions:                                                        **/
/** 1. The origin of this software must not be misrepresented; you must not claim that **/
/**    you wrote the original software. If you use this software in a product,         **/
/**    an acknowledgment in the product documentation would be appreciated but is not  **/
/**    required.                                                                       **/
/** 2. Altered source versions must be plainly marked as such, and must not be         **/
/**    misrepresented as being the original software.                                  **/
/** 3. This notice may not be removed or altered from any source distribution.         **/
/****************************************************************************************/

#ifndef _SYNC_TIMING_COMMON_H_
#define _SYNC_TIMING_COMMON_H_

/*! Enumeration definition for the supported chip reset types */
typedef enum
{
    SYNC_TIMING_DEVICE_RESET_TOGGLE = 0xC0,      
    //!< Toggle and bring device out of reset in regular firmware mode - HARD RESET
    SYNC_TIMING_DEVICE_RESET_HOLD,               
    //!< Put device into reset - HARD RESET
    SYNC_TIMING_DEVICE_RESET_RELEASE,            
    //!< Release reset line and bring device out of reset in regular firmware mode - HARD RESET
    SYNC_TIMING_DEVICE_RESET_BOOTLOADER_MODE,    
    //!< Bring device out of reset but keep it in bootloader mode
    SYNC_TIMING_DEVICE_RESET_SOFT,
    //!< Toggle and bring device out of reset in regular firmware mode - SOFT RESET
    SYNC_TIMING_DEVICE_RESET_INVALID,
    //!< Invalid Reset Type - used only for range check; DONOT use in application

} SYNC_TIMING_DEVICE_RESET_TYPE_E;

#endif // _SYNC_TIMING_COMMON_H_
