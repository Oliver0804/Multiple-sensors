/**************************************************************************************************
  Filename:       zcl_samplesw.h
  Revised:        $Date: 2015-08-19 17:11:00 -0700 (Wed, 19 Aug 2015) $
  Revision:       $Revision: 44460 $


  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef ZCL_SAMPLESW_H
#define ZCL_SAMPLESW_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
* Sensor Type
*/


#define CMDID_POWER             0x0
#define	CMDID_VOC               0x0001
#define CMDID_O3                0x0002
#define CMDID_CHO2              0x0003
#define	CMDID_CO                0x0004
#define CMDID_CO2               0x0005
#define CMDID_PM25              0x0006
#define CMDID_TEMPERATURE       0x0007
#define	CMDID_HUMIDITY          0x0008
#define CMDID_O2                0x0009
#define CMDID_YEAR              0x000A
#define CMDID_MONTH             0x000B
#define CMDID_DATE              0x000C
#define CMDID_HOUR              0x000D
#define CMDID_MINUTE            0x000E
#define CMDID_SECOND            0x000F
#define	CMDID_CO2_PARA          0x0010
#define CMDID_HUMIDITY_PARA     0x0011
#define CMDID_PM25_PARA         0x0012
#define	CMDID_VOC_PARA          0x0013
#define CMDID_WORK_MODE         0x0014
#define CMDID_FAN_SPEED         0x0015
#define CMDID_AUTO_POWER        0x0016
#define CMDID_HIGH_EFFICIENCY_FILTER            0x0017
#define	CMDID_LOW_EFFICIENCY_FILTER             0x0018
#define CMDID_PWM_PROGRESS_VALUE                0x0019

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define SAMPLESW_ENDPOINT               8

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Events for the sample app
#define SAMPLEAPP_END_DEVICE_REJOIN_EVT   0x0001
  
  
#ifdef ZDO_COORDINATOR
#else
  // rejoin
  #define SAMPLEAPP_REJOIN_EVT          0x0080
  #define SAMPLEAPP_REJOIN_PERIOD       1000
  // Report Event
  #define SAMPLEAPP_REPORT_EVT          0x0040
  #define SAMPLEAPP_REPORT_PERIOD       3000
#endif  
  

#define SAMPLEAPP_END_DEVICE_REJOIN_DELAY 5000

/*********************************************************************
 * MACROS cstx.taobao.com
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleSw_SimpleDesc;

extern SimpleDescriptionFormat_t zclSampleSw9_SimpleDesc;

extern CONST zclAttrRec_t zclSampleSw_Attrs[];

extern uint8  zclSampleSw_OnOff;

extern uint16 zclSampleSw_IdentifyTime;

extern uint8 zclSampleSw_OnOffSwitchType;

extern uint8 zclSampleSw_OnOffSwitchActions;

extern CONST uint8 zclSampleSw_NumAttributes;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleSw_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleSw_event_loop( byte task_id, UINT16 events );

/*
 *  Reset all writable attributes to their default values.
 */
extern void zclSampleSw_ResetAttributesToDefaultValues(void); //implemented in zcl_samplesw_data.c

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLEAPP_H */
