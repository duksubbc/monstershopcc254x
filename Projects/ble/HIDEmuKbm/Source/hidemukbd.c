/******************************************************************************

 @file  hidemukbd.c

 @brief This file contains the HID emulated keyboard sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2011-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include <stdio.h>
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "osal_snv.h"   
   
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "hidtouchservice.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"
#include "hidemukbd.h"

#include "simpleGATTprofile.h"

#define CONFIG_AUTO_CLICK
/*********************************************************************
 * MACROS
 */

// Selected HID keycodes
#define KEY_RIGHT_ARROW             0x4F
#define KEY_LEFT_ARROW              0x50
#define KEY_NONE                    0x00

// Selected HID LED bitmaps
#define LED_NUM_LOCK                0x01
#define LED_CAPS_LOCK               0x02

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        7

// HID touch input report length
#define HID_TOUCH_IN_RPT_LEN        8 //8   
/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              60000

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE //GAPBOND_PAIRING_MODE_WAIT_FOR_REQ /*GAPBOND_PAIRING_MODE_INITIATE*/

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE //TRUE/*FALSE*/

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

#define PASSCODE_LEN                          6


#define TOUCH_INTERVAL                        180
#define FW_VERSION(x,y,z)                     ((( x & 0x0F) << 12)|((y & 0x0F) << 8)|(z & 0xFF))
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 hidEmuKbdTaskId;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x08,                             // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'M','O','N','S','T','E','R'
};

// Advertising data
static uint8 advData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  //LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
  //HI_UINT16(GAP_APPEARE_HID_KEYBOARD),
  LO_UINT16(GAP_APPEARE_HID_MOUSE),
  HI_UINT16(GAP_APPEARE_HID_MOUSE),
  //LO_UINT16(GAP_APPEARE_HID_DIGITIZER_TYABLET),
  //HI_UINT16(GAP_APPEARE_HID_DIGITIZER_TYABLET),

  // service UUIDs
  0x07,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERV_UUID),
  HI_UINT16(HID_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID),
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
};

// Device name attribute value
static CONST uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "MONSTER";

// HID Dev configuration
static hidDevCfg_t hidEmuKbdCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};


static hidDevStatus_t hidEmuKbdStatus =
{
  GAPROLE_INIT
};


// TRUE if boot mouse enabled
static uint8 hidBootMouseEnabled = FALSE;

static uint8 remainingPasscodeDigits = 0;
static uint32 passcode;

static uint16  configUpdate = 0;

static uint8  startDevice = 0;
static uint8  power_on = 0;


static uint8   hidConnected = 0;

static uint8   touchEvent = 0;
static int16   mWheel = -1;
static uint32  mWheelTimeOut = 1000;
static uint8   mWheelMode = 1;

static uint8   gConfigSaved = 0;
static uint8   keyPress[4] = {0,0,0,0};
static uint16  xypoint[4][2] = {{200,500},{800,500},{200,800},{800,800}};
static uint16  keyPressInterval[4] = {500,500,500,500};
static uint8   keyPressMode[4] = {0,0,0,0};
static uint8   keyWheelMode[4] = {0,1,1,1};


static uint8   keyCurrentMode[SIMPLEPROFILE_CHAR4_LEN] = {0,0,0,0,0,0,0,0};

/* default S9 */
static uint32  displayResolution[2] = {1440,2560};

static uint32  displayWidth = 1440; //720;
static uint32  displayHight = 2560; //1280;

static uint32  displayMaxWidth = 5000;
static uint32  displayMaxHight = 5000;


static uint16  touchInterval = TOUCH_INTERVAL;

uint8   gEnableSendMsg = 1;

#define SW_A    0
#define SW_B    1
#define SW_C    2
#define SW_D    3

#define KEY_RELEASE_TIME   100
#define SWLONG_PRESS_TIME  1500

#define CAL_TOUCHX(X)   ((X*displayMaxWidth)/displayWidth)
#define CAL_TOUCHY(Y)   ((Y*displayMaxHight)/displayHight)

#define KEY_DIG           0x74


#define WHEEL_ENABLE    1
#define WHEEL_SCROLL    2
#define WHEEL_RETURN    3

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void hidEmuKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void hidEmuKbd_HandleKeys( uint8 shift, uint8 keys );
static void hidEmuKbd_ProcessGattMsg( gattMsgEvent_t *pMsg );
static void hidEmuKbdSendReport( uint8 Modifier , uint8 keycode );
//static void hidEmuKbdSendMouseReport( uint8 buttons );
static void hidEmuKbdSendMouseReport( uint8 buttons , uint8 xp , uint8 yp );
static void hidEmuKbdSendTouchReport( uint8 buttons , uint16 xp , uint16 yp );
static void hidEmuKbdSendForceTouchReport( uint8 buttons , uint16 xp , uint16 yp );
static void hidEmuKbdSendWhellReport( uint8 buttons , uint16 xp , uint16 yp ,uint8 dir);



static uint8 hidEmuKbdRcvReport( uint8 len, uint8 *pData );
static uint8 hidEmuKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint8 *pLen, uint8 *pData );

static void hidEmuKbdEvtCB( uint8 evt );

static void hidEmuKbdPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                    uint8 uiInputs, uint8 uiOutputs );

static void hidEmuKbdGetPasscode( uint8 keycode );

static void simpleProfileChangeCB( uint8 paramID );



uint8 getTouchEvent(void);

void enableTouchEvent(uint8 id);
void disableTouchEvent(uint8 id);
void saveConfigureData(void);

void loadConfigureData(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

//static hidDevCB_t hidEmuKbdHidCBs =
//{
//  hidEmuKbdRptCB,
//  hidEmuKbdEvtCB,
//  hidEmuKbdPasscodeCB
//};

static hidDevCB_t hidEmuKbdHidCBs =
{
  hidEmuKbdRptCB,
  hidEmuKbdEvtCB,
  NULL
};


// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmuKbd_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void HidEmuKbd_Init( uint8 task_id )
{
  hidEmuKbdTaskId = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *) attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 1234/*DEFAULT_PASSCODE*/;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }

  // Setup Battery Characteristic Values
  {
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8), &critical );
  }

  // Set up HID keyboard service
  HidKbM_AddService( );

  // Register for HID Dev callback
  HidDev_Register( &hidEmuKbdCfg, &hidEmuKbdHidCBs , &hidEmuKbdStatus );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( hidEmuKbdTaskId );
  
#if 1
  SimpleProfile_AddService( GATT_ALL_SERVICES );
  
  // Setup the SimpleProfile Characteristic Values
  {
    //uint16 version = FW_VERSION(0,4,25);  // wheel + auto
    uint16 version = FW_VERSION(1,2,0);    // wheel only
    uint8 charValue3 = touchEvent;
    uint8 charValue4 = 0;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 ,6 , 7, 8 };
    
    //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1,2,&displayWidth);
    //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,2,&displayHight);
    
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 2, &version );
    //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, 2, &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }
#endif
  
#if defined( CC2540_MINIDK )
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )
  
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  //PWR_SW_SBIT = 1;
  //power_on = 1;
  // Setup a delayed profile startup
  osal_snv_init();
 
  hidConnected = 0;
  touchEvent = 0;
  power_on = 0;
  startDevice = 0;
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  HalReKeyConfig();
  loadConfigureData();
  
  osal_start_timerEx( hidEmuKbdTaskId, START_DEVICE_EVT, 1000);
  //osal_set_event( hidEmuKbdTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      HidEmuKbd_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */

char BitChan8(char bitCheckValue, char bitLocation, char setBit)
{
   if (bitLocation > 7)
      return -1;

   if (setBit == 1)
      return bitCheckValue | (1 << bitLocation);
   else
      return bitCheckValue & (~(1 << bitLocation));
}

char BitCheck8(char bitCheckValue, char bitLocation)
{
   if (bitLocation > 7)
      return -1;

   return (bitCheckValue >> bitLocation) & 1;
}

uint16 HidEmuKbd_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
  uint16 xp,yp;

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( hidEmuKbdTaskId )) != NULL )
    {
      hidEmuKbd_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if (events & START_DEVICE_EVT )
  {
    if ((PWR_SW_PORT & PWR_SW_BV) != 0) 
    {
      startDevice = 1;
      PWR_EN_DDR |= PWR_EN_BV;
      PWR_EN_SBIT = 1;
    } else {
      PWR_EN_DDR |= PWR_EN_BV;
      PWR_EN_SBIT = 0;
      KEY_E_DDR |= KEY_E_BV;
    }
    
    return ( events ^ START_DEVICE_EVT );
  }
  
  if (events & HID_SW_RELEASE_EVT )
  {

    if(power_on == 1 && mWheelMode == 1)
    {
      hidEmuKbdSendWhellReport(1,CAL_TOUCHX(500),CAL_TOUCHY(500),1);
    }
    return ( events ^ HID_SW_RELEASE_EVT );
  }
  
  if( events & HID_POWER_OFF_EVT )
  {
    power_on = 0;
    PWR_EN_SBIT = 0;
    return ( events ^ HID_POWER_OFF_EVT );
  }
  
  if( events & HID_POWER_SW_PRESS_EVT )
  {
    if ((PWR_SW_PORT & PWR_SW_BV) != 0)
    {
      //
      if(power_on == 1) {
        // maybe Power Off
        osal_stop_timerEx( hidEmuKbdTaskId, HID_LED_EVT);
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
        if(configUpdate != 0)
          saveConfigureData();
        HidDev_Close();

        //
        // ´ë±â 
        //
        osal_start_timerEx( hidEmuKbdTaskId, HID_POWER_OFF_EVT, 200);
        //power_on = 0;
        //PWR_EN_SBIT = 0;
      } else if(startDevice == 1) {
        //loadConfigureData();
        osal_start_timerEx( hidEmuKbdTaskId, HID_LED_EVT, 500);
        HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
        PWR_EN_DDR |= PWR_EN_BV;
        PWR_EN_SBIT = 1;
        power_on = 1;
        startDevice = 0;
      }      
    }

    return ( events ^ HID_POWER_SW_PRESS_EVT );
  }
  
  if ( events & HID_SW_PRESS_EVT )
  {
    if (!(KEY_A_PORT & KEY_A_BV)) {
      /* Switch A Long Press Event */
      if(touchEvent & (HAL_KEY_SW_1)) {
        touchEvent &= ~HAL_KEY_SW_1;
        osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT);
        if(keyPress[SW_A] == 1) {
          keyPress[SW_A] = 0;
          xp = xypoint[SW_A][0];
          yp = xypoint[SW_A][1];
          hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
        }
        
        keyCurrentMode[SW_A]   = 0x00;
      }
      else {
        touchEvent |= HAL_KEY_SW_1;
        keyPress[SW_A] = 0;
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT, keyPressInterval[SW_A]);
        keyCurrentMode[SW_A] = 0x10;
        keyCurrentMode[SW_A] |= (keyPressInterval[SW_A]/1000);
      }
      
      //
      SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4,SIMPLEPROFILE_CHAR4_LEN, keyCurrentMode);
    } else if (!(KEY_B_PORT & KEY_B_BV)) {
      /* Switch B Long Press Event */
      if(touchEvent & (HAL_KEY_SW_2)) {
        touchEvent &= ~HAL_KEY_SW_2;
        osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT);
        if(keyPress[SW_B] == 1) {
          keyPress[SW_B] = 0;
          xp = xypoint[SW_B][0];
          yp = xypoint[SW_B][1];
          hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
        }
        keyCurrentMode[SW_B] = 0;
      }
      else {
        touchEvent |= HAL_KEY_SW_2;
        keyPress[SW_B] = 0;
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT, keyPressInterval[SW_B]);
        keyCurrentMode[SW_B] = 0x10;
        keyCurrentMode[SW_B] |= (keyPressInterval[SW_B]/1000);
      }
      SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4,SIMPLEPROFILE_CHAR4_LEN, keyCurrentMode);
      
    } else if (!(KEY_C_PORT & KEY_C_BV)) {
      /* Switch C Long Press Event */
      if(touchEvent & (HAL_KEY_SW_3)) {
        touchEvent &= ~HAL_KEY_SW_3;
        osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT);
        if(keyPress[SW_C] == 1) {
          keyPress[SW_C] = 0;
          xp = xypoint[SW_C][0];
          yp = xypoint[SW_C][1];
          hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
        }
        keyCurrentMode[SW_C] = 0;
       
      }
      else {
        touchEvent |= HAL_KEY_SW_3;
        keyPress[SW_C] = 0;
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT, keyPressInterval[SW_C]);
        keyCurrentMode[SW_C] = 0x10;
        keyCurrentMode[SW_C] |= keyPressInterval[SW_C]/1000;
      }
      SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4,SIMPLEPROFILE_CHAR4_LEN, keyCurrentMode);
      
    } else if (!(KEY_D_PORT & KEY_D_BV)) {
      /* Switch D Long Press Event */
      if(touchEvent & (HAL_KEY_SW_4)) {
        touchEvent &= ~HAL_KEY_SW_4;
        osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT);
        if(keyPress[SW_D] == 1) {
          keyPress[SW_D] = 0;
          xp = xypoint[SW_D][0];
          yp = xypoint[SW_D][1];
          hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
        }
        keyCurrentMode[SW_D] = 0;
        
      }
      else {
        touchEvent |= HAL_KEY_SW_4;
        keyPress[SW_D] = 0;
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT, keyPressInterval[SW_D]);
        keyCurrentMode[SW_D] = 0x10;
        keyCurrentMode[SW_D] |= keyPressInterval[SW_D]/1000;;
      }
      SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4,SIMPLEPROFILE_CHAR4_LEN, keyCurrentMode);
    }
    
    //
    //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &touchEvent );
    
    HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
    
    return ( events ^ HID_SW_PRESS_EVT );
  }
  
  if ( events & HID_KEY1_EVT )
  {
    if(touchEvent & (HAL_KEY_SW_1)) {
      xp = xypoint[SW_A][0];
      yp = xypoint[SW_A][1];
      keyPress[SW_A] ^= 1;
      
      hidEmuKbdSendTouchReport(keyPress[SW_A],CAL_TOUCHX(xp),CAL_TOUCHY(yp));

      if(keyPress[SW_A] == 1) {
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT, touchInterval);
      } else {
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT, keyPressInterval[SW_A]-touchInterval); 
      }
    }
    return ( events ^ HID_KEY1_EVT );
  }
  
  if ( events & HID_KEY2_EVT )
  {
    if(touchEvent & (HAL_KEY_SW_2)) {
      xp = xypoint[SW_B][0];
      yp = xypoint[SW_B][1];
      keyPress[SW_B] ^= 1;
      hidEmuKbdSendTouchReport(keyPress[SW_B],CAL_TOUCHX(xp),CAL_TOUCHY(yp));
      
      if(keyPress[SW_B] == 1) {
        //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &btn );
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT, touchInterval);
      } else {
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT, keyPressInterval[SW_B]-touchInterval); 
      }
      
    }
    return ( events ^ HID_KEY2_EVT );
  }
  
  if ( events & HID_KEY3_EVT )
  {
    if(touchEvent & (HAL_KEY_SW_3)) {
      xp = xypoint[SW_C][0];
      yp = xypoint[SW_C][1];
      keyPress[SW_C] ^= 1;
      hidEmuKbdSendTouchReport(keyPress[SW_C],CAL_TOUCHX(xp),CAL_TOUCHY(yp));

      if(keyPress[SW_C] == 1)
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT, touchInterval);
      else
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT, keyPressInterval[SW_C]-touchInterval); 

    }
    return ( events ^ HID_KEY3_EVT );
  }
  
  if ( events & HID_KEY4_EVT )
  {
    if(touchEvent & (HAL_KEY_SW_4)) {
      xp = xypoint[SW_D][0];
      yp = xypoint[SW_D][1];
      keyPress[SW_D] ^= 1;
      hidEmuKbdSendTouchReport(keyPress[SW_D],CAL_TOUCHX(xp),CAL_TOUCHY(yp));

      if(keyPress[SW_D] == 1)
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT, touchInterval);
      else
        osal_start_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT, keyPressInterval[SW_D]-touchInterval); 
      
    }
    return ( events ^ HID_KEY4_EVT );
  }

  if(events & HID_LED_EVT )
  {
    if( hidEmuKbdStatus.status == GAPROLE_CONNECTED)
    {
      if(hidConnected != 1)
      {
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );
        HalLedSet(HAL_LED_1, HAL_LED_MODE_FLASH );
        hidConnected = 1;
      } else {
        HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );
      }
    } else {
      hidEmuKbdSendTouchReport(0,0,0);
      hidConnected = 0;
    }
        
    osal_start_timerEx( hidEmuKbdTaskId, HID_LED_EVT, 1000);
    return ( events ^ HID_LED_EVT );
  }
  
  if(events & HID_SAVE_EVT )
  {
    saveConfigureData();
    configUpdate = 0;
    HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
    return ( events ^ HID_SAVE_EVT );
  }

   return 0;
}

/*********************************************************************
 * @fn      hidEmuKbd_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidEmuKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      hidEmuKbd_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      hidEmuKbd_ProcessGattMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      hidEmuKbd_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void hidEmuKbd_HandleKeys( uint8 shift, uint8 keys )
{
  static uint16 xp ,yp ;
  static uint8 prevKey1 = 0;
  static uint8 prevKey2 = 0;
  static uint8 prevKey3 = 0;
  static uint8 prevKey4 = 0;
  static uint8 prevKey5 = 0;
  
  static uint8  auto_mode = 0;
  
  (void)shift;  // Intentionally unreferenced parameter
  
  //KEY_E
  if ((keys & HAL_KEY_SW_5) && (prevKey5 == 0) )
  {
    // pressed
    prevKey5 = 1;
    osal_start_timerEx( hidEmuKbdTaskId, HID_POWER_SW_PRESS_EVT, SWLONG_PRESS_TIME);
    osal_stop_timerEx(hidEmuKbdTaskId, HID_SW_RELEASE_EVT);
    if(power_on == 1 && mWheelMode == 1)
    {
      hidEmuKbdSendWhellReport(1,CAL_TOUCHX(500),CAL_TOUCHY(500),0);
    }
    return;
  }
  else if (!(keys & HAL_KEY_SW_5) && (prevKey5 == 1) ) 
  {
    prevKey5 = 0;
    
    osal_stop_timerEx( hidEmuKbdTaskId, HID_POWER_SW_PRESS_EVT);
    if(mWheelTimeOut != 0)
      osal_start_timerEx(hidEmuKbdTaskId, HID_SW_RELEASE_EVT,mWheelTimeOut);
    return;
  }
  
  
  ///////////////////////////////////////////////////////////////
  if(keys != 0)
  {
    auto_mode = getTouchEvent();
  
    if((auto_mode & (HAL_KEY_SW_1)) == HAL_KEY_SW_1)
    {
      if ((keys & HAL_KEY_SW_1) != HAL_KEY_SW_1)
        disableTouchEvent(0);
    }
    
    if((auto_mode & (HAL_KEY_SW_2)) == HAL_KEY_SW_2)
    {
       if ((keys & HAL_KEY_SW_2) != HAL_KEY_SW_2)
        disableTouchEvent(1);
    }
    
    if((auto_mode & (HAL_KEY_SW_3)) == HAL_KEY_SW_3)
    {
       if ((keys & HAL_KEY_SW_3) != HAL_KEY_SW_3)
        disableTouchEvent(2);
    }
    
    if((auto_mode & (HAL_KEY_SW_4)) == HAL_KEY_SW_4)
    {
       if ((keys & HAL_KEY_SW_4) != HAL_KEY_SW_4)
        disableTouchEvent(3);
    }
  } 
  ///////////////////////////////////////////////////////////////

  //KEY_A
  if ((keys & HAL_KEY_SW_1) && (prevKey1 == 0) )
  {
    // pressed
    if ( remainingPasscodeDigits == 0 )
    {
      if(keyPressMode[SW_A] == 1)
      {
        xp = xypoint[SW_A][0];
        yp = xypoint[SW_A][1];
        hidEmuKbdSendTouchReport(1,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK
        osal_start_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT, SWLONG_PRESS_TIME);
#endif
      }
    } 
    else
    {
      hidEmuKbdGetPasscode(KEY_1);
    }
    prevKey1 = 1;
    
    if( hidEmuKbdStatus.status != GAPROLE_CONNECTED)
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK );
    }
  }
  else if (!(keys & HAL_KEY_SW_1) && (prevKey1 == 1) )
  {
    // released
    if(keyPressMode[SW_A] == 1)
    {
      xp = xypoint[SW_A][0];
      yp = xypoint[SW_A][1];
      hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK      
      osal_stop_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT);
#endif
    }
    
    //if((auto_mode & (HAL_KEY_SW_1)) == HAL_KEY_SW_1)
    //  enableTouchEvent(0);
    if((auto_mode & (HAL_KEY_SW_2)) == HAL_KEY_SW_2)
      enableTouchEvent(1);
    if((auto_mode & (HAL_KEY_SW_3)) == HAL_KEY_SW_3)
      enableTouchEvent(2);
    if((auto_mode & (HAL_KEY_SW_4)) == HAL_KEY_SW_4)
      enableTouchEvent(3);
    
    prevKey1 = 0;
  }
  
  //KEY_B
  if ( (keys & HAL_KEY_SW_2) && (prevKey2 == 0) )
  {
    // pressed
    if (remainingPasscodeDigits == 0 )
    {
      if(keyPressMode[SW_B] == 1)
      {
        xp = xypoint[SW_B][0];
        yp = xypoint[SW_B][1];
        //hidEmuKbdSendWhellReport(1,CAL_TOUCHX(xp),CAL_TOUCHY(yp),1);
        hidEmuKbdSendTouchReport(1,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK              
        osal_start_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT, SWLONG_PRESS_TIME);
#endif        
      }
    } 
    else
    {
      hidEmuKbdGetPasscode(KEY_2);
    }
    prevKey2 = 1;
    if( hidEmuKbdStatus.status != GAPROLE_CONNECTED)
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK );
    }
  }
  else if ( !(keys & HAL_KEY_SW_2) && (prevKey2 == 1) )
  {
    // released
    if(keyPressMode[SW_B] == 1)
    {
      xp = xypoint[SW_B][0];
      yp = xypoint[SW_B][1];
      hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK            
      osal_stop_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT);
#endif      
    }
    
    
    if((auto_mode & (HAL_KEY_SW_1)) == HAL_KEY_SW_1)
      enableTouchEvent(0);
    //if((auto_mode & (HAL_KEY_SW_2)) == HAL_KEY_SW_2)
    //  enableTouchEvent(1);
    if((auto_mode & (HAL_KEY_SW_3)) == HAL_KEY_SW_3)
      enableTouchEvent(2);
    if((auto_mode & (HAL_KEY_SW_4)) == HAL_KEY_SW_4)
      enableTouchEvent(3);
    
    prevKey2 = 0;
  }
  
  //KEY_C
  if ( (keys & HAL_KEY_SW_3) && (prevKey3 == 0) )
  {
    // pressed
    // pressed
    if ( remainingPasscodeDigits == 0 )
    {
      if(keyPressMode[SW_C] == 1)
      {
        xp = xypoint[SW_C][0];
        yp = xypoint[SW_C][1];
        hidEmuKbdSendTouchReport(1,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK              
        osal_start_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT, SWLONG_PRESS_TIME);
#endif        
      }
    } 
    else
    {
      hidEmuKbdGetPasscode(KEY_3);
    }
    prevKey3 = 1;
   if( hidEmuKbdStatus.status != GAPROLE_CONNECTED)
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK );
    }
  }
  else if ( !(keys & HAL_KEY_SW_3) && (prevKey3 == 1) )
  {
    // released
    if(keyPressMode[SW_C] == 1)
    {
      xp = xypoint[SW_C][0];
      yp = xypoint[SW_C][1];
      hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK            
      osal_stop_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT);
#endif      
    }
    
    if((auto_mode & (HAL_KEY_SW_1)) == HAL_KEY_SW_1)
      enableTouchEvent(0);
    if((auto_mode & (HAL_KEY_SW_2)) == HAL_KEY_SW_2)
      enableTouchEvent(1);
    //if((auto_mode & (HAL_KEY_SW_3)) == HAL_KEY_SW_3)
    //  enableTouchEvent(2);
    if((auto_mode & (HAL_KEY_SW_4)) == HAL_KEY_SW_4)
      enableTouchEvent(3);
    prevKey3 = 0;
  }
  
  //KEY_D
  if ( (keys & HAL_KEY_SW_4) && (prevKey4 == 0) )
  {
    // pressed
    if ( remainingPasscodeDigits == 0 )
    {
      if(keyPressMode[SW_D] == 1)
      {
        xp = xypoint[SW_D][0];
        yp = xypoint[SW_D][1];
        //hidEmuKbdSendWhellReport(1,CAL_TOUCHX(xp),CAL_TOUCHY(yp),0);
        hidEmuKbdSendTouchReport(1,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
        //hidEmuKbdSendMouseReport(1,xp,yp);
        //hidEmuKbdSendReport(0, KEY_D );
#ifdef CONFIG_AUTO_CLICK              
        osal_start_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT, SWLONG_PRESS_TIME);
#endif        
      }
    } 
    else
    {
      hidEmuKbdGetPasscode(KEY_4);
    }
    prevKey4 = 1;
    if( hidEmuKbdStatus.status != GAPROLE_CONNECTED)
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK );
    }
  }
  else if ( !(keys & HAL_KEY_SW_4) && (prevKey4 == 1) )
  {
    // released
    if(keyPressMode[SW_D] == 1)
    {
      xp = xypoint[SW_D][0];
      yp = xypoint[SW_D][1];
      hidEmuKbdSendTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
#ifdef CONFIG_AUTO_CLICK            
      osal_stop_timerEx( hidEmuKbdTaskId, HID_SW_PRESS_EVT);
#endif
    }
    
   if((auto_mode & (HAL_KEY_SW_1)) == HAL_KEY_SW_1)
      enableTouchEvent(0);
    if((auto_mode & (HAL_KEY_SW_2)) == HAL_KEY_SW_2)
      enableTouchEvent(1);
    if((auto_mode & (HAL_KEY_SW_3)) == HAL_KEY_SW_3)
      enableTouchEvent(2);
    //if((auto_mode & (HAL_KEY_SW_4)) == HAL_KEY_SW_4)
    //  enableTouchEvent(3);
    prevKey4 = 0;
    
  }
}


/*********************************************************************
 * @fn      hidEmuKbdGetPasscode
 *
 * @brief   Build and send a passcode.
 *
 * @param   keycode - HID keycode
 *
 * @return  none
 */
static void hidEmuKbdGetPasscode( uint8 keycode )
{
  if (( keycode >= KEY_1 ) && ( keycode <= KEY_0 ) )
  {
    // Append new digit to passcode
    passcode *= 10;
    passcode += ( ( keycode - KEY_1 + 1 ) % 10 );

    if ( --remainingPasscodeDigits == 0 )
    {
       // Send passcode response
      HidDev_PasscodeRsp( SUCCESS, passcode );
    }
  }
  else
  {
    // Send passcode response
    HidDev_PasscodeRsp( SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED, passcode );
    remainingPasscodeDigits = 0;
  }
  
}

/*********************************************************************
 * @fn      hidEmuKbd_ProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void hidEmuKbd_ProcessGattMsg( gattMsgEvent_t *pMsg )
{
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      hidEmuKbdSendReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void hidEmuKbdSendReport( uint8 Modifier , uint8 keycode )
{
  uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = Modifier;         // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidEmuKbdSendMouseReport
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   buttons - Mouse button code
 *
 * @return  none
 */
static void hidEmuKbdSendMouseReport( uint8 buttons , uint8 xp , uint8 yp )
{
  uint8 buf[HID_MOUSE_IN_RPT_LEN];

  buf[0] = buttons;   // Buttons
  buf[1] = 0;         // X
  buf[2] = 100;        // 
  buf[3] = 0;         // 
  buf[4] = 100;        // Y
  buf[5] = 0;         // Wheel
  buf[6] = 0;         // AC Pan
  if(gEnableSendMsg == 1)
  {
    HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                   HID_MOUSE_IN_RPT_LEN, buf );
  }
}


static void hidEmuKbdSendTouchReport( uint8 buttons , uint16 xp , uint16 yp )
{
  uint8 buf[HID_TOUCH_IN_RPT_LEN];

  buf[0] =  buttons | ( 1 << 1);     // Buttons
  buf[1] =  (uint8)(xp & 0xFF);  
  buf[2] =  (uint8)(xp >> 8) & 0xFF;
  buf[3] =  (uint8)(yp & 0xFF); 
  buf[4] =  (uint8)(yp >> 8) & 0xFF; 
  //buf[5] =  (uint8)(yp & 0xFF); ;  // Wheel
  //buf[6] =  (uint8)(yp >> 8) & 0xFF; ;  // AC Pan
  buf[5] =  0;  // Wheel
  buf[6] =  0;  // Wheel
  buf[7] =  0;  // AC Pan

  if(gEnableSendMsg == 1)
  {
    HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                   HID_TOUCH_IN_RPT_LEN, buf );
  }
}


static void hidEmuKbdSendForceTouchReport( uint8 buttons , uint16 xp , uint16 yp )
{
  uint8 buf[HID_TOUCH_IN_RPT_LEN];
  
  buf[0] =  buttons | ( 1 << 1);     // Buttons
  buf[1] =  (uint8)(xp & 0xFF);  
  buf[2] =  (uint8)(xp >> 8) & 0xFF;
  buf[3] =  (uint8)(yp & 0xFF); 
  buf[4] =  (uint8)(yp >> 8) & 0xFF; 

  buf[5] =  0;  // Wheel
  buf[6] =  0;  // AC Pan
  buf[7] =  0;  // AC Pan


  HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                 HID_TOUCH_IN_RPT_LEN, buf );
}

static void hidEmuKbdSendWhellReport( uint8 buttons , uint16 xp , uint16 yp , uint8  dir )
{
  int16 wheel ;
  uint8 buf[HID_TOUCH_IN_RPT_LEN];

  buf[0] =  0 | ( 1 << 1);     // Buttons
  buf[1] =  (uint8)(xp & 0xFF);  
  buf[2] =  (uint8)(xp >> 8) & 0xFF;
  buf[3] =  (uint8)(yp & 0xFF); ;  // Wheel
  buf[4] =  (uint8)(yp >> 8) & 0xFF; ;  // AC Pan
  buf[7] =  0;  // AC Pan
  if(dir == 1)
  {
    wheel = 0x7FF0;
    buf[5] =  (uint8)(wheel & 0xFF); ;  // Wheel
    buf[6] =  (uint8)(wheel >> 8) & 0xFF; ;  // AC Pan
  }
  else 
  {
    buf[5] =  (uint8)(mWheel & 0xFF); ;  // Wheel
    buf[6] =  (uint8)(mWheel >> 8) & 0xFF; ;  // AC Pan
  }
 
  HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                 HID_TOUCH_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidEmuKbdRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8 hidEmuKbdRcvReport( uint8 len, uint8 *pData )
{
  // verify data length
  if ( len == HID_LED_OUT_RPT_LEN )
  {
    // set keyfob LEDs
    HalLedSet( HAL_LED_1, ((*pData & LED_CAPS_LOCK) == LED_CAPS_LOCK) );
    HalLedSet( HAL_LED_2, ((*pData & LED_NUM_LOCK) == LED_NUM_LOCK) );

    return SUCCESS;
  }
  else
  {
    return ATT_ERR_INVALID_VALUE_SIZE;
  }
}

/*********************************************************************
 * @fn      hidEmuKbdRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8 hidEmuKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint8 *pLen, uint8 *pData )
{
  uint8 status = SUCCESS;

  // write
  if ( oper == HID_DEV_OPER_WRITE )
  {
    if ( uuid == REPORT_UUID )
    {
      // process write to LED output report; ignore others
      if ( type == HID_REPORT_TYPE_OUTPUT )
      {
        status = hidEmuKbdRcvReport( *pLen, pData );
      }
    }

    if ( status == SUCCESS )
    {
      status = HidKbM_SetParameter( id, type, uuid, *pLen, pData );
    }
  }
  // read
  else if ( oper == HID_DEV_OPER_READ )
  {
    status = HidKbM_GetParameter( id, type, uuid, pLen, pData );
  }
  // notifications enabled
  else if ( oper == HID_DEV_OPER_ENABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = TRUE;
    }
  }
  // notifications disabled
  else if ( oper == HID_DEV_OPER_DISABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = FALSE;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      hidEmuKbdEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidEmuKbdEvtCB( uint8 evt )
{
  // process enter/exit suspend or enter/exit boot mode

  return;
}

static void hidEmuKbdPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                    uint8 uiInputs, uint8 uiOutputs )
{
  remainingPasscodeDigits = PASSCODE_LEN;
  passcode = 0;
}



/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue[SIMPLEPROFILE_CHAR5_LEN];

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValue );
      //HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR2:
      {
        //
        uint16 value ;
        if(SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2,&value) == SUCCESS)
        {
          touchInterval = value;
        }
      }
      break;
   case SIMPLEPROFILE_CHAR5:
     {
      uint8 idx = 0;
      uint8 type ;
      uint16 lo,hi,sec;
      
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, newValue );
      
      idx =  newValue[0]; // 
      type = newValue[1]; // type
      
      if(idx == 0xFF)
      {
        if(type == 'A')
        {
          if((touchEvent & (HAL_KEY_SW_1)))
            osal_start_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT, keyPressInterval[SW_A]);
          if(touchEvent & (HAL_KEY_SW_2))
            osal_start_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT, keyPressInterval[SW_B]);
          if(touchEvent & (HAL_KEY_SW_3))
            osal_start_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT, keyPressInterval[SW_C]);
          if(touchEvent & (HAL_KEY_SW_4))
            osal_start_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT, keyPressInterval[SW_D]);
            
          gEnableSendMsg = 1;
          
        }
        else if( type == 'D')
        {
          //
          //touchEvent &= ~(HAL_KEY_SW_1|HAL_KEY_SW_2|HAL_KEY_SW_3|HAL_KEY_SW_4);
          if(touchEvent & (HAL_KEY_SW_1))
            osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT);
          if(touchEvent & (HAL_KEY_SW_2))
            osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT);
          if(touchEvent & (HAL_KEY_SW_3))
            osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT);
          if(touchEvent & (HAL_KEY_SW_4))
            osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT);
        }
      }
      else
      {
        if(type == 0) {
          // set press interval
          sec =  newValue[2]*1000; // reflash interval
          if(sec == 0)
            sec = 500;
          
          keyPressInterval[idx] = sec;

          hi = newValue[4]*100;
          lo = newValue[5];
          xypoint[idx][0] = hi+lo;
          
          hi = newValue[6]*100;
          lo = newValue[7];
          xypoint[idx][1] = hi+lo;

        } else if(type == 1) {
          
          hi = newValue[4]*100;
          lo = newValue[5];
          displayWidth = hi+lo;
          
          hi = newValue[6]*100;
          lo = newValue[7];
          displayHight = hi+lo;
          
          
          displayResolution[0] = displayWidth;
          displayResolution[1] = displayHight;
          //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1,2,&displayWidth);
          //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,2,&displayHight);
          
        } else if(type == 'A') {
          keyPressMode[idx] = 1;
        } else if(type == 'D') {
          keyPressMode[idx] = 0;
          disableTouchEvent(idx);
        } else if(type == 'S') {
          if(idx == 0x05) {              
            keyWheelMode[WHEEL_ENABLE] = newValue[2];
            keyWheelMode[WHEEL_SCROLL] = newValue[3];
            keyWheelMode[WHEEL_RETURN] = newValue[4];
            
            mWheel = (-1)*keyWheelMode[WHEEL_SCROLL];
            mWheelTimeOut = keyWheelMode[WHEEL_RETURN] * 1000;
            mWheelMode = keyWheelMode[WHEEL_ENABLE];
          }
          
        }
        
        HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
        osal_start_timerEx( hidEmuKbdTaskId, HID_SAVE_EVT,10000);

        configUpdate++;
      }
      
     }
      break;
    default:
      // should not reach here!
      break;
  }
}




uint8 getTouchEvent(void)
{
  return touchEvent;
}

void enableTouchEvent(uint8 id)
{
  switch(id)
  {
  case 0:
    if((touchEvent & (HAL_KEY_SW_1)) == 0)
    {
      touchEvent |= HAL_KEY_SW_1;
      osal_start_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT, keyPressInterval[SW_A]);
    }
    break;
  case 1:
    if((touchEvent & (HAL_KEY_SW_2)) == 0)
    {
      touchEvent |= HAL_KEY_SW_2;
      osal_start_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT, keyPressInterval[SW_B]);
    }
    break;
  case 2:
    if((touchEvent & (HAL_KEY_SW_3)) == 0)
    {
      touchEvent |= HAL_KEY_SW_3;
      osal_start_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT, keyPressInterval[SW_C]);
    }
    break;
  case 3:
    if((touchEvent & (HAL_KEY_SW_4)) == 0)
    {
      touchEvent |= HAL_KEY_SW_4;
      osal_start_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT, keyPressInterval[SW_D]);
    }
    break;  
  }
}

void disableTouchEvent(uint8 id)
{
  uint16 xp,yp;
  switch(id)
  {
  case 0:
    if(touchEvent & (HAL_KEY_SW_1))
    {
      touchEvent &= ~HAL_KEY_SW_1;
      osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY1_EVT);
      if(keyPress[SW_A] == 1) {
        keyPress[SW_A] = 0;
        xp = xypoint[SW_A][0];
        yp = xypoint[SW_A][1];
        hidEmuKbdSendForceTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
      }
    }
    break;
  case 1:
    if(touchEvent & (HAL_KEY_SW_2))
    {
      touchEvent &= ~HAL_KEY_SW_2;
      osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY2_EVT);
      if(keyPress[SW_B] == 1) {
        keyPress[SW_B] = 0;
        xp = xypoint[SW_B][0];
        yp = xypoint[SW_B][1];
        hidEmuKbdSendForceTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
      }
    }
    break;
  case 2:
    if(touchEvent & (HAL_KEY_SW_3))
    {
      touchEvent &= ~HAL_KEY_SW_3;
      osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY3_EVT);
      if(keyPress[SW_C] == 1) {
        keyPress[SW_C] = 0;
        xp = xypoint[SW_C][0];
        yp = xypoint[SW_C][1];
        hidEmuKbdSendForceTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
      }
    }
    break;
  case 3:
    if(touchEvent & (HAL_KEY_SW_4))
    {
      touchEvent &= ~HAL_KEY_SW_4;
      osal_stop_timerEx( hidEmuKbdTaskId, HID_KEY4_EVT);
      if(keyPress[SW_D] == 1) {
        keyPress[SW_D] = 0;
        xp = xypoint[SW_D][0];
        yp = xypoint[SW_D][1];
        hidEmuKbdSendForceTouchReport(0,CAL_TOUCHX(xp),CAL_TOUCHY(yp));
      }
    }
    break;  
  }
}

void saveConfigureData(void)
{
  //static uint16  xypoint[4][2] = {{200,500},{800,500},{200,800},{800,800}};
  //static uint16  keyPressInterval[4] = {500,500,500,500};
  /* default S9 */
  //static uint32  displayResolution[2] = {1080,2220};
  
  uint8 len;
  
  if(gConfigSaved != 1)
  {
    gConfigSaved = 1;
    osal_snv_write( 0x80, sizeof(gConfigSaved), &gConfigSaved);
  }
  
  len = sizeof(xypoint);
  osal_snv_write( 0x81, len, xypoint);
  
  len = sizeof(keyPressInterval);
  osal_snv_write( 0x82, len, keyPressInterval );
  
  len = sizeof(displayResolution);
  osal_snv_write( 0x83,len, displayResolution );
  
  len = sizeof(keyPressMode);
  osal_snv_write( 0x84, len, keyPressMode );
  
  len = sizeof(touchInterval);
  osal_snv_write( 0x85, len, &touchInterval );
  
  len = sizeof(keyWheelMode);
  osal_snv_write( 0x86, len, keyWheelMode );
  
}

void loadConfigureData(void)
{
  uint8 len;
  uint16  tmp_xypoint[4][2];
  uint16  tmp_keyPressInterval[4];
  uint32  tmp_displayResolution[2];
  uint8   tmp_keyPressMode[4];
  uint16  tmp_touchInterval;
    
  if(osal_snv_read( 0x80, sizeof(gConfigSaved), &gConfigSaved ) != SUCCESS) 
  {
      return;
  }
  
  if(gConfigSaved != 1)
    return;
  
  len = sizeof(xypoint);
  if(osal_snv_read( 0x81, len, tmp_xypoint ) == SUCCESS) 
  { 
      memcpy(xypoint,tmp_xypoint,len);
  }
  
  len = sizeof(keyPressInterval);
  if(osal_snv_read( 0x82,len, tmp_keyPressInterval ) == SUCCESS) 
  { 
     memcpy(keyPressInterval,tmp_keyPressInterval,len);
  }
  
  len = sizeof(displayResolution);
  if(osal_snv_read( 0x83, sizeof(displayResolution), tmp_displayResolution ) == SUCCESS) 
  { 
     memcpy(displayResolution,tmp_displayResolution,len);
     displayWidth = displayResolution[0];
     displayHight = displayResolution[1];
  }
  
  len = sizeof(keyPressMode);
  if(osal_snv_read( 0x84,len, tmp_keyPressMode ) == SUCCESS) 
  {
     memcpy(keyPressMode,tmp_keyPressMode,len);
  }
  
  len = sizeof(touchInterval);
  if(osal_snv_read(0x85,len, &tmp_touchInterval ) == SUCCESS) 
  {
     memcpy(&touchInterval,&tmp_touchInterval,len);
  }
  
  len = sizeof(keyWheelMode);
  if(osal_snv_read(0x86,len, &tmp_keyPressMode ) == SUCCESS) 
  {
     memcpy(keyWheelMode,tmp_keyPressMode,len);
     mWheel = (-1)*keyWheelMode[WHEEL_SCROLL];
     mWheelTimeOut = keyWheelMode[WHEEL_RETURN] * 1000;
     mWheelMode = keyWheelMode[WHEEL_ENABLE];
  }
}

/*********************************************************************
*********************************************************************/
