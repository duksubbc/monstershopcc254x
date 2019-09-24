/**************************************************************************************************
  Filename:       hal_key.h
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the KEY Service.


  Copyright 2005-2012 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* Interrupt option - Enable or disable */
#define HAL_KEY_INTERRUPT_DISABLE    0x00
#define HAL_KEY_INTERRUPT_ENABLE     0x01

/* Key state - shift or nornal */
#define HAL_KEY_STATE_NORMAL          0x00
#define HAL_KEY_STATE_SHIFT           0x01

#define HAL_KEY_SW_1 0x01  // Joystick up
#define HAL_KEY_SW_2 0x02  // Joystick right
#define HAL_KEY_SW_5 0x04  // Joystick center
#define HAL_KEY_SW_4 0x08  // Joystick left
#define HAL_KEY_SW_3 0x10  // Joystick down

#define HAL_KEY_SW_6 0x20  // Button S1 if available
#define HAL_KEY_SW_7 0x40  // Button S2 if available
#define HAL_KEY_SW_8 0x80  // Button S2 if available
  
#define HAL_KEY_SW_SHIFT 0x40  // Button S2 if available
#define HAL_KEY_SW_ALT   0x80  // Button S2 if available    

/* Joystick */
#define HAL_KEY_UP     0x01  // Joystick up
#define HAL_KEY_RIGHT  0x02  // Joystick right
#define HAL_KEY_CENTER 0x04  // Joystick center
#define HAL_KEY_LEFT   0x08  // Joystick left
#define HAL_KEY_DOWN   0x10  // Joystick down

/* Buttons */  
#define HAL_PUSH_BUTTON_RIGHT  0x01  // Button right
#define HAL_PUSH_BUTTON_LEFT   0x02  // Button left
#define HAL_PUSH_BUTTON_SELECT 0x04  // Button select
#define HAL_KEY_BUTTON_UP      0x40  // Button up
#define HAL_KEY_BUTTON_DOWN    0x80  // Button down

   
/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef void (*halKeyCBack_t) (uint8 keys, uint8 state);


/// Codes for non-modifier-keys.
enum keycodes {
    KEY_Reserved = 0,     // unused
    KEY_ErrorRollOver,
    KEY_POSTFail,
    KEY_ErrorUndefined,
    KEY_A,                // 0x04
    KEY_B,
    KEY_C,
    KEY_D,
    KEY_E,
    KEY_F,
    KEY_G,
    KEY_H,
    KEY_I,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_M,                // 0x10
    KEY_N,
    KEY_O,
    KEY_P,
    KEY_Q,
    KEY_R,
    KEY_S,
    KEY_T,
    KEY_U,
    KEY_V,
    KEY_W,
    KEY_X,
    KEY_Y,
    KEY_Z,
    KEY_1,                //       1 and !
    KEY_2,                //       2 and @
    KEY_3,                // 0x20  3 and #
    KEY_4,                //       4 and $
    KEY_5,                //       5 and %
    KEY_6,                //       6 and ^
    KEY_7,                //       7 and &
    KEY_8,                //       8 and *
    KEY_9,                //       9 and (
    KEY_0,                // 0x27  0 and )
    KEY_Return,           // 0x28  enter
    KEY_ESCAPE,           // 0x29
    KEY_DELETE,           // 0x2A  backspace
    KEY_Tab,              // 0x2B
    KEY_Spacebar,         // 0x2C
    KEY_minus,            // 0x2D  - and _
    KEY_equals,           // 0x2E  = and +
    KEY_lbracket,         // 0x2F  [ and {
    KEY_rbracket,         // 0x30  ] and }
    KEY_backslash,        // 0x31  \ and |
    KEY_hash,             // 0x32  non-US # and ~
    KEY_semicolon,        // 0x33  ; and :
    KEY_apostroph,        // 0x34  ' and "
    KEY_grave,            // 0x35  grave accent and tilde
    KEY_comma,            // 0x36  , and <
    KEY_dot,              // 0x37  . and >
    KEY_slash,            // 0x38  / and ?
    KEY_capslock,         // 0x39
    KEY_F1,
    KEY_F2,
    KEY_F3,
    KEY_F4,
    KEY_F5,
    KEY_F6,
    KEY_F7,               // 0x40
    KEY_F8,
    KEY_F9,
    KEY_F10,
    KEY_F11,
    KEY_F12,
    KEY_PrintScreen,
    KEY_ScrollLock,
    KEY_Pause,            //       Break
    KEY_Insert,
    KEY_Home,
    KEY_PageUp,
    KEY_DeleteForward,
    KEY_End,
    KEY_PageDown,
    KEY_RightArrow,
    KEY_LeftArrow,        // 0x50
    KEY_DownArrow,
    KEY_UpArrow,
    KEY_NumLock,          //       Clear
    KEY_KPslash,
    KEY_KPasterisk,
    KEY_KPminus,
    KEY_KPplus,
    KEY_KPenter,
    KEY_KP1,              //       End
    KEY_KP2,              //       Down Arrow
    KEY_KP3,              //       Page Down
    KEY_KP4,              //       Left Arrow
    KEY_KP5,
    KEY_KP6,              //       Right Arrow
    KEY_KP7,              //       Home
    KEY_KP8,              // 0x60  Up Arrow
    KEY_KP9,              //       Page Up
    KEY_KP0,              //       Insert
    KEY_KPcomma,          //       Delete
    KEY_Euro,             //       non-US \ and |
    KEY_Application
};

#define KEY_CTRL_L 0xE0
#define KEY_SHIFT_L 0xE1
#define KEY_ALT_L 0xE2
#define KEY_GUI_L 0xE3
#define KEY_CTRL_R 0xE4
#define KEY_SHIFT_R 0xE5
#define KEY_ALT_R 0xE6
#define KEY_GUI_R 0xE7

#if 1
#define KEY_JUML1       0x0100
#define KEY_JUML2       0x0200
#define KEY_JUML3       0x0400
#define KEY_JUMR1       0x1000
#define KEY_JUMR2       0x2000
#define KEY_JUMR3       0x4000
#else
#define KEY_JUML1       0x1000
#define KEY_JUML2       0x2000
#define KEY_JUML3       0x4000
#define KEY_JUMR1       0x0100
#define KEY_JUMR2       0x0200
#define KEY_JUMR3       0x0400
#endif
/**************************************************************************************************
 *                                             GLOBAL VARIABLES
 **************************************************************************************************/
extern bool Hal_KeyIntEnable;

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize the Key Service
 */
extern void HalKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalKeyConfig( bool interruptEnable, const halKeyCBack_t cback);

extern void HalReKeyConfig ( void );
/*
 * Read the Key status
 */
extern uint16 HalKeyRead( void);

extern uint16 HalKeyReadJuM ( void );
/*
 * Enter sleep mode, store important values
 */
extern void HalKeyEnterSleep ( void );

/*
 * Exit sleep mode, retore values
 */
extern uint8 HalKeyExitSleep ( void );

/*
 * This is for internal used by hal_driver
 */
extern void HalKeyPoll ( void );

/*
 * This is for internal used by hal_sleep
 */
extern bool HalKeyPressed( void );

extern uint8 hal_key_keys(void);                                           

extern uint8 hal_key_int_keys(void);

/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
