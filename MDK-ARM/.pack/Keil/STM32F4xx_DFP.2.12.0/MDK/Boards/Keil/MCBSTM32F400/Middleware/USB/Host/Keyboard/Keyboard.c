/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Host
 * Copyright (c) 2004-2018 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    Keyboard.c
 * Purpose: USB Host - HID Keyboard example
 *----------------------------------------------------------------------------*/

#include <stdio.h>

#include "RTE_Components.h"

#include "main.h"
#include "rl_usb.h"                     /* RL-USB function prototypes         */
#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

// Main stack size must be multiple of 8 Bytes
#define APP_MAIN_STK_SZ (2048U)
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};

extern int stdout_init (void);

/*------------------------------------------------------------------------------
 *        Application
 *----------------------------------------------------------------------------*/

__NO_RETURN void app_main (void *arg) {
  usbStatus usb_status;                 // USB status
  usbStatus hid_status;                 // HID status
  int       status;
  int       ch;                         // Character
  uint8_t   con = 0U;                   // Connection status of keyboard

  (void)arg;

#ifdef RTE_Compiler_EventRecorder
  EventRecorderInitialize(0, 1);
  EventRecorderEnable (EventRecordError, 0xB0U, 0xB5U);  /* USBH Error Events */
  EventRecorderEnable (EventRecordAll  , 0xB0U, 0xB0U);  /* USBH Core Events */
  EventRecorderEnable (EventRecordAll  , 0xB4U, 0xB4U);  /* USBH HID Events */
#endif

  status = stdout_init ();              // Initialize retargeted stdout
  if (status != 0) {
    for (;;) {}                         // Handle stdout init failure
  }

  usb_status = USBH_Initialize(0U);     // Initialize USB Host 0
  if (usb_status != usbOK) {
    for (;;) {}                         // Handle USB Host 0 init failure
  }
  usb_status = USBH_Initialize(1U);     // Initialize USB Host 1
  if (usb_status != usbOK) {
    for (;;) {}                         // Handle USB Host 1 init failure
  }

  for (;;) {
    hid_status = USBH_HID_GetDeviceStatus(0U);  // Get HID device status
    if (hid_status == usbOK) {
      if (con == 0U) {                  // If keyboard was not connected previously
        con = 1U;                       // Keyboard got connected
        printf("Keyboard connected!\n");
      }
    } else {
      if (con == 1U) {                  // If keyboard was connected previously
        con = 0U;                       // Keyboard got disconnected
        printf("\nKeyboard disconnected!\n");
      }
    }
    if (con != 0U) {                    // If keyboard is active
      ch = USBH_HID_GetKeyboardKey(0U); // Get pressed key
      if (ch != -1) {                   // If valid key value
        if ((ch & 0x10000) != 0) {      // Handle non-ASCII translated keys (Keypad 0 .. 9)
                                        // Bit  16:    non-ASCII bit (0 = ASCII, 1 = not ASCII)
                                        // Bits 15..8: modifiers (SHIFT, ALT, CTRL, GUI)
                                        // Bits  7..0: ASCII or HID key Usage ID if not ASCII
                                        // HID Usage ID values can be found in following link:
                                        // http://www.usb.org/developers/hidpage/Hut1_12v2.pdf
          ch &= 0xFF;                   // Remove non-ASCII bit and modifiers
          if ((ch>=0x59)&&(ch<=0x61)) { // Keypad 1 .. 9 key convert to
            ch = (ch - 0x59) + '1';     // ASCII  1 .. 9
          } else if (ch == 0x62) {      // Keypad 0 key convert to
            ch = '0';                   // ASCII  0
          } else {                      // If not Keypad 0 .. 9
            ch = -1;                    // invalidate the key
          }
        }
        if ((ch > 0) && (ch < 128)) {   // Output ASCII 0 .. 127 range
          putchar(ch);
          fflush(stdout);
        }
      }
    }
    osDelay(10U);
  }
}
