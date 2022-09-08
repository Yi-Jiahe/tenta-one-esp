#ifndef REPORT_MAPS_H
#define REPORT_MAPS_H

#include "config.h"

// HID Report IDs for the service
#define HID_RPT_ID_MOUSE_IN 1   // Mouse input report ID
#define HID_RPT_ID_KEY_IN 2     // Keyboard input report ID
#define HID_RPT_ID_CC_IN 3      // Consumer Control input report ID
#define HID_RPT_ID_VENDOR_OUT 4 // Vendor output report ID
#define HID_RPT_ID_JOY_IN 5     // Vendor output report ID
#define HID_RPT_ID_LED_OUT 0    // LED output report ID
#define HID_RPT_ID_FEATURE 0    // Feature report ID

/* HID Report type */
#define HID_REPORT_TYPE_INPUT 1
#define HID_REPORT_TYPE_OUTPUT 2
#define HID_REPORT_TYPE_FEATURE 3

#define HID_KEYBOARD_IN_RPT_LEN 66

const unsigned char hidapiReportMap[] = {
    // 8 bytes input, 8 bytes feature
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x0A, 0x00, 0x01, // Usage (0x0100)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x08,       //   Report Count (8)
    0x09, 0x01,       //   Usage (0x01)
    0x82, 0x02, 0x01, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Buffered Bytes)
    0x95, 0x08,       //   Report Count (8)
    0x09, 0x02,       //   Usage (0x02)
    0xB2, 0x02, 0x01, //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile,Buffered Bytes)
    0x95, 0x08,       //   Report Count (8)
    0x09, 0x03,       //   Usage (0x03)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,             // End Collection

    // 38 bytes
};

const unsigned char mediaReportMap[] = {
    0x05, 0x0C, // Usage Page (Consumer)
    0x09, 0x01, // Usage (Consumer Control)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x03, //   Report ID (3)
    0x09, 0x02, //   Usage (Numeric Key Pad)
    0xA1, 0x02, //   Collection (Logical)
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (0x01)
    0x29, 0x0A, //     Usage Maximum (0x0A)
    0x15, 0x01, //     Logical Minimum (1)
    0x25, 0x0A, //     Logical Maximum (10)
    0x75, 0x04, //     Report Size (4)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x00, //     Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,       //   End Collection
    0x05, 0x0C, //   Usage Page (Consumer)
    0x09, 0x86, //   Usage (Channel)
    0x15, 0xFF, //   Logical Minimum (-1)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x02, //   Report Size (2)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x46, //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,Null State)
    0x09, 0xE9, //   Usage (Volume Increment)
    0x09, 0xEA, //   Usage (Volume Decrement)
    0x15, 0x00, //   Logical Minimum (0)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x02, //   Report Count (2)
    0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0xE2, //   Usage (Mute)
    0x09, 0x30, //   Usage (Power)
    0x09, 0x83, //   Usage (Recall Last)
    0x09, 0x81, //   Usage (Assign Selection)
    0x09, 0xB0, //   Usage (Play)
    0x09, 0xB1, //   Usage (Pause)
    0x09, 0xB2, //   Usage (Record)
    0x09, 0xB3, //   Usage (Fast Forward)
    0x09, 0xB4, //   Usage (Rewind)
    0x09, 0xB5, //   Usage (Scan Next Track)
    0x09, 0xB6, //   Usage (Scan Previous Track)
    0x09, 0xB7, //   Usage (Stop)
    0x15, 0x01, //   Logical Minimum (1)
    0x25, 0x0C, //   Logical Maximum (12)
    0x75, 0x04, //   Report Size (4)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x00, //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x80, //   Usage (Selection)
    0xA1, 0x02, //   Collection (Logical)
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (0x01)
    0x29, 0x03, //     Usage Maximum (0x03)
    0x15, 0x01, //     Logical Minimum (1)
    0x25, 0x03, //     Logical Maximum (3)
    0x75, 0x02, //     Report Size (2)
    0x81, 0x00, //     Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,       //   End Collection
    0x81, 0x03, //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,       // End Collection
};

const unsigned char mouseReportMap[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)

    0x09, 0x01, //   USAGE (Pointer)
    0xa1, 0x00, //   COLLECTION (Physical)

    0x05, 0x09, //     USAGE_PAGE (Button)
    0x19, 0x01, //     USAGE_MINIMUM (Button 1)
    0x29, 0x03, //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0x01, //     LOGICAL_MAXIMUM (1)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x75, 0x01, //     REPORT_SIZE (1)
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x95, 0x01, //     REPORT_COUNT (1)
    0x75, 0x05, //     REPORT_SIZE (5)
    0x81, 0x03, //     INPUT (Cnst,Var,Abs)

    0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    0x09, 0x38, //     USAGE (Wheel)
    0x15, 0x81, //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f, //     LOGICAL_MAXIMUM (127)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x81, 0x06, //     INPUT (Data,Var,Rel)

    0xc0, //   END_COLLECTION
    0xc0  // END_COLLECTION
};

const unsigned char keyboardReportMap[] = {
    0x05, 0x01, // Usage Pg (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection: (Application)
    0x85, 0x02, // Report Id (2)
    //
    0x05, 0x07, //   Usage Pg (Key Codes)
    0x19, 0xE0, //   Usage Min (224)
    0x29, 0xE7, //   Usage Max (231)
    0x15, 0x00, //   Log Min (0)
    0x25, 0x01, //   Log Max (1)
    //
    //   Modifier byte
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input: (Data, Variable, Absolute)
    //
    //   Reserved byte
    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x01, //   Input: (Constant)
    //
    //   LED report
    0x95, 0x05, //   Report Count (5)
    0x75, 0x01, //   Report Size (1)
    0x05, 0x08, //   Usage Pg (LEDs)
    0x19, 0x01, //   Usage Min (1)
    0x29, 0x05, //   Usage Max (5)
    0x91, 0x02, //   Output: (Data, Variable, Absolute)
    //
    //   LED report padding
    0x95, 0x01, //   Report Count (1)
    0x75, 0x03, //   Report Size (3)
    0x91, 0x01, //   Output: (Constant)
    //
    //   Key arrays (6 bytes)
    0x95, REPORT_COUNT_BYTES, //   Report Count (6)
    0x75, 0x08,               //   Report Size (8)
    0x15, 0x00,               //   Log Min (0)
    0x25, 0x65,               //   Log Max (101)
    0x05, 0x07,               //   Usage Pg (Key Codes)
    0x19, 0x00,               //   Usage Min (0)
    0x29, 0x65,               //   Usage Max (101)
    0x81, 0x00,               //   Input: (Data, Array)
    //
    0xC0, // End Collection
};

#endif