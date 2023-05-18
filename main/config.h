#ifndef CONIFG_H
#define CONFIG_H

static const int kEncoderSwitchPin = GPIO_NUM_33;
static const int kEncoderA = GPIO_NUM_39;
static const int kEncoderB = GPIO_NUM_32;

static const int kJoystickSwitchPin = GPIO_NUM_27;
static const int kJoystickXADCChannel = ADC2_CHANNEL_8; // GPIO25
static const int kJoystickYADCChannel = ADC2_CHANNEL_9; // GPIO26

static const double kJoystickDeadZone = 0.9;
static const double kJoystickAngleOffset = 0;

static const int knKeypadCols = 4;
static const int knKeypadRows = 4;
const int kKeypadCols[] = {GPIO_NUM_38, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_17};
const int kKeypadRows[] = {GPIO_NUM_2, GPIO_NUM_15, GPIO_NUM_13, GPIO_NUM_12};

// SPI Connections to ST7789V LCD
const int kMOSIPin = 19;
const int kSCLKPin = 18;
const int kDCPin = 16;
const int kRSTPin = 23;
const int kCSPin = 5;
const int kBLPin = 4;

const int kLCDPixelClockHz = (20 * 1000 * 1000);

// Bit number used to represent command and parameter
const int kLCDCmdBits = 8;
const int kLCDParamBits = 8;

const int kDisplayWidth = 135;
const int kDisplayHeight = 240;

#endif