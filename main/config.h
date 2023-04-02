#ifndef CONIFG_H
#define CONFIG_H

static const int kEncoderSwitchPin = 33;
static const int kEncoderA = 39;
static const int kEncoderB = 32;

static const int kJoystickSwitchPin = 27;
static const int kJoystickXADCChannel = ADC2_CHANNEL_8;
static const int kJoystickYADCChannel = ADC2_CHANNEL_9;

static const double kJoystickDeadZone = 0.9;
static const double kJoystickAngleOffset = 0;

#endif