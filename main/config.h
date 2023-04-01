#ifndef CONIFG_H
#define CONFIG_H

static const int kEncoderSwitchPin = 33;
static const int kEncoderA = 39;
static const int kEncoderB = 32;

static const int kJoystickSwitchPin = 27;
static const int kJoystickXADCChannel = ADC2_CHANNEL_8;
static const int kJoystickYADCChannel = ADC2_CHANNEL_9;

static const double kJoystickDeadZone = 0.7;
static const double kJoystickAngleOffset = 0;

// TODO: figure out the right values here
static const int kReportLen = 66;

#endif