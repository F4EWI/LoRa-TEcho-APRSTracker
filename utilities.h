#pragma once

String VERSION            = "Version 1.01c";
String BuildDate          = "2023.03.28";       // dc7os 20.03.2023: New, date of build

#include <Arduino.h>

#ifndef _PINNUM
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#endif

// Pinout ePaper Display
#define ePaper_Miso         _PINNUM(1,6)
#define ePaper_Mosi         _PINNUM(0,29)
#define ePaper_Sclk         _PINNUM(0,31)
#define ePaper_Cs           _PINNUM(0,30)
#define ePaper_Dc           _PINNUM(0,28)
#define ePaper_Rst          _PINNUM(0,2)
#define ePaper_Busy         _PINNUM(0,3)
#define ePaper_Backlight    _PINNUM(1,11)

//Pinout Lora Radio Module
#define RADIO_MISO_PIN      _PINNUM(0,23)
#define RADIO_MOSI_PIN      _PINNUM(0,22)
#define RADIO_SCLK_PIN      _PINNUM(0,19)
#define RADIO_CS_PIN        _PINNUM(0,24)
#define RADIO_RST_PIN       _PINNUM(0,25)
#define RADIO_DI0_PIN       _PINNUM(0,22)
#define RADIO_DIO1_PIN      _PINNUM(0,20)
#define RADIO_DIO2_PIN      //_PINNUM(0,3)
#define RADIO_DIO3_PIN      _PINNUM(0,21)
#define RADIO_DIO4_PIN      //_PINNUM(0,3)
#define RADIO_DIO5_PIN      //_PINNUM(0,3)
#define RADIO_BUSY_PIN      _PINNUM(0,17)

// Pinout Flash Chip
#define Flash_Cs            _PINNUM(1,15)
#define Flash_Miso          _PINNUM(1,13)
#define Flash_Mosi          _PINNUM(1,12)
#define Flash_Sclk          _PINNUM(1,14)

#define Touch_Pin           _PINNUM(0,11)
#define Adc_Pin             _PINNUM(0,4)

#define I2C_SDA             _PINNUM(0,26)
#define I2C_SCL             _PINNUM(0,27)

#define RTC_Int_Pin         _PINNUM(0,16)

// Pinout GPS Module
#define GPS_RX_PIN          _PINNUM(1,9)
#define GPS_TX_PIN          _PINNUM(1,8)
#define Gps_Wakeup_Pin      _PINNUM(1,2)
#define Gps_Reset_Pin       _PINNUM(1,5)
#define Gps_pps_Pin         _PINNUM(1,4)




#define UserButton_Pin      _PINNUM(1,10)

#define Power_Enable_Pin    _PINNUM(0,13)
#define Power_On_Pin        _PINNUM(0,12)



#define GreenLed_Pin        _PINNUM(1,1)
#define RedLed_Pin          _PINNUM(1,3)
#define BlueLed_Pin         _PINNUM(0,14)


#define SerialMon           Serial
#define SerialGPS           Serial2


#define MONITOR_SPEED       115200
#define GPS_BAUD_RATE       9600

#define FREQ                433.775

//
// DC7OS, 15.03.2023: Switch some Informations off or on. 
// Telemetry should be off, because there are some wrong data fields.
//
#define Telemetry           false

// Smart Beacon Parameters
// int ANGLE_MIN             = 25;               // minimum Turn Angle in degrees
// int SPEED_MIN             = 10;               // kmh slow speed
// int SPEED_MAX             = 90;               // kmh slow speed
// int INTERVAL_MIN          = 180;              // The interval in seconds for speeds below the min limit
// int INTERVAL_MAX          = 50;               // The interval in seconds for speeds above the max limit
// int INTERVAL_DEFAULT      = 30;               // The interval in seconds for speeds beetwen min and max limit speed

// 28.03.2023, dc7os: modified for city urban transport usage

int ANGLE_MIN             = 15;               // minimum Turn Angle in degrees
int SPEED_MIN             = 10;               // kmh slow speed
int SPEED_MAX             = 45;               // kmh slow speed
int INTERVAL_MIN          = 180;              // The interval in seconds for speeds below the min limit
int INTERVAL_MAX          = 60;               // The interval in seconds for speeds above the max limit
int INTERVAL_DEFAULT      = 20;               // The interval in seconds for speeds beetwen min and max limit speed

int SF                    = 12;               // Spreading Factor 12 -> for LoRa-APRS
int TX_OUTPUT_POWER       = 20;               // Power maxi 20 dBm



