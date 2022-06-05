// Configuration

// BME setting
String VERSION            = "Version 1.01";
bool BME280               = true;             // false if no BME280 option
int BME_READING_INTERVAL  = 20;               // BME280 data reading interval in seconde

// GPS setting
int GPS_READING_INTERVAL  = 10;               // GPS data reading interval in seconde

// General setting
int TIME_ZONE             = 2;                // Time zone management : 0 -> UTC 

// APRS
String APRS_MESSAGE = "LoRa Test Tracker by F4AVI / F4EWI";

// Smart Beacon Parameters
int ANGLE_MIN             = 25;               // minimum Turn Angle in degrees
int SPEED_MIN             = 10;               // kmh slow speed
int SPEED_MAX             = 90;               // kmh slow speed
int INTERVAL_MIN          = 120;              // The interval in seconds for speeds below the min limit
int INTERVAL_MAX          = 50;               // The interval in seconds for speeds above the max limit
int INTERVAL_DEFAULT      = 30;               // The interval in seconds for speeds beetwen min and max limit speed

int SF                    = 12;               // Spreading Factor 12 -> for LoRa-APRS
int TX_OUTPUT_POWER       = 22;               // Power maxi 20 dBm

// User setting
String Call = "F4EWI-7";                      // Call Sign of the operator
String BEACON_SYMBOL = ">";                   // Caractère définissant le symbôle affiché sur aprs.fi 
                                              // "[" -> Runner
                                              // "<" -> Motocycle
                                              // ">" -> Car
int Language = 0;                             // 0 -> French
                                              // 1 -> English
                                              // 2 -> German
