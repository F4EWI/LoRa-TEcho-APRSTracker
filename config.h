// Configuration

// BME setting
bool BME280               = false;             // false if no BME280 option
// int BME_READING_INTERVAL  = 20;               // BME280 data reading interval in seconde
int SENSOR_READING_INTERVALL  = 20;

//dc7os, 21.03.2023: fopr DHT22 otion
bool DHT22_Used           = false;

// GPS setting
int GPS_READING_INTERVAL  = 5;               // GPS data reading interval in seconde

// General setting
int TIME_ZONE             = 0;                // Time zone management : 0 -> UTC
int NUMBER_TRAMES         = 10;               // Number of trames with position before sending telemetry.
int Language = 2;                             // 0 -> French
                                              // 1 -> English
                                              // 2 -> German

// APRS
String APRS_MESSAGE = "Karsten testet den T-Echo";

// User setting
String Call = "DC7OS-12";                      // Call Sign of the operator
String BEACON_SYMBOL = "Q";                   // Caractère définissant le symbôle affiché sur aprs.fi 
                                              // "[" -> Runner
                                              // "<" -> Motocycle
                                              // ">" -> Car

// Receiver functions.
bool RECEIVER             = false;            // true -> activate the receiver

//
// dc7os, 15.03.2023: Expand UserComment
// With VoltageSat you could add the voltage and the Number auf Satelites used to your APRS Comment
//
#define VoltageSat          true
