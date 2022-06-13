#include "utilities.h"
#include "images.h"
#include "config.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <AceButton.h>

//Libraries for E-paper Display
#include <GxEPD.h>
#include <GxDEPG0150BN/GxDEPG0150BN.h>  // 1.54" b/w
#include GxEPD_BitmapExamples
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>



TinyGPSPlus MyGPS;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

SPIClass dispPort(NRF_SPIM2, ePaper_Miso, ePaper_Sclk, ePaper_Mosi);
SPIClass rfPort(NRF_SPIM3, RADIO_MISO_PIN, RADIO_SCLK_PIN, RADIO_MOSI_PIN);
SPISettings spiSettings; 
GxIO_Class io(dispPort, ePaper_Cs, ePaper_Dc, ePaper_Rst);
GxEPD_Class display(io, ePaper_Rst, ePaper_Busy);

SX1262 radio = nullptr;
TinyGPSPlus     *gps;

int       counter=0;
int       SB_TIMER     = 0;
int       SB_COURSE    = 0;
int       SPEED;
int       sat;
int       GPShour;
int       GPSminute;
char      Hour[3];
char      Minute[3];
char      Lat[9]       = "";
char      Long[10]     = "";
char      HLat;
char      MLong;
float     current;
int       voltage;
char      UBatt[6]     = "";
char      Tension[25]  = "";
bool      charge       = false;
bool      prot_bat     = false;
bool      Pwr_en       = false;
bool      Pwr_on       = false;

// APRS Variable
String APRSmsg = "";
String LoRaheader = "";
String APRSheader = "";
String APRSheadertlm = "";
String APRStlm0 = "";
String APRStlm1 = "";
String APRStlm2 = "";
String APRStlm3 = "";
String APRStlm4 = "";
int N_tlm = 0;
int D_par = 0;

char TLM[4] = "";
char APRStlmpar1[5] = "";
char APRStlmpar2[5] = "";
char APRStlmpar3[5] = "";
char APRStlmpar6[2] = "";
int N_trame = 0;

uint32_t  last  = 0;
uint32_t  last1 = 0;
uint32_t  last2 = 0;

unsigned long delayTime;
void setupDisplay();
void boardInit();
void StartupScreen();
void enableBacklight(bool en);

void setup() {
  Serial.begin(115200);
  boardInit();
  delay(200);
  StartupScreen();
  //enableBacklight(true);            //light on the screen
  //delay(2000);
  //enableBacklight(false);           // light off the screen
  //display.fillScreen(GxEPD_WHITE);  // Clear the sreen by filling it with white
  //display.setRotation(3);           // ????
  //display.update();                 // Display update

  //while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    if (BME280) {
      status = bme.begin();  
      if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
      }
    }

    
    Serial.println("-- Default Test --");
    delayTime = 1000;

    Serial.println();

    display.fillScreen(GxEPD_WHITE);
    display.fillRect(0, 23, GxEPD_WIDTH, 3, GxEPD_BLACK);
    display.fillRect(59, 0, 3, 23, GxEPD_BLACK);
    display.fillRect(134, 0, 3, 23, GxEPD_BLACK);
    display.fillRect(0, 80, GxEPD_WIDTH, 3, GxEPD_BLACK);
    display.fillRect(0, 115, GxEPD_WIDTH, 3, GxEPD_BLACK);
    display.setCursor(0,108);
    display.setFont(&FreeMonoBold18pt7b);
    display.println(Call);
    display.setFont(&FreeMonoBold9pt7b);

}

void loop() {

    // Manage the light of the screen with the upper button
    if (digitalRead(Touch_Pin) == 0)
      enableBacklight(true); //ON backlight 
    else
      enableBacklight(false);

    // Manage the power on and off with the button
    if ((digitalRead(UserButton_Pin) == LOW )&&(Pwr_on == false))
       {
        // Alimentation périphériques
        digitalWrite(Power_On_Pin,HIGH);
        enableBacklight(true);
        display.fillScreen(GxEPD_WHITE);
        display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
        enableBacklight(false);        
        //digitalWrite(GreenLed_Pin,HIGH);
        //digitalWrite(RedLed_Pin,LOW);
        Pwr_on = true;
        SerialMon.println("PWR ON");
        while (digitalRead(UserButton_Pin) == LOW);
       }
    else if ((digitalRead(UserButton_Pin) == LOW) && (Pwr_on == true))
       {
        SerialMon.println("PWR OFF");
        switchOFF();
        // Arrêt alimentation périphériques 
        digitalWrite(Power_On_Pin,LOW);
        //digitalWrite(GreenLed_Pin,LOW);
        //digitalWrite(RedLed_Pin,HIGH);        
        Pwr_on = false;
        while (digitalRead(UserButton_Pin) == LOW);
       }
       
    // BME280 data reading
    if ((millis() - last1 > BME_READING_INTERVAL*1000) && BME280) {

      Serial.print("Compteur :");
      Serial.println(counter);

      displayBME();
      counter++;
      last1 = millis();
    }
    //printValues();

    loopGPS();

    //delay(delayTime);
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void setupLora()
{
    display.setRotation(3);
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(30,15);
    display.print(VERSION);                 // Display the firmware version
    display.setCursor(0,30);
    rfPort.begin();
    radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN , RADIO_BUSY_PIN, rfPort, spiSettings);
 
   //if (!radio.begin(433.775,125.0,SF,5,0x12,TX_OUTPUT_POWER))
    if (!radio.begin(FREQ,125.0,SF,5,0x12,TX_OUTPUT_POWER))
      display.print("[LoRa] Init. : OK");
    else
      display.print("[LoRa] Init. : KO");

    display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}

bool setupGPS()
{
    display.setCursor(0,45);
    display.print("[GPS] Init. ... ");
    SerialGPS.setPins(GPS_RX_PIN, GPS_TX_PIN);
    display.setCursor(0,60);
    display.print("[GPS] Pins :   OK");
    SerialGPS.begin(GPS_BAUD_RATE);
    SerialGPS.flush();
    display.setCursor(0,75);
    display.print("[GPS] Port :   OK");

    pinMode(Gps_pps_Pin, INPUT);
    pinMode(Gps_Wakeup_Pin, OUTPUT);
    digitalWrite(Gps_Wakeup_Pin, HIGH);

    delay(10);
    pinMode(Gps_Reset_Pin, OUTPUT);
    digitalWrite(Gps_Reset_Pin, HIGH); delay(10);
    digitalWrite(Gps_Reset_Pin, LOW); delay(10);
    digitalWrite(Gps_Reset_Pin, HIGH);

    display.setCursor(0,90);
    display.print("[GPS] Reset :  OK");

    gps = new TinyGPSPlus();
  
    display.setCursor(0,105);
    display.print("[GPS] Start :  OK");

    display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

    return true;
}

void loopGPS()
{
      byte bytetab[] = {0x3C, 0xFF, 0x01};
      char bytechar[3];
      bytechar[0] = char(bytetab[0]);
      bytechar[1] = char(bytetab[1]);
      bytechar[2] = char(bytetab[2]);
      LoRaheader = String(bytechar); 
      APRSheader = LoRaheader + Call + String(">") + "APLT00-1" + String(",") + "WIDE1-1" + String(":");
      APRSheadertlm = LoRaheader + Call + String(">") + "APMI04" + String(",") + "WIDE1-1" + String(":");
      
      while (SerialGPS.available() > 0)
        gps->encode(SerialGPS.read());

    if (millis() - last > GPS_READING_INTERVAL*1000) {
        //display.fillScreen(GxEPD_WHITE);
        display.fillRect(0, 0, GxEPD_WIDTH, 22, GxEPD_WHITE);
        //display.fillRect(0, 23, GxEPD_WIDTH, 3, GxEPD_BLACK);
        display.fillRect(59, 0, 3, 23, GxEPD_BLACK);
        display.fillRect(134, 0, 3, 23, GxEPD_BLACK);

        // Determination of voltage
        voltage = analogRead(Adc_Pin);
        float volt_tmp = float(voltage)/1000;
        volt_tmp = volt_tmp * 7.0125;           // coefficent à appliquer depuis lecture ADC ( Ubat /2 /3.3V /1.0625)

        if (volt_tmp > 4.6)
          charge = true;
        else
          charge = false;

        if (gps->location.isUpdated()) {
            /*SerialMon.print(F("Lat="));
            SerialMon.print(gps->location.lat(), 6);
            SerialMon.print(F("Long="));
            SerialMon.println(gps->location.lng(), 6);
            SerialMon.print(gps->time.hour());
            SerialMon.print(F(":"));
            SerialMon.println(gps->time.minute());
            SerialMon.print(F(" Meters="));
            SerialMon.println(gps->altitude.meters());*/
            
            sat       = gps->satellites.value();
            GPShour   = gps->time.hour();
            GPSminute = gps->time.minute();
            SPEED     = gps->speed.kmph();
            SB_COURSE = gps->course.deg();
            
            sprintf(Hour,"%02d",GPShour + TIME_ZONE);
            sprintf(Minute,"%02d",GPSminute);    

            if (SPEED <= SPEED_MIN) {
              SB_TIMER = INTERVAL_MIN;
            }
            else if (SPEED > SPEED_MIN && SPEED < SPEED_MIN) {
              SB_TIMER = INTERVAL_DEFAULT;
            }
            else {
              SB_TIMER = INTERVAL_MAX;
            }

 
 /*           SerialMon.print(F("SB_COURSE="));
            SerialMon.println(SB_TIMER);
*/

            display.setFont(&FreeMonoBold9pt7b);
            display.setCursor(0,15);
            display.print(Hour);
            display.print(":");
            display.print(Minute);
            display.setCursor(65,15);
            display.print("Sat:");
            display.print(sat);

            float latitude_mdeg = gps->location.lat();
            float longitude_mdeg = gps->location.lng();

            display.fillRect(0, 26, GxEPD_WIDTH, 53, GxEPD_WHITE);
            display.setFont(&FreeMonoBold12pt7b);
            display.setCursor(0,45);
            display.print("Lat=");
            display.print(latitude_mdeg, 6);
            display.setCursor(0,70);
            display.print("Long=");
            display.print(longitude_mdeg, 6);

            // Latitude conversion
            int lat_tmp1 = latitude_mdeg;
            float lat_tmp2 = abs(latitude_mdeg) - abs(lat_tmp1);
            float lat_tmp3 = lat_tmp2 *60;
            char lat_tmp4[6] = "";
            snprintf(lat_tmp4, sizeof(lat_tmp4),"%.2f",lat_tmp3);
        
            if (latitude_mdeg < 0)
              HLat = 'S';
            else
              HLat = 'N';        

            snprintf(Lat, sizeof(Lat), "%02d%05s%c", abs(lat_tmp1),lat_tmp4,HLat);

            // Longitude conversion
            int long_tmp1 = longitude_mdeg;
            float long_tmp2 = abs(longitude_mdeg) - abs(long_tmp1);
            float long_tmp3 = float(long_tmp2) * 60;
            char long_tmp4[6] = "";
            snprintf(long_tmp4, sizeof(long_tmp4),"%.2f",long_tmp3);        
        
            if (longitude_mdeg < 0)
              MLong = 'W';
            else
              MLong = 'E';
              
            snprintf(Long, sizeof(Long), "%03d%05s%c", abs(long_tmp1),long_tmp4,MLong);

            /* SerialMon.print("Lat : ");
            SerialMon.println(Lat);
            SerialMon.print("Long : ");
            SerialMon.println(Long);*/   

            APRSmsg = APRSheader;

            APRSmsg += "!" + String(Lat) + "/" + String(Long) + BEACON_SYMBOL; 
//            APRSmsg += "LoRa Test Tracker by F4AVI & F4EWI";
            APRSmsg += APRS_MESSAGE;
            int stlong = APRSmsg.length();
 
            APRStlm0 = APRSheadertlm;
            APRStlm0 += ":" + Call + "  :PARM.UBat,IBat,Sat,-,-,Batt.Charge,B2,B3,B4,B5,B6,B7,B8";
            int stlong0 = APRStlm0.length(); 
 
            APRStlm1 = APRSheadertlm;
            APRStlm1 += ":" + Call + "  :UNIT.V,mA,-,-,-,O/N,O/N,O/N,O/N,O/N,O/N,O/N,O/N";
            int stlong1 = APRStlm1.length();

            APRStlm2 = APRSheadertlm;
            APRStlm2 += ":" + Call + "  :EQNS.0,0.005,0,0,1,0,0,1,0,0,1,0,0,1,0";
            int stlong2 = APRStlm2.length();

            APRStlm3 = APRSheadertlm;
            APRStlm3 += ":" + Call + "  :BITS.11111111,LoRa Tracker Telemetry";
            int stlong3 = APRStlm3.length();

            APRStlm4 = APRSheadertlm;

            // Trame number
            sprintf(TLM,"%03d",N_tlm);
            
            // Battery voltage
            int tlmpar1 = int(voltage * 1.4025);  // 7.0125 / 5  
            sprintf(APRStlmpar1,"%03d",tlmpar1);
        
            // Battery courant
            int tlmpar2 = int(current * 2);
            sprintf(APRStlmpar2,"%03d",tlmpar2);
            
            // Number of satellites available
            sprintf(APRStlmpar3,"%03d",sat);

            // Battery status, charging or discharging?
            if (charge)
              APRStlmpar6[1] = '1';
            else
              APRStlmpar6[1] = '0';

            APRStlm4 += "T#" + String(TLM) + "," + String(APRStlmpar1) + "," + String(APRStlmpar2) + "," + String(APRStlmpar3) + "," + "0" + "," + "0" + "," + String(APRStlmpar6[1]);
            int stlong4 = APRStlm4.length();  

            SerialMon.print("millis() : ");
            SerialMon.println(millis());   
            SerialMon.println(last2);
            SerialMon.print("millis() - last2 : ");
            SerialMon.println(millis() - last2);     

            // Envoi données via LoRa selon la valeur du Timer.
            if (millis() - last2 > SB_TIMER * 1000){

              SerialMon.print("D_par : ");
              SerialMon.println(D_par);   
 
              //Send LoRa packet to receiver

              if (D_par != N_tlm-10)
                {
                // transmission parametres = false
                if ( N_trame > 0 && N_trame < 5 )
                  {
                  N_trame = 5;
                  }
                }
              else 
                {
                //transmission parametres = true
                if (N_trame == 4)
                  {
                  D_par = N_tlm;
                  } 
                }
            SerialMon.print("N_trame : ");
            SerialMon.println(N_trame);  
              switch (N_trame){
                  case 0:
                      //APRS Data
                      radio.startTransmit(APRSmsg,stlong);
                      Serial.println(APRSmsg);
                      N_trame++;
                      break;
                  case 1:
                      //APRS Telemetry
                      radio.startTransmit(APRStlm0,stlong0);
                      Serial.println(APRStlm0);
                      N_trame++; 
                      break;
                  case 2:
                      //APRS Telemetry
                      radio.startTransmit(APRStlm1,stlong1);
                      Serial.println(APRStlm1);
                      N_trame++;
                      break;                                    
                  case 3:
                      //APRS Telemetry
                      radio.startTransmit(APRStlm2,stlong2);
                      Serial.println(APRStlm2);
                      N_trame++;
                      break;
                  case 4:
                      //APRS Telemetry
                      radio.startTransmit(APRStlm3,stlong3);
                      Serial.println(APRStlm3);
                      N_trame++;
                      break;
                  case 5:
                      //APRS Telemetry
                      radio.startTransmit(APRStlm4,stlong4);
                      Serial.println(APRStlm4);
                      N_tlm++;
                      N_trame++;
                  break;                  
              }

              display.fillRect(0, 83, GxEPD_WIDTH, 32, GxEPD_BLACK);
              //display.updateWindow(0, 83, GxEPD_WIDTH, GxEPD_HEIGHT, false); 
              display.setCursor(15,108);
              display.setFont(&FreeMonoBold18pt7b);
              display.setTextColor(GxEPD_WHITE);
              display.println("<< TX >>");
              digitalWrite(RedLed_Pin, LOW);
              display.setFont(&FreeMonoBold9pt7b);
              display.setTextColor(GxEPD_BLACK);
              display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false); 
              delay(400);
              digitalWrite(RedLed_Pin, HIGH);  

              /*display.setCursor(12,170);
              display.setFont(&FreeMonoBold18pt7b);
              display.println("<< TX >>");*/
              //Serial.println("<< TX >>");
           
              //LoRa.endPacket();
              if (N_trame > 5)
                N_trame = 0;
          
              if (N_tlm > 999)
                N_tlm = 0;

              last2 = millis();
            }
          else
            {
            //display.setCursor(0,192);
            //Serial.print("TX dans : ");
            //Serial.println(60-((millis() - last2)/1000));
            //display.print("TX dans : ");
            //display.print(60-((millis() - last2)/1000));
            //display.println("s");
            }


        }     
         else {
            display.fillRect(0, 26, GxEPD_WIDTH, 53, GxEPD_WHITE);
            display.setCursor(15,65);
            display.setFont(&FreeMonoBold24pt7b);
            display.println("No Fix");
            display.setFont(&FreeMonoBold9pt7b);
            display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
         }
         


        sprintf (Tension,"%.2fV",volt_tmp);       // Create a string of characters for displaying to the screen     
        display.setFont(&FreeMonoBold9pt7b);
        display.setCursor(140,15);
        display.print(Tension);

        //display.fillRect(0, 80, GxEPD_WIDTH, 3, GxEPD_BLACK);
        display.fillRect(0, 83, GxEPD_WIDTH, 32, GxEPD_WHITE);
        //display.updateWindow(0, 83, GxEPD_WIDTH, GxEPD_HEIGHT, false); 
        display.setCursor(0,108);
        display.setFont(&FreeMonoBold18pt7b);
        display.println(Call);
        display.setFont(&FreeMonoBold9pt7b);    
        //display.fillRect(0, 115, GxEPD_WIDTH, 3, GxEPD_BLACK);

        // Display info from GPS
        digitalWrite(BlueLed_Pin, LOW);
        displayGPS();
        digitalWrite(BlueLed_Pin, HIGH);     
        
        display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

    last = millis();
    }
}

// Display the information coming from the BME
void displayGPS() {

        display.fillRect(0, 119, 28, 82, GxEPD_BLACK);
        display.fillRect(29, 119, 171, 82, GxEPD_WHITE);
        display.setTextColor(GxEPD_WHITE);
        display.setFont(&FreeMonoBold18pt7b);
        display.setCursor(3,140);
        display.println("G");
        display.setCursor(3,165);
        display.println("P");
        display.setCursor(3,190);
        display.println("S");
        display.setTextColor(GxEPD_BLACK);        
        display.setFont(&FreeMonoBold9pt7b); 
        display.setCursor(35,133);
        display.print("Alt.:");
        display.print(gps->altitude.meters());
        display.print("m");
        display.setCursor(35,153);
        display.print("Vit.:");
        display.print(gps->speed.kmph());
        display.print("km/h");
        display.setCursor(35,173);
        display.print("Dir.:");
        display.print(gps->course.deg());
        display.print("deg");      
        display.setCursor(35,193);
        display.print("Next TX in:");
        display.print(SB_TIMER - ((millis() - last2)/1000));
        display.print("s");             
        display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);        
}

// Display the information coming from the BME
void displayBME() {

        display.fillRect(0, 119, 28, 82, GxEPD_BLACK);
        display.fillRect(29, 119, 171, 82, GxEPD_WHITE);
        display.setTextColor(GxEPD_WHITE);
        display.setFont(&FreeMonoBold18pt7b);
        display.setCursor(3,140);
        display.println("B");
        display.setCursor(3,165);
        display.println("M");
        display.setCursor(3,190);
        display.println("E");
        display.setTextColor(GxEPD_BLACK);        
        display.setFont(&FreeMonoBold9pt7b); 
        display.setCursor(35,133);
        display.print("Alt.:");
        display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        display.print("m");
        display.setCursor(35,153);
        display.print("Temp.:");
        display.print(bme.readTemperature());
        display.print("°C");
        display.setCursor(35,173);
        display.print("Hum.:");
        display.print(bme.readHumidity());
        display.print("%");           
        display.setCursor(35,193);
        display.print("Pres.:");
        display.print(bme.readPressure() / 100.0F);
        display.print("hPa");
        display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);        
}

void enableBacklight(bool en)
{
    digitalWrite(ePaper_Backlight, en);
}

void setupDisplay()
{
    dispPort.begin();
    display.init(/*115200*/); // enable diagnostic output on Serial
    display.setRotation(0);
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold9pt7b);
}

void boardInit()
{
    uint8_t rlst = 0;

    SerialMon.begin(MONITOR_SPEED);
    SerialMon.println("Start\n");

    uint32_t reset_reason;
    sd_power_reset_reason_get(&reset_reason);
    SerialMon.print("sd_power_reset_reason_get:");
    SerialMon.println(reset_reason, HEX);

    pinMode(Power_Enable_Pin, OUTPUT);
    digitalWrite(Power_Enable_Pin, HIGH);
    Pwr_en = true;

    pinMode(Power_On_Pin, OUTPUT);
    digitalWrite(Power_On_Pin, HIGH);
    Pwr_on = true;

    pinMode(ePaper_Backlight, OUTPUT);

    pinMode(GreenLed_Pin, OUTPUT);
    pinMode(RedLed_Pin, OUTPUT);
    pinMode(BlueLed_Pin, OUTPUT);

    pinMode(UserButton_Pin, INPUT_PULLUP);
    pinMode(Touch_Pin, INPUT_PULLUP);

    setupDisplay();
    setupLora();
    setupGPS();

    if (BME280) {
      display.setCursor(0,125);
      display.print("[BME] active : Yes");
    }
    else {
      display.setCursor(0,125);
      display.print("[BME] active : No");      
    }

    display.update();
    delay(5000);
}

void StartupScreen(void)
{
    display.setRotation(2);
    display.fillScreen(GxEPD_WHITE);
    display.drawExampleBitmap(BitmapCallSign, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    display.update();                 // Display update
    enableBacklight(true);            //light on the screen
    delay(2000);
    enableBacklight(false);           // light off the screen
    display.fillScreen(GxEPD_WHITE);  // Clear the sreen by filling it with white
    display.setRotation(3);           // ????
    display.update();                 // Display update
}

void switchOFF(void)
{
    display.setRotation(0);
    display.fillScreen(GxEPD_WHITE);
    display.drawExampleBitmap(T_Echo_OFF, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    display.setRotation(3);
    display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}
