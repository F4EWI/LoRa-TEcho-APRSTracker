Change Log for LoRa TEcho APRS Tracker by F4EWI
===============================================

28.03.2023, dc7os
- modified: Move some settings from config.h to utilsities.h
- modified: Change smart beacon settings for better use in urban transports

27.03.2023, dc7os
- modified: Setup screen shows build date in the line und the version number
21.03.2023, dc7os
- modified: Startup screen

20.03.2023, dc7os
- added: first steps to include the DHT22 sensor (temperature, humidity)
         it crashes ift DHT22 is enabled but not present, i'm searching why.
- modified: shows only inverted call sign if T-Echo transmits data

17.03.2023, dc7os
- modified: Check if BME280 present without endless waiting, only disable BME and continue

15.30.2023, dc7os
- added: suppress sending telemetry data, because there are a lot of invalid frames detected
- added: Voltage and number of sats could added to user comment
        this could set in config.h with set VoltageSat to true
- urgent note: if you get an error in startup to initialize LoRa, you should check the RadioLib used. 
        It must be the lib provided by Xinyuan (https://github.com/Xinyuan-LilyGO/T-Echo) and you have to select the "Nordic nRF52840 DK" board
        
13.03.2023, dc7os
- Modified: Toogle backlight with push button on top side