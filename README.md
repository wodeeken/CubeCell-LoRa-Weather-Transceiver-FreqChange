# CubeCell LoRa Weather Transceiver (w/ Region/Frequency change)
A VSCode / Platform IO project for the HelTec CubeCell HTCC-AB01 LoRa board wired to an Adafruit BMP390 sensor that periodically sends air pressure and temperature readings to a LoRaWAN network (The Things Network / Helium) in CayenneLPP format. The sketch can also receive downlinks to trigger the board to reset and toggle the operating frequency between North America 915MHz  and Europe 868MHz. This program contains a frequency change failsafe, where after a certain amount of data transfers and attemped joins the operating frequency automatically changes to EU868MHz. For a list of components and a wiring diagram, visit the Documentation folder.

# Notes
1. This project is intended to be used in picoballoon applications that are launched from North America and are intended to reach Europe. This program must use the modified cubecell_board library located at repo https://github.com/wodeeken/CubeCell-Library-LoraWAN-Region-Change.
# Usage
1. This program was created using the PlatformIO extension for VSCode. You will need to add the libraries for HTCC-AB01 (the modified one above), Adafruit BMP390, and CayenneLPP.
2. The user must configure OTAA (Dev/App EUI and key) or ABP (nwk/app key and addr) params located at main.cpp:11-17 before building and uploading the program.
3. When the LoRa board first boots up, the board will send a pressure/temperature reading every 5 minutes. After an hour, the board will only send once every 10 mins. The board sleeps in between readings.
4. If the air pressure increases by 4 hPa or more since the previous reading, the board will only sleep for 5 minutes. If the increase is 7 hPa or more the board will sleep for only 1 minute.
5. If the sum of total join attempts and successful sends reach 600, and the current frequency is NA915MHz, the frequency is changed to EU868MHz and the board is reset.
6. When the board successfully joins the currently configured network (NA915 or EU868) for the first time, this is written to EEPROM.
7. If the board resets and the current network has never been joined before, the wait time in between join attempts is 3 hours (in order to preserve power if board is over the ocean).
8. If the board resets and the current network has been joined before, the wait time in between join attempts is 5 minutes.
9. If the board receives a downlink payload of 0x52 ("R"), the board will reset.
10. If the board receives a downlink payload of 0x4E ("N"), the board frequency will be changed to NA915 after the next send. The board will be reset.
11. If the board receives a downlink payload of 0x45 ("E"), the board frequency will be changed to EU868 after the next send. The board will be reset.
12. If the board receives a downlink payload of 0x43 ("C"), the board EEPROM (join/send counters, current configured region (defaulted to NA915)) will be cleared to zeros. The board will be reset.
13. If the board receives a downlink payload of 0x4F ("O"), the frequency change failsafe will be overridden (frequency will not automatically change after 600 join attempts/sends).
14. If the board receives a downlink payload of 0x46 ("F"), the frequency change failsage override will be cancelled.
