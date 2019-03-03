# Esp32GeigerCounter
This project is an esp32 version of the [opengeiger](https://github.com/opengeiger/LoRaGeiger-MKRWAN1300.git) project.
It supports heltec and ttgo lorawan boards.

## Installation

### Dependencies
To compile the code you will need the arduino ide (which you can get [here](https://www.arduino.cc/en/Main/Software)).
Additionaly you will have to install the esp32 boards definitions for arduino, which can be found on
[github](https://github.com/espressif/arduino-esp32). The readme of that project contains a link and instructions
for installing the board definitions using the arduino board manager.  

### Configuration
All the configuration is done via a set of defines at the top of `Esp32GeigerCounter.ino`. All defines between the
`USER CONFIG` and `CONFIG CHECKS` headings are configuration variables. For a minimal configuration you will have
to uncomment the correct board definition (and comment the default board), set the `DEVEUI` `APPEUI` and `APPKEY`
in lsb and set the correct `GEIGER_PIN` (which is the data pin to which the sensor is attached).

#### linux
If you run into error messages regarding the serial command during compilation installing pyserial using
`sudo pip install pyserial` could fix the problem.

### Data Format
By default the data of 10 measurements over the period of quarter of an hour is agglomorated and then send as a single lora
packet. The packet contains the following values:
```
 - 4 bytes timestamp in network byte order (least significant byte first)
 - NUM_MEASUREMENTS times a variable byte encoded count that was shifted by DISCARDED_BITS to the right
```
A decoder for the packets will be provided soon.
