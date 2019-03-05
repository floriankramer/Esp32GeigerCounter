# Esp32GeigerCounter
This project is an esp32 version of the
[opengeiger](https://github.com/opengeiger/LoRaGeiger-MKRWAN1300.git) project.
It supports heltec and ttgo lorawan boards.

## Installation

### Dependencies
To compile the code you will need the arduino ide (which you can get
[here](https://www.arduino.cc/en/Main/Software)).
Additionaly you will have to install the esp32 boards definitions for arduino,
which can be found on [github](https://github.com/espressif/arduino-esp32).
The readme of that project contains a link and instructions for installing the
board definitions using the arduino board manager.  

### Configuration
All the configuration is done via a set of defines at the top of
`Esp32GeigerCounter.ino`. All defines between the `USER CONFIG` and
`CONFIG CHECKS` headings are configuration variables. For a minimal
configuration you will have to uncomment the correct board definition (and
comment the default board), set the `DEVEUI` `APPEUI` and `APPKEY`
in lsb and set the correct `GEIGER_PIN` (which is the data pin to which the
sensor is attached).

#### linux
If you run into error messages regarding the serial command during compilation
installing pyserial using `sudo pip install pyserial` could fix the problem.

### Data Format
By default the data of 10 measurements over the period of quarter of an hour is
agglomorated and then send as a single lora packet. The packet contains the
following values:
```
 - 2 bytes timestamp in seconds encoded in network byte order (least significant
   byte first)
 - NUM_MEASUREMENTS times a variable byte encoded count that was shifted by
   DISCARDED_BITS to the right
```
The variable byte encoding uses the most significant bit of every byte to
indicate if the byte is the last byte of the integer (with 0 indicating that
it is not the last byte and 1 indicating that it is the last byte).

#### Decoder
A decoder for the ttn console can be found in decoder.js. Simply copy the
contents of that file into the field for the decoder function in the consoles
web interface.

## Hardware
The code should work with any form of sensor that sends signals by pulling a line to ground, but it was designed to work with the open source [PiGI](https://github.com/apollo-ng/PiGI) platform, and was only tested together with that.

### PiGI + TTGo v2.1
Pins to connect:  

| PiGI  | TTGo    |
| ----- | ------- |
| 1     | 3.3V    |
| 2     | 5V      |
| 7     | GPIO 0  |
| 25    | GnD     |
