RDZ_TTGO_SONDE
==============

This a simple, experimental, not (well) tested, and incomplete decoder for
radiosonde RS41 and DFM06/09 on a TTGO LoRa ESP32 with OLED display board.

There have been made some additions for TTGO LoRa ESP32 with only RST button.
Please check also your OLED port settings, both versions use different ports.
You can setup the depending ports in config.txt, OLED Setup is depending on hardware of LoRa board
-  TTGO v1:  SDA=4  SCL=15, RST=16 
-  TTGO v2:  SDA=21 SCL=22, RST=16

## Button commands
You can use the button on the board (not the reset button, the second one) to
issue some commands. The software distinguishes between several inputs:

- SHORT	Short button press (<1.5 seconds)
- DOUBLE  Short button press, followed by another button press within 0.5 seconds
- MID	Medium-length button press (2-4 seconds)
- LONG	Long button press (>5 seconds)

## Wireless configuration

On startup, as well as after a LONG button press, the WiFI configuration will
be started.  The board will scan available WiFi networks, if the scan results
contains a WiFi network configured with ID and Password in networks.txt, it
will connect to that network in station mode. If no known network is found, or
the connection does not suceed after 5 seconds, it instead starts in access point
mode. In both cases, the ESP32's IP address will be shown in tiny letters in the
bottom line. Then the board will switch to scanning mode.

## Scanning mode

In the scanning mode, the board will iterate over all channels configured in
channels.txt, trying to decode a radio sonde on each channel for about 1 second
for RS41, a bit less for DMF06/09. If a valid signal is found, the board switches
to receiving mode on that channel.  a SHORT buttong press will also switch to
receiving mode.

## Receiving mode

In receiving mode, a single frequency will be decoded, and sonde info (ID, GPS
coordinates, RSSI) will be displayed. The bar above the IP address indicates,
for the last 18 frames, if reception was successfull (|) or failed (.) 
A DOUBLE press will switch to scanning mode.
A SHORT press will switch to the next channel in channels.txt

## Spectrum mode

A medium press will active scan the whole band (400..406 MHz) and display a
spectrum diagram (each line == 50 kHz)
For TTGO boards without configurable button there are some new parameter in config.txt:
- spectrum=10       // 0=off / 1-99 number of seconds to show spectrum after restart
- timer=1           // 0=off / 1= show spectrum countdown timer in spectrum display
- marker=1          // 0=off / 1= show channel edge freq in spectrum display

## Setup

see Setup.md

