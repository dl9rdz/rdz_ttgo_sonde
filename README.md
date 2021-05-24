rdzTTGOsonde
============

This a decoder for radiosonde RS41, RS92, DFM06/09/17, M10/M20, and MP3H
based on a TTGO LoRa ESP32 board.

It supports OLED displays (SSD1306, SH1106) and TFT displays (ILI9225).

It also supports feeding data to external applications using WiFi (NOT bluetooth):
- Arduino app by dl9rdz (see https://github.com/dl9rdz/rdzwx-go for apk download)
- AXUDP (for aprsmap application by oe5dxl, among others)
- KISS TNC (aprs format, mainly useful for APRSdroid app)
- MQTT
- SondeHub tracker (experimental)


Please consult the Wiki at https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Supported-boards
for details on supported boards, and additional setup instructions.


### Radiosonde Support Matrix

Manufacturer | Model | Position | Temperature | Humidity | Pressure
-------------|-------|----------|-------------|----------|----------
Vaisala | RS92-SGP/NGP | :heavy_check_mark: | :heavy_check_mark: | :x: | :x:
Vaisala | RS41-SG/SGP/SGM | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :x:
Graw | DFM06/09/17 | :heavy_check_mark: | :x: | :x: | :x:
Meteomodem | M10 | :heavy_check_mark: | :x: | :x: | Not Sent
Meteomodem | M20 | :heavy_check_mark: | :x: | :x: | Not Sent
Meteo-Radiy | MP3-H1 (MRZ-H1) | :heavy_check_mark: | :x: | :x: | :x: 

SondeHub integration has mainly been tested with RS41 and DFM. MP3-H1 currently lacks decoding
timestamps from telemetry, thus not yet suitable for SondeHub.


Support for other radiosondes that use AFSK modulation is not feasible with the TTGO hardware.
In particular, decoding iMet radiosondes is not practical.

Adding support for LMS6 (see issue #48) and ims100 (see branch ims100) could be feasible,
but currently I don't have plans to do add this myself. Well-tested pull requests will of
course be considered for inclusion :-).

## Button commands

You can use the button on the board (not the reset button, the second one) to
issue some commands. The software distinguishes between several inputs:

- SHORT	Short button press (<1.5 seconds)
- DOUBLE  Short button press, followed by another button press within 0.5 seconds
- MID	Medium-length button press (2-4 seconds)
- LONG	Long button press (>5 seconds)

You can optionally use a second button, which you have to add manually to your board.
See https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Hardware-configuration for details.


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
channels.txt, trying to decode a radio sonde on each channel for about 1 second.
If a valid signal is found, the board switches to receiving mode on that channel.
A SHORT buttong press will also switch to receiving mode.

## Receiving mode

In receiving mode, a single frequency will be decoded, and sonde info (ID, GPS
coordinates, RSSI) will be displayed. The bar above the IP address indicates,
for the last 18 frames, if reception was successfull (|) or failed (.), or had
some errors (E), e.g., CRC check failed.
 
A DOUBLE press will switch to scanning mode.

A SHORT press will switch to the next channel in channels.txt

A SHORT press on the second button will switch to a different display screen.

## Spectrum mode

A medium press will active scan the whole band (400..406 MHz) and display a
spectrum diagram (each line == 50 kHz)
For TTGO boards without configurable button there are some new parameter in config.txt:
- spectrum=10       // 0=off / 1-99 number of seconds to show spectrum after restart
- timer=1           // 0=off / 1= show spectrum countdown timer in spectrum display
- marker=1          // 0=off / 1= show channel edge freq in spectrum display

## Setup

see Setup.md

