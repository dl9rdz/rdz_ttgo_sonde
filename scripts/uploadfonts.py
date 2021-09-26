#!/usr/bin/env python3
import requests
import sys
import os
import socket
import esptool

if len(sys.argv)<3:
  print("Usage: uploadfonts <font.bin> <partition.csv")
  exit(1)

fontbin = sys.argv[1]
partition = sys.argv[2]

OFFSET=-1
SIZE=-1

# Fetch font partition info
file = open(partition, "r")
for line in file:
  if line.startswith("fonts"):
    l = line.split(",")
    OFFSET = l[3]
    SIZE = l[4]

#OFFSET="0x3F0000"
#SIZE="0x10000"

print("Using offset ",OFFSET,"; size is ",SIZE)

sys._argv = sys.argv[:]
sys.argv=[sys._argv[0],"--chip", "esp32", "--baud", "921600", "--before", "default_reset", "--after", "hard_reset", "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "detect", OFFSET, ".pio/build/ttgo-lora32/fonts.bin"]
esptool.main()
