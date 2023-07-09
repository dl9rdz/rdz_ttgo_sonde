#!/usr/bin/python
import os
import os.path
import sys
import csv
import subprocess

#default.csv content:
#nvs,      data, nvs,     0x9000,  0x5000,
#otadata,  data, ota,     0xe000,  0x2000,
#app0,     app,  ota_0,   0x10000, 0x140000,
#app1,     app,  ota_1,   0x150000,0x140000,
#spiffs,   data, spiffs,  0x290000,0x170000,

MKSPIFFS = os.environ['MKSPIFFS']
print("mkspiffs is "+MKSPIFFS)

OFFSET_BOOTLOADER = 0x1000
OFFSET_PARTITIONS = 0x8000

## now taken from default.csv
OFFSET_BOOTAPP0 = 0xE000
OFFSET_APPLICATION = 0x10000
OFFSET_SPIFFS = 0x291000
SIZE_SPIFFS = 0x16F000

esp32tools = sys.argv[1]
file_in = sys.argv[2]
data_dir = sys.argv[3]
file_out = sys.argv[4]

partition = esp32tools + "/partitions/default.csv"
if os.path.isfile("RX_FSK/partitions.csv"):
  partition = "RX_FSK/partitions.csv"

with open(partition, 'rt') as csvfile:
	partreader = csv.reader(csvfile, delimiter=',')
	for row in partreader:
		if row[0] == "otadata":
			OFFSET_BOOTAPP0 = int(row[3],16)
		if row[0] == "app0":
			OFFSET_APPLICATION = int(row[3],16)
		if row[0] == "spiffs":
			OFFSET_SPIFFS = int(row[3],16)
			SIZE_SPIFFS = int(row[4],16)

print("bootapp0: "+hex(OFFSET_BOOTAPP0))
print("app0: "+hex(OFFSET_APPLICATION))
print("spiffs: "+hex(OFFSET_SPIFFS)+" size "+hex(SIZE_SPIFFS))

# create binary partition
file_part = "/tmp/partition.bin"
partproc = subprocess.Popen(['python', esp32tools+'/gen_esp32part.py', partition, file_part]);
partproc.wait();

# create SPI file system
file_spiffs = "/tmp/spiffs.bin"
spiproc = subprocess.Popen([MKSPIFFS,'-c',data_dir,'-b','4096','-p','256','-s',str(SIZE_SPIFFS),file_spiffs]);
spiproc.wait();

files_in = [
## for arduino esp32 2.0    ('bootloader', OFFSET_BOOTLOADER, esp32tools+"/sdk/esp32/bin/bootloader_dio_80m.bin"),
    ('bootloader', OFFSET_BOOTLOADER, esp32tools+"/sdk/bin/bootloader_dio_80m.bin"),
    ('partitions', OFFSET_PARTITIONS, file_part),
    ('bootapp0', OFFSET_BOOTAPP0, esp32tools+"/partitions/boot_app0.bin"),
    ('application', OFFSET_APPLICATION, file_in),
    ('spiffs', OFFSET_SPIFFS, file_spiffs),
]

cur_offset = OFFSET_BOOTLOADER
with open(file_out, 'wb') as fout:
    for name, offset, file_in in files_in:
        assert offset >= cur_offset
        fout.write(b'\xff' * (offset - cur_offset))
        cur_offset = offset
        with open(file_in, 'rb') as fin:
            data = fin.read()
            fout.write(data)
            cur_offset += len(data)
            print('%-12s% 8d' % (name, len(data))) 
    print('%-12s% 8d' % ('total', cur_offset))
