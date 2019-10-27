#!/usr/bin/python
import sys

OFFSET_BOOTLOADER = 0x1000
OFFSET_PARTITIONS = 0x8000
OFFSET_BOOTAPP0 = 0xE000
OFFSET_APPLICATION = 0x10000
OFFSET_SPIFFS = 0x291000

esp32tools = sys.argv[1]
file_in = sys.argv[2]
file_spiffs = sys.argv[3]
file_out = sys.argv[4]

files_in = [
    ('bootloader', OFFSET_BOOTLOADER, esp32tools+"/sdk/bin/bootloader_dio_80m.bin"),
    ('partitions', OFFSET_PARTITIONS, esp32tools+"/partitions/default.bin"),
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
