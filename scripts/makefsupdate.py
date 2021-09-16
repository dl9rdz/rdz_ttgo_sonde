#!/usr/bin/env python3
from os import listdir
import sys

path = sys.argv[1]

# create update.fs.bin with content:
# filename size CR LF
# data for filename ...

files = listdir(path)
files = list(filter(lambda x: x.endswith('.js') or x.endswith('.html') or x.endswith('.css'), files))
for f in files:
  with open(path+"/"+f,"rb") as myf:
    data=myf.read(-1)
  head = str.encode(f + " " + str(len(data)) + "\r\n")
  sys.stdout.buffer.write(head)
  sys.stdout.buffer.write(data)

