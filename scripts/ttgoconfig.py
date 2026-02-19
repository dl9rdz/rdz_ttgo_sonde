#!/usr/bin/python3
import hashlib
import re
import requests
import sys
import os
import socket
import tempfile
import esptool

ttgohost = "rdzsonde.local"
auth_user = None
auth_pass = None

# usually, rdzsonde.mooo.com should be an alias for that:
# or, more specifically:
updatehost = "https://github.com/dl9rdz/rdz_ttgo_sonde/blob/gh-pages/{}/{}?raw=true"

screens = ("screens1.txt", "screens2.txt", "screens3.txt")
allfiles = ("config.txt", "qrg.txt", "networks.txt") + screens

optprint = False
optdir = ""

def login(session, base_url, user, password):
  """Challenge-response login: GET preauth from login.html, compute SHA256(user:preauth:password), POST to login."""
  r = session.get(base_url + "login.html")
  r.raise_for_status()
  m = re.search(r'name="preauth"\s+value="([^"]+)"', r.text)
  if not m:
    print("Could not get login form / preauth", file=sys.stderr)
    sys.exit(1)
  preauth = m.group(1)
  auth_hex = hashlib.sha256(f"{user}:{preauth}:{password}".encode()).hexdigest()
  r2 = session.post(base_url + "login.html", data={"user": user, "preauth": preauth, "auth": auth_hex}, allow_redirects=True)
  if r2.status_code == 401:
    print("Invalid credentials or session expired", file=sys.stderr)
    sys.exit(1)

def getfile(name, session):
  urlg = url+"file/"+name;
  print("Downloading: ",urlg);
  data = session.get(urlg);
  if data.status_code == 401:
    print("Permission denied. Use --user and --pass for authenticated devices.", file=sys.stderr)
    sys.exit(1)
  if optprint:
    print(data.text)
  elif len(data.content)>0:
    f = open(optdir+name, "wb");
    f.write(data.content);
    f.close();
  else:
    print("Error: empty response")

def putfile(name, session):
  print("Uploading: ",optdir+name)
  with open(optdir+name, "rb") as f:
    files = { 'data': (name, f) }
    response = session.post(url+"file", files=files)
  if response.status_code == 401:
    print("Permission denied. Use --user and --pass for authenticated devices.", file=sys.stderr)
    sys.exit(1)

while len(sys.argv)>=2:
  if sys.argv[1]=="--print":
    del(sys.argv[1])
    optprint = True
    print("Printing file content on screen\n")
  elif sys.argv[1].startswith("--dir="):
    optdir = sys.argv[1][6:]+"/"
    print("Using file directory ",optdir)
    os.makedirs(optdir, exist_ok=True)
    del(sys.argv[1])
  elif sys.argv[1].startswith("--ttgo="):
    ttgohost = sys.argv[1][7:]
    del(sys.argv[1])
  elif sys.argv[1]=="--user" and len(sys.argv)>=3:
    auth_user = sys.argv[2]
    del(sys.argv[1:3])
  elif sys.argv[1]=="--pass" and len(sys.argv)>=3:
    auth_pass = sys.argv[2]
    del(sys.argv[1:3])
  else:
    break
  
if len(sys.argv)<=2:
  print("Usage: ",sys.argv[0]," [--ttgo={ip}] [--user USER --pass PASS] [--print|--dir={dir}] <get|put> <all|config|qrg|networks|screens>");
  print("or:    ",sys.argv[0]," <get|put> file {filename}");
  print("or:    ",sys.argv[0]," update <devel-xxx|master-yyy>");
  print("or:    ",sys.argv[0]," <backup|restore> file.bin");
  print("or:    ",sys.argv[0]," uploadfs directory");
  print("\n",
        "     screens is screens1.txt, screens2.txt, screens3.txt");
  print("     networks is networks.txt (Wifi ssid and password)")
  print("     qrg is qrg.txt (List with scan frequencies)")
  print("     all is screens + network + qrg")
  print("     --user/--pass for authenticated device (challenge-response login)")
  sys.exit(1)

if sys.argv[1]=="backup":
  # backup installed firmware (+ all data) to backup.bin
  sys._argv = sys.argv[:]
  # sys.argv=[sys._argv[0],"--chip", "esp32", "--baud", "921600", "--before", "default_reset", "--after", "hard_reset", "read_flash", "0x1000", "0x3FF000", sys.argv[2]]
  sys.argv=[sys._argv[0],"--chip", "esp32", "--baud", "115200", "--before", "default_reset", "--after", "hard_reset", "read_flash", "0x1000", "0x3FF000", sys.argv[2]]
  esptool.main()
  exit(0)

if sys.argv[1]=="restore":
  # restore system from backup.bin
  sys._argv = sys.argv[:]
  sys.argv=[sys._argv[0],"--chip", "esp32", "--baud", "921600", "--before", "default_reset", "--after", "hard_reset", "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "detect", "0x1000", sys.argv[2]]
  esptool.main()
  exit(0)
  

if sys.argv[1]=="update":
  # update to a new version...
  what = sys.argv[2]
  imgdir = "devel"
  if what.startswith("master"):
    imgdir = "master"
  host = updatehost.format(imgdir, what)
  data = requests.get(host)
  f = open("firmware.bin", "wb")
  f.write(data.content)
  f.close()
  sys._argv = sys.argv[:]
  sys.argv=[sys._argv[0],"--chip", "esp32", "--baud", "921600", "--before", "default_reset", "--after", "hard_reset", "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "detect", "0x1000", "firmware.bin"]
  esptool.main()
  exit(0)


def getpartinfo(partname):
  import gen_esp32part as pt
  # flash complete file system
  # automatically get file system parameters from ESP (i.e. you need to program the partition table first)
  if True:
    tmpdir = tempfile.mkdtemp()
    partbin = os.path.join(tmpdir, "partition.bin")
    sys._argv = sys.argv[:]
    sys.argv=[sys._argv[0], "--chip", "esp32", "--baud", "921600", "--before", "default_reset",
      "--after", "no_reset", "read_flash", "0x8000", "0x1000", partbin]
    esptool.main()
  else:
    # test only
    partbin="partitions-esp32v2.csv"
  with open(partbin,"rb") as f:
    table, input_is_binary = pt.PartitionTable.from_file(f)
  print("Partition table:")
  tab = table.to_csv()
  print(tab)
  OFFSET = -1
  SIZE = -1
  for line in tab.split("\n"):
    if line.startswith(partname):
      l = line.split(",")
      OFFSET = int(l[3],0)
      SIZE = l[4]
      mult = 1
      if SIZE[-1].upper() == 'K':
          SIZE = SIZE[:-1]
          mult = 1024
      print("SIZE is:"+SIZE+"!")
      SIZE = int(SIZE,0) * mult
 
  print("File system at ",hex(OFFSET)," size=",hex(SIZE))
  return [OFFSET, SIZE]

#OFFSET="0x3F0000"
#SIZE="0x10000"

if sys.argv[1]=="uploadfs":
  (offset, size) = getpartinfo("spiffs")
  print("Using offset ",offset,"; size is ",size)
  exit(0)


addrinfo = socket.gethostbyname(ttgohost)
url = "http://"+addrinfo+"/"
print("Using URL ",url)

session = requests.Session()
if auth_user and auth_pass:
  login(session, url, auth_user, auth_pass)

files=()

if sys.argv[2]=="file":
  if len(sys.argv)<=3:
    print("get/put file: missing filename\n");
    sys.exit(1);
  files=(sys.argv[3],)
elif sys.argv[2]=="config":
  files=("config.txt",)
elif sys.argv[2]=="qrg":
  files=("qrg.txt",)
elif sys.argv[2]=="networks":
  files=("networks.txt",)
elif sys.argv[2]=="screens":
  files=screens
elif sys.argv[2]=="all":
  files=allfiles
else:
  print("Invalid file specification: ",sys.argv[2])
  sys.exit(1)

if(sys.argv[1]=="get"):
  for f in files:
    getfile(f, session)
elif(sys.argv[1]=="put"):
  for f in files:
    putfile(f, session)
else:
  print("Invalid command ",sys.argv[1])

