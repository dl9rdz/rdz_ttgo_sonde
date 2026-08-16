# listen_multicast.py
import socket
import struct

## Receives sondeseeker udp multicasts (JSON data, similar to rdzjson/TCP)

MCAST_GRP = '239.255.0.1'
MCAST_PORT = 62655

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
mreq = struct.pack('4s4s', socket.inet_aton(MCAST_GRP), socket.inet_aton('0.0.0.0'))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print(f"Listening on {MCAST_GRP}:{MCAST_PORT}...")
while True:
    data, addr = sock.recvfrom(65535)
    print(data.decode(errors='replace'))

