"""
IMPROV serial protocol for WiFi provisioning (aligned with RX_FSK logger.cpp).
Wire format: line-based, each packet = IMPROV\\x01 + type + len + data + checksum + \\n.
"""

from __future__ import annotations

from typing import Any, List, Optional, Tuple

# Packet header (7 bytes): "IMPROV" + protocol version 1
HEADER = b"IMPROV\x01"

# Message types
CURRENT_STATE = 0x01
ERROR_STATE = 0x02
RPC = 0x03
RPC_RESULT = 0x04

# RPC command bytes (sent in RPC packet data[0])
RPC_GET_INFO = 0x03
RPC_REQUEST_STATE = 0x02
RPC_WIFI_SCAN = 0x04
RPC_SEND_WIFI = 0x01


def build_packet(msg_type: int, data: bytes) -> bytes:
    """Build IMPROV packet: header + type + len + data + checksum + \\n."""
    payload = HEADER + bytes([msg_type, len(data)]) + data
    checksum = sum(payload) & 0xFF
    return payload + bytes([checksum, ord("\n")])


def parse_packet(line: bytes) -> Optional[Tuple[int, bytes]]:
    """
    Parse one IMPROV packet line (with or without trailing \\n).
    Returns (type, data) or None if invalid.
    """
    line = line.rstrip(b"\n\r")
    if len(line) < 10 or not line.startswith(HEADER):
        return None
    msg_type = line[7]
    length = line[8]
    if len(line) < 9 + length + 1:
        return None
    data = line[9 : 9 + length]
    checksum = line[9 + length]
    calculated = sum(line[: 9 + length]) & 0xFF
    if calculated != checksum:
        return None
    return (msg_type, data)


def send_packet(ser: Any, packet: bytes) -> None:
    """Write packet to serial and flush."""
    ser.write(packet)
    ser.flush()


def read_packet(ser: Any, timeout_sec: float = 2.0) -> Optional[Tuple[int, bytes]]:
    """
    Read one IMPROV packet. Reads lines until one starts with IMPROV header (device may send leading \\n).
    Returns (type, data) or None on timeout/parse error.
    """
    old_timeout = ser.timeout
    try:
        ser.timeout = timeout_sec
        while True:
            line = ser.readline()
            if not line:
                return None
            if line.startswith(HEADER):
                return parse_packet(line)
    finally:
        ser.timeout = old_timeout


def _send_rpc(ser: Any, command: int, data: bytes) -> None:
    """Send RPC packet: type RPC, data = [command, 0] or custom data."""
    if len(data) == 0:
        data = bytes([command, 0])
    send_packet(ser, build_packet(RPC, data))


def _parse_rpc_result_strings(data: bytes) -> List[str]:
    """Parse RPC result payload: data[0]=reply_to, data[1]=total_len, then length-prefixed strings."""
    if len(data) < 2:
        return []
    total_len = data[1]
    result = []
    idx = 2
    while idx < 2 + total_len and idx < len(data):
        slen = data[idx]
        idx += 1
        if idx + slen > len(data):
            break
        result.append(data[idx : idx + slen].decode("utf-8", errors="replace"))
        idx += slen
    return result


def rpc_get_info(ser: Any, timeout_sec: float = 3.0) -> Optional[List[str]]:
    """Send RPC GET_INFO; read one RPC_RESULT; return list of strings (firmware name, version, chip, ...)."""
    _send_rpc(ser, RPC_GET_INFO, bytes([RPC_GET_INFO, 0]))
    pkt = read_packet(ser, timeout_sec)
    if pkt is None or pkt[0] != RPC_RESULT:
        return None
    data = pkt[1]
    if len(data) < 2 or data[0] != RPC_GET_INFO:
        return None
    return _parse_rpc_result_strings(data)


def rpc_request_state(ser: Any, timeout_sec: float = 2.0) -> Optional[int]:
    """Send RPC REQUEST_STATE; read CURRENT_STATE; return state byte (e.g. 2=READY, 4=PROVISIONED)."""
    _send_rpc(ser, RPC_REQUEST_STATE, bytes([RPC_REQUEST_STATE, 0]))
    pkt = read_packet(ser, timeout_sec)
    if pkt is None or pkt[0] != CURRENT_STATE or len(pkt[1]) < 1:
        return None
    return pkt[1][0]


def rpc_wifi_scan(ser: Any, timeout_sec: float = 10.0) -> List[Tuple[str, str, str]]:
    """
    Send RPC WIFI_SCAN; read RPC_RESULTs until result with empty first string (end marker).
    Each result = (ssid, rssi, secured). Returns list of networks.
    """
    _send_rpc(ser, RPC_WIFI_SCAN, bytes([RPC_WIFI_SCAN, 0]))
    networks: List[Tuple[str, str, str]] = []
    while True:
        pkt = read_packet(ser, timeout_sec)
        if pkt is None or pkt[0] != RPC_RESULT:
            break
        data = pkt[1]
        if len(data) < 2 or data[0] != RPC_WIFI_SCAN:
            continue
        strings = _parse_rpc_result_strings(data)
        if not strings:
            break
        ssid = strings[0]
        if not ssid:
            break
        rssi = strings[1] if len(strings) > 1 else ""
        secured = strings[2] if len(strings) > 2 else ""
        networks.append((ssid, rssi, secured))
    return networks


def rpc_send_wifi(
    ser: Any, ssid: str, password: str, timeout_sec: float = 15.0
) -> Optional[str]:
    """
    Send RPC SEND_WIFI with ssid and password. Data: [0x01, total_len, ssid_len, ssid..., pw_len, pw...].
    Returns first string from result (URL) if success, None on failure.
    """
    ssid_b = ssid.encode("utf-8")
    pw_b = password.encode("utf-8")
    total_len = 1 + len(ssid_b) + 1 + len(pw_b)
    data = bytes([RPC_SEND_WIFI, total_len, len(ssid_b)]) + ssid_b + bytes([len(pw_b)]) + pw_b
    _send_rpc(ser, RPC_SEND_WIFI, data)
    pkt = read_packet(ser, timeout_sec)
    if pkt is None or pkt[0] != RPC_RESULT:
        return None
    payload = pkt[1]
    if len(payload) < 2 or payload[0] != RPC_SEND_WIFI:
        return None
    strings = _parse_rpc_result_strings(payload)
    return strings[0] if strings else None
