#!/usr/bin/env python3
"""Send MAVLink v1 GCS heartbeats to satisfy PX4 SITL GCS connection check.

PX4 SITL requires a GCS heartbeat to pass the 'No connection to GCS' preflight
check. This script fakes that connection so the swarm can arm without QGC.
"""
import socket
import struct
import sys
import time


def x25crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc


def make_heartbeat(seq: int, sysid: int = 255, compid: int = 0) -> bytes:
    msgid = 0   # HEARTBEAT
    # custom_mode(u32)=0, type(u8)=6(GCS), autopilot(u8)=8(INVALID),
    # base_mode(u8)=0, system_status(u8)=4(ACTIVE), mavlink_version(u8)=3
    payload = struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3)
    crc_extra = 50  # CRC_EXTRA for HEARTBEAT msgid=0
    crc_in = bytes([len(payload), seq & 0xFF, sysid, compid, msgid]) + payload + bytes([crc_extra])
    crc = x25crc(crc_in)
    frame = bytes([0xFE, len(payload), seq & 0xFF, sysid, compid, msgid])
    frame += payload + struct.pack('<H', crc)
    return frame


def main() -> None:
    num_uavs = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    # Instance i listens on 18570+i for GCS MAVLink traffic
    ports = [18570 + i for i in range(num_uavs)]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    while True:
        hb = make_heartbeat(seq)
        for port in ports:
            try:
                sock.sendto(hb, ('127.0.0.1', port))
            except OSError:
                pass
        seq = (seq + 1) & 0xFF
        time.sleep(1)


if __name__ == '__main__':
    main()
