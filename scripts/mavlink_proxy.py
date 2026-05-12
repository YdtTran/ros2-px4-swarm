#!/usr/bin/env python3
"""MAVLink proxy for a 3-UAV PX4 SITL swarm.

UAV0 (sysid 1): talks directly to QGC on :14550 via PX4's -o 14550 flag.
                Proxy does NOT touch UAV0 — no interference.
UAV1 (sysid 2): proxy owns :14560, sends heartbeat to PX4 :18571,
                forwards MAVLink → QGC :14550, routes QGC commands back.
UAV2 (sysid 3): same as UAV1 on :14570 / :18572.

If PX4 is already running with a different partner port (proxy was restarted),
the script detects the actual partner ports via raw-socket sniff and binds
to those instead of the fixed ports.

Start this script BEFORE PX4 for the fixed-port path (preferred).
"""
import socket
import struct
import threading
import time

QGC_HOST = "127.0.0.1"
QGC_PORT = 14550

# sysid → (proxy_recv_port, px4_gcs_listen_port)
UAV_PROXY = {
    2: (14560, 18571),
    3: (14570, 18572),
}


def _x25crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc


def _heartbeat(seq: int) -> bytes:
    payload = struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3)
    crc_extra = 50
    crc_in = bytes([len(payload), seq & 0xFF, 255, 0, 0]) + payload + bytes([crc_extra])
    crc = _x25crc(crc_in)
    return bytes([0xFE, len(payload), seq & 0xFF, 255, 0, 0]) + payload + struct.pack('<H', crc)


def _detect_partners(timeout: float = 2.0) -> dict[int, int]:
    """Sniff loopback UDP to find where each PX4 instance is currently sending.

    Returns {px4_gcs_port: current_partner_port}.
    Only looks at ports for UAV1 and UAV2 (not UAV0 which goes direct to QGC).
    """
    px4_src = {gp for _, gp in UAV_PROXY.values()}
    ignore_dst = {8888, QGC_PORT}
    try:
        rs = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_UDP)
        rs.bind(('127.0.0.1', 0))
        rs.settimeout(0.1)
        seen: dict[tuple[int, int], int] = {}
        t_end = time.time() + timeout
        while time.time() < t_end:
            try:
                data, _ = rs.recvfrom(65536)
                ihl = (data[0] & 0xF) * 4
                if data[9] != 17:
                    continue
                udp = data[ihl:]
                sp = struct.unpack('!H', udp[0:2])[0]
                dp = struct.unpack('!H', udp[2:4])[0]
                if sp in px4_src and dp not in ignore_dst:
                    seen[(sp, dp)] = seen.get((sp, dp), 0) + 1
            except socket.timeout:
                pass
        rs.close()
        result: dict[int, int] = {}
        for (sp, dp), n in seen.items():
            if sp not in result or seen.get((sp, result[sp]), 0) < n:
                result[sp] = dp
        return result
    except Exception as e:
        print(f"  partner detect failed: {e}")
    return {}


def _reset_uav0_partner(uav0_gcs_port: int = 18570, attempts: int = 8) -> None:
    """If UAV0's partner port ≠ QGC_PORT, redirect it back to QGC_PORT.

    Binds to QGC_PORT with SO_REUSEPORT (shares with QGC when it's running)
    and sends a burst of heartbeats to UAV0.  PX4 will adopt QGC_PORT as its
    new partner once the old partner times out and this burst is received.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(('', QGC_PORT))
    except OSError as e:
        print(f"  UAV0 partner-reset: cannot bind :{QGC_PORT}: {e}")
        return
    print(f"  UAV0 partner-reset: sending {attempts} heartbeats from :{QGC_PORT} → :{uav0_gcs_port}")
    for i in range(attempts):
        try:
            sock.sendto(_heartbeat(i), ('127.0.0.1', uav0_gcs_port))
        except OSError:
            pass
        time.sleep(0.5)
    sock.close()


def _uav_thread(recv_port: int, px4_gcs_port: int) -> None:
    """Bidirectional proxy for one UAV.

    - Sends GCS heartbeats FROM recv_port → PX4 (so PX4 adopts recv_port as partner)
    - Receives PX4 MAVLink at recv_port → forwards to QGC :14550
    - Receives QGC commands at recv_port → routes to PX4 :px4_gcs_port
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('', recv_port))
    except OSError as e:
        print(f"  ERROR: cannot bind :{recv_port}: {e}")
        return
    sock.settimeout(1.0)
    print(f"  :{recv_port} ↔ PX4 :{px4_gcs_port} → QGC :{QGC_PORT}")

    seq = 0
    last_hb = 0.0

    while True:
        now = time.time()
        if now - last_hb >= 1.0:
            try:
                sock.sendto(_heartbeat(seq), ('127.0.0.1', px4_gcs_port))
            except OSError:
                pass
            seq = (seq + 1) & 0xFF
            last_hb = now

        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            continue

        if not data or data[0] not in (0xFE, 0xFD):
            continue

        if addr[1] == px4_gcs_port:
            # PX4 MAVLink → forward to QGC (source = recv_port, QGC replies here)
            sock.sendto(data, (QGC_HOST, QGC_PORT))
        else:
            # QGC command → route back to PX4 using same sock so source stays recv_port
            sock.sendto(data, ('127.0.0.1', px4_gcs_port))


def main() -> None:
    print("MAVLink proxy starting...")

    # Correct UAV0's GCS partner back to QGC_PORT in case a previous session
    # left it pointing at a stale ephemeral port. Must run before UAV1/2
    # threads start so the port-14550 socket is available.
    threading.Thread(
        target=_reset_uav0_partner,
        daemon=True,
        name="uav0_reset",
    ).start()

    # If PX4 is already running, detect its current partner ports
    partners = _detect_partners(timeout=2.0)
    if partners:
        print(f"  PX4 already running, partners: {partners}")

    fixed_ports = {recv for recv, _ in UAV_PROXY.values()}

    for sysid, (fixed_recv, px4_gcs_port) in UAV_PROXY.items():
        partner = partners.get(px4_gcs_port, 0)
        if partner and partner not in fixed_ports and partner != QGC_PORT:
            recv_port = partner   # take over the existing partner port
        else:
            recv_port = fixed_recv  # PX4 not running yet; use fixed port
        threading.Thread(
            target=_uav_thread,
            args=(recv_port, px4_gcs_port),
            daemon=True,
            name=f"uav{sysid}",
        ).start()

    print("Proxy running. UAV0 talks directly to QGC :14550.")
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
