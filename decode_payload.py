#!/usr/bin/env python3
"""Decode a WearTrack manufacturer-data payload (PRD sec. 5) and verify its CRC-8.

Usage:
  decode_payload.py FFFF010203...        # hex of manufacturer data (company ID + 14 bytes)
  decode_payload.py 01020304...          # hex of the 14-byte payload only
Spaces / colons / 0x prefixes are ignored.
"""
import sys

STATES = {0: "IDLE", 1: "WALKING", 2: "ALERT", 3: "CALIBRATING"}


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def decode(raw: bytes) -> dict:
    if len(raw) == 16 and raw[:2] == b"\xff\xff":
        raw = raw[2:]
    if len(raw) != 14:
        raise ValueError(f"expected 14 payload bytes (or 16 with company ID), got {len(raw)}")
    f = raw[2]
    return {
        "version": raw[0],
        "sequence": raw[1],
        "flags": {
            "fall_alert": bool(f & 1), "bad_posture": bool(f & 2), "walking": bool(f & 4),
            "sensor_fault": bool(f & 8), "deadline_violation_seen": bool(f & 16),
        },
        "state": STATES.get(raw[3], f"?{raw[3]}"),
        "steps": int.from_bytes(raw[4:8], "big"),
        "tilt_deg": raw[8],
        "falls": raw[9],
        "uptime_min": int.from_bytes(raw[10:12], "big"),
        "violations": raw[12],
        "crc_ok": crc8(raw[:13]) == raw[13],
    }


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    h = "".join(sys.argv[1:]).replace(":", "").replace("0x", "").replace(" ", "")
    for k, v in decode(bytes.fromhex(h)).items():
        print(f"{k:12s} {v}")
