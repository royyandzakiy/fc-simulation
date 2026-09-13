#!/usr/bin/env python3
"""The wire format, mirroring the #pragma pack(1) structs in src/app/fc_core.hpp.

Deliberately built on `struct` and a hand-rolled CRC rather than construct/crc
so this folder installs nothing extra. The bytes are identical either way -
same sync bytes, same little-endian layout, same CRC-8.

    SensorPacket  22 bytes  0xA5  seq u32, dt f32, gyro[3] f32, crc u8
    MotorPacket   39 bytes  0x5A  seq u32, m[4] f32, rc[4] f32, armed u8, crc u8

CRC-8 over every byte before the crc field itself: poly 0xD5, init 0, no
reflection, no final xor. Same polynomial CRSF uses, and the same crc8() the
C++ side computes in fc_core.hpp.
"""

import struct

SYNC_SENSOR = 0xA5
SYNC_MOTOR = 0x5A
CRC_POLY = 0xD5

# "<" little endian, no padding - matches #pragma pack(push, 1)
_SENSOR_BODY = struct.Struct("<BIf3f")
_MOTOR_BODY = struct.Struct("<BI4f4fB")

SENSOR_LEN = _SENSOR_BODY.size + 1
MOTOR_LEN = _MOTOR_BODY.size + 1

# The C++ side static_asserts these, so a mismatch means the two definitions
# have drifted apart.
assert SENSOR_LEN == 22, SENSOR_LEN
assert MOTOR_LEN == 39, MOTOR_LEN


def crc8(data):
    """Bitwise CRC-8, matching fc::crc8() byte for byte."""
    c = 0
    for byte in data:
        c ^= byte
        for _ in range(8):
            c = ((c << 1) ^ CRC_POLY) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def build_sensor(seq, dt, gyro):
    """One sensor frame: sync, seq, dt, body rates in rad/s, trailing CRC."""
    body = _SENSOR_BODY.pack(SYNC_SENSOR, seq, dt, *gyro)
    return body + bytes([crc8(body)])


def parse_motor(frame):
    """Motor frame -> dict. Raises on a bad sync byte or a bad CRC."""
    if len(frame) != MOTOR_LEN:
        raise ValueError(f"motor frame is {len(frame)}B, expected {MOTOR_LEN}")
    if frame[-1] != crc8(frame[:-1]):
        raise ValueError("motor frame failed CRC")

    sync, seq, m0, m1, m2, m3, rc0, rc1, rc2, rc3, armed = _MOTOR_BODY.unpack(
        frame[:-1])
    if sync != SYNC_MOTOR:
        raise ValueError(f"motor frame sync is 0x{sync:02X}, expected 0x5A")

    return {"seq": seq, "m": [m0, m1, m2, m3],
            "rc": [rc0, rc1, rc2, rc3], "armed": armed}
