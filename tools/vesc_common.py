#!/usr/bin/env python3
"""Small dependency-free helpers shared by VESC host tools."""
from __future__ import annotations

TUNING_FIELDS = (
    "kpq", "kiq", "kpd", "kid",
    "kps", "kis", "kds",
    "kpp", "kip", "kdp",
    "telem_filter_q16",
)


def crc16(data: bytes) -> int:
    """VESC CRC-16/CCITT with initial value 0x0000."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def u32_delta(before: int, after: int) -> int:
    """Unsigned 32-bit monotonic counter delta with wraparound."""
    return (int(after) - int(before)) & 0xFFFFFFFF


def quantize_u16(value: float, scale: float) -> int:
    """Scale a physical gain to the firmware unsigned-16 representation."""
    return max(0, min(0xFFFF, round(float(value) * float(scale))))


def tuning_key(tuning) -> tuple[int, ...]:
    """Stable equality key for the tunable controller fields."""
    return tuple(int(getattr(tuning, field)) for field in TUNING_FIELDS)
