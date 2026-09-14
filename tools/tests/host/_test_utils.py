"""Shared helpers for host-only regression scripts."""
from __future__ import annotations

from pathlib import Path


def read_source(root: Path, relative_path: str) -> str:
    return (root / relative_path).read_text(errors="ignore")


def unframe(raw: bytes) -> bytes:
    if not raw:
        raise ValueError("empty VESC frame")
    kind = raw[0]
    if kind == 2:
        header_size = 2
        payload_size = raw[1]
    elif kind == 3:
        header_size = 3
        payload_size = (raw[1] << 8) | raw[2]
    elif kind == 4:
        header_size = 4
        payload_size = (raw[1] << 16) | (raw[2] << 8) | raw[3]
    else:
        raise ValueError(f"invalid VESC frame type {kind}")
    return raw[header_size:header_size + payload_size]
