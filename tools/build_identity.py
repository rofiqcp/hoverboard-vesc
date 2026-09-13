#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess

def firmware_inputs(root: Path):
    files=[]
    for p in (root/'Src').rglob('*'):
        if p.is_file() and p.suffix.lower() in {'.c','.h','.s','.ld'}:
            files.append(p)
    files += [p for p in (root/'platformio.ini',root/'STM32F103RCTx_APP.ld',root/'STM32F103RCTx_BOOTLOADER.ld') if p.is_file()]
    return sorted(set(files), key=lambda p: p.relative_to(root).as_posix())

def source_hash32(root: Path) -> int:
    h=hashlib.sha256()
    for p in firmware_inputs(root):
        rel=p.relative_to(root).as_posix().encode()
        h.update(len(rel).to_bytes(2,'big')); h.update(rel); h.update(p.read_bytes())
    return int.from_bytes(h.digest()[:4],'big')

def git_sha12(root: Path) -> str:
    try:
        s=subprocess.check_output(['git','rev-parse','--short=12','HEAD'],cwd=root,text=True,stderr=subprocess.DEVNULL).strip().lower()
        if len(s)==12 and all(c in '0123456789abcdef' for c in s): return s
    except Exception: pass
    return '000000000000'

def compute(root: Path):
    sha=git_sha12(root); return {'git_sha12':sha,'git_hi32':int(sha[:8],16),'git_lo16':int(sha[8:],16),'build_id32':source_hash32(root)}
