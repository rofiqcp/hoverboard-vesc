#!/usr/bin/env python3
import argparse, fcntl, hashlib, os, signal, socket, struct, subprocess, sys, time
from pathlib import Path

COMM_FW_VERSION=0; COMM_JUMP_TO_BOOTLOADER=1; COMM_ERASE_NEW_APP=2; COMM_WRITE_NEW_APP_DATA=3
COMM_CUSTOM_APP_DATA=36
MAX_FW=240*1024
HB_MAGIC=b"HB"; HB_VERSION=1; HB_BOOT_GET_INFO=0xF0; HB_BOOT_READ_APP=0xF1
STATE_STREAM=0x5354524D; STATE_TEST=0x54455354; STATE_RECOVERY=0x52454356; STATE_CONFIRMED=0x434E464D
HB_CUSTOM_GET_FW_UPDATE_STATE=23


def _cmdline_local(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
    except Exception:
        return ''

class RestartUploadSession(RuntimeError):
    pass

class IntentionalStreamStop(RuntimeError):
    pass

class UploadProcessLock:
    def __init__(self, key: str):
        safe=''.join(c if c.isalnum() else '_' for c in key)[:96]
        self.path=Path('/tmp')/f'pio_vesc_upload_{safe}.lock'
        self.fd=None
    def __enter__(self):
        self.fd=os.open(self.path, os.O_CREAT|os.O_RDWR, 0o660)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            try:
                os.lseek(self.fd,0,os.SEEK_SET); owner=os.read(self.fd,128).decode(errors='replace').strip()
            except Exception:
                owner=''
            raise RuntimeError(f'another firmware uploader is already active{f" ({owner})" if owner else ""}')
        os.ftruncate(self.fd,0); os.write(self.fd, f'pid={os.getpid()} started={time.time():.3f}'.encode()); os.fsync(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        if self.fd is not None:
            try: fcntl.flock(self.fd, fcntl.LOCK_UN)
            finally: os.close(self.fd); self.fd=None

def crc16(data: bytes)->int:
    crc=0
    for x in data:
        crc ^= x<<8
        for _ in range(8): crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc

def frame(payload: bytes)->bytes:
    n=len(payload)
    h=bytes((2,n)) if n<=255 else bytes((3,(n>>8)&255,n&255))
    c=crc16(payload)
    return h+payload+bytes((c>>8,c&255,3))


def _stable_linux_port(device:str)->str:
    if os.name!='posix': return device
    root=Path('/dev/serial/by-id')
    try:
        target=os.path.realpath(device)
        for p in sorted(root.iterdir()):
            if os.path.realpath(str(p))==target: return str(p)
    except OSError: pass
    return device

def _serial_candidates():
    import serial.tools.list_ports
    ranked=[]
    for p in serial.tools.list_ports.comports():
        dev=_stable_linux_port(p.device)
        meta=' '.join(str(x or '') for x in (p.description,p.manufacturer,p.hwid)).lower()
        usb_candidate=(p.vid is not None or '/dev/ttyUSB' in p.device or
                       '/dev/ttyACM' in p.device or '/dev/serial/by-id/' in dev or 'usb' in meta)
        if not usb_candidate: continue
        score=(100 if p.vid is not None else 0)+(40 if 'usb' in meta else 0)
        if 'ch340' in meta or 'ch341' in meta or p.vid==0x1A86: score+=30
        if '/dev/serial/by-id/' in dev: score+=25
        ranked.append((-score,dev,p.description or ''))
    ranked.sort(key=lambda x:(x[0],x[1])); out=[]
    for _,dev,desc in ranked:
        if dev not in [x[0] for x in out]: out.append((dev,desc))
    return out

def resolve_serial_port(args):
    requested=(args.serial_port or 'auto').strip()
    if requested.lower() not in ('auto','detect'): return requested
    candidates=_serial_candidates()
    if not candidates: raise RuntimeError('no USB serial ports detected; use --serial-port explicitly')
    errors=[]
    for dev,desc in candidates:
        probe=argparse.Namespace(**vars(args)); probe.serial_port=dev; link=None
        try:
            link=Link(probe); hw=fw_version(link,1.2)
            print(f'[PORT] selected {dev} ({desc}) target={hw}',flush=True); return dev
        except Exception as e: errors.append(f'{dev}:{type(e).__name__}')
        finally:
            if link is not None:
                try: link.close()
                except Exception: pass
    raise RuntimeError('no VESC/F103 target responded: '+', '.join(errors))

class Link:
    def __init__(self,args):
        self.args=args; self.sock=None; self.ser=None; self.buf=bytearray(); self.linebuf=bytearray()
        self.f411_direct=False; self.last_maintenance_refresh=0.0; self.route="unknown"
        self._suspended_launch_pids=[]
        try:
            self.open()
        except Exception:
            try:
                if self.sock is not None:
                    self.sock.close()
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.sock=None; self.ser=None; self.f411_direct=False
            self._resume_ros_launch()
            raise

    def _holders(self, port):
        real=os.path.realpath(port)
        r=subprocess.run(['fuser',real],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        out=[]
        for tok in r.stdout.split():
            if not tok.isdigit(): continue
            pid=int(tok)
            if pid==os.getpid(): continue
            try: cmd=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            except Exception: cmd=''
            out.append((pid,cmd))
        return out

    def _parent_pid(self, pid):
        try:
            raw=Path(f'/proc/{pid}/stat').read_text().split()
            return int(raw[3])
        except Exception:
            return 0

    def _suspend_ros_launch_ancestors(self, pids):
        suspended=[]
        seen=set()
        for child in pids:
            pid=self._parent_pid(child)
            while pid>1 and pid not in seen:
                seen.add(pid)
                cmd=_cmdline_local(pid)
                if 'ros2 launch ' in cmd or '/opt/ros/' in cmd and ' launch ' in cmd:
                    try:
                        os.kill(pid, signal.SIGSTOP)
                        suspended.append(pid)
                        print(f'[F411] paused ROS launch pid={pid} during direct CDC fallback',flush=True)
                    except (ProcessLookupError,PermissionError):
                        pass
                    break
                pid=self._parent_pid(pid)
        self._suspended_launch_pids.extend(x for x in suspended if x not in self._suspended_launch_pids)

    def _resume_ros_launch(self):
        for pid in reversed(self._suspended_launch_pids):
            try:
                os.kill(pid,signal.SIGCONT)
                print(f'[F411] resumed ROS launch pid={pid}',flush=True)
            except (ProcessLookupError,PermissionError):
                pass
        self._suspended_launch_pids.clear()

    def _prefer_tcp_f411(self, max_wait=5.0):
        wait=max(0.0,min(float(max_wait),5.0))
        deadline=time.monotonic()+wait
        last=None; attempt=0
        while True:
            attempt+=1
            try:
                self._open_tcp(attempts=1)
                self.route='tcp65101'
                holders=self._holders(self.args.serial_port)
                hpids=[pid for pid,_ in holders]
                print(f'[F411] TCP maintenance selected {self.args.host}:{self.args.port} '
                      f'after {attempt} probe(s); CDC holders={hpids or "none"}',flush=True)
                return True
            except Exception as e:
                last=e
                if time.monotonic()>=deadline:
                    break
                time.sleep(min(.20,max(0.0,deadline-time.monotonic())))
        print(f'[F411] TCP maintenance unavailable within {wait:.1f}s: {last}; falling back to direct CDC',flush=True)
        return False

    def _prepare_direct_f411(self):
        holders=self._holders(self.args.serial_port)
        if not holders:
            return
        official=[x for x in holders if 'stmf4_hmi_bridge' in x[1]]
        unknown=[x for x in holders if x not in official]
        if unknown:
            raise RuntimeError('F411 CDC busy by unknown process(es): '+', '.join(f'{pid}:{cmd[:80]}' for pid,cmd in unknown))
        pids=[pid for pid,_ in official]
        self._suspend_ros_launch_ancestors(pids)
        self._stop_official_bridge(official)

    def _open_tcp(self, attempts=2):
        last=None
        for attempt in range(1, attempts + 1):
            try:
                self.sock=socket.create_connection((self.args.host,self.args.port),timeout=.75)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(.10); self.buf.clear(); time.sleep(.12)
                return
            except OSError as e:
                last=e; self.sock=None
                if attempt < attempts: time.sleep(.12)
        raise RuntimeError(f'cannot connect F411 ROS gateway TCP {self.args.host}:{self.args.port}: {last}')

    def _tcp_probe(self, timeout=.8):
        try:
            p=self.transact(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION,timeout)
            return bool(p and p[0]==COMM_FW_VERSION)
        except Exception:
            return False

    def _stop_official_bridge(self, official):
        if not official: return
        pids=[pid for pid,_ in official]
        print(f'[F411] stopping CDC holder pid={pids} for direct USB upload',flush=True)
        for pid in pids:
            try: os.kill(pid, signal.SIGTERM)
            except ProcessLookupError: pass
        deadline=time.monotonic()+4.0
        while time.monotonic()<deadline:
            remain=[x for x in self._holders(self.args.serial_port) if x[0] in pids]
            if not remain: return
            time.sleep(.10)
        for pid in pids:
            try: os.kill(pid, signal.SIGKILL)
            except ProcessLookupError: pass
        deadline=time.monotonic()+2.0
        while time.monotonic()<deadline:
            if not any(x[0] in pids for x in self._holders(self.args.serial_port)): return
            time.sleep(.10)
        raise RuntimeError(f'F411 CDC still held after stopping pid={pids}')

    def reconnect_tcp(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass
        self.sock=None; self.buf.clear(); time.sleep(.25); self._open_tcp()

    def reconnect_transport(self):
        self.buf.clear()
        if self.args.transport=='tcp':
            self.reconnect_tcp(); return
        if self.args.transport=='serial':
            if self.ser:
                try: self.ser.close()
                except Exception: pass
            self.ser=None; time.sleep(.20)
            import serial
            self.ser=serial.Serial(self.args.serial_port,self.args.baud,timeout=.1,write_timeout=2)
            try: self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
            except Exception: pass
            self.route='serial'; return
        raise RuntimeError('automatic reconnect unsupported for this transport')

    def _f411_read_line(self, timeout=.1):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            nl=self.linebuf.find(b'\n')
            if nl>=0:
                raw=bytes(self.linebuf[:nl]); del self.linebuf[:nl+1]
                return raw.rstrip(b'\r').decode(errors='replace')
            d=self.ser.read(512)
            if d: self.linebuf.extend(d)
        return None

    def _f411_command(self,text,expect=None,timeout=3.0):
        self.ser.write((text+'\n').encode()); self.ser.flush()
        if expect is None: return ''
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            line=self._f411_read_line(.15)
            if line is None: continue
            if line.startswith('VESC:ERR:'): raise RuntimeError(line)
            if expect in line: return line
        raise TimeoutError(f'F411 command timeout: {text}')

    def _refresh_f411_maintenance(self, force=False):
        if not self.f411_direct or self.ser is None:
            return
        now=time.monotonic()
        if not force and now-self.last_maintenance_refresh < 2.0:
            return
        # Refresh only between complete VESC request/reply transactions. Older
        # F411 gateway builds used a short maintenance lease. Re-entering the
        # same mode is idempotent and keeps direct USB firmware uploads alive.
        line=self._f411_command('VESC:MODE:MAINTENANCE','VESC:MODE:MAINTENANCE',2.0)
        if line.strip() != 'VESC:MODE:MAINTENANCE':
            raise RuntimeError(f'F411 maintenance handshake invalid: {line!r}')
        self.last_maintenance_refresh=time.monotonic()
        if force:
            status=self._f411_command('VESC:STATUS','mode=MAINTENANCE',2.0)
            if 'VESC:STAT:' not in status or 'mode=MAINTENANCE' not in status:
                raise RuntimeError(f'F411 maintenance status invalid: {status!r}')

    def _open_f411_direct(self):
        import serial
        self.ser=serial.Serial(self.args.serial_port,1000000,timeout=.05,write_timeout=2,exclusive=True)
        self.f411_direct=True; self.ser.reset_input_buffer(); self.ser.reset_output_buffer(); self.linebuf.clear()
        self.ser.write(b'\n'); self.ser.flush(); time.sleep(.03); self.ser.reset_input_buffer()
        self._refresh_f411_maintenance(force=True)
        print('[F411] direct CDC maintenance confirmed',flush=True)
        # F411 changes UART ownership synchronously, but allow its CDC/status
        # output and UART RX flush to settle before the first binary packet.
        time.sleep(.20)

    def open(self):
        if self.args.transport=='tcp':
            self._open_tcp(); self.route='tcp'; return
        if self.args.transport=='f411':
            # Normal AGV runtime: stmf4_hmi_bridge owns the BlackPill CDC and
            # vesc_tool_bridge exposes the priority-100 Python maintenance route
            # on localhost:65101. Never tear ROS down when that route is alive.
            # Probe for at most five seconds; only then reclaim the CDC directly.
            if self._prefer_tcp_f411(self.args.tcp_wait):
                return
            self._prepare_direct_f411()
            print(f'[F411] using direct USB CDC fallback: {self.args.serial_port} @ 1000000',flush=True)
            self._open_f411_direct(); self.route='f411_direct'; return
        import serial
        self.ser=serial.Serial(self.args.serial_port,self.args.baud,timeout=.1,write_timeout=2)
        self.route='serial'

    def close(self):
        try:
            if self.sock: self.sock.close()
            if self.ser:
                if self.f411_direct:
                    try: self._f411_command('VESC:MODE:RUNTIME','VESC:MODE:RUNTIME',1.5)
                    except Exception as e: print(f'[F411] runtime restore warning: {e}',file=sys.stderr,flush=True)
                self.ser.close()
        finally:
            self.sock=None; self.ser=None; self.f411_direct=False
            self._resume_ros_launch()

    def write(self,b):
        if self.sock:
            self.sock.sendall(b); return
        if self.f411_direct:
            self._refresh_f411_maintenance()
            for off in range(0,len(b),48):
                chunk=b[off:off+48]
                self.ser.write(b'VESC:TX:M:'+chunk.hex().upper().encode()+b'\n'); self.ser.flush(); time.sleep(.002)
            return
        self.ser.write(b); self.ser.flush()

    def read_some(self):
        if self.sock:
            try:
                d=self.sock.recv(4096)
                if d: self.buf.extend(d)
                else: raise ConnectionError('TCP bridge closed connection')
            except (socket.timeout,TimeoutError): pass
            return
        if self.f411_direct:
            for _ in range(32):
                line=self._f411_read_line(.01)
                if line is None: break
                if line.startswith('VESC:ERR:'): raise RuntimeError(line)
                if line.startswith('VESC:RX:'):
                    hx=line[8:].strip()
                    try: self.buf.extend(bytes.fromhex(hx))
                    except ValueError: raise RuntimeError(f'bad F411 VESC hex: {hx[:80]}')
            return
        d=self.ser.read(4096)
        if d: self.buf.extend(d)

    def recv_payload(self,timeout=2.0):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.read_some()
            for start in range(len(self.buf)):
                st=self.buf[start]
                if st not in (2,3): continue
                if st==2:
                    if len(self.buf)-start < 2: continue
                    n=self.buf[start+1]; hdr=2
                else:
                    if len(self.buf)-start < 3: continue
                    n=(self.buf[start+1]<<8)|self.buf[start+2]; hdr=3
                if n<=0 or n>4096: continue
                total=hdr+n+3
                if len(self.buf)-start < total: continue
                raw=bytes(self.buf[start:start+total])
                if raw[-1]!=3: continue
                p=raw[hdr:hdr+n]; got=(raw[hdr+n]<<8)|raw[hdr+n+1]
                if got!=crc16(p): continue
                del self.buf[:start+total]
                return p
            if len(self.buf)>8192: del self.buf[:-4096]
        raise TimeoutError('VESC response timeout')

    def transact(self,payload,expected,timeout=3.0):
        self.write(frame(payload)); end=time.monotonic()+timeout
        while time.monotonic()<end:
            p=self.recv_payload(max(.05,end-time.monotonic()))
            if p and p[0]==expected: return p
        raise TimeoutError(f'no response id={expected}')

def boot_info(link, timeout=2.0):
    req=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_VERSION,HB_BOOT_GET_INFO))
    p=link.transact(req,COMM_CUSTOM_APP_DATA,timeout)
    if len(p)<26 or p[1:3]!=HB_MAGIC or p[3]!=HB_VERSION or p[4]!=HB_BOOT_GET_INFO or p[5]!=0:
        raise RuntimeError(f'boot info invalid: {p.hex()}')
    return {
        'app_region':struct.unpack('>I',p[6:10])[0],
        'state':struct.unpack('>I',p[10:14])[0],
        'size':struct.unpack('>I',p[14:18])[0],
        'crc16':struct.unpack('>H',p[18:20])[0],
        'resume':struct.unpack('>I',p[20:24])[0],
        'attempt':struct.unpack('>H',p[24:26])[0],
        'app_valid': bool(p[26]) if len(p)>=27 else True,
    }


def read_active_image(link, size:int)->bytes:
    if size<=0 or size>MAX_FW: raise RuntimeError(f'invalid active region size {size}')
    out=bytearray()
    step=240
    for off in range(0,size,step):
        want=min(step,size-off)
        req=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_VERSION,HB_BOOT_READ_APP))+struct.pack('>IH',off,want)
        p=link.transact(req,COMM_CUSTOM_APP_DATA,3.0)
        if len(p)<12 or p[1:3]!=HB_MAGIC or p[3]!=HB_VERSION or p[4]!=HB_BOOT_READ_APP or p[5]!=0:
            raise RuntimeError(f'active backup read failed at {off}: {p.hex()[:120]}')
        got_off=struct.unpack('>I',p[6:10])[0]; got_len=struct.unpack('>H',p[10:12])[0]
        if got_off!=off or got_len!=want or len(p)!=12+want:
            raise RuntimeError(f'active backup framing mismatch at {off}')
        out.extend(p[12:])
        if off==0 or off+want>=size or off%(32*1024)<step:
            print(f'[LKG] backup read {min(off+want,size)}/{size}',flush=True)
    return bytes(out)


def _lkg_root():
    return Path(os.environ.get('F103_LKG_ROOT', str(Path(__file__).resolve().parents[1]/'recovery'/'host_lkg')))

def latest_lkg_path():
    root=_lkg_root()
    root.mkdir(parents=True,exist_ok=True)
    latest=root/'LATEST'
    if latest.is_file():
        try:
            q=Path(latest.read_text().strip())
            if q.is_file(): return q
        except Exception: pass
    files=sorted(root.glob('f103_lkg_*.bin'))
    return files[-1] if files else None


def ensure_lkg_backup(link, info):
    if not info.get('app_valid', True):
        print('[LKG] no valid application; resident bootloader is the recovery authority',flush=True)
        return None
    if info['state'] in (STATE_STREAM,STATE_TEST,STATE_RECOVERY):
        prev=latest_lkg_path()
        if prev is None:
            print(f'[LKG] recovery state=0x{info["state"]:08X}; no prior CONFIRMED LKG, resident bootloader remains recovery authority',flush=True)
            return None
        print(f'[LKG] recovery state=0x{info["state"]:08X}; preserving existing {prev}',flush=True)
        return prev
    if info['state'] != STATE_CONFIRMED:
        prev=latest_lkg_path()
        if prev is not None:
            print(f'[LKG] target metadata is not CONFIRMED; refusing to bless it as LKG, preserving {prev}',flush=True)
            return prev
        print('[LKG] target metadata is not CONFIRMED; first migration proceeds without legacy LKG',flush=True)
        return None
    image_size=info.get('size',0)
    if image_size<=0 or image_size>info['app_region']:
        raise RuntimeError(f'confirmed metadata has invalid image size {image_size}')
    image=read_active_image(link,image_size)
    digest=hashlib.sha256(image).hexdigest()
    root=_lkg_root()
    root.mkdir(parents=True,exist_ok=True)
    stamp=time.strftime('%Y%m%d_%H%M%S')
    out=root/f'f103_lkg_{stamp}_{digest[:12]}.bin'
    tmp=out.with_suffix('.tmp'); tmp.write_bytes(image); os.replace(tmp,out)
    (root/'LATEST').write_text(str(out)+'\n')
    print(f'[LKG] saved {out} bytes={len(image)} sha256={digest}',flush=True)
    return out


def fw_version(link,timeout=2.0):
    p=link.transact(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION,timeout)
    if len(p)<4: return 'unknown'
    z=p.find(b'\0',3); return p[3:z if z>=0 else len(p)].decode(errors='replace')

def wait_for_bootloader(link, initial_hw: str) -> str:
    if 'bootloader' in initial_hw.lower():
        return initial_hw
    print(f'[VESC] application connected: {initial_hw}; entering resident bootloader', flush=True)
    # The application writes only a dual-word SRAM boot request and resets.
    # No flash write and no motor command is issued; recovery starts fail-safe.
    link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    deadline=time.monotonic()+20.0
    last=''
    while time.monotonic()<deadline:
        time.sleep(.25)
        try:
            last=fw_version(link,1.0)
            if 'bootloader' in last.lower():
                print(f'[VESC] bootloader ready: {last}', flush=True)
                return last
        except Exception:
            pass
    raise RuntimeError(f'bootloader did not appear; last={last!r}')


def _recover_bootloader_transport(link, reason: str):
    print(f'[VESC] transport recovery: {reason}', flush=True)
    last_error=None
    for attempt in range(1,5):
        try:
            if link.sock is not None or link.args.transport in ('tcp','f411'):
                if link.sock is not None:
                    link.reconnect_tcp()
                elif link.args.transport=='tcp':
                    link._open_tcp(attempts=4)
            hw=fw_version(link,3.0)
            if 'bootloader' in hw.lower():
                print(f'[VESC] recovery probe bootloader ready: {hw}', flush=True)
                return
            wait_for_bootloader(link,hw)
            raise RestartUploadSession('target application restarted; staging session must restart from erase')
        except RestartUploadSession:
            raise
        except Exception as e:
            last_error=e
            time.sleep(.20*attempt)
    raise RuntimeError(f'transport recovery failed: {last_error}')


def _stage_once(link,fw:bytes,session:int):
    p=link.transact(bytes((COMM_ERASE_NEW_APP,))+struct.pack('>IH',len(fw),crc16(fw)),COMM_ERASE_NEW_APP,10)
    if len(p)<2 or p[1]!=1: raise RuntimeError('external stream session rejected')
    staged=struct.pack('>IH',len(fw),crc16(fw))+fw
    resume=struct.unpack('>I',p[2:6])[0] if len(p)>=6 else 0
    if resume<0 or resume>len(staged): raise RuntimeError(f'invalid bootloader resume offset {resume}')
    if resume: print(f'[VESC] session={session} resume at stream offset={resume}/{len(staged)}',flush=True)
    # The F411 path crosses TCP -> ROS -> USB CDC -> 115200 UART. Smaller
    # packets reduce worst-case blocking and USB/UART burst pressure while the
    # bootloader's idempotent writes make retries safe.
    step=128 if link.args.transport=='f411' else 192
    for off in range(resume,len(staged),step):
        chunk=staged[off:off+step]
        request=bytes((COMM_WRITE_NEW_APP_DATA,))+struct.pack('>I',off)+chunk
        last_error=None
        for attempt in range(1,5):
            try:
                p=link.transact(request,COMM_WRITE_NEW_APP_DATA,2.5)
                if len(p)>=6 and p[1]==1 and struct.unpack('>I',p[2:6])[0]==off:
                    last_error=None; break
                last_error=RuntimeError(f'bad write ACK at {off}: {p.hex()}')
            except Exception as e:
                last_error=e
                if link.f411_direct and 'OWNER:RUNTIME' in str(e):
                    try:
                        link._refresh_f411_maintenance(force=True)
                        print(f'[F411] maintenance ownership restored at offset={off}', flush=True)
                    except Exception as me:
                        last_error=RuntimeError(f'{e}; maintenance restore failed: {me}')
            if attempt<4:
                print(f'[VESC] retry offset={off} attempt={attempt+1} reason={last_error}', flush=True)
                # One immediate idempotent retry handles a lost ACK. If two
                # attempts fail on TCP, rebuild the socket/maintenance route.
                if attempt==2 and link.sock is not None:
                    _recover_bootloader_transport(link,f'offset={off}')
                time.sleep(.08*attempt)
        if last_error is not None:
            raise RuntimeError(f'write failed at {off}: {last_error}')
        done=min(off+len(chunk),len(staged))
        if off==0 or done>=len(staged) or off%(step*40)==0:
            print(f'[VESC] session={session} write {done}/{len(staged)}', flush=True)
        stop_after=int(getattr(link.args,'fault_stop_after',0) or 0)
        if stop_after>0 and done>=stop_after:
            print(f'[FAULT-TEST] intentional host stop after ACK at {done} bytes',flush=True)
            raise IntentionalStreamStop(f'intentional stop at stream byte {done}')
        time.sleep(.001)


def app_update_info(link, timeout=2.0):
    req=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_VERSION,HB_CUSTOM_GET_FW_UPDATE_STATE))
    p=link.transact(req,COMM_CUSTOM_APP_DATA,timeout)
    if len(p)<16 or p[1:3]!=HB_MAGIC or p[3]!=HB_VERSION or p[4]!=HB_CUSTOM_GET_FW_UPDATE_STATE:
        raise RuntimeError(f'update-state response malformed: {p.hex()[:120]}')
    status=p[5]; state,size=struct.unpack('>II',p[6:14]); crc=struct.unpack('>H',p[14:16])[0]
    if status!=0: raise RuntimeError('application update metadata invalid')
    return {'state':state,'size':size,'crc':crc}

def _wait_confirmed(link, fw:bytes, label:str, timeout:float=20.0):
    wanted_crc=crc16(fw); deadline=time.monotonic()+timeout; last=None
    while time.monotonic()<deadline:
        try:
            info=app_update_info(link,2.0); last=info
            if info['state']==STATE_CONFIRMED and info['size']==len(fw) and info['crc']==wanted_crc:
                print(f'[VESC] {label} CONFIRMED size={len(fw)} crc16=0x{wanted_crc:04X}',flush=True)
                return info
        except Exception as e:
            last=e
        time.sleep(.25)
    raise RuntimeError(f'{label} application ran but did not reach CONFIRMED metadata: {last}')

def _wait_application(link, label:str, timeout:float=65.0):
    deadline=time.monotonic()+timeout
    last=''; stable=0; failures=0
    while time.monotonic()<deadline:
        time.sleep(.4)
        try:
            last=fw_version(link,1.5); failures=0
            if last and 'bootloader' not in last.lower():
                stable += 1
                if stable>=2:
                    print(f'[VESC] {label} application stable: {last}',flush=True)
                    return last
            else: stable=0
        except Exception as e:
            stable=0; failures+=1
            if failures>=3 and link.sock is not None:
                try:
                    link.reconnect_tcp(); failures=0
                    print(f'[VESC] reconnect while waiting {label}: {e}',flush=True)
                except Exception: pass
    raise RuntimeError(f'{label} application did not return stably; last={last!r}')


def _stream_install(link, fw:bytes, label:str):
    stage_error=None
    for session in range(1,4):
        try:
            _stage_once(link,fw,session); stage_error=None; break
        except IntentionalStreamStop:
            raise
        except RestartUploadSession as e: stage_error=e
        except Exception as e: stage_error=e
        if session>=3: break
        print(f'[VESC] {label} stream session {session} failed: {stage_error}; resuming/retrying',flush=True)
        _recover_bootloader_transport(link,f'{label} stream session {session+1}')
    if stage_error is not None: raise RuntimeError(f'{label} streaming failed: {stage_error}')
    link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    print(f'[VESC] {label} full CRC complete; TEST boot requested',flush=True)
    link.buf.clear()
    if hasattr(link,'linebuf'): link.linebuf.clear()
    app=_wait_application(link,label)
    _wait_confirmed(link,fw,label)
    return app


def upload(link,fw:bytes):
    if not fw or len(fw)>MAX_FW: raise RuntimeError(f'firmware size {len(fw)} exceeds {MAX_FW}')
    hw=None; last_error=None
    probe_deadline=time.monotonic()+75.0; attempt=0
    while time.monotonic()<probe_deadline:
        attempt += 1
        try:
            hw=fw_version(link,1.2); break
        except Exception as e:
            last_error=e
            if link.sock is not None and link.args.transport=='tcp':
                try: link.reconnect_tcp()
                except Exception as re: last_error=re
            if attempt==1 or attempt%4==0:
                remain=max(0,int(probe_deadline-time.monotonic()))
                print(f'[VESC] waiting for F103 response attempt={attempt} remaining={remain}s',flush=True)
            time.sleep(.15)
    if hw is None: raise RuntimeError(f'initial firmware probe failed after recovery window: {last_error}')
    wait_for_bootloader(link,hw)
    info=boot_info(link,3.0)
    print(f'[BOOT] app_region={info["app_region"]} state=0x{info["state"]:08X} resume={info["resume"]}',flush=True)
    lkg=ensure_lkg_backup(link,info)
    try:
        return _stream_install(link,fw,'candidate')
    except IntentionalStreamStop:
        raise
    except Exception as candidate_error:
        print(f'[ROLLBACK] candidate failed: {candidate_error}',file=sys.stderr,flush=True)
        if lkg is None:
            raise RuntimeError(f'candidate failed; resident bootloader remains in recovery and no previous relocated LKG exists: {candidate_error}')
        backup=lkg.read_bytes()
        if len(backup)>MAX_FW: raise RuntimeError(f'{candidate_error}; LKG size invalid {len(backup)}')
        last=None
        for _ in range(12):
            try:
                last=fw_version(link,2.0)
                if 'bootloader' in last.lower(): break
                wait_for_bootloader(link,last); last='f103rc_bootloader'; break
            except Exception:
                time.sleep(.5)
        if not last or 'bootloader' not in last.lower():
            raise RuntimeError(f'{candidate_error}; rollback bootloader unavailable')
        print(f'[ROLLBACK] restoring host LKG {lkg}',flush=True)
        _stream_install(link,backup,'rollback')
        raise RuntimeError(f'candidate failed and LKG rollback completed: {candidate_error}')

def selftest():
    p=b'\x00\x06\x00test\x00'; f=frame(p)
    assert f[0]==2 and f[1]==len(p) and f[-1]==3
    assert crc16(b'123456789')==0x31C3
    print('PIO_VESC_UPLOADER_SELFTEST_PASS')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--transport',choices=['serial','tcp','f411'])
    ap.add_argument('--serial-port',default='auto'); ap.add_argument('--baud',type=int,default=1000000)
    ap.add_argument('--host',default='127.0.0.1'); ap.add_argument('--port',type=int,default=65101)
    ap.add_argument('--tcp-wait',type=float,default=5.0,help='maksimum deteksi TCP maintenance sebelum fallback F411 CDC (maks 5 s)')
    ap.add_argument('--firmware'); ap.add_argument('--selftest',action='store_true')
    ap.add_argument('--probe-only',action='store_true')
    ap.add_argument('--boot-info-only',action='store_true')
    ap.add_argument('--fault-stop-after',type=int,default=0)
    a=ap.parse_args()
    if a.selftest: selftest(); return
    if not a.transport: ap.error('--transport required')
    if a.transport=='serial': a.serial_port=resolve_serial_port(a)
    elif a.transport=='f411' and (not a.serial_port or a.serial_port.lower() in ('auto','detect')):
        ap.error('--serial-port must be explicit for f411')
    lock_key=f'{a.transport}_{a.host}_{a.port}_{a.serial_port or "none"}'
    with UploadProcessLock(lock_key):
        link=Link(a)
        try:
            if a.probe_only or a.boot_info_only:
                hw=fw_version(link,2.0); print(f'VESC_TARGET_PROBE_PASS hw={hw}',flush=True)
                if a.boot_info_only:
                    if 'bootloader' not in hw.lower(): hw=wait_for_bootloader(link,hw)
                    print('BOOT_INFO',boot_info(link,3.0),flush=True)
                return
            if not a.firmware: ap.error('--firmware required')
            upload(link,Path(a.firmware).read_bytes())
        finally: link.close()
if __name__=='__main__':
    try: main()
    except IntentionalStreamStop as e:
        print(f'UPLOAD_INTENTIONAL_STOP: {e}',file=sys.stderr); raise SystemExit(75)
    except Exception as e:
        print(f'UPLOAD_FAIL: {e}',file=sys.stderr); raise SystemExit(2)
