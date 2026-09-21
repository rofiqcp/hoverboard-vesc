#!/usr/bin/env python3
import argparse, fcntl, os, struct, sys, time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
from vesc_common import crc16
from vesc_dual import open_serial_compat

COMM_FW_VERSION=0; COMM_JUMP_TO_BOOTLOADER=1; COMM_ERASE_NEW_APP=2; COMM_WRITE_NEW_APP_DATA=3
COMM_CUSTOM_APP_DATA=36
MAX_FW=240*1024
HB_MAGIC=b"HB"; HB_BOOT_VERSION=1; HB_APP_VERSION=2; HB_BOOT_GET_INFO=0xF0
STATE_STREAM=0x5354524D; STATE_TEST=0x54455354; STATE_RECOVERY=0x52454356; STATE_CONFIRMED=0x434E464D
HB_CUSTOM_GET_FW_UPDATE_STATE=23; HB_CUSTOM_BOOT_HANDOFF=29



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

def resolve_serial_port(args, timeout_s: float = 1.5):
    requested=(args.serial_port or 'auto').strip()
    auto=requested.lower() in ('auto','detect')
    deadline=time.monotonic()+max(0.1,float(timeout_s))
    attempt=0; last_errors=[]
    while True:
        attempt += 1
        if auto:
            candidates=_serial_candidates()
            if not candidates:
                last_errors=['no USB serial ports detected']
        else:
            candidates=[(_stable_linux_port(requested),'explicit USB-UART')]
        errors=[]
        for dev,desc in candidates:
            probe=argparse.Namespace(**vars(args)); probe.serial_port=dev; link=None
            try:
                link=Link(probe); hw=_probe_known_bauds(link,1.2)
                if not hw or hw=='unknown': raise RuntimeError('empty COMM_FW_VERSION identity')
                print(f'[PORT] verified direct F103 UART {dev} ({desc}) target={hw}',flush=True); return dev
            except Exception as e: errors.append(f'{dev}:{type(e).__name__}:{e}')
            finally:
                if link is not None:
                    try: link.close()
                    except Exception: pass
        if errors: last_errors=errors
        if time.monotonic()>=deadline: break
        if attempt==1 or attempt%3==0:
            remain=max(0.0,deadline-time.monotonic())
            print(f'[PORT] waiting for F103 UART startup attempt={attempt} remaining={remain:.1f}s',flush=True)
        time.sleep(.25)
    detail=', '.join(last_errors) if last_errors else 'no candidates'
    raise RuntimeError('no F103 VESC target responded within startup grace window: '+detail)

class Link:
    """Direct USB-UART transport to STM32F103 USART3."""
    def __init__(self,args):
        self.args=args; self.ser=None; self.buf=bytearray(); self.route="serial"
        self.current_baud=int(args.baud)
        self.open()

    def open(self):
        self.ser=open_serial_compat(self.args.serial_port,self.current_baud,.10,2)
        if not getattr(self.ser,"_hb_skip_buffer_reset",False):
            try: self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
            except Exception: pass

    def close(self):
        if self.ser is not None:
            try: self.ser.close()
            finally: self.ser=None

    def reconnect_transport(self):
        self.close(); self.buf.clear(); time.sleep(.20); self.open()

    def set_baud(self, baud:int):
        baud=int(baud)
        if baud==self.current_baud: return
        self.close(); self.buf.clear(); self.current_baud=baud; time.sleep(.08); self.open()
        print(f'[VESC] host UART -> {baud} baud', flush=True)

    def write(self,b):
        self.ser.write(b)
        # PL2303 on Jetson can block indefinitely in tcdrain() even after the
        # full frame has entered the kernel/USB queue. open_serial_compat marks
        # that adapter with _hb_skip_buffer_reset; transaction ACK/readback is
        # sufficient proof that the frame reached the F103, so never tcdrain it.
        if not getattr(self.ser,"_hb_skip_buffer_reset",False):
            self.ser.flush()

    def read_some(self):
        d=self.ser.read(self.ser.in_waiting or 1)
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
                payload=raw[hdr:hdr+n]; got=(raw[hdr+n]<<8)|raw[hdr+n+1]
                if got!=crc16(payload): continue
                del self.buf[:start+total]
                return payload
            if len(self.buf)>8192: del self.buf[:-4096]
        raise TimeoutError('VESC response timeout')

    def transact(self,payload,expected,timeout=3.0):
        self.write(frame(payload)); end=time.monotonic()+timeout
        while time.monotonic()<end:
            p=self.recv_payload(max(.05,end-time.monotonic()))
            if p and p[0]==expected: return p
        raise TimeoutError(f'no response id={expected}')

def boot_info(link, timeout=2.0):
    req=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_BOOT_VERSION,HB_BOOT_GET_INFO))
    p=link.transact(req,COMM_CUSTOM_APP_DATA,timeout)
    if len(p)<26 or p[1:3]!=HB_MAGIC or p[3]!=HB_BOOT_VERSION or p[4]!=HB_BOOT_GET_INFO or p[5]!=0:
        raise RuntimeError(f'boot info invalid: {p.hex()}')
    return {
        'app_region':struct.unpack('>I',p[6:10])[0],
        'state':struct.unpack('>I',p[10:14])[0],
        'size':struct.unpack('>I',p[14:18])[0],
        'crc16':struct.unpack('>H',p[18:20])[0],
        'resume':struct.unpack('>I',p[20:24])[0],
        'attempt':struct.unpack('>H',p[24:26])[0],
        'app_valid': bool(p[26]) if len(p)>=27 else True,
        'reset_reason': struct.unpack('>I',p[27:31])[0] if len(p)>=31 else None,
        'reset_stage': struct.unpack('>I',p[31:35])[0] if len(p)>=35 else None,
    }


def fw_version(link,timeout=2.0):
    p=link.transact(bytes((COMM_FW_VERSION,)),COMM_FW_VERSION,timeout)
    if len(p)<4: return 'unknown'
    z=p.find(b'\0',3); return p[3:z if z>=0 else len(p)].decode(errors='replace')


def _set_link_baud(link, baud:int):
    if hasattr(link,'set_baud'):
        link.set_baud(int(baud))

def _probe_known_bauds(link, timeout=1.2):
    app_baud=int(getattr(link.args,'baud',921600))
    boot_baud=int(getattr(link.args,'boot_baud',921600))
    errors=[]
    for baud in dict.fromkeys((app_baud,boot_baud,115200)):
        try:
            _set_link_baud(link,baud)
            return fw_version(link,timeout)
        except Exception as exc:
            errors.append(f'{baud}:{type(exc).__name__}:{exc}')
    raise RuntimeError('no response at known UART bauds: '+', '.join(errors))

def wait_for_bootloader(link, initial_hw: str) -> str:
    boot_baud=int(getattr(link.args,'boot_baud',921600))
    if 'bootloader' in initial_hw.lower():
        _set_link_baud(link,boot_baud)
        return initial_hw
    print(f'[VESC] application connected: {initial_hw}; entering resident bootloader', flush=True)
    # New APP protocol v2 ACKs the handoff and resets only after UART DMA + shift
    # register are drained. Legacy APPs do not know op29; fall back only after
    # the ACK transaction times out so field upgrades from older firmware remain possible.
    req=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_APP_VERSION,HB_CUSTOM_BOOT_HANDOFF))
    try:
        p=link.transact(req,COMM_CUSTOM_APP_DATA,1.5)
        if len(p)<6 or p[1:3]!=HB_MAGIC or p[3]!=HB_APP_VERSION or p[4]!=HB_CUSTOM_BOOT_HANDOFF or p[5]!=0:
            raise RuntimeError(f'boot handoff ACK malformed: {p.hex()[:80]}')
        print('[VESC] application boot-handoff ACK received; waiting for drained-UART reset',flush=True)
    except Exception as exc:
        print(f'[VESC] legacy application handoff fallback: {exc}',flush=True)
        link.write(frame(bytes((COMM_JUMP_TO_BOOTLOADER,))))
    # Resetting only the F103 must not require manual action on the host. Probe
    # preferred 921600 first, then legacy 115200 for one-time field migration.
    link.buf.clear()
    deadline=time.monotonic()+20.0
    last=''; reconnects=0
    bauds=list(dict.fromkeys((boot_baud,115200)))
    while time.monotonic()<deadline:
        time.sleep(.20)
        for baud in bauds:
            try:
                _set_link_baud(link,baud)
                last=fw_version(link,0.8)
                if 'bootloader' in last.lower():
                    print(f'[VESC] bootloader ready: {last} baud={baud} reconnects={reconnects}', flush=True)
                    return last
            except Exception as exc:
                last=f'{baud}:{type(exc).__name__}: {exc}'
        try:
            link.reconnect_transport(); reconnects += 1
        except Exception:
            pass
    raise RuntimeError(f'bootloader did not appear; last={last!r} reconnects={reconnects}')


def _recover_bootloader_transport(link, reason: str):
    print(f'[VESC] direct-UART recovery: {reason}', flush=True)
    last_error=None
    for attempt in range(1,5):
        try:
            link.reconnect_transport()
            hw=_probe_known_bauds(link,1.5)
            if 'bootloader' in hw.lower():
                print(f'[VESC] recovery probe bootloader ready: {hw} baud={link.current_baud}', flush=True)
                return
            wait_for_bootloader(link,hw)
            raise RestartUploadSession('target application restarted; staging session must restart from erase')
        except RestartUploadSession:
            raise
        except Exception as exc:
            last_error=exc; time.sleep(.20*attempt)
    raise RuntimeError(f'direct-UART recovery failed: {last_error}')

def _stage_once(link,fw:bytes,session:int):
    p=link.transact(bytes((COMM_ERASE_NEW_APP,))+struct.pack('>IH',len(fw),crc16(fw)),COMM_ERASE_NEW_APP,10)
    if len(p)<2 or p[1]!=1: raise RuntimeError('external stream session rejected')
    staged=struct.pack('>IH',len(fw),crc16(fw))+fw
    resume=struct.unpack('>I',p[2:6])[0] if len(p)>=6 else 0
    if resume<0 or resume>len(staged): raise RuntimeError(f'invalid bootloader resume offset {resume}')
    if resume: print(f'[VESC] session={session} resume at stream offset={resume}/{len(staged)}',flush=True)
    # Resident-bootloader UART. Runtime application and current bootloader baud are selected dynamically; production target is 921600. Bounded chunks keep ACK latency predictable
    # while bootloader idempotent writes make retries safe.
    step=192
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
            if attempt<4:
                print(f'[VESC] retry offset={off} attempt={attempt+1} reason={last_error}', flush=True)
                # One immediate idempotent retry handles a lost ACK. After two
                # failures reopen the direct USB-UART and resume from metadata.
                if attempt==2:
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
    req=bytes((COMM_CUSTOM_APP_DATA,))+HB_MAGIC+bytes((HB_APP_VERSION,HB_CUSTOM_GET_FW_UPDATE_STATE))
    p=link.transact(req,COMM_CUSTOM_APP_DATA,timeout)
    if len(p)<16 or p[1:3]!=HB_MAGIC or p[3]!=HB_APP_VERSION or p[4]!=HB_CUSTOM_GET_FW_UPDATE_STATE:
        raise RuntimeError(f'update-state response malformed: {p.hex()[:120]}')
    status=p[5]; state,size=struct.unpack('>II',p[6:14]); crc=struct.unpack('>H',p[14:16])[0]
    if status!=0: raise RuntimeError('application update metadata invalid')
    return {'state':state,'size':size,'crc':crc}

def _wait_confirmed(link, fw:bytes, label:str, timeout:float=25.0):
    wanted_crc=crc16(fw); deadline=time.monotonic()+timeout; last=None; failures=0
    while time.monotonic()<deadline:
        try:
            info=app_update_info(link,1.5); last=info; failures=0
            if info['state']==STATE_CONFIRMED and info['size']==len(fw) and info['crc']==wanted_crc:
                print(f'[VESC] {label} CONFIRMED size={len(fw)} crc16=0x{wanted_crc:04X}',flush=True)
                return info
        except Exception as e:
            last=e; failures+=1
            # Confirmation intentionally resets the F103 through resident bootloader.
            # Reopen the host transport instead of assuming one serial fd survives every
            # UART peripheral reset/USB-UART driver combination.
            if failures>=2:
                try:
                    link.reconnect_transport(); failures=0
                    hw=fw_version(link,1.0)
                    if 'bootloader' in hw.lower():
                        bi=boot_info(link,1.5); last=bi
                        st=bi.get('state',0); rr=bi.get('reset_reason'); rs=bi.get('reset_stage')
                        print(f'[VESC] {label} confirmation transition boot_state=0x{st:08X} '
                              f'reason={None if rr is None else hex(rr)} stage={None if rs is None else hex(rs)}',flush=True)
                        if st in (STATE_RECOVERY,STATE_TEST) and rr not in (None,0x544F4B21):
                            raise RuntimeError(f'candidate entered recovery during probation: {bi}')
                except RuntimeError:
                    raise
                except Exception as re:
                    last=re
        time.sleep(.20)
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
            if failures>=3:
                try:
                    link.reconnect_transport(); failures=0
                    print(f'[VESC] reopened direct UART while waiting {label}: {e}',flush=True)
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
    _set_link_baud(link,int(getattr(link.args,'baud',921600)))
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
            hw=_probe_known_bauds(link,1.2); break
        except Exception as e:
            last_error=e
            if attempt%4==0:
                try: link.reconnect_transport()
                except Exception as re: last_error=re
            if attempt==1 or attempt%4==0:
                remain=max(0,int(probe_deadline-time.monotonic()))
                print(f'[VESC] waiting for F103 response attempt={attempt} remaining={remain}s',flush=True)
            time.sleep(.15)
    if hw is None: raise RuntimeError(f'initial firmware probe failed after recovery window: {last_error}')
    wait_for_bootloader(link,hw)
    info=boot_info(link,3.0)
    print(f'[BOOT] app_region={info["app_region"]} state=0x{info["state"]:08X} resume={info["resume"]} '
          f'reason={None if info.get("reset_reason") is None else hex(info["reset_reason"])} '
          f'stage={None if info.get("reset_stage") is None else hex(info["reset_stage"])}',flush=True)
    print('[VESC] direct install: host LKG backup disabled; resident bootloader is recovery authority',flush=True)
    try:
        return _stream_install(link,fw,'candidate')
    except IntentionalStreamStop:
        raise
    except Exception as candidate_error:
        print(f'[VESC] candidate failed; resident bootloader remains recovery authority: {candidate_error}',file=sys.stderr,flush=True)
        raise RuntimeError(f'candidate install failed: {candidate_error}')

def selftest():
    p=b'\x00\x06\x00test\x00'; f=frame(p)
    assert f[0]==2 and f[1]==len(p) and f[-1]==3
    assert crc16(b'123456789')==0x31C3
    print('PIO_VESC_UPLOADER_SELFTEST_PASS')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--transport',choices=['serial'],default='serial')
    ap.add_argument('--serial-port',default='auto'); ap.add_argument('--baud',type=int,default=921600); ap.add_argument('--boot-baud',type=int,default=921600)
    ap.add_argument('--firmware'); ap.add_argument('--selftest',action='store_true')
    ap.add_argument('--probe-only',action='store_true')
    ap.add_argument('--boot-info-only',action='store_true')
    ap.add_argument('--fault-stop-after',type=int,default=0)
    a=ap.parse_args()
    if a.selftest: selftest(); return
    a.serial_port=resolve_serial_port(a,15.0 if a.firmware else 3.0)
    lock_key=f'serial_{a.serial_port or "none"}'
    with UploadProcessLock(lock_key):
        link=Link(a)
        try:
            if a.probe_only or a.boot_info_only:
                hw=_probe_known_bauds(link,2.0); print(f'VESC_TARGET_PROBE_PASS hw={hw} baud={getattr(link,"current_baud","mock")}',flush=True)
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
