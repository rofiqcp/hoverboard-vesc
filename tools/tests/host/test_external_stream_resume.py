#!/usr/bin/env python3
import os,socket,struct,subprocess,sys,tempfile,threading,time
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
SCRIPT=R/'tools/pio_vesc_upload.py'; APPREG=240*1024; PAGE=2048
STATE_STREAM=0x5354524D; STATE_CONFIRMED=0x434E464D

def crc16(d):
 c=0
 for x in d:
  c^=x<<8
  for _ in range(8): c=((c<<1)^0x1021)&0xffff if c&0x8000 else (c<<1)&0xffff
 return c

def frame(p):
 n=len(p); h=bytes((2,n)) if n<=255 else bytes((3,(n>>8)&255,n&255)); c=crc16(p)
 return h+p+bytes((c>>8,c&255,3))

def recv(c,b):
 c.settimeout(5)
 while True:
  while b and b[0] not in (2,3): del b[0]
  if len(b)>=2:
   if b[0]==2: n=b[1]; h=2
   elif len(b)>=3: n=(b[1]<<8)|b[2]; h=3
   else: n=-1; h=3
   if n>=0 and len(b)>=h+n+3:
    r=bytes(b[:h+n+3]); del b[:h+n+3]; p=r[h:h+n]
    assert r[-1]==3 and ((r[h+n]<<8)|r[h+n+1])==crc16(p); return p
  d=c.recv(4096)
  if not d: raise EOFError
  b.extend(d)

fw=bytes((i*37+11)&255 for i in range(8193))
active=bytearray([0xA5])*APPREG
st={'mode':'app','state':0,'size':0,'crc':0,'buf':bytearray(),'written':0,'resume_seen':0,'updated':False,'err':None,'connections':0}
with tempfile.TemporaryDirectory() as td:
 td=Path(td); f=td/'fw.bin'; f.write_bytes(fw); lkg=td/'lkg'; lkg.mkdir()
 srv=socket.socket(); srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); srv.bind(('127.0.0.1',0)); srv.listen(8); srv.settimeout(20); port=srv.getsockname()[1]
 def worker():
  try:
   while not st['updated'] or st['connections']<2:
    c,_=srv.accept(); st['connections']+=1; b=bytearray()
    try:
     while True:
      p=recv(c,b); cmd=p[0]
      if cmd==0:
       name=(b'f103rc_bootloader\0' if st['mode']=='boot' else (b'motor_left_updated\0' if st['updated'] else b'motor_left\0'))
       c.sendall(frame(bytes((0,6,0))+name))
      elif cmd==1:
       if st['mode']=='app': st['mode']='boot'
       else:
        assert st['state']==STATE_STREAM and st['written']>=st['size']
        assert crc16(bytes(st['buf'][:st['size']]))==st['crc']
        active[:st['size']]=st['buf'][:st['size']]; st['state']=STATE_CONFIRMED; st['mode']='app'; st['updated']=True
      elif cmd==36:
       assert p[1:3]==b'HB' and p[3]==1; op=p[4]
       if op==0xF0:
        done=(st['written']//PAGE)*PAGE if st['state']==STATE_STREAM else 0
        if done>st['size']: done=st['size']
        resume=0 if done==0 else done+6
        r=bytes((36,))+b'HB'+bytes((1,op,0))+struct.pack('>IIIHIH',APPREG,st['state'],st['size'],st['crc'] if st['state'] else 0,resume,0xFFFF)
        c.sendall(frame(r))
       elif op==0xF1:
        off,want=struct.unpack('>IH',p[5:11]); data=bytes(active[off:off+want])
        c.sendall(frame(bytes((36,))+b'HB'+bytes((1,op,0))+struct.pack('>IH',off,want)+data))
       elif op==23:
        assert st['mode']=='app' and st['updated'] and st['state']==STATE_CONFIRMED
        c.sendall(frame(bytes((36,))+b'HB'+bytes((1,op,0))+struct.pack('>IIH',st['state'],st['size'],st['crc'])))
       else: raise AssertionError(('op',op))
      elif cmd==2:
       size,cr=struct.unpack('>IH',p[1:7]); assert size==len(fw)
       if not (st['state']==STATE_STREAM and st['size']==size and st['crc']==cr):
        st['state']=STATE_STREAM; st['size']=size; st['crc']=cr; st['buf']=bytearray(b'\xff'*size); st['written']=0
       complete=(st['written']//PAGE)*PAGE
       if complete<st['written']:
        st['buf'][complete:]=b'\xff'*(len(st['buf'])-complete); st['written']=complete
       resume=0 if complete==0 else complete+6
       if resume: st['resume_seen']=max(st['resume_seen'],resume)
       c.sendall(frame(bytes((2,1))+struct.pack('>I',resume)))
      elif cmd==3:
       off=struct.unpack('>I',p[1:5])[0]; data=p[5:]
       if off==0:
        size,cr=struct.unpack('>IH',data[:6]); assert size==st['size'] and cr==st['crc']; data=data[6:]; appoff=0
       else: appoff=off-6
       assert appoff==st['written'],(appoff,st['written'])
       st['buf'][appoff:appoff+len(data)]=data; st['written']+=len(data)
       c.sendall(frame(bytes((3,1))+struct.pack('>I',off)))
      else: raise AssertionError(('cmd',cmd))
    except (EOFError,ConnectionResetError,BrokenPipeError,socket.timeout,OSError): pass
    finally:
     try:c.close()
     except OSError:pass
  except Exception as e: st['err']=repr(e)
 th=threading.Thread(target=worker,daemon=True); th.start()
 env=os.environ.copy(); env['F103_LKG_ROOT']=str(lkg)
 base=[sys.executable,str(SCRIPT),'--transport','tcp','--host','127.0.0.1','--port',str(port),'--firmware',str(f)]
 r1=subprocess.run(base+['--fault-stop-after','5000'],capture_output=True,text=True,timeout=30,env=env)
 assert r1.returncode==75,r1.stdout+r1.stderr
 assert st['mode']=='boot' and st['state']==STATE_STREAM and 0<st['written']<len(fw),st
 # Represents MCU reset/power-cycle: volatile connection/session is gone; persistent STREAM state remains.
 r2=subprocess.run(base,capture_output=True,text=True,timeout=30,env=env)
 th.join(3); srv.close()
 assert r2.returncode==0,r2.stdout+r2.stderr
 assert st['err'] is None,st['err']; assert st['updated']; assert st['resume_seen']>=PAGE+6,st
 assert 'resume at stream offset=' in r2.stdout and 'candidate application stable: motor_left_updated' in r2.stdout and 'candidate CONFIRMED' in r2.stdout,r2.stdout
 print(f'EXTERNAL_STREAM_RESUME_PASS stop_written={st["written"]} resume={st["resume_seen"]} connections={st["connections"]}')
