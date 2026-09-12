#!/usr/bin/env python3
import os,socket,struct,subprocess,sys,tempfile,threading
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists()); SCRIPT=R/'tools/pio_vesc_upload.py'
APPREG=240*1024; STATE_STREAM=0x5354524D; STATE_CONFIRMED=0x434E464D

def crc16(d):
 c=0
 for x in d:
  c^=x<<8
  for _ in range(8): c=((c<<1)^0x1021)&0xffff if c&0x8000 else (c<<1)&0xffff
 return c

def frame(p):
 n=len(p); h=bytes((2,n)) if n<=255 else bytes((3,n>>8,n&255)); c=crc16(p); return h+p+bytes((c>>8,c&255,3))

def recv(c,b):
 c.settimeout(4)
 while True:
  while b and b[0] not in (2,3): del b[0]
  if len(b)>=2:
   n=b[1] if b[0]==2 else ((b[1]<<8)|b[2] if len(b)>=3 else -1); h=2 if b[0]==2 else 3
   if n>=0 and len(b)>=h+n+3:
    r=bytes(b[:h+n+3]); del b[:h+n+3]; p=r[h:h+n]
    assert r[-1]==3 and ((r[h+n]<<8)|r[h+n+1])==crc16(p); return p
  d=c.recv(4096)
  if not d: raise EOFError
  b.extend(d)

fw=bytes((i*37+11)&255 for i in range(2049))
st={'mode':'app','state':0,'size':0,'crc':0,'buf':bytearray(),'written':0,'updated':False,'writes':0,'probes':0,'confirmed_reads':0,'err':None}
with tempfile.TemporaryDirectory() as td:
 td=Path(td); f=td/'fw.bin'; f.write_bytes(fw); lkg=td/'lkg'; lkg.mkdir()
 srv=socket.socket(); srv.bind(('127.0.0.1',0)); srv.listen(2); port=srv.getsockname()[1]
 def worker():
  try:
   c,_=srv.accept(); b=bytearray()
   while st['probes']<2 or not st['updated'] or st['confirmed_reads']<1:
    p=recv(c,b); cmd=p[0]
    if cmd==0:
     name=b'f103rc_bootloader\0' if st['mode']=='boot' else (b'motor_left_updated\0' if st['updated'] else b'motor_left\0')
     c.sendall(frame(bytes((0,6,0))+name))
     if st['updated'] and st['mode']=='app': st['probes']+=1
    elif cmd==1:
     if st['mode']=='app': st['mode']='boot'
     else:
      assert st['state']==STATE_STREAM and st['written']==st['size'] and crc16(bytes(st['buf']))==st['crc']
      st['state']=STATE_CONFIRMED; st['mode']='app'; st['updated']=True
    elif cmd==36:
     assert p[1:3]==b'HB' and p[3]==1; op=p[4]
     if op==0xF0:
      r=bytes((36,))+b'HB'+bytes((1,op,0))+struct.pack('>IIIHIH',APPREG,st['state'],st['size'],st['crc'] if st['state'] else 0,0,0xFFFF)+bytes((0,))
      c.sendall(frame(r))
     elif op==23:
      assert st['mode']=='app' and st['state']==STATE_CONFIRMED
      c.sendall(frame(bytes((36,))+b'HB'+bytes((1,op,0))+struct.pack('>IIH',st['state'],st['size'],st['crc']))); st['confirmed_reads']+=1
     else: raise AssertionError(('op',op))
    elif cmd==2:
     size,cr=struct.unpack('>IH',p[1:7]); assert size==len(fw)
     st['state']=STATE_STREAM; st['size']=size; st['crc']=cr; st['buf']=bytearray(b'\xff'*size); st['written']=0
     c.sendall(frame(bytes((2,1))+struct.pack('>I',0)))
    elif cmd==3:
     off=struct.unpack('>I',p[1:5])[0]; data=p[5:]
     if off==0:
      size,cr=struct.unpack('>IH',data[:6]); assert size==st['size'] and cr==st['crc']; data=data[6:]; appoff=0
     else: appoff=off-6
     assert appoff==st['written']; st['buf'][appoff:appoff+len(data)]=data; st['written']+=len(data); st['writes']+=1
     c.sendall(frame(bytes((3,1))+struct.pack('>I',off)))
    else: raise AssertionError(cmd)
   c.close()
  except Exception as e: st['err']=repr(e)
 th=threading.Thread(target=worker,daemon=True); th.start(); env=os.environ.copy(); env['F103_LKG_ROOT']=str(lkg)
 r=subprocess.run([sys.executable,str(SCRIPT),'--transport','tcp','--host','127.0.0.1','--port',str(port),'--firmware',str(f)],capture_output=True,text=True,timeout=25,env=env)
 th.join(2); srv.close()
 assert r.returncode==0,r.stdout+r.stderr; assert st['err'] is None,st['err']; assert st['updated'] and st['writes']>=6 and st['probes']>=2 and st['confirmed_reads']>=1
 assert 'bootloader ready:' in r.stdout and 'candidate application stable: motor_left_updated' in r.stdout and 'candidate CONFIRMED' in r.stdout
 print(f'PIO_VESC_UPLOADER_MOCK_PASS bytes={len(fw)} writes={st["writes"]} crc=0x{crc16(fw):04x} confirmed=1')
