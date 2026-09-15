#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,csv,json,statistics,struct,time,threading
from dataclasses import replace
from vesc_dual import VescDual,COMM_SET_CURRENT,COMM_SET_RPM

class AbortRun(RuntimeError): pass

def send_scalar(link,right,cmd,value):
    p=bytes((cmd,))+struct.pack('>i',int(round(value)))
    link.send_no_reply(p,right)

def release(link,right):
    link.set_id_test(0.0,0.0,right)

def motor_name(right): return 'right' if right else 'left'

def phys_to_tuning(base,**kw):
    d=dict(base.physical); d.update({k:v for k,v in kw.items() if v is not None})
    return replace(base,
        kpq=max(0,min(65535,round(d['foc_q_kp']*1536))),
        kiq=max(0,min(65535,round(d['foc_q_ki']*4.608))),
        kpd=max(0,min(65535,round(d['foc_d_kp']*1536))),
        kid=max(0,min(65535,round(d['foc_d_ki']*4.608))),
        kps=max(0,min(65535,round(d['speed_kp']*100000))),
        kis=max(0,min(65535,round(d['speed_ki']*100000))),
        kds=max(0,min(65535,round(d['speed_kd']*100000))),
        kpp=max(0,min(65535,round(d['pos_kp']*1000))),
        kip=max(0,min(65535,round(d['pos_ki']*1000))),
        kdp=max(0,min(65535,round(d['pos_kd']*1000))),
        telem_filter_q16=max(1,min(65535,round(d['telemetry_filter']*65535))))

def metrics(rows,target,key,tol=0.02):
    rr=[r for r in rows if r['phase']=='run']
    if not rr: return dict(valid=False)
    t=[r['t'] for r in rr]; y=[r[key] for r in rr]
    y0=y[0]; amp=target-y0; direction=1 if amp>=0 else -1; mag=max(abs(amp),1e-9)
    z=[direction*(v-y0) for v in y]; tgt=abs(amp)
    def first_cross(frac):
        th=frac*tgt
        for ti,zi in zip(t,z):
            if zi>=th:return ti
        return None
    t10,t90=first_cross(.1),first_cross(.9)
    rise=None if t10 is None or t90 is None else max(0.0,t90-t10)
    peak=max(z); overs=max(0.0,(peak-tgt)/mag*100.0)
    band=max(tol*mag, 0.01 if key in ('iq','id') else 1.0)
    settle=None
    for i,ti in enumerate(t):
        if all(abs(v-target)<=band for v in y[i:]): settle=ti; break
    tail=y[max(0,int(len(y)*.8)):]; steady=statistics.mean(tail) if tail else y[-1]
    sse=steady-target; sse_pct=100.0*sse/mag
    iae=ise=itae=0.0
    for i in range(1,len(rr)):
        dt=max(0.0,t[i]-t[i-1]); e0=target-y[i-1]; e1=target-y[i]; em=.5*(e0+e1)
        iae+=abs(em)*dt; ise+=em*em*dt; itae+=.5*(t[i-1]+t[i])*abs(em)*dt
    return dict(valid=True,initial=y0,target=target,rise_10_90_s=rise,settling_2pct_s=settle,
                overshoot_pct=overs,steady=steady,steady_error=sse,steady_error_pct=sse_pct,
                iae=iae,ise=ise,itae=itae,peak=max(y),minimum=min(y),samples=len(y))

def safety(v,max_erpm,max_current,max_duty):
    if v.fault: raise AbortRun(f'fault={v.fault}')
    if abs(v.rpm)>max_erpm: raise AbortRun(f'erpm={v.rpm:.0f}>{max_erpm:.0f}')
    if abs(v.current_motor)>max_current: raise AbortRun(f'Imotor={v.current_motor:.2f}>{max_current:.2f}A')
    if abs(v.duty)>max_duty: raise AbortRun(f'duty={100*v.duty:.1f}%>{100*max_duty:.1f}%')
    if not (20.0<=v.vin<=60.0): raise AbortRun(f'Vin={v.vin:.1f}V')

def capture_step(link,right,loop,target,duration,hz,guard,position_target=None):
    rows=[]; t0=time.monotonic(); abort=''; stop_evt=threading.Event(); writer_errors=[]

    def issue_command():
        if loop=='speed': send_scalar(link,right,COMM_SET_RPM,target)
        elif loop=='currentq': send_scalar(link,right,COMM_SET_CURRENT,target*1000.0)
        elif loop=='currentd': link.set_id_test(abs(target),0.0,right)
        elif loop=='position': link.set_position_counts(int(position_target),right)

    # SET_RPM/SET_CURRENT have no reply. Keep them refreshed independently from
    # telemetry so a slow direct USB-UART GET_VALUES transaction can never exhaust the
    # 500-ms motor watchdog. Other loops use request/reply commands and stay
    # single-threaded to preserve response correlation.
    def writer():
        if loop not in ('speed','currentq'): return
        next_t=time.monotonic()
        while not stop_evt.is_set():
            try:
                issue_command()
            except Exception as e:
                writer_errors.append(repr(e)); return
            next_t += 0.10
            stop_evt.wait(max(0.0,next_t-time.monotonic()))

    th=threading.Thread(target=writer,name='vesc-setpoint-refresh',daemon=True)
    try:
        if loop in ('speed','currentq'): th.start()
        while True:
            now=time.monotonic(); elapsed=now-t0
            if elapsed>=duration: break
            if loop not in ('speed','currentq'): issue_command()
            try:
                v=link.values(right)
            except TimeoutError:
                # A single telemetry miss is diagnostic, not a motor failure;
                # the background command refresher keeps the watchdog alive.
                if writer_errors: raise AbortRun('setpoint writer failed: '+writer_errors[-1])
                time.sleep(0.02); continue
            safety(v,*guard)
            d=link.diag(right) if loop=='position' else None
            rows.append(dict(phase='run',t=time.monotonic()-t0,erpm=v.rpm,iq=v.iq,id=v.id,imotor=v.current_motor,
                             ibat=v.current_in,duty=v.duty,vd=v.vd,vq=v.vq,vin=v.vin,fault=v.fault,
                             position=(d.position if d else 0),position_target=(d.position_target if d else 0)))
            if writer_errors: raise AbortRun('setpoint writer failed: '+writer_errors[-1])
            time.sleep(max(0.02,1.0/max(hz,1.0)))
    except AbortRun as e: abort=str(e)
    finally:
        stop_evt.set()
        if th.is_alive(): th.join(timeout=.4)
        try: release(link,right)
        except Exception: pass
    ts=time.monotonic()
    while time.monotonic()-ts<0.8:
        try:
            v=link.values(right); d=link.diag(right) if loop=='position' else None
            rows.append(dict(phase='stop',t=time.monotonic()-ts,erpm=v.rpm,iq=v.iq,id=v.id,imotor=v.current_motor,
                             ibat=v.current_in,duty=v.duty,vd=v.vd,vq=v.vq,vin=v.vin,fault=v.fault,
                             position=(d.position if d else 0),position_target=(d.position_target if d else 0)))
        except Exception: pass
        time.sleep(max(.05,1.0/max(hz,1.0)))
    key={'speed':'erpm','currentq':'iq','currentd':'id','position':'position'}[loop]
    mt=position_target if loop=='position' else target
    m=metrics(rows,float(mt),key); m['abort']=abort; m['loop']=loop; m['motor']=motor_name(right); m['writer_errors']=writer_errors
    if rows:
        run_rows=[r for r in rows if r['phase']=='run']
        coast_rows=[r for r in rows if r['phase']=='stop']
        if run_rows:
            m['peak_current_a']=max(abs(r['imotor']) for r in run_rows)
            m['peak_duty']=max(abs(r['duty']) for r in run_rows)
        if coast_rows:
            m['coast_peak_current_untrusted_a']=max(abs(r['imotor']) for r in coast_rows)
    return rows,m

def score(m):
    if not m.get('valid') or m.get('abort'): return 1e9
    rise=m.get('rise_10_90_s'); settle=m.get('settling_2pct_s')
    return abs(m['steady_error_pct'])*3.0+m['overshoot_pct']+(50.0 if rise is None else rise*20.0)+(30.0 if settle is None else settle*8.0)

def idle_monitor(link,right,seconds,hz):
    vals=[]; end=time.monotonic()+seconds
    while time.monotonic()<end:
        v=link.values(right); vals.append(v); time.sleep(1.0/hz)
    def st(name):
        x=[getattr(v,name) for v in vals]; return dict(mean=statistics.mean(x),std=statistics.pstdev(x),minimum=min(x),maximum=max(x))
    return {k:st(k) for k in ('current_motor','current_in','id','iq','rpm','duty')}

def parse_candidates(text,loop):
    out=[]
    for item in text.split(';'):
        if not item.strip():continue
        v=[float(x) for x in item.split(',')]
        need=3 if loop in ('speed','position') else 2
        if len(v)!=need: raise ValueError(f'{loop} candidate needs {need} values')
        out.append(v)
    return out

def write_run(outdir,name,rows,summary):
    outdir.mkdir(parents=True,exist_ok=True); stamp=time.strftime('%Y%m%d_%H%M%S')
    csvp=outdir/f'{name}_{stamp}.csv'; jsp=outdir/f'{name}_{stamp}.json'
    if rows:
        with csvp.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    jsp.write_text(json.dumps(summary,indent=2,sort_keys=True))
    return csvp,jsp

def main():
    ap=argparse.ArgumentParser(description='VESC dual FOC response measurement and PID tuning lab')
    ap.add_argument('mode',choices=['idle','step','sweep','max-erpm','show','apply'])
    ap.add_argument('--port',default='auto'); ap.add_argument('--motor',choices=['left','right'],default='left')
    ap.add_argument('--loop',choices=['speed','currentq','currentd','position'],default='speed')
    ap.add_argument('--target',type=float,default=300); ap.add_argument('--duration',type=float,default=1.5); ap.add_argument('--hz',type=float,default=40)
    ap.add_argument('--max-erpm',type=float,default=3000); ap.add_argument('--max-current',type=float,default=1.25); ap.add_argument('--max-duty',type=float,default=.90)
    ap.add_argument('--candidates',default=''); ap.add_argument('--targets',default=''); ap.add_argument('--arm',action='store_true'); ap.add_argument('--store',action='store_true')
    ap.add_argument('--foc-q-kp',type=float); ap.add_argument('--foc-q-ki',type=float); ap.add_argument('--foc-d-kp',type=float); ap.add_argument('--foc-d-ki',type=float)
    ap.add_argument('--speed-kp',type=float); ap.add_argument('--speed-ki',type=float); ap.add_argument('--speed-kd',type=float)
    ap.add_argument('--pos-kp',type=float); ap.add_argument('--pos-ki',type=float); ap.add_argument('--pos-kd',type=float); ap.add_argument('--filter',type=float)
    ap.add_argument('--out',default='tools/results/response_lab')
    a=ap.parse_args(); right=a.motor=='right'; link=VescDual(a.port,921600,timeout=.65); link.require_platform_compatible(True); outdir=Path(a.out)
    try:
        base=link.get_tuning(right)
        if a.mode=='show': print(json.dumps(base.physical,indent=2)); return
        if a.mode=='idle': print(json.dumps(idle_monitor(link,right,a.duration,a.hz),indent=2)); return
        if a.mode=='apply':
            t=phys_to_tuning(base,foc_q_kp=a.foc_q_kp,foc_q_ki=a.foc_q_ki,foc_d_kp=a.foc_d_kp,foc_d_ki=a.foc_d_ki,
                speed_kp=a.speed_kp,speed_ki=a.speed_ki,speed_kd=a.speed_kd,pos_kp=a.pos_kp,pos_ki=a.pos_ki,pos_kd=a.pos_kd,telemetry_filter=a.filter)
            print(json.dumps(link.set_tuning(t,right,store=a.store).physical,indent=2)); return
        if not a.arm: raise SystemExit('active test requires --arm')
        guard=(a.max_erpm,a.max_current,a.max_duty)
        if a.mode=='step':
            pt=int(round(a.target)) if a.loop=='position' else None
            rows,m=capture_step(link,right,a.loop,a.target,a.duration,a.hz,guard,pt); m['score']=score(m)
            p=write_run(outdir,f'{a.motor}_{a.loop}_step',rows,m); print(json.dumps(m,indent=2)); print(*p); return
        if a.mode=='sweep':
            cand=parse_candidates(a.candidates,a.loop); targets=[float(x) for x in (a.targets or str(a.target)).split(',')]
            allrows=[]; results=[]
            for c in cand:
                t=base
                if a.loop=='speed': t=phys_to_tuning(base,speed_kp=c[0],speed_ki=c[1],speed_kd=c[2])
                elif a.loop=='position': t=phys_to_tuning(base,pos_kp=c[0],pos_ki=c[1],pos_kd=c[2])
                elif a.loop=='currentq': t=phys_to_tuning(base,foc_q_kp=c[0],foc_q_ki=c[1])
                else: t=phys_to_tuning(base,foc_d_kp=c[0],foc_d_ki=c[1])
                link.set_tuning(t,right,store=False); time.sleep(.15)
                for target in targets:
                    rows,m=capture_step(link,right,a.loop,target,a.duration,a.hz,guard,int(round(target)) if a.loop=='position' else None)
                    m.update(candidate=c,target_command=target,score=score(m)); results.append(m)
                    for r in rows:r.update(candidate=str(c),target_command=target); allrows.extend(rows)
                    print(json.dumps(m,sort_keys=True))
            best=min(results,key=lambda z:z['score']) if results else {}; link.set_tuning(base,right,store=False)
            p=write_run(outdir,f'{a.motor}_{a.loop}_sweep',allrows,dict(best=best,results=results)); print('BEST',json.dumps(best,indent=2)); print(*p); return
        if a.mode=='max-erpm':
            step=max(50,abs(int(a.target))); rows=[]; results=[]
            for target in list(range(step,int(a.max_erpm)+1,step))+list(range(-step,-int(a.max_erpm)-1,-step)):
                r,m=capture_step(link,right,'speed',target,a.duration,a.hz,guard); m['target_command']=target; results.append(m)
                for x in r:x['target_command']=target; rows.extend(r)
                print(json.dumps(m,sort_keys=True))
                if m.get('abort') or m.get('peak_duty',0)>.88: break
            p=write_run(outdir,f'{a.motor}_max_erpm',rows,dict(results=results)); print(*p)
    finally:
        try: release(link,right)
        except Exception: pass
        link.close()
if __name__=='__main__': main()
