#!/usr/bin/env python3
import csv,json,math,statistics,sys,time
from dataclasses import replace
from pathlib import Path
sys.path.insert(0,'/home/otomasi/agv/hoverboard-vesc/tools')
from vesc_dual import VescDual,Tuning
PORT='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
OUT=Path('/home/otomasi/agv/data/esc/tuning_staged_20260911/foc')
TARGET_A=0.50
STEP_S=0.45
BASELINE_S=0.12
DT=0.008
ABORT_A=2.5

def q(v,s): return max(0,min(65535,round(v*s)))
def tune(orig, **kw):
    p=orig.physical
    vals=dict(kpq=orig.kpq,kiq=orig.kiq,kpd=orig.kpd,kid=orig.kid,kps=orig.kps,kis=orig.kis,kds=orig.kds,kpp=orig.kpp,kip=orig.kip,kdp=orig.kdp,telem_filter_q16=orig.telem_filter_q16,current_limit_q4=orig.current_limit_q4)
    if 'q_kp' in kw: vals['kpq']=q(kw['q_kp'],1536)
    if 'q_ki' in kw: vals['kiq']=q(kw['q_ki'],4.608)
    if 'd_kp' in kw: vals['kpd']=q(kw['d_kp'],1536)
    if 'd_ki' in kw: vals['kid']=q(kw['d_ki'],4.608)
    return Tuning(**vals)

def stop(v,right):
    try:v.terminal('stop',right)
    except Exception:pass
    time.sleep(.12)

def run_step(v,right,axis,tun,label):
    v.set_tuning(tun,right,store=False); stop(v,right)
    d0=v.diag(right); rows=[]; t0=time.monotonic()
    while time.monotonic()-t0<BASELINE_S:
        x=v.values(right); rows.append((time.monotonic()-t0,0.0,x.id,x.iq,x.vd,x.vq,x.current_motor,x.duty,x.rpm,x.vin,x.fault)); time.sleep(DT)
    ts=time.monotonic()
    if axis=='q':
        if right:v.set_current(0.0,TARGET_A)
        else:v.set_current(TARGET_A,0.0)
    else:
        v.set_id_test(TARGET_A,0.0,right)
    aborted=''
    try:
        while time.monotonic()-ts<STEP_S:
            v.alive(right)
            x=v.values(right); tgt=TARGET_A
            rows.append((time.monotonic()-t0,tgt,x.id,x.iq,x.vd,x.vq,x.current_motor,x.duty,x.rpm,x.vin,x.fault))
            if x.fault: aborted=f'fault={x.fault}'; break
            if max(abs(x.id),abs(x.iq),abs(x.current_motor))>ABORT_A: aborted=f'current>{ABORT_A}A'; break
            if x.vin<35.0: aborted=f'vin={x.vin:.1f}'; break
            time.sleep(DT)
    finally: stop(v,right)
    d1=v.diag(right)
    step=[r for r in rows if r[1]!=0.0]
    sig=[r[2] if axis=='d' else r[3] for r in step]
    cross=[r[3] if axis=='d' else r[2] for r in step]
    tt=[r[0]-BASELINE_S for r in step]
    steady=sig[-max(3,min(12,len(sig))):] if sig else []
    mean=statistics.mean(steady) if steady else 0.0
    std=statistics.pstdev(steady) if len(steady)>1 else 0.0
    err=TARGET_A-mean
    peak=max(sig) if sig else 0.0
    overs=max(0.0,(peak-TARGET_A)/TARGET_A*100.0)
    crosspk=max((abs(x) for x in cross),default=0.0)
    rise=9.99
    for t,y in zip(tt,sig):
        if y>=0.9*TARGET_A: rise=max(0.0,t); break
    trip_delta=(d1.current_trips or 0)-(d0.current_trips or 0)
    score=80*abs(err)+25*std+0.25*overs+6*crosspk+0.8*min(rise,1.0)+50*max(0,trip_delta)
    if aborted:score+=1000
    status='PASS' if not aborted and trip_delta==0 else 'ABORT'
    out=OUT/(('right' if right else 'left')+'_'+axis)/f'{label}.csv'; out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['t_s','target_a','id_a','iq_a','vd_v','vq_v','motor_current_a','duty','erpm','vin_v','fault'])
        w.writerows(rows)
        w.writerow([]); w.writerow(['SUMMARY','status','score','steady_mean_a','steady_error_a','steady_std_a','overshoot_pct','rise90_s','cross_peak_a','trip_delta','note'])
        w.writerow(['SUMMARY',status,score,mean,err,std,overs,rise,crosspk,trip_delta,aborted])
    return dict(status=status,score=score,mean=mean,error=err,std=std,overshoot=overs,rise=rise,cross_peak=crosspk,trip_delta=trip_delta,note=aborted,file=str(out),physical=tun.physical)

def sweep(v,right,axis,param,orig,base_phys,scales):
    res=[]
    for i,s in enumerate(scales,1):
        kw={param:base_phys*s}; tun=tune(orig,**kw)
        label=f'{param}_v{i}_{s:.2f}x'; print('BEGIN',('R' if right else 'L'),axis,label,kw,flush=True)
        r=run_step(v,right,axis,tun,label); r.update(var=i,scale=s,param=param,value=base_phys*s)
        res.append(r); print('RESULT',label,json.dumps({k:r[k] for k in ('status','score','mean','error','std','overshoot','rise','cross_peak','trip_delta')},sort_keys=True),flush=True)
        if r['status']!='PASS': break
    good=[r for r in res if r['status']=='PASS']
    return (min(good,key=lambda x:x['score']) if good else None),res

def main():
    OUT.mkdir(parents=True,exist_ok=True); v=VescDual(PORT,baud=115200,timeout=.8)
    allres={}; finals={}
    try:
        orig={False:v.get_tuning(False),True:v.get_tuning(True)}
        for right in (False,True):
            name='right' if right else 'left'; cur=orig[right]; p=cur.physical
            # Q: Kp then Ki
            b,r=sweep(v,right,'q','q_kp',cur,p['foc_q_kp'],[0.85,0.95,1.05,1.15,1.25]); allres[name+'_q_kp']=r
            if not b: raise RuntimeError(name+' q_kp no safe winner')
            cur=tune(cur,q_kp=b['value']); p=cur.physical
            b2,r=sweep(v,right,'q','q_ki',cur,p['foc_q_ki'],[0.75,0.90,1.00,1.10,1.25]); allres[name+'_q_ki']=r
            if not b2: raise RuntimeError(name+' q_ki no safe winner')
            cur=tune(cur,q_ki=b2['value'])
            # D: Kp then Ki
            p=cur.physical; b3,r=sweep(v,right,'d','d_kp',cur,p['foc_d_kp'],[0.85,0.95,1.05,1.15,1.25]); allres[name+'_d_kp']=r
            if not b3: raise RuntimeError(name+' d_kp no safe winner')
            cur=tune(cur,d_kp=b3['value']); p=cur.physical
            b4,r=sweep(v,right,'d','d_ki',cur,p['foc_d_ki'],[0.75,0.90,1.00,1.10,1.25]); allres[name+'_d_ki']=r
            if not b4: raise RuntimeError(name+' d_ki no safe winner')
            cur=tune(cur,d_ki=b4['value']); finals[name]=cur
            v.set_tuning(cur,right,store=False); stop(v,right)
            print('FOC_WINNER',name,json.dumps(cur.physical,sort_keys=True),flush=True)
        (OUT/'summary.json').write_text(json.dumps({'results':allres,'finals':{k:v.physical for k,v in finals.items()}},indent=2,sort_keys=True)+'\n')
        print('FOC_STAGE_PASS',json.dumps({k:v.physical for k,v in finals.items()},sort_keys=True),flush=True)
    finally:
        for r in (False,True): stop(v,r)
        v.close()
if __name__=='__main__': main()
