#!/usr/bin/env python3
"""Sensor-aware Stage-2 autotune: Hall=>SPEED+POSITION, LEFT ABI=>steering POSITION."""
from __future__ import annotations
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,json,math,statistics,time
from dataclasses import replace
from vesc_common import u32_delta, quantize_u16, tuning_key
from vesc_dual import VescDual,Tuning

Q4_PER_A=800.0; SPEED_SCALE=100000; POS_SCALE=1000

def wrap_deg(x): return float(x)%360.0
def angle_error(target,actual): return (float(target)-float(actual)+180.0)%360.0-180.0

def apply_stage1_foc(orig:Tuning,best:dict)->Tuning:
    t=best.get('tuning',{}); req=('foc_q_kp','foc_q_ki','foc_d_kp','foc_d_ki')
    if any(k not in t for k in req): raise RuntimeError('stage1 winner missing physical FOC gains')
    return replace(orig,kpq=quantize_u16(t['foc_q_kp'],1536),kiq=quantize_u16(t['foc_q_ki'],4.608),kpd=quantize_u16(t['foc_d_kp'],1536),kid=quantize_u16(t['foc_d_ki'],4.608))

def tune_speed(base:Tuning,kp,ki,kd=0.0)->Tuning:
    return replace(base,kps=quantize_u16(kp,SPEED_SCALE),kis=quantize_u16(ki,SPEED_SCALE),kds=quantize_u16(kd,SPEED_SCALE))
def tune_pos(base:Tuning,kp,ki,kd)->Tuning:
    return replace(base,kpp=quantize_u16(kp,POS_SCALE),kip=quantize_u16(ki,POS_SCALE),kdp=quantize_u16(kd,POS_SCALE))

def relay_identify(link,mode,right,target,hyst,current_a,crossings,timeout_s,max_isr):
    p0=link.isr_profile(reset=True); seq=link.start_relay_autotune(mode,target,hyst,current_a,crossings,timeout_s,right)
    deadline=time.monotonic()+timeout_s+1.0; st=None
    while time.monotonic()<deadline:
        st=link.relay_autotune_status(right)
        if st['sequence']!=seq: raise RuntimeError(f'{mode}: relay sequence changed')
        if st['done'] and not st['active']: break
        time.sleep(.04)
    else:
        try: link.abort_relay_autotune(right)
        except Exception: pass
        raise RuntimeError(f'{mode}: relay timeout status={st}')
    prof=link.isr_profile(False); dma=u32_delta(p0['dma_tc_pending_exit'],prof['dma_tc_pending_exit'])
    if st['failed']: raise RuntimeError(f'{mode}: firmware fail={st["failed"]} status={st}')
    if st['period_count']<4 or st['maximum']<=st['minimum']: raise RuntimeError(f'{mode}: insufficient oscillation {st}')
    if prof['deadline_miss'] or dma or prof['slot_sequence_errors'] or prof['total_max']>=max_isr:
        raise RuntimeError(f'{mode}: timing gate fail isr={prof["total_max"]} miss={prof["deadline_miss"]} dma={dma}')
    return st,(st['period_sum_ms']/st['period_count'])/1000.0,(st['maximum']-st['minimum'])/2.0,prof

def speed_seed(st,pu,current_limit_a):
    amp=max((st['maximum']-st['minimum'])/2.0,1.0); d=st['relay_current_ma']/1000.0
    ku_i=4.0*d/(math.pi*amp); ku=20.0*ku_i/max(current_limit_a,0.1); kp=ku/3.2; ti=2.2*pu
    return dict(method='Tyreus-Luyben PI',ku_current_a_per_erpm=ku_i,ku=ku,pu_s=pu,kp=kp,ki=kp/max(ti,1e-6),kd=0.0)

def position_seed(st,pu,current_limit_a):
    amp_deg=max((st['maximum']-st['minimum'])/2000.0,0.05); d=st['relay_current_ma']/1000.0
    ku_i=4.0*d/(math.pi*amp_deg); ku=ku_i/max(current_limit_a,0.1); kp=ku/2.2; ti=2.2*pu; td=pu/6.3
    return dict(method='Tyreus-Luyben PID',ku_current_a_per_deg=ku_i,ku=ku,pu_s=pu,kp=kp,ki=kp/max(ti,1e-6),kd=kp*td)

def sample_speed_candidate(link,tune,right,points,hold_s,dt=.05):
    got=link.set_tuning(tune,right,store=False)
    if tuning_key(got)!=tuning_key(tune): raise RuntimeError('speed candidate readback mismatch')
    segs=[]; abort=''
    try:
        for target in points:
            rows=[]; start=time.monotonic()
            while time.monotonic()-start<hold_s:
                link.set_rpm_one(target,right); v=link.values(right); rows.append((v.rpm,v.current_motor,v.duty,v.vin,v.fault))
                if v.fault: abort=f'fault={v.fault}'; break
                if v.vin<35.0: abort=f'vbus={v.vin:.2f}'; break
                if abs(v.current_motor)>7.5: abort=f'current={v.current_motor:.2f}'; break
                if abs(v.rpm)>10000: abort=f'overspeed={v.rpm:.0f}'; break
                time.sleep(dt)
            link.set_rpm_one(0,right); time.sleep(.15)
            if abort or len(rows)<5: break
            tail=rows[-max(4,len(rows)//4):]; err=[target-r[0] for r in rows]; stead=[r[0] for r in tail]
            rmse=math.sqrt(sum(e*e for e in err)/len(err)); se=target-statistics.mean(stead); sd=statistics.pstdev(stead) if len(stead)>1 else 0.0
            y0=rows[0][0]; a=target-y0; overs=0.0
            if abs(a)>1: overs=max(0.0,(max((r[0]-y0)/a for r in rows)-1.0)*100.0)
            segs.append(dict(target=target,rmse=rmse,steady_error=se,steady_std=sd,overshoot_pct=overs,peak_current=max(abs(r[1]) for r in rows),peak_duty=max(abs(r[2]) for r in rows)))
    finally:
        try: link.shutdown_safe(right)
        except Exception: pass
    if abort:return dict(pass_=False,score=1e9,reason=abort,segments=segs)
    score=sum(2.5*abs(x['steady_error'])/max(abs(x['target']),100)+1.5*x['rmse']/max(abs(x['target']),100)+.01*x['overshoot_pct']+.8*x['steady_std']/max(abs(x['target']),100)+.02*x['peak_current'] for x in segs)
    ok=len(segs)==len(points) and all(abs(x['steady_error'])<=max(120,.08*abs(x['target'])) and x['overshoot_pct']<=35 for x in segs)
    return {'pass_':ok,'score':score,'segments':segs}

def sample_encoder_position(link,tune,points,hold_s,dt=.05):
    got=link.set_tuning(tune,False,store=False)
    if tuning_key(got)!=tuning_key(tune): raise RuntimeError('encoder position candidate readback mismatch')
    segs=[]; abort=''
    try:
        for target in points:
            rows=[]; start=time.monotonic()
            while time.monotonic()-start<hold_s:
                link.set_steering_deg(target); v=link.values(False); rows.append((v.position,v.current_motor,v.duty,v.vin,v.fault))
                if v.fault:abort=f'fault={v.fault}';break
                if v.vin<35:abort=f'vbus={v.vin:.2f}';break
                if abs(v.current_motor)>5.5:abort=f'current={v.current_motor:.2f}';break
                if abs(v.position)>31:abort=f'position={v.position:.2f}';break
                time.sleep(dt)
            if abort or len(rows)<5:break
            tail=rows[-max(4,len(rows)//4):]; err=[target-r[0] for r in rows]; stead=[r[0] for r in tail]
            rmse=math.sqrt(sum(e*e for e in err)/len(err));se=target-statistics.mean(stead);sd=statistics.pstdev(stead) if len(stead)>1 else 0.0
            y0=rows[0][0];a=target-y0;overs=0.0
            if abs(a)>.1:overs=max(0.0,(max((r[0]-y0)/a for r in rows)-1)*100)
            segs.append(dict(target=target,rmse=rmse,steady_error=se,steady_std=sd,overshoot_pct=overs,peak_current=max(abs(r[1]) for r in rows)))
    finally:
        try:link.set_steering_deg(0);time.sleep(.2);link.shutdown_safe(False)
        except Exception:pass
    if abort:return dict(pass_=False,score=1e9,reason=abort,segments=segs)
    score=sum(3*abs(x['steady_error'])/max(abs(x['target']),5)+1.7*x['rmse']/max(abs(x['target']),5)+.012*x['overshoot_pct']+x['steady_std']/max(abs(x['target']),5)+.025*x['peak_current'] for x in segs)
    ok=len(segs)==len(points) and all(abs(x['steady_error'])<=1.5 and x['overshoot_pct']<=30 for x in segs)
    return {'pass_':ok,'score':score,'segments':segs}

def sample_hall_position(link,tune,right,offsets,hold_s,dt=.05):
    got=link.set_tuning(tune,right,store=False)
    if tuning_key(got)!=tuning_key(tune): raise RuntimeError('Hall position candidate readback mismatch')
    center=wrap_deg(link.values(right).position);segs=[];abort=''
    try:
        for off in offsets:
            target=wrap_deg(center+off);rows=[];start=time.monotonic()
            while time.monotonic()-start<hold_s:
                link.set_pos_one(target,right);v=link.values(right);rows.append((v.position,v.current_motor,v.duty,v.vin,v.fault))
                if v.fault:abort=f'fault={v.fault}';break
                if v.vin<35:abort=f'vbus={v.vin:.2f}';break
                if abs(v.current_motor)>7.5:abort=f'current={v.current_motor:.2f}';break
                time.sleep(dt)
            if abort or len(rows)<5:break
            tail=rows[-max(4,len(rows)//4):];errs=[angle_error(target,r[0]) for r in rows];tailerr=[angle_error(target,r[0]) for r in tail]
            rmse=math.sqrt(sum(e*e for e in errs)/len(errs));se=statistics.mean(tailerr);sd=statistics.pstdev(tailerr) if len(tailerr)>1 else 0.0
            y0=rows[0][0];cmd=angle_error(target,y0);overs=0.0
            if abs(cmd)>.5:
                progress=[angle_error(r[0],y0) for r in rows];overs=max(0.0,(max(p/cmd for p in progress)-1)*100)
            segs.append(dict(target=target,offset=off,rmse=rmse,steady_error=se,steady_std=sd,overshoot_pct=overs,peak_current=max(abs(r[1]) for r in rows)))
    finally:
        try:link.set_pos_one(center,right);time.sleep(.2);link.shutdown_safe(right)
        except Exception:pass
    if abort:return dict(pass_=False,score=1e9,reason=abort,center_deg=center,segments=segs)
    score=sum(3*abs(x['steady_error'])/30+1.7*x['rmse']/30+.012*x['overshoot_pct']+x['steady_std']/30+.025*x['peak_current'] for x in segs)
    ok=len(segs)==len(offsets) and all(abs(x['steady_error'])<=8.0 and x['overshoot_pct']<=40 for x in segs)
    return {'pass_':ok,'score':score,'center_deg':center,'segments':segs}

def timing_gate(link,p0,max_isr):
    prof=link.isr_profile(False);dma=u32_delta(p0['dma_tc_pending_exit'],prof['dma_tc_pending_exit'])
    ok=not(prof['deadline_miss'] or dma or prof['slot_sequence_errors'] or prof['total_max']>=max_isr)
    return ok,prof,dma

def combined_role_gate(link,left_role,final,max_isr,hold=.8):
    p0=link.isr_profile(reset=True);rows=[];abort=''
    try:
        if left_role=='hall':
            for lr,rr in ((1000,1000),(3000,-3000),(-3000,3000),(0,0)):
                start=time.monotonic();last=None
                while time.monotonic()-start<hold:
                    link.set_rpm_one(lr,False);link.set_rpm_one(rr,True);vl=link.values(False);vr=link.values(True);last=(vl,vr)
                    if vl.fault or vr.fault or min(vl.vin,vr.vin)<35 or max(abs(vl.current_motor),abs(vr.current_motor))>7.5:abort='dual Hall speed safety';break
                    time.sleep(.05)
                if abort:break
                rows.append(dict(mode='speed',left_target=lr,right_target=rr,left_error=abs(lr-last[0].rpm),right_error=abs(rr-last[1].rpm)))
            link.shutdown_safe(False);link.shutdown_safe(True);time.sleep(.2)
        if not abort:
            lc=wrap_deg(link.values(False).position) if left_role=='hall' else 0.0;rc=wrap_deg(link.values(True).position)
            for lo,ro in ((-30,30),(30,-30),(0,0)):
                start=time.monotonic();last=None
                while time.monotonic()-start<hold:
                    if left_role=='hall':link.set_pos_one(wrap_deg(lc+lo),False)
                    else:link.set_steering_deg(lo/2.0)
                    link.set_pos_one(wrap_deg(rc+ro),True);vl=link.values(False);vr=link.values(True);last=(vl,vr)
                    lguard=7.5 if left_role=='hall' else 5.5
                    if vl.fault or vr.fault or min(vl.vin,vr.vin)<35 or abs(vl.current_motor)>lguard or abs(vr.current_motor)>7.5:abort='dual position safety';break
                    time.sleep(.05)
                if abort:break
                le=abs(angle_error(wrap_deg(lc+lo),last[0].position)) if left_role=='hall' else abs(lo/2.0-last[0].position)
                re=abs(angle_error(wrap_deg(rc+ro),last[1].position));rows.append(dict(mode='position',left_error=le,right_error=re))
    finally:
        for r in (False,True):
            try:link.shutdown_safe(r)
            except Exception:pass
    timing,prof,dma=timing_gate(link,p0,max_isr)
    tracking=all((x['left_error']<=max(180,.12*abs(x.get('left_target',0))) and x['right_error']<=max(180,.12*abs(x.get('right_target',0)))) if x['mode']=='speed' else (x['left_error']<=(8 if left_role=='hall' else 2) and x['right_error']<=8) for x in rows)
    expected=7 if left_role=='hall' else 3
    return dict(pass_=not abort and timing and tracking and len(rows)==expected,reason=abort,rows=rows,isr=prof,dma_pending_exit_delta=dma,timing_pass=timing,tracking_pass=tracking)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('port',nargs='?',default='auto');ap.add_argument('--arm',action='store_true')
    ap.add_argument('--stage1',default=str(TOOLS_DIR.parent.parent / 'data/esc/stage1/foc_tuning.json'));ap.add_argument('--output',default=str(TOOLS_DIR.parent.parent / 'data/esc/stage2/autotune.json'))
    ap.add_argument('--speed-target-erpm',type=int,default=1500);ap.add_argument('--speed-hysteresis-erpm',type=int,default=150);ap.add_argument('--speed-relay-a',type=float,default=.8)
    ap.add_argument('--position-hysteresis-deg',type=float,default=1.5);ap.add_argument('--position-relay-a',type=float,default=.6);ap.add_argument('--position-current-limit-a',type=float,default=5.0)
    ap.add_argument('--crossings',type=int,default=10);ap.add_argument('--relay-timeout-s',type=float,default=12.0);ap.add_argument('--candidate-hold-s',type=float,default=1.4)
    ap.add_argument('--max-isr-cycles',type=int,default=4000);ap.add_argument('--apply-best-ram',action='store_true');ap.add_argument('--store-best',action='store_true');a=ap.parse_args()
    if not a.arm:raise SystemExit('ARM_REQUIRED: stage-2 relay and validation move actuators')
    p=Path(a.stage1)
    if not p.exists():raise SystemExit(f'STAGE2_REFUSED: stage1 result not found: {p}')
    stage1=json.loads(p.read_text())
    if not stage1.get('pass') or not all(k in stage1.get('best',{}) for k in ('left','right')):raise SystemExit('STAGE2_REFUSED: stage1 FOC qualification is not PASS')
    link=VescDual(a.port,921600,timeout=1.0);out={'stage':2,'stage1':a.stage1};success=False;persistence=False;originals={}
    try:
        out['platform']=link.require_platform_compatible(require_build=True);cal=link.steering_calibration();left_role=cal['role']
        if left_role not in ('encoder','hall'):raise RuntimeError(f'LEFT unsupported sensor role {cal}')
        if left_role=='encoder' and not(cal['calibrated'] and cal['homed'] and cal['encoder_synced'] and cal['span']):raise RuntimeError(f'LEFT encoder requires valid span/home/sync {cal}')
        out['sensor_plan']={'left':left_role,'right':'hall','left_calibration':cal,'policy':'Hall=SPEED+POSITION; LEFT Encoder=POSITION'}
        originals={r:link.get_tuning(r) for r in (False,True)}
        current={False:apply_stage1_foc(originals[False],stage1['best']['left']),True:apply_stage1_foc(originals[True],stage1['best']['right'])}
        for r in (False,True):link.set_tuning(current[r],r,store=False)

        speed_sides=[False,True] if left_role=='hall' else [True];out['speed']={}
        for right in speed_sides:
            name='right' if right else 'left';st,pu,amp,prof=relay_identify(link,'speed',right,a.speed_target_erpm,a.speed_hysteresis_erpm,a.speed_relay_a,a.crossings,a.relay_timeout_s,a.max_isr_cycles)
            seed=speed_seed(st,pu,current[right].current_limit_q4/Q4_PER_A);cands=[]
            for sp,si in ((.8,.8),(1,.8),(1,1),(1.15,.9),(1.15,1.1)):
                t=tune_speed(current[right],seed['kp']*sp,seed['ki']*si,0);m=sample_speed_candidate(link,t,right,[1000,-1000,3000,-3000],a.candidate_hold_s);cands.append((m,t,sp,si))
            good=[x for x in cands if x[0]['pass_']]
            if not good:raise RuntimeError(f'{name}: no speed candidate passed')
            m,t,sp,si=min(good,key=lambda x:x[0]['score']);current[right]=t;final=sample_speed_candidate(link,t,right,[8000,-8000,3000,-3000,1000,-1000],max(1.5,a.candidate_hold_s))
            if not final['pass_']:raise RuntimeError(f'{name}: speed final failed {final}')
            out['speed'][name]={'relay':st,'pu_s':pu,'amplitude_erpm':amp,'seed':seed,'isr':prof,'best_scale':[sp,si],'best_metrics':m,'final':final,'tuning':t.physical}

        out['position']={}
        for right in (False,True):
            name='right' if right else 'left';role='hall' if right else left_role
            if role=='encoder':
                start=time.monotonic()
                while time.monotonic()-start<1.2:link.set_steering_deg(0);time.sleep(.05)
                link.shutdown_safe(False);time.sleep(.1);hyst=max(1,round(a.position_hysteresis_deg*1000));ilim=a.position_current_limit_a
            else:
                hyst=max(8000,round(a.position_hysteresis_deg*1000));ilim=current[right].current_limit_q4/Q4_PER_A
            st,pu,amp,prof=relay_identify(link,'position',right,0,hyst,a.position_relay_a,a.crossings,a.relay_timeout_s,a.max_isr_cycles);seed=position_seed(st,pu,ilim);cands=[]
            for sp,si,sd in ((.8,.5,.8),(1,.5,1),(1,1,1),(1.15,.5,1.15),(1.15,1,1.2)):
                t=tune_pos(current[right],seed['kp']*sp,seed['ki']*si,seed['kd']*sd)
                m=sample_encoder_position(link,t,[-15,0,15,0],a.candidate_hold_s) if role=='encoder' else sample_hall_position(link,t,right,[-30,0,30,0],a.candidate_hold_s)
                cands.append((m,t,sp,si,sd))
            good=[x for x in cands if x[0]['pass_']]
            if not good:raise RuntimeError(f'{name}: no {role} position candidate passed')
            m,t,sp,si,sd=min(good,key=lambda x:x[0]['score']);current[right]=t
            final=sample_encoder_position(link,t,[-25,-15,0,15,25,0],max(1.5,a.candidate_hold_s)) if role=='encoder' else sample_hall_position(link,t,right,[-60,-30,0,30,60,0],max(1.5,a.candidate_hold_s))
            if not final['pass_']:raise RuntimeError(f'{name}: {role} position final failed {final}')
            out['position'][name]={'role':role,'relay':st,'pu_s':pu,'amplitude_deg':amp/1000,'seed':seed,'isr':prof,'best_scale':[sp,si,sd],'best_metrics':m,'final':final,'tuning':t.physical}

        for r in (False,True):link.set_tuning(current[r],r,store=False)
        combo=combined_role_gate(link,left_role,current,a.max_isr_cycles);out['combined_gate']=combo
        if not combo['pass_']:raise RuntimeError(f'combined sensor-aware gate failed {combo}')
        if a.store_best:
            persistence=True
            for r in (False,True):link.set_tuning(current[r],r,store=True)
        elif not a.apply_best_ram:
            for r in (False,True):link.set_tuning(originals[r],r,store=False)
        out['final']={'left':current[False].physical,'right':current[True].physical};out['pass']=True;out['persistent_commit']=bool(a.store_best);out['ram_applied']=bool(a.apply_best_ram or a.store_best);success=True
    finally:
        for r in (False,True):
            try:link.shutdown_safe(r)
            except Exception:pass
        if not success and originals:
            for r in (False,True):
                try:link.set_tuning(originals[r],r,store=persistence)
                except Exception:pass
        link.close();dst=Path(a.output);dst.parent.mkdir(parents=True,exist_ok=True);dst.write_text(json.dumps(out,indent=2,sort_keys=True,default=str)+'\n');print('STAGE2_RESULT',dst,'PASS' if out.get('pass') else 'FAIL',flush=True)
if __name__=='__main__':main()
