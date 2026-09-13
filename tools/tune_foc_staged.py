#!/usr/bin/env python3
"""Trace-authoritative FOC bandwidth qualification. D and Q are always equal unless a future saliency model proves otherwise."""
from __future__ import annotations
import argparse,csv,json,math,statistics,time
from dataclasses import replace
from pathlib import Path
from vesc_dual import VescDual,Tuning
Q4_PER_A=800.0
def d32(a,b): return (b-a)&0xffffffff

def q(v,s): return max(0,min(65535,round(v*s)))
def tuning_from_model(orig:Tuning,r:float,l:float,tc_us:float)->Tuning:
    bw=1.0/(tc_us*1e-6); kp=l*bw; ki=r*bw
    return replace(orig,kpq=q(kp,1536),kpd=q(kp,1536),kiq=q(ki,4.608),kid=q(ki,4.608))

def metrics(rows,right,pre_a,step_a,ts):
    p='right_' if right else 'left_'; sig=[r[p+'iq_q4']/Q4_PER_A for r in rows]; cross=[r[p+'id_q4']/Q4_PER_A for r in rows]
    event=next((i for i,r in enumerate(rows) if r['event_bits']&(1<<3)),None)
    if event is None: return {"pass":False,"reason":"no exact t0 event"}
    y0=statistics.mean(sig[:max(1,event)]) if event else pre_a; amp=step_a-y0
    post=sig[event:]; pcross=cross[event:]; rise=None
    if abs(amp)>1e-6:
        th=y0+0.9*amp
        for i,y in enumerate(post):
            if (amp>0 and y>=th) or (amp<0 and y<=th): rise=i*ts; break
    norm=[(y-y0)/(amp if abs(amp)>1e-9 else 1.0) for y in post]
    # norm is already oriented by amp, so positive and negative steps share one overshoot formula.
    overs=max(0.0,(max(norm)-1.0)*100.0)
    tail=post[-max(3,min(8,len(post))):]; mean=statistics.mean(tail); sd=statistics.pstdev(tail) if len(tail)>1 else 0.0
    quality_key=p+'quality'; faults=p+'fault'; sat=sum(1 for r in rows[event:] if r[quality_key]&0x30)
    badq=sum(1 for r in rows[event:] if (r[quality_key]&0xCF)!=0xCF)
    fault=max((r[faults] for r in rows),default=0); crosspk=max((abs(x) for x in pcross),default=0.0)
    err=abs(step_a-mean); risev=rise if rise is not None else 9.99
    score=120*err+25*sd+0.35*overs+8*crosspk+3*min(risev,1.0)+100*sat+100*badq+(1000 if fault else 0)
    ok=(fault==0 and sat==0 and badq==0 and rise is not None)
    return {"pass":ok,"score":score,"rise90_s":risev,"overshoot_pct":overs,"steady_mean_a":mean,"steady_error_a":step_a-mean,
            "steady_std_a":sd,"cross_peak_a":crosspk,"saturation_samples":sat,"bad_quality_samples":badq,"fault":fault,"event_index":event}

def _gain_key(t):
    return (t.kpq,t.kiq,t.kpd,t.kid,t.kps,t.kis,t.kds,t.kpp,t.kip,t.kdp,t.telem_filter_q16)

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('port',nargs='?',default='auto'); ap.add_argument('--arm',action='store_true')
    ap.add_argument('--model',default='/home/otomasi/agv/data/esc/model_qualification_latest.json'); ap.add_argument('--motor',choices=('left','right','both'),default='both')
    ap.add_argument('--tc-us',default='600,800,1000,1250,1500'); ap.add_argument('--pre-a',type=float,default=0.25); ap.add_argument('--step-a',type=float,default=0.75)
    ap.add_argument('--pre-samples',type=int,default=8); ap.add_argument('--post-samples',type=int,default=24); ap.add_argument('--apply-best-ram',action='store_true'); ap.add_argument('--store-best',action='store_true')
    ap.add_argument('--output',default='/home/otomasi/agv/data/esc/foc_trace_tuning_latest.json'); a=ap.parse_args()
    if not a.arm: raise SystemExit('ARM_REQUIRED: synchronized current steps energize the motor')
    model=json.loads(Path(a.model).read_text())
    if not model.get('pass'): raise SystemExit('TUNING_REFUSED: motor model qualification did not PASS')
    tcs=[float(x) for x in a.tc_us.split(',') if x.strip()]
    if not tcs or any(x<=0 for x in tcs): raise SystemExit('TUNING_REFUSED: tc-us must be positive')
    if a.pre_samples<2 or a.post_samples<2 or a.pre_samples+a.post_samples>40:
        raise SystemExit('TUNING_REFUSED: pre/post samples must each be >=2 and total <=40')
    link=VescDual(a.port,115200,timeout=1.0); out={"candidates":{},"model":str(a.model)}
    choices=[]; originals={}; winners={}; success=False; persistence_started=False
    try:
        plat=link.require_platform_compatible(require_build=True); out['platform']=plat; ts=plat['control_div']/plat['pwm_hz']
        choices=[('left',False),('right',True)] if a.motor=='both' else [(a.motor,a.motor=='right')]
        originals={r:link.get_tuning(r) for _,r in choices}; best={}
        for name,right in choices:
            mm=model['motors'].get(name)
            if not mm or not mm.get('pass') or not mm.get('force_dq_equal'): raise RuntimeError(f'TUNING_REFUSED invalid {name} model')
            r=mm['r_ohm']['median']; l=mm['l_h']['median']; rows_out=[]
            for tc in tcs:
                cand=tuning_from_model(originals[right],r,l,tc)
                applied=link.set_tuning(cand,right,store=False)
                if _gain_key(applied)!=_gain_key(cand): raise RuntimeError(f'{name}: candidate readback mismatch tc={tc}')
                time.sleep(.05)
                p0=link.isr_profile(reset=True); seq=link.arm_current_step(a.pre_a,a.step_a,a.pre_samples,a.post_samples,right)
                deadline=time.monotonic()+2.0; st=None
                while time.monotonic()<deadline:
                    st=link.step_status(right)
                    if st['sequence']==seq and st['done']: break
                    time.sleep(.01)
                else: raise RuntimeError(f'{name} step timeout tc={tc}')
                if not st or not st['step_fired']: raise RuntimeError(f'{name} step aborted before t0 tc={tc}: {st}')
                actual_pre=st['pre_q4']/Q4_PER_A; actual_step=st['step_q4']/Q4_PER_A
                trace=link.download_trace(); prof=link.isr_profile(False); m=metrics(trace,right,actual_pre,actual_step,ts)
                dma_delta=d32(p0['dma_tc_pending_exit'],prof['dma_tc_pending_exit'])
                m.update(tc_us=tc,tuning=cand.physical,trace_count=len(trace),requested_pre_a=a.pre_a,requested_step_a=a.step_a,
                         actual_pre_a=actual_pre,actual_step_a=actual_step,deadline_miss=prof['deadline_miss'],dma_tc_pending_exit_delta=dma_delta,
                         slot_sequence_errors=prof['slot_sequence_errors'],isr_max_cycles=prof['total_max'])
                if prof['deadline_miss'] or dma_delta or prof['slot_sequence_errors'] or prof['total_max']>=3000:
                    m['pass']=False; m['reason']='ISR gate failed'
                rows_out.append(m); print('FOC_TRACE_CANDIDATE',name,json.dumps(m,sort_keys=True),flush=True)
                link.shutdown_safe(right); time.sleep(.08)
            good=[x for x in rows_out if x['pass']]; out['candidates'][name]=rows_out
            if not good: raise RuntimeError(f'{name}: no candidate passed trace+ISR gates')
            win=min(good,key=lambda x:x['score']); best[name]=win; print('FOC_TRACE_WINNER',name,json.dumps(win,sort_keys=True),flush=True)
            winners[right]=tuning_from_model(originals[right],r,l,win['tc_us'])

        if a.apply_best_ram or a.store_best:
            for _,right in choices:
                got=link.set_tuning(winners[right],right,store=False)
                if _gain_key(got)!=_gain_key(winners[right]): raise RuntimeError('winner RAM readback mismatch')
        else:
            for _,right in choices: link.set_tuning(originals[right],right,store=False)

        if a.store_best:
            persistence_started=True
            try:
                for _,right in choices:
                    got=link.set_tuning(winners[right],right,store=True)
                    if _gain_key(got)!=_gain_key(winners[right]): raise RuntimeError('winner persistent readback mismatch')
            except Exception:
                for _,right in choices:
                    try: link.set_tuning(originals[right],right,store=True)
                    except Exception: pass
                persistence_started=False
                raise

        expected=winners if (a.apply_best_ram or a.store_best) else originals
        for _,right in choices:
            got=link.get_tuning(right)
            if _gain_key(got)!=_gain_key(expected[right]): raise RuntimeError('final tuning verification mismatch')
        out['best']=best; out['pass']=True; out['persistent_commit']=bool(a.store_best); success=True
        dst=Path(a.output); dst.parent.mkdir(parents=True,exist_ok=True); dst.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n'); print('FOC_TRACE_TUNING_PASS',dst,flush=True)
    finally:
        if not success and originals:
            for _,right in choices:
                try: link.set_tuning(originals[right],right,store=persistence_started)
                except Exception: pass
        try:
            for _,right in choices: link.shutdown_safe(right)
        except Exception: pass
        link.close()
if __name__=='__main__': main()
