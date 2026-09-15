#!/usr/bin/env python3
"""Stage-1 FOC autotune: qualified R/L model -> D/Q-equal PI -> synchronized D/Q current-step qualification."""
from __future__ import annotations
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse,json,statistics,time
from dataclasses import replace
from vesc_common import u32_delta, quantize_u16, tuning_key
from vesc_dual import VescDual,Tuning

Q4_PER_A=800.0


def tuning_from_model(orig:Tuning,r:float,l:float,tc_us:float)->Tuning:
    # VESC current controller model: G(s)=1/(Ls+R), pole cancellation at 1/tc.
    bw=1.0/(tc_us*1e-6); kp=l*bw; ki=r*bw
    return replace(orig,kpq=quantize_u16(kp,1536),kpd=quantize_u16(kp,1536),kiq=quantize_u16(ki,4.608),kid=quantize_u16(ki,4.608))

def metrics(rows,right,axis,pre_a,step_a,ts):
    p='right_' if right else 'left_'
    main='iq_q4' if axis=='q' else 'id_q4'; cross='id_q4' if axis=='q' else 'iq_q4'
    sig=[r[p+main]/Q4_PER_A for r in rows]; cross_sig=[r[p+cross]/Q4_PER_A for r in rows]
    event=next((i for i,r in enumerate(rows) if r['event_bits']&(1<<3)),None)
    if event is None:return {"pass":False,"reason":"no exact t0 event","axis":axis}
    y0=statistics.mean(sig[:max(1,event)]) if event else pre_a; amp=step_a-y0
    post=sig[event:]; pcross=cross_sig[event:]
    rise=None
    if abs(amp)>1e-6:
        th=y0+0.9*amp
        for i,y in enumerate(post):
            if (amp>0 and y>=th) or (amp<0 and y<=th): rise=i*ts;break
    norm=[(y-y0)/(amp if abs(amp)>1e-9 else 1.0) for y in post]
    overs=max(0.0,(max(norm)-1.0)*100.0) if norm else 999.0
    tail=post[-max(3,min(8,len(post))):]
    mean=statistics.mean(tail) if tail else 0.0
    sd=statistics.pstdev(tail) if len(tail)>1 else 0.0
    quality_key=p+'quality'; fault_key=p+'fault'
    sat=sum(1 for r in rows[event:] if r[quality_key]&0x30)
    badq=sum(1 for r in rows[event:] if (r[quality_key]&0xCF)!=0xCF)
    fault=max((r[fault_key] for r in rows),default=0)
    crosspk=max((abs(x) for x in pcross),default=0.0)
    err=abs(step_a-mean); risev=rise if rise is not None else 9.99
    score=120*err+25*sd+0.35*overs+8*crosspk+3*min(risev,1.0)+100*sat+100*badq+(1000 if fault else 0)
    ok=(fault==0 and sat==0 and badq==0 and rise is not None)
    return {"axis":axis,"pass":ok,"score":score,"rise90_s":risev,"overshoot_pct":overs,
            "steady_mean_a":mean,"steady_error_a":step_a-mean,"steady_std_a":sd,
            "cross_peak_a":crosspk,"saturation_samples":sat,"bad_quality_samples":badq,
            "fault":fault,"event_index":event}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('port',nargs='?',default='auto');ap.add_argument('--arm',action='store_true')
    ap.add_argument('--model',default=str(TOOLS_DIR.parent.parent / 'data/esc/stage1/model_qualification.json'))
    ap.add_argument('--motor',choices=('left','right','both'),default='both')
    ap.add_argument('--tc-us',default='750,1000,1250,1500,2000')
    ap.add_argument('--pre-a',type=float,default=0.25);ap.add_argument('--step-a',type=float,default=0.75)
    ap.add_argument('--pre-samples',type=int,default=8);ap.add_argument('--post-samples',type=int,default=24)
    ap.add_argument('--repeat',type=int,default=2);ap.add_argument('--max-isr-cycles',type=int,default=4000)
    ap.add_argument('--apply-best-ram',action='store_true');ap.add_argument('--store-best',action='store_true')
    ap.add_argument('--output',default=str(TOOLS_DIR.parent.parent / 'data/esc/stage1/foc_tuning.json'));a=ap.parse_args()
    if not a.arm:raise SystemExit('ARM_REQUIRED: synchronized D/Q current steps energize the motor')
    if not 1<=a.repeat<=5:raise SystemExit('--repeat must be 1..5')
    if not 1000<=a.max_isr_cycles<=4000:raise SystemExit('--max-isr-cycles must be 1000..4000')
    if not (0.05<=abs(a.pre_a)<=3.0 and 0.05<=abs(a.step_a)<=3.0):raise SystemExit('current step must stay within +/-3 A')
    if a.pre_samples<2 or a.post_samples<2 or a.pre_samples+a.post_samples>40:raise SystemExit('invalid trace sample count')
    model=json.loads(Path(a.model).read_text())
    if not model.get('pass'):raise SystemExit('TUNING_REFUSED: motor-model qualification did not PASS')
    tcs=[float(x) for x in a.tc_us.split(',') if x.strip()]
    if not tcs or any(x<=0 for x in tcs):raise SystemExit('TUNING_REFUSED: tc-us must be positive')

    link=VescDual(a.port,921600,timeout=1.0);out={"stage":1,"method":"R/L pole cancellation + D/Q trace bandwidth sweep","model":str(a.model),"candidates":{}}
    choices=[];originals={};winners={};success=False;persistence_started=False
    try:
        plat=link.require_platform_compatible(require_build=True);out['platform']=plat
        ts=plat['control_div']/plat['pwm_hz'];min_tc_us=2.0*ts*1e6
        if any(tc+1e-9<min_tc_us for tc in tcs):raise RuntimeError(f'TUNING_REFUSED tc < 2 regulator samples ({min_tc_us:.1f} us)')
        choices=[('left',False),('right',True)] if a.motor=='both' else [(a.motor,a.motor=='right')]
        originals={r:link.get_tuning(r) for _,r in choices};best={}
        for name,right in choices:
            mm=model['motors'].get(name)
            if not mm or not mm.get('pass') or not mm.get('force_dq_equal'):raise RuntimeError(f'TUNING_REFUSED invalid {name} model')
            r=mm['r_ohm']['median'];l=mm['l_h']['median'];candidates=[]
            for tc in tcs:
                cand=tuning_from_model(originals[right],r,l,tc);applied=link.set_tuning(cand,right,store=False)
                if tuning_key(applied)!=tuning_key(cand):raise RuntimeError(f'{name}: candidate readback mismatch tc={tc}')
                traces=[]
                for axis in ('d','q'):
                    for rep in range(1,a.repeat+1):
                        time.sleep(.05);p0=link.isr_profile(reset=True)
                        seq=link.arm_current_step(a.pre_a,a.step_a,a.pre_samples,a.post_samples,right,axis=axis)
                        deadline=time.monotonic()+2.0;st=None
                        while time.monotonic()<deadline:
                            st=link.step_status(right)
                            if st['sequence']==seq and st['done']:break
                            time.sleep(.01)
                        else:raise RuntimeError(f'{name} {axis}-axis step timeout tc={tc} rep={rep}')
                        if not st or not st['step_fired']:raise RuntimeError(f'{name} {axis}-axis aborted tc={tc}: {st}')
                        actual_pre=st['pre_q4']/Q4_PER_A;actual_step=st['step_q4']/Q4_PER_A
                        trace=link.download_trace();prof=link.isr_profile(False)
                        m=metrics(trace,right,axis,actual_pre,actual_step,ts);dma_delta=u32_delta(p0['dma_tc_pending_exit'],prof['dma_tc_pending_exit'])
                        m.update(repeat=rep,tc_us=tc,trace_count=len(trace),requested_pre_a=a.pre_a,requested_step_a=a.step_a,
                                 actual_pre_a=actual_pre,actual_step_a=actual_step,deadline_miss=prof['deadline_miss'],
                                 dma_tc_pending_exit_delta=dma_delta,slot_sequence_errors=prof['slot_sequence_errors'],isr_max_cycles=prof['total_max'])
                        if prof['deadline_miss'] or dma_delta or prof['slot_sequence_errors'] or prof['total_max']>=a.max_isr_cycles:
                            m['pass']=False;m['reason']='ISR/DMA timing gate failed'
                        traces.append(m);print('FOC_STAGE1_TRACE',name,json.dumps(m,sort_keys=True),flush=True)
                        link.shutdown_safe(right);time.sleep(.08)
                valid=all(x['pass'] for x in traces)
                scores=[x['score'] for x in traces];agg=max(scores)+0.25*statistics.mean(scores)
                row={"tc_us":tc,"kp_v_per_a":l/(tc*1e-6),"ki_v_per_as":r/(tc*1e-6),"tuning":cand.physical,
                     "pass":valid,"score":agg,"traces":traces}
                candidates.append(row);print('FOC_STAGE1_CANDIDATE',name,json.dumps({k:v for k,v in row.items() if k!='traces'},sort_keys=True),flush=True)
            good=[x for x in candidates if x['pass']];out['candidates'][name]=candidates
            if not good:raise RuntimeError(f'{name}: no D/Q candidate passed all trace/timing gates')
            win=min(good,key=lambda x:x['score']);best[name]=win
            winners[right]=tuning_from_model(originals[right],r,l,win['tc_us'])
            print('FOC_STAGE1_WINNER',name,json.dumps({k:v for k,v in win.items() if k!='traces'},sort_keys=True),flush=True)

        if a.apply_best_ram or a.store_best:
            for _,right in choices:
                got=link.set_tuning(winners[right],right,store=False)
                if tuning_key(got)!=tuning_key(winners[right]):raise RuntimeError('winner RAM readback mismatch')
        else:
            for _,right in choices:link.set_tuning(originals[right],right,store=False)
        if a.store_best:
            persistence_started=True
            try:
                for _,right in choices:
                    got=link.set_tuning(winners[right],right,store=True)
                    if tuning_key(got)!=tuning_key(winners[right]):raise RuntimeError('winner persistent readback mismatch')
            except Exception:
                for _,right in choices:
                    try:link.set_tuning(originals[right],right,store=True)
                    except Exception:pass
                persistence_started=False;raise
        expected=winners if (a.apply_best_ram or a.store_best) else originals
        for _,right in choices:
            if tuning_key(link.get_tuning(right))!=tuning_key(expected[right]):raise RuntimeError('final tuning verification mismatch')
        out['best']=best;out['pass']=True;out['persistent_commit']=bool(a.store_best);out['dq_equal']=True
        out['note']='D/Q gains remain equal because Stage-1 identification does not prove saliency.'
        dst=Path(a.output);dst.parent.mkdir(parents=True,exist_ok=True);dst.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
        success=True;print('FOC_STAGE1_PASS',dst,flush=True)
    finally:
        if not success and originals:
            for _,right in choices:
                try:link.set_tuning(originals[right],right,store=persistence_started)
                except Exception:pass
        try:
            for _,right in choices:link.shutdown_safe(right)
        except Exception:pass
        link.close()
if __name__=='__main__':main()
