#!/usr/bin/env python3
import csv,json,math,statistics,sys,time
from dataclasses import replace
from pathlib import Path
ROOT=Path('/home/otomasi/agv')
TOOLS=ROOT/'hoverboard-vesc/tools'; sys.path.insert(0,str(TOOLS))
from vesc_dual import VescDual,Tuning
PORT='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
OUTROOT=ROOT/'data/esc'
POS_KD_PROC_FIXED=0.00035
SPEED_RAMP=12000.0
SAMPLE_DT=0.02
STEP_HOLD=3.0
STEADY_HOLD=0.8
CURRENT_ABORT_A=7.0
VOLTAGE_MIN=35.0
FIELDS=['record_type','test','tuning','mode','segment','phase','t_s','motor','target','actual','error',
'left_pos_deg','left_erpm','right_erpm','left_motor_current_a','left_battery_current_a','left_id_a','left_iq_a','left_vd_v','left_vq_v','left_duty','left_vbus_v','left_temp_mos_c','left_fault',
'right_motor_current_a','right_battery_current_a','right_id_a','right_iq_a','right_vd_v','right_vq_v','right_duty','right_vbus_v','right_temp_mos_c','right_fault',
'left_iq_target_a','left_iq_ref_a','left_pos_count','left_pos_target_count','left_foc_isr_cycles','left_foc_isr_cycles_max','left_rx_qdrop','right_iq_target_a','right_iq_ref_a','right_foc_isr_cycles','right_foc_isr_cycles_max','right_rx_qdrop',
'left_foc_q_kp','left_foc_q_ki','left_foc_d_kp','left_foc_d_ki','left_speed_kp','left_speed_ki','left_speed_kd','left_pos_kp','left_pos_ki','left_pos_kd','left_pos_kd_proc',
'right_foc_q_kp','right_foc_q_ki','right_foc_d_kp','right_foc_d_ki','right_speed_kp','right_speed_ki','right_speed_kd','right_pos_kp','right_pos_ki','right_pos_kd','right_pos_kd_proc',
'rise90_s','settling_s','overshoot_pct','steady_mean','steady_std','steady_error','rmse','iae','peak_motor_current_a','peak_battery_current_a','peak_abs_iq_a','peak_abs_id_a','peak_abs_vq_v','peak_abs_vd_v','peak_abs_duty','min_vbus_v','max_temp_mos_c','score','result','note']
CASES=[
(1,180.0,1000),(2,180.0,-1000),(3,120.0,2000),(4,240.0,-2000),(5,60.0,4000),
(6,300.0,-4000),(7,0.0,6000),(8,360.0,-6000),(9,90.0,8000),(10,270.0,-8000)]
# Five global candidates around the NUC known-good region. Lower score wins.
CANDS=[
 dict(name='v1',focL=.90,focR=.90,speed=(.0016,.0012,0.0),pos=(.20,.002,0.0)),
 dict(name='v2',focL=.95,focR=.95,speed=(.0018,.0015,0.0),pos=(.23,.004,0.0)),
 dict(name='v3',focL=1.00,focR=1.00,speed=(.0020,.0020,0.0),pos=(.26,.008,0.0)),
 dict(name='v4',focL=1.05,focR=1.05,speed=(.0022,.0020,.00002),pos=(.28,.010,.001)),
 dict(name='v5',focL=1.10,focR=1.10,speed=(.0024,.0022,.00004),pos=(.30,.012,.001)),
]
BASE_L=dict(foc_q_kp=.7604166666666666,foc_q_ki=253.47222222222223,
            foc_d_kp=.7604166666666666,foc_d_ki=253.47222222222223)
BASE_R=dict(foc_q_kp=.83984375,foc_q_ki=279.9479166666667,
            foc_d_kp=.5598958333333334,foc_d_ki=167.96875)

def q(v,s): return max(0,min(65535,round(v*s)))
def build_tune(orig,base,fscale,speed,pos):
    return Tuning(q(base['foc_q_kp']*fscale,1536),q(base['foc_q_ki']*fscale,4.608),
                  q(base['foc_d_kp']*fscale,1536),q(base['foc_d_ki']*fscale,4.608),
                  q(speed[0],100000),q(speed[1],100000),q(speed[2],100000),
                  q(pos[0],1000),q(pos[1],1000),q(pos[2],1000),
                  telem_filter_q16=orig.telem_filter_q16,current_limit_q4=orig.current_limit_q4)

def phys_from_vesc_pos(v): return max(-30.0,min(30.0,v/6.0-30.0))
def signed_metrics(rows,target,tol,pref):
    rr=[r for r in rows if r['record_type']=='SAMPLE' and r['motor']==pref and r['phase'] in ('step','steady')]
    if not rr:return {}
    ts=[float(r['t_s']) for r in rr]; ys=[float(r['actual']) for r in rr]
    y0=ys[0]; amp=target-y0; rise=None
    if abs(amp)>1e-6:
        th=y0+.9*amp
        for t,y in zip(ts,ys):
            if (amp>0 and y>=th) or (amp<0 and y<=th): rise=t; break
    settle=None
    for i,r in enumerate(rr):
        t=float(r['t_s'])
        if t+0.45>ts[-1]: break
        win=[q for q in rr[i:] if float(q['t_s'])<=t+0.45]
        if win and all(abs(float(q['error']))<=tol for q in win): settle=t; break
    steady=rr[-min(15,len(rr)):]
    act=[float(r['actual']) for r in steady]; err=[float(r['error']) for r in rr]
    excursion=[(y-y0)/(amp if abs(amp)>1e-9 else 1.0) for y in ys]
    overs=max(0.0,(max(excursion)-1.0)*100.0)
    rmse=math.sqrt(sum(e*e for e in err)/len(err))
    iae=sum(abs(err[i])*(ts[i]-ts[i-1]) for i in range(1,len(err)))
    side='left_' if pref=='LEFT' else 'right_'
    def mx(k): return max(abs(float(r[k])) for r in rr)
    m=dict(rise90_s=rise if rise is not None else 999.0,
           settling_s=settle if settle is not None else 999.0,
           overshoot_pct=overs,steady_mean=statistics.mean(act),
           steady_std=statistics.pstdev(act) if len(act)>1 else 0.0,
           steady_error=target-statistics.mean(act),rmse=rmse,iae=iae,
           peak_motor_current_a=mx(side+'motor_current_a'),
           peak_battery_current_a=mx(side+'battery_current_a'),
           peak_abs_iq_a=mx(side+'iq_a'),peak_abs_id_a=mx(side+'id_a'),
           peak_abs_vq_v=mx(side+'vq_v'),peak_abs_vd_v=mx(side+'vd_v'),
           peak_abs_duty=mx(side+'duty'),
           min_vbus_v=min(float(r[side+'vbus_v']) for r in rr),
           max_temp_mos_c=max(float(r[side+'temp_mos_c'] or 0) for r in rr))
    scale=max(abs(amp),1.0)
    m['score']=(2.5*abs(m['steady_error'])/scale*100 + 1.2*m['overshoot_pct'] +
                0.7*m['steady_std']/scale*100 + .25*min(m['settling_s'],20) +
                .20*m['peak_motor_current_a'])
    return m

def emptyrow(): return {k:'' for k in FIELDS}
class Campaign:
    def __init__(self):
        self.link=VescDual(PORT,115200,timeout=1.0)
        self.origL=self.link.get_tuning(False); self.origR=self.link.get_tuning(True)
        self.tempL=0.0; self.tempR=0.0; self.abort=''; self.rows=[]
        cfg=self.link.terminal('config',True)
        if 'inv=1' not in cfg: raise RuntimeError('RIGHT EEPROM inversion is not active')
        self.link.terminal(f'set speed_ramp {SPEED_RAMP}',True)
    def stop_all(self):
        for right in (False,True):
            try:self.link.terminal('stop',right)
            except Exception:pass
        time.sleep(.35)
    def center(self):
        self.stop_all(); t=time.monotonic()
        while time.monotonic()-t<1.6:
            self.link.set_pos_one(180.0,False); self.link.set_rpm_one(0,True)
            v=self.link.values(False)
            if abs(phys_from_vesc_pos(v.position))<0.8: break
            time.sleep(.04)
        self.stop_all(); time.sleep(.35)
    def temps(self):
        try:self.tempL=self.link.setup_values(False).temp_mos
        except Exception:pass
        try:self.tempR=self.link.setup_values(True).temp_mos
        except Exception:pass
    def sample_pair(self,test,var,t0,phase,pos_target,rpm_target,idx,tL,tR):
        vl=self.link.values(False); vr=self.link.values(True)
        dl=dr=None
        if idx%4==0:
            try:dl=self.link.diag(False); dr=self.link.diag(True)
            except Exception:pass
        if idx%16==0:self.temps()
        now=time.monotonic()-t0; pL=tL.physical; pR=tR.physical
        pos_actual=vl.position; common=dict(left_pos_deg=phys_from_vesc_pos(vl.position),
          left_erpm=vl.rpm,right_erpm=vr.rpm,left_motor_current_a=vl.current_motor,
          left_battery_current_a=vl.current_in,left_id_a=vl.id,left_iq_a=vl.iq,left_vd_v=vl.vd,
          left_vq_v=vl.vq,left_duty=vl.duty,left_vbus_v=vl.vin,left_temp_mos_c=self.tempL,left_fault=vl.fault,
          right_motor_current_a=vr.current_motor,right_battery_current_a=vr.current_in,right_id_a=vr.id,
          right_iq_a=vr.iq,right_vd_v=vr.vd,right_vq_v=vr.vq,right_duty=vr.duty,right_vbus_v=vr.vin,
          right_temp_mos_c=self.tempR,right_fault=vr.fault,left_pos_kd_proc=POS_KD_PROC_FIXED,right_pos_kd_proc=POS_KD_PROC_FIXED)
        if dl: common.update(left_iq_target_a=dl.iq_target_a,left_iq_ref_a=dl.iq_ref_a,left_pos_count=dl.position,
          left_pos_target_count=dl.position_target,left_foc_isr_cycles=dl.foc_isr_cycles,left_foc_isr_cycles_max=dl.foc_isr_cycles_max,left_rx_qdrop=dl.rx_queue_drops)
        if dr: common.update(right_iq_target_a=dr.iq_target_a,right_iq_ref_a=dr.iq_ref_a,right_foc_isr_cycles=dr.foc_isr_cycles,
          right_foc_isr_cycles_max=dr.foc_isr_cycles_max,right_rx_qdrop=dr.rx_queue_drops)
        for pre,p in [('left_',pL),('right_',pR)]:
            for k in ('foc_q_kp','foc_q_ki','foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','pos_kp','pos_ki','pos_kd'):
                common[pre+k]=p[k]
        for motor,target,actual in [('LEFT',pos_target,pos_actual),('RIGHT',rpm_target,vr.rpm)]:
            r=emptyrow(); r.update(common); r.update(record_type='SAMPLE',test=f'uji{test}',tuning=f'variasi{test}_{var}',
                mode='combined_step',segment=f'pos{pos_target:g}_rpm{rpm_target:+d}',phase=phase,t_s=f'{now:.6f}',
                motor=motor,target=target,actual=actual,error=target-actual)
            self.rows.append(r)
        if vl.fault or vr.fault:self.abort=f'fault L={vl.fault} R={vr.fault}'
        if min(vl.vin,vr.vin)<VOLTAGE_MIN:self.abort=f'vbus low {min(vl.vin,vr.vin):.1f}'
        if max(abs(vl.current_motor),abs(vr.current_motor),abs(vl.iq),abs(vr.iq))>CURRENT_ABORT_A:
            self.abort=f'current>{CURRENT_ABORT_A}A L={vl.current_motor:.2f}/{vl.iq:.2f} R={vr.current_motor:.2f}/{vr.iq:.2f}'
        if not 0.0<=vl.position<=360.0:self.abort=f'LEFT pos out of range {vl.position:.2f}'
        if abs(vr.rpm)>9000:self.abort=f'RIGHT overspeed {vr.rpm:.0f}'
        return vl,vr
    def run_one(self,test,var,pos_target,rpm_target,cand):
        self.abort=''; self.rows=[]
        tL=build_tune(self.origL,BASE_L,cand['focL'],cand['speed'],cand['pos'])
        tR=build_tune(self.origR,BASE_R,cand['focR'],cand['speed'],cand['pos'])
        self.link.set_tuning(tL,False,store=False); self.link.set_tuning(tR,True,store=False)
        self.link.terminal(f'set speed_ramp {SPEED_RAMP}',True)
        self.center(); self.temps()
        t0=time.monotonic(); idx=0
        end=time.monotonic()+0.45
        while time.monotonic()<end:
            self.sample_pair(test,var,t0,'baseline',180.0,0,idx,tL,tR); idx+=1
            time.sleep(SAMPLE_DT)
        start_step=time.monotonic(); sat_since=None; saturation=False
        while time.monotonic()-start_step<STEP_HOLD:
            self.link.set_pos_one(pos_target,False); self.link.set_rpm_one(rpm_target,True)
            vl,vr=self.sample_pair(test,var,t0,'step',pos_target,rpm_target,idx,tL,tR); idx+=1
            if abs(vr.duty)>=.95 and abs(rpm_target)>1000 and abs(vr.rpm)<.85*abs(rpm_target):
                if sat_since is None:sat_since=time.monotonic()
                if time.monotonic()-sat_since>.55:saturation=True
            else:sat_since=None
            if self.abort:break
            time.sleep(SAMPLE_DT)
        self.stop_all(); time.sleep(.35)
        return tL,tR,saturation
    def finish_one(self,test,var,pos_target,rpm_target,tL,tR,saturation,cand):
        ml=signed_metrics(self.rows,pos_target,6.0,'LEFT')
        mr=signed_metrics(self.rows,rpm_target,max(100.0,.03*abs(rpm_target)),'RIGHT')
        result='ABORT' if self.abort else ('SATURATION' if saturation else 'PASS')
        note=self.abort or ('voltage/duty saturation' if saturation else '')
        sums=[]
        for motor,target,m in [('LEFT',pos_target,ml),('RIGHT',rpm_target,mr)]:
            r=emptyrow(); r.update(record_type='SUMMARY',test=f'uji{test}',tuning=f'variasi{test}_{var}',mode='combined_step',
                segment=f'pos{pos_target:g}_rpm{rpm_target:+d}',phase='summary',motor=motor,target=target,result=result,note=note)
            r.update(m); sums.append(r)
        comb=(float(ml.get('score',999))+float(mr.get('score',999)))
        if saturation:comb+=200.0
        if self.abort:comb+=1000.0
        p=emptyrow(); p.update(record_type='PARAMETERS',test=f'uji{test}',tuning=f'variasi{test}_{var}',mode='combined_step',phase='parameters',
            score=comb,result=result,note=json.dumps({'candidate':cand,'left':tL.physical,'right':tR.physical,'speed_ramp':SPEED_RAMP,'status_note':note},sort_keys=True))
        out=OUTROOT/f'uji{test}'/f'variasi{test}_{var}.csv'; out.parent.mkdir(parents=True,exist_ok=True)
        with out.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader();w.writerows(self.rows);w.writerows(sums);w.writerow(p)
        print('RESULT',test,var,result,'score',round(comb,3),'L',ml.get('steady_error'),'R',mr.get('steady_error'),'Ipk',ml.get('peak_motor_current_a'),mr.get('peak_motor_current_a'),'file',out,flush=True)
        return dict(test=test,var=var,result=result,note=note,score=comb,left=ml,right=mr,candidate=cand,file=str(out))
    def restore(self):
        try:self.stop_all()
        except Exception:pass
        try:self.link.set_tuning(self.origL,False,store=False); self.link.set_tuning(self.origR,True,store=False)
        except Exception:pass
    def close(self):
        self.restore(); self.link.close()

def save_summary(results):
    p=OUTROOT/'campaign_summary.csv'; p.parent.mkdir(parents=True,exist_ok=True)
    fields=['test','var','result','score','left_score','right_score','left_sse','right_sse','left_rise','right_rise','left_settle','right_settle','left_overshoot','right_overshoot','left_peak_current','right_peak_current','note','file','candidate']
    with p.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for r in results:
            L=r['left'];R=r['right'];w.writerow(dict(test=r['test'],var=r['var'],result=r['result'],score=r['score'],
                left_score=L.get('score'),right_score=R.get('score'),left_sse=L.get('steady_error'),right_sse=R.get('steady_error'),
                left_rise=L.get('rise90_s'),right_rise=R.get('rise90_s'),left_settle=L.get('settling_s'),right_settle=R.get('settling_s'),
                left_overshoot=L.get('overshoot_pct'),right_overshoot=R.get('overshoot_pct'),left_peak_current=L.get('peak_motor_current_a'),
                right_peak_current=R.get('peak_motor_current_a'),note=r['note'],file=r['file'],candidate=json.dumps(r['candidate'],sort_keys=True)))
    return p
def aggregate_best(results):
    by={i:[] for i in range(1,6)}
    for r in results:by[r['var']].append(r)
    agg={}
    for i,rr in by.items():
        if not rr:continue
        agg[i]=sum(r['score'] for r in rr)/len(rr)
    return min(agg,key=agg.get),agg

def main():
    c=Campaign(); results=[]
    try:
        for test,pos,rpm in CASES:
            for var,cand in enumerate(CANDS,1):
                print(f'BEGIN uji{test} variasi{test}_{var} pos={pos} rpm={rpm} cand={cand}',flush=True)
                try:
                    tL,tR,sat=c.run_one(test,var,pos,rpm,cand)
                    results.append(c.finish_one(test,var,pos,rpm,tL,tR,sat,cand))
                except Exception as e:
                    c.abort=f'exception {type(e).__name__}: {e}'
                    print('EXCEPTION',test,var,repr(e),flush=True)
                    try:
                        tL=build_tune(c.origL,BASE_L,cand['focL'],cand['speed'],cand['pos'])
                        tR=build_tune(c.origR,BASE_R,cand['focR'],cand['speed'],cand['pos'])
                        results.append(c.finish_one(test,var,pos,rpm,tL,tR,False,cand))
                    except Exception:pass
                    c.stop_all(); time.sleep(.8)
                save_summary(results)
                time.sleep(.25)
        best,agg=aggregate_best(results)
        cand=CANDS[best-1]
        finalL=build_tune(c.origL,BASE_L,cand['focL'],cand['speed'],cand['pos'])
        finalR=build_tune(c.origR,BASE_R,cand['focR'],cand['speed'],cand['pos'])
        c.stop_all(); c.link.set_tuning(finalL,False,store=True); c.link.set_tuning(finalR,True,store=True)
        c.link.terminal('set invert 1',True); c.link.terminal('save mcconf',True)
        final={'best_variation':best,'aggregate_scores':agg,'candidate':cand,'left':finalL.physical,'right':finalR.physical,'right_invert_eeprom':1}
        (OUTROOT/'best_tuning.json').write_text(json.dumps(final,indent=2,sort_keys=True)+'\n')
        print('BEST',json.dumps(final,sort_keys=True),flush=True)
    finally:
        c.stop_all(); c.link.close()
if __name__=='__main__': main()
