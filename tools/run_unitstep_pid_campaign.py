#!/usr/bin/env python3
import csv,json,math,statistics,sys,time,threading
from dataclasses import replace
from pathlib import Path
ROOT=Path('/home/otomasi/agv')
TOOLS=ROOT/'hoverboard-vesc/tools'; sys.path.insert(0,str(TOOLS))
from vesc_dual import VescDual,Tuning
PORT='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
OUTROOT=ROOT/'data/esc'
POS_KD_PROC_FIXED=0.00035
SPEED_RAMP=12000.0
SAMPLE_DT=0.04
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
# Ten repeat trials of the same full-range simultaneous unit step.
# LEFT: VESC position 0 -> 360 deg (calibrated steering safe span).
# RIGHT: speed 0 -> +8000 ERPM.
CASES=[(i,360.0,8000) for i in range(1,11)]
# Five outer-loop candidates. Every variation changes all six requested PID gains.
# FOC current-loop gains remain exactly as stored in EEPROM from qualified R/L/flux.
CANDS=[
 dict(name='v1',speed=(.0016,.0014,.00001),pos=(.20,.002,.001)),
 dict(name='v2',speed=(.0018,.0017,.00002),pos=(.23,.004,.002)),
 dict(name='v3',speed=(.0020,.0020,.00003),pos=(.26,.008,.003)),
 dict(name='v4',speed=(.0022,.0023,.00004),pos=(.28,.010,.004)),
 dict(name='v5',speed=(.0024,.0026,.00005),pos=(.30,.012,.005)),
]

def q(v,s): return max(0,min(65535,round(v*s)))
def build_tune(orig,speed,pos):
    # Preserve qualified current-loop D/Q gains; tune outer loops only.
    return Tuning(orig.kpq,orig.kiq,orig.kpd,orig.kid,
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
        self.tempL=0.0; self.tempR=0.0; self.abort=''; self.rows=[]; self.tripL=0; self.tripR=0
        self._pump_stop=None; self._pump_thread=None; self._pump_target=(180.0,0); self._pump_error=''
        cfg=self.link.terminal('config',True)
        if 'inv=1' not in cfg: raise RuntimeError('RIGHT EEPROM inversion is not active')
        self.link.terminal(f'set speed_ramp {SPEED_RAMP}',True)
    def _pump_loop(self):
        # Keep actuator watchdog independent from slow telemetry/CSV transactions.
        # VescDual.send_no_reply deliberately bypasses io_lock and only serializes TX frames.
        next_t=time.monotonic()
        while self._pump_stop is not None and not self._pump_stop.is_set():
            pos,rpm=self._pump_target
            try:
                self.link.set_pos_one(pos,False)
                self.link.set_rpm_one(rpm,True)
            except Exception as e:
                self._pump_error=f'{type(e).__name__}: {e}'
                self.abort=f'setpoint pump error {self._pump_error}'
                break
            next_t+=0.020
            wait=next_t-time.monotonic()
            if wait>0:self._pump_stop.wait(wait)
            else:next_t=time.monotonic()
    def pump_set(self,pos,rpm):
        self._pump_target=(float(pos),int(rpm))
        if self._pump_thread is None or not self._pump_thread.is_alive():
            self._pump_error=''; self._pump_stop=threading.Event()
            self._pump_thread=threading.Thread(target=self._pump_loop,name='vesc-setpoint-pump',daemon=True)
            self._pump_thread.start()
    def pump_stop(self):
        if self._pump_stop is not None:self._pump_stop.set()
        if self._pump_thread is not None:self._pump_thread.join(timeout=.4)
        self._pump_stop=None; self._pump_thread=None
    def stop_all(self):
        self.pump_stop()
        for right in (False,True):
            try:self.link.terminal('stop',right)
            except Exception:pass
        time.sleep(.35)
    def move_left(self,target,timeout=3.0,tol=3.0):
        self.pump_set(target,0)
        t=time.monotonic(); good=0
        while time.monotonic()-t<timeout:
            vl=self.link.values(False); vr=self.link.values(True)
            if vl.fault or vr.fault: raise RuntimeError(f'baseline fault L={vl.fault} R={vr.fault}')
            if min(vl.vin,vr.vin)<VOLTAGE_MIN: raise RuntimeError(f'baseline vbus low {min(vl.vin,vr.vin):.1f}')
            if max(abs(vl.current_motor),abs(vl.iq))>CURRENT_ABORT_A: raise RuntimeError(f'baseline steering current high {vl.current_motor:.2f}/{vl.iq:.2f}')
            good = good+1 if abs(vl.position-target)<=tol and abs(vr.rpm)<=150 else 0
            if good>=4:return
            time.sleep(.04)
        raise RuntimeError(f'LEFT failed to settle target={target} actual={vl.position:.2f}')
    def baseline(self):
        self.stop_all(); self.move_left(0.0,timeout=3.5,tol=15.0); time.sleep(.15)
    def recover_center(self):
        try:self.move_left(180.0,timeout=3.0,tol=4.0)
        except Exception as e: print('RECOVER_CENTER_WARN',repr(e),flush=True)
        self.stop_all(); time.sleep(.25)
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
        if dl and dl.phase_trip_count is not None and dl.phase_trip_count!=self.tripL:self.abort=f'LEFT phase trip {dl.phase_trip_count}'
        if dr and dr.phase_trip_count is not None and dr.phase_trip_count!=self.tripR:self.abort=f'RIGHT phase trip {dr.phase_trip_count}'
        return vl,vr
    def run_one(self,test,var,pos_target,rpm_target,cand):
        self.abort=''; self.rows=[]
        tL=build_tune(self.origL,cand['speed'],cand['pos'])
        tR=build_tune(self.origR,cand['speed'],cand['pos'])
        self.link.set_tuning(tL,False,store=False); self.link.set_tuning(tR,True,store=False)
        self.link.terminal(f'set speed_ramp {SPEED_RAMP}',True)
        self.baseline(); self.temps()
        dl0=self.link.diag(False); dr0=self.link.diag(True); self.tripL=dl0.phase_trip_count or 0; self.tripR=dr0.phase_trip_count or 0
        t0=time.monotonic(); idx=0
        self.pump_set(0.0,0)
        end=time.monotonic()+0.45
        while time.monotonic()<end:
            self.sample_pair(test,var,t0,'baseline',0.0,0,idx,tL,tR); idx+=1
            if self.abort:break
            time.sleep(SAMPLE_DT)
        self.pump_set(pos_target,rpm_target)
        start_step=time.monotonic(); sat_since=None; saturation=False
        while time.monotonic()-start_step<STEP_HOLD:
            phase='steady' if time.monotonic()-start_step >= STEP_HOLD-STEADY_HOLD else 'step'
            vl,vr=self.sample_pair(test,var,t0,phase,pos_target,rpm_target,idx,tL,tR); idx+=1
            if abs(vr.duty)>=.95 and abs(rpm_target)>1000 and abs(vr.rpm)<.85*abs(rpm_target):
                if sat_since is None:sat_since=time.monotonic()
                if time.monotonic()-sat_since>.55:saturation=True
            else:sat_since=None
            if self.abort:break
            time.sleep(SAMPLE_DT)
        self.stop_all(); time.sleep(.20)
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
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument('--test',type=int); ap.add_argument('--var',type=int); a=ap.parse_args()
    c=Campaign(); results=[]
    try:
        for test,pos,rpm in CASES:
            if a.test and test!=a.test: continue
            for var,cand in enumerate(CANDS,1):
                if a.var and var!=a.var: continue
                print(f'BEGIN uji{test} variasi{test}_{var} pos={pos} rpm={rpm} cand={cand}',flush=True)
                try:
                    tL,tR,sat=c.run_one(test,var,pos,rpm,cand)
                    results.append(c.finish_one(test,var,pos,rpm,tL,tR,sat,cand))
                except Exception as e:
                    c.abort=f'exception {type(e).__name__}: {e}'
                    print('EXCEPTION',test,var,repr(e),flush=True)
                    try:
                        tL=build_tune(c.origL,cand['speed'],cand['pos'])
                        tR=build_tune(c.origR,cand['speed'],cand['pos'])
                        results.append(c.finish_one(test,var,pos,rpm,tL,tR,False,cand))
                    except Exception:pass
                    c.stop_all(); time.sleep(.8)
                save_summary(results)
                c.recover_center()
                time.sleep(.15)
        if a.test or a.var:
            print('FILTERED_RUN_COMPLETE',len(results),flush=True); return
        best,agg=aggregate_best(results)
        cand=CANDS[best-1]
        finalL=build_tune(c.origL,cand['speed'],cand['pos'])
        finalR=build_tune(c.origR,cand['speed'],cand['pos'])
        final={'best_variation':best,'aggregate_scores':agg,'candidate':cand,'left':finalL.physical,'right':finalR.physical,'persistent_write':False}
        (OUTROOT/'best_tuning_analysis.json').write_text(json.dumps(final,indent=2,sort_keys=True)+'\n')
        print('BEST_ANALYSIS_ONLY',json.dumps(final,sort_keys=True),flush=True)
    finally:
        c.stop_all(); c.link.close()
if __name__=='__main__': main()
