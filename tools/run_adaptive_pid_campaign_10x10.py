#!/usr/bin/env python3
import csv, json, math, statistics, struct, sys, threading, time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from pathlib import Path

ROOT=Path('/home/otomasi/agv')
TOOLS=ROOT/'hoverboard-vesc/tools'
sys.path.insert(0,str(TOOLS))
from vesc_dual import VescDual, Tuning, Values, COMM_GET_VALUES

PORT='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
OUTROOT=ROOT/'data/esc'
SUMMARY=OUTROOT/'campaign_10x10_summary.csv'
STATE=OUTROOT/'campaign_10x10_state.json'
BESTJSON=OUTROOT/'best_tuning_10x10.json'
SPEED_RAMP=12000.0
TX_PERIOD=0.020
SAMPLE_PERIOD=0.040
CURRENT_ABORT_A=7.0
VBUS_MIN=35.0
RIGHT_RPM_ABORT=9000.0
LEFT_MIN=0.0
LEFT_MAX=360.0

# Each round increases excitation while keeping all 10 variations in that round comparable.
# (position_deg, right_erpm, hold_seconds)
ROUND_PROFILES={
  1:[(180,0,.35),(240,2000,.95),(180,0,.80),(120,1000,.90),(180,0,.80)],
  2:[(180,0,.35),(270,3000,1.00),(180,1000,.90),(90,2000,1.00),(180,0,.80)],
  3:[(180,0,.35),(300,4000,1.10),(180,2000,.95),(60,3000,1.05),(180,0,.85)],
  4:[(180,0,.35),(330,5000,1.20),(180,2500,1.00),(30,3500,1.10),(180,0,.90)],
  5:[(180,0,.35),(360,6000,1.30),(240,3000,1.00),(0,4000,1.20),(180,0,.95)],
  6:[(180,0,.35),(60,6000,1.30),(300,3500,1.10),(120,1000,1.00),(180,0,.95)],
  7:[(180,0,.35),(0,7000,1.45),(300,4000,1.15),(60,2000,1.05),(180,0,1.00)],
  8:[(180,0,.35),(360,8000,1.55),(180,4500,1.20),(0,2500,1.15),(180,0,1.00)],
  9:[(180,0,.35),(0,8000,1.55),(360,5000,1.30),(120,2000,1.10),(180,0,1.00)],
 10:[(180,0,.35),(360,8000,1.60),(0,4000,1.35),(300,7000,1.45),(60,2000,1.15),(180,0,1.05)],
}

BOUNDS={
 'skp':(0.0005,0.0120),'ski':(0.0000,0.0150),'skd':(0.0000,0.0010),
 'pkp':(0.020,1.200),'pki':(0.000,0.080),'pkd':(0.000,0.080),
}
# starting point is conservative but already stronger than EEPROM position baseline
START={'skp':0.0020,'ski':0.0020,'skd':0.00000,'pkp':0.200,'pki':0.002,'pkd':0.001}
STEP0={'skp':0.00035,'ski':0.00040,'skd':0.00002,'pkp':0.040,'pki':0.002,'pkd':0.001}
PRIMARY=['skp','ski','pkp','pkd','skd','pki','skp','pkp','ski','pkd']
DIRECTION=[+1,+1,+1,+1,+1,+1,-1,-1,-1,-1]

RAW_FIELDS=['round','variation','segment','sample','t_s','target','actual','error','current_motor_a','current_in_a','id_a','iq_a','vd_v','vq_v','duty','vbus_v','temp_mos_c','fault','vesc_id','other_target','other_actual','candidate_id']
SUMMARY_FIELDS=['round','variation','result','accepted','score','best_score','left_score','right_score','left_steady_error','right_steady_error','left_steady_std','right_steady_std','left_rmse','right_rmse','left_iae','right_iae','left_itae','right_itae','left_rise90','right_rise90','left_settling','right_settling','left_overshoot','right_overshoot','left_oscillation','right_oscillation','left_peak_current','right_peak_current','left_rms_current','right_rms_current','left_peak_duty','right_peak_duty','min_vbus','max_temp','fault_left','fault_right','phase_trip_delta_left','phase_trip_delta_right','dc_trip_delta_left','dc_trip_delta_right','isr_max_cycles','isr_deadline_miss','isr_miss_rate','dma_pending_exit','skp','ski','skd','pkp','pki','pkd','note','left_csv','right_csv','combined_csv','left_png','right_png']


def clamp(x,lo,hi): return min(hi,max(lo,x))
def qspeed(v): return max(0,min(65535,round(v*100000.0)))
def qpos(v): return max(0,min(65535,round(v*1000.0)))
def quantized(g):
    return {'skp':qspeed(g['skp'])/100000.0,'ski':qspeed(g['ski'])/100000.0,'skd':qspeed(g['skd'])/100000.0,
            'pkp':qpos(g['pkp'])/1000.0,'pki':qpos(g['pki'])/1000.0,'pkd':qpos(g['pkd'])/1000.0}

def build_tune(orig,g):
    g=quantized(g)
    return Tuning(orig.kpq,orig.kiq,orig.kpd,orig.kid,
                  qspeed(g['skp']),qspeed(g['ski']),qspeed(g['skd']),
                  qpos(g['pkp']),qpos(g['pki']),qpos(g['pkd']),
                  telem_filter_q16=orig.telem_filter_q16,current_limit_q4=orig.current_limit_q4)

def parse_full_values(p):
    if len(p)<74 or p[0]!=COMM_GET_VALUES: raise ValueError(f'bad COMM_GET_VALUES len={len(p)}')
    temp=struct.unpack_from('>h',p,1)[0]/10.0
    cm=struct.unpack_from('>i',p,5)[0]/100.0
    ci=struct.unpack_from('>i',p,9)[0]/100.0
    idv=struct.unpack_from('>i',p,13)[0]/100.0
    iq=struct.unpack_from('>i',p,17)[0]/100.0
    duty=struct.unpack_from('>h',p,21)[0]/1000.0
    rpm=float(struct.unpack_from('>i',p,23)[0])
    vin=struct.unpack_from('>h',p,27)[0]/10.0
    fault=p[53]
    pos=struct.unpack_from('>i',p,54)[0]/1_000_000.0
    vid=p[58]
    vd=struct.unpack_from('>i',p,65)[0]/1000.0
    vq=struct.unpack_from('>i',p,69)[0]/1000.0
    return Values(cm,ci,idv,iq,duty,rpm,vin,pos,fault,vid,vd,vq),temp

class Link(VescDual):
    EXPECTED_FW = {
        'platform_schema': 2, 'diag_schema': 4, 'isr_schema': 3, 'trace_schema': 3,
        'profile_revision': 0x00030000, 'build_id': 1372176165,
        'git_sha': '3bacd9bd3216', 'control_div': 6,
        'cpu_hz': 64_000_000, 'pwm_hz': 16_000,
    }
    def require_platform_compatible(self, require_build=True):
        x=self.platform_info()
        for k,v in self.EXPECTED_FW.items():
            if x.get(k)!=v:
                raise RuntimeError(f'TUNING_REFUSED active firmware mismatch {k}: got={x.get(k)!r} expected={v!r}')
        return x
    def values_full(self,right=False):
        req=bytes((COMM_GET_VALUES,))
        expected=2 if right else 1
        deadline=time.monotonic()+max(.35,self.timeout)
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            while True:
                rem=deadline-time.monotonic()
                if rem<=0: raise TimeoutError(f'no full values from VESC {expected}')
                p=self.recv(COMM_GET_VALUES,rem)
                v,t=parse_full_values(p)
                if v.vesc_id==expected:return v,t

@dataclass
class SegmentMetric:
    rise90:float=999.0; settling:float=999.0; overshoot:float=0.0
    steady_error:float=0.0; steady_std:float=0.0; rmse:float=0.0; iae:float=0.0; itae:float=0.0
    oscillation:float=0.0; peak_current:float=0.0; rms_current:float=0.0; peak_duty:float=0.0
    min_vbus:float=999.0; max_temp:float=0.0; score:float=999.0

def segment_metric(rows,prev_target,target,is_left):
    if len(rows)<3:return SegmentMetric(score=999.0)
    t0=rows[0]['t_rel']; ts=[r['t_rel']-t0 for r in rows]
    y=[r['actual'] for r in rows]; e=[target-v for v in y]
    y0=y[0]; amp=target-y0
    scale=max(abs(target-prev_target),30.0 if is_left else 1000.0)
    tol=max(2.0,0.015*scale) if is_left else max(120.0,0.03*max(abs(target),scale))
    rise=999.0
    if abs(amp)>1e-9:
        th=y0+0.9*amp
        for t,v in zip(ts,y):
            if (amp>0 and v>=th) or (amp<0 and v<=th):rise=t;break
    settle=999.0
    for i in range(len(rows)):
        if all(abs(x)<=tol for x in e[i:]) and len(e)-i>=2:
            settle=ts[i];break
    norm=[(v-y0)/(amp if abs(amp)>1e-9 else scale) for v in y]
    overs=max(0.0,(max(norm)-1.0)*100.0) if amp>0 else max(0.0,(max(norm)-1.0)*100.0)
    tail=max(2,len(y)//4); se=e[-tail:]; sy=y[-tail:]
    steady_err=statistics.mean(se); steady_std=statistics.pstdev(sy) if len(sy)>1 else 0.0
    rmse=math.sqrt(sum(x*x for x in e)/len(e))
    iae=0.0; itae=0.0
    for i in range(1,len(e)):
        dt=max(0.0,ts[i]-ts[i-1]); iae+=abs(e[i])*dt; itae+=ts[i]*abs(e[i])*dt
    zc=0
    for a,b in zip(e[1:],e[2:]):
        if a*b<0:zc+=1
    tv=sum(abs(y[i]-y[i-1]) for i in range(1,len(y)))
    osc=max(0.0,(tv/max(abs(amp),scale))-1.0)+0.25*zc
    cur=[abs(r['current_motor_a']) for r in rows]; duty=[abs(r['duty']) for r in rows]
    pc=max(cur); rc=math.sqrt(sum(c*c for c in cur)/len(cur)); pd=max(duty)
    minv=min(r['vbus_v'] for r in rows); maxt=max(r['temp_mos_c'] for r in rows)
    # Lower is better; normalized so position and speed contribute comparably.
    score=(30.0*abs(steady_err)/scale + 12.0*steady_std/scale + 12.0*rmse/scale +
           0.80*min(rise,3.0) + 0.90*min(settle,3.0) + 0.12*min(overs,100.0) +
           2.0*min(osc,10.0) + 0.18*pc + 2.0*max(0.0,pd-0.92))
    return SegmentMetric(rise,settle,overs,steady_err,steady_std,rmse,iae,itae,osc,pc,rc,pd,minv,maxt,score)

def aggregate_metrics(seg_metrics):
    if not seg_metrics:return SegmentMetric(score=999.0)
    valid=seg_metrics
    def mean(name):return statistics.mean(getattr(x,name) for x in valid)
    return SegmentMetric(
      rise90=mean('rise90'),settling=mean('settling'),overshoot=mean('overshoot'),steady_error=mean('steady_error'),
      steady_std=mean('steady_std'),rmse=mean('rmse'),iae=sum(x.iae for x in valid),itae=sum(x.itae for x in valid),
      oscillation=mean('oscillation'),peak_current=max(x.peak_current for x in valid),rms_current=mean('rms_current'),
      peak_duty=max(x.peak_duty for x in valid),min_vbus=min(x.min_vbus for x in valid),max_temp=max(x.max_temp for x in valid),
      score=mean('score'))

class Campaign:
    def __init__(self):
        self.link=Link(PORT,115200,timeout=.6)
        self.link.require_platform_compatible(require_build=True)
        self.origL=self.link.get_tuning(False);self.origR=self.link.get_tuning(True)
        self.link.terminal(f'set speed_ramp {SPEED_RAMP}',True)
        self.stop_evt=None;self.tx_thread=None;self.target=(180.0,0);self.tx_error=''
    def start_tx(self,pos,rpm):
        self.target=(float(pos),int(rpm))
        if self.tx_thread and self.tx_thread.is_alive():return
        self.stop_evt=threading.Event();self.tx_error=''
        def worker():
            n=0;deadline=time.monotonic()
            while not self.stop_evt.is_set():
                p,r=self.target
                try:
                    self.link.set_pos_one(p,False);self.link.set_rpm_one(r,True)
                    if n%5==0:self.link.alive(False);self.link.alive(True)
                except Exception as e:
                    self.tx_error=f'{type(e).__name__}:{e}';self.stop_evt.set();break
                n+=1;deadline+=TX_PERIOD;wait=deadline-time.monotonic()
                if wait>0:self.stop_evt.wait(wait)
                else:deadline=time.monotonic()
        self.tx_thread=threading.Thread(target=worker,daemon=True,name='vesc-standard-tx');self.tx_thread.start()
    def set_target(self,pos,rpm):self.target=(float(pos),int(rpm));self.start_tx(pos,rpm)
    def stop_tx(self):
        if self.stop_evt:self.stop_evt.set()
        if self.tx_thread:self.tx_thread.join(timeout=.5)
        self.stop_evt=None;self.tx_thread=None
    def stop_all(self):
        self.stop_tx()
        for right in (False,True):
            try:self.link.terminal('stop',right)
            except Exception:pass
        time.sleep(.15)
    def restore_original(self):
        self.stop_all()
        try:self.link.set_tuning(self.origL,False,store=False);self.link.set_tuning(self.origR,True,store=False)
        except Exception:pass
    def apply(self,g,store=False):
        tl=build_tune(self.origL,g);tr=build_tune(self.origR,g)
        rl=self.link.set_tuning(tl,False,store=store);rr=self.link.set_tuning(tr,True,store=store)
        self.link.terminal(f'set speed_ramp {SPEED_RAMP}',True)
        return rl,rr
    def acquire(self):
        # Official VESC selective values are the realtime path; they avoid the
        # ~2x larger full GET_VALUES payload while preserving all control metrics.
        l=self.link.values(False); r=self.link.values(True)
        self._temp_div=getattr(self,'_temp_div',0)+1
        if not hasattr(self,'_temp_l'): self._temp_l=self._temp_r=0.0
        if self._temp_div>=20:
            self._temp_div=0
            try:self._temp_l=self.link.setup_values(False).temp_mos
            except Exception:pass
            try:self._temp_r=self.link.setup_values(True).temp_mos
            except Exception:pass
        return l,r,self._temp_l,self._temp_r
    def preposition(self,pos=180.0,timeout=2.5):
        self.set_target(pos,0);good=0;end=time.monotonic()+timeout
        while time.monotonic()<end:
            l,r,_,_=self.acquire()
            if l.fault or r.fault:raise RuntimeError(f'preposition fault L={l.fault} R={r.fault}')
            if abs(l.current_motor)>CURRENT_ABORT_A:raise RuntimeError('preposition current high')
            good=good+1 if abs(l.position-pos)<=5.0 and abs(r.rpm)<=180 else 0
            if good>=3:return
            time.sleep(.03)
        # Do not hide a sluggish candidate; test can start and metrics will penalize it.
    def run_variation(self,round_id,var_id,g):
        g=quantized(g);self.apply(g,store=False);self.stop_all();self.preposition(180.0)
        try:p0=self.link.isr_profile(reset=False)
        except Exception:p0={}
        d0l=self.link.diag(False);d0r=self.link.diag(True)
        trip0=(d0l.phase_trip_count or 0,d0r.phase_trip_count or 0,d0l.dc_trip_count or 0,d0r.dc_trip_count or 0)
        left_rows=[];right_rows=[];combined=[];abort='';sample_n=0;t_campaign=time.monotonic()
        profile=ROUND_PROFILES[round_id];prev_pos=profile[0][0];prev_rpm=profile[0][1]
        segment_metrics_left=[];segment_metrics_right=[]
        for seg,(pos,rpm,hold) in enumerate(profile):
            self.set_target(pos,rpm);seg_start=time.monotonic();sl=[];sr=[]
            while time.monotonic()-seg_start<hold:
                if self.tx_error:abort='tx '+self.tx_error;break
                try:l,r,tl,tr=self.acquire()
                except Exception as e:abort=f'telemetry {type(e).__name__}:{e}';break
                now=time.monotonic();trel=now-seg_start
                common={'round':round_id,'variation':var_id,'segment':seg,'sample':sample_n,'t_s':now-t_campaign,'candidate_id':f'uji{round_id}_var{var_id}'}
                L=dict(common,target=float(pos),actual=l.position,error=float(pos)-l.position,current_motor_a=l.current_motor,current_in_a=l.current_in,id_a=l.id,iq_a=l.iq,vd_v=l.vd,vq_v=l.vq,duty=l.duty,vbus_v=l.vin,temp_mos_c=tl,fault=l.fault,vesc_id=l.vesc_id,other_target=float(rpm),other_actual=r.rpm,t_rel=trel)
                R=dict(common,target=float(rpm),actual=r.rpm,error=float(rpm)-r.rpm,current_motor_a=r.current_motor,current_in_a=r.current_in,id_a=r.id,iq_a=r.iq,vd_v=r.vd,vq_v=r.vq,duty=r.duty,vbus_v=r.vin,temp_mos_c=tr,fault=r.fault,vesc_id=r.vesc_id,other_target=float(pos),other_actual=l.position,t_rel=trel)
                left_rows.append(L);right_rows.append(R);sl.append(L);sr.append(R)
                combined.append({'t_s':now-t_campaign,'round':round_id,'variation':var_id,'segment':seg,'target_left_pos':pos,'actual_left_pos':l.position,'error_left':pos-l.position,'target_right_rpm':rpm,'actual_right_rpm':r.rpm,'error_right':rpm-r.rpm,'left_current_a':l.current_motor,'right_current_a':r.current_motor,'left_iq_a':l.iq,'right_iq_a':r.iq,'left_duty':l.duty,'right_duty':r.duty,'left_vbus_v':l.vin,'right_vbus_v':r.vin,'left_fault':l.fault,'right_fault':r.fault})
                sample_n+=1
                if l.fault or r.fault:abort=f'fault L={l.fault} R={r.fault}'
                elif min(l.vin,r.vin)<VBUS_MIN:abort=f'vbus low {min(l.vin,r.vin):.1f}'
                elif max(abs(l.current_motor),abs(r.current_motor),abs(l.iq),abs(r.iq))>CURRENT_ABORT_A:abort=f'current>{CURRENT_ABORT_A}A'
                elif not LEFT_MIN-1.0<=l.position<=LEFT_MAX+1.0:abort=f'left position {l.position:.2f}'
                elif abs(r.rpm)>RIGHT_RPM_ABORT:abort=f'right overspeed {r.rpm:.0f}'
                if abort:break
                elapsed=time.monotonic()-now
                if elapsed<SAMPLE_PERIOD:time.sleep(SAMPLE_PERIOD-elapsed)
            if sl:
                if seg>0:
                    segment_metrics_left.append(segment_metric(sl,prev_pos,pos,True))
                    segment_metrics_right.append(segment_metric(sr,prev_rpm,rpm,False))
            prev_pos,prev_rpm=pos,rpm
            if abort:break
        self.stop_all()
        try:d1l=self.link.diag(False);d1r=self.link.diag(True)
        except Exception:d1l=d0l;d1r=d0r
        try:p1=self.link.isr_profile(reset=False)
        except Exception:p1={}
        phase_dl=max(0,(d1l.phase_trip_count or 0)-trip0[0]);phase_dr=max(0,(d1r.phase_trip_count or 0)-trip0[1])
        dc_dl=max(0,(d1l.dc_trip_count or 0)-trip0[2]);dc_dr=max(0,(d1r.dc_trip_count or 0)-trip0[3])
        if not abort and (phase_dl or phase_dr or dc_dl or dc_dr):abort=f'current trip phase={phase_dl}/{phase_dr} dc={dc_dl}/{dc_dr}'
        ml=aggregate_metrics(segment_metrics_left);mr=aggregate_metrics(segment_metrics_right)
        miss=max(0,p1.get('deadline_miss',0)-p0.get('deadline_miss',0));steady=max(1,p1.get('steady_isr_count',0)-p0.get('steady_isr_count',0));miss_rate=miss/steady
        # Detailed profiler intentionally samples stages; reject only sustained overload >2%.
        if not abort and miss_rate>0.02:abort=f'ISR miss rate {100*miss_rate:.2f}%'
        score=0.5*ml.score+0.5*mr.score
        if abort:score+=1000.0
        if ml.peak_current>5.5 or mr.peak_current>5.5:score+=2.0*(max(ml.peak_current,mr.peak_current)-5.5)
        return dict(gains=g,left_rows=left_rows,right_rows=right_rows,combined=combined,left=ml,right=mr,abort=abort,score=score,
                    phase_dl=phase_dl,phase_dr=phase_dr,dc_dl=dc_dl,dc_dr=dc_dr,profile=p1,
                    fault_l=getattr(d1l,'fault',0),fault_r=getattr(d1r,'fault',0),profile0=p0)
    def close(self):
        self.restore_original();self.link.close()

def csv_safe_rows(rows):
    out=[]
    for r in rows:
        out.append({k:r.get(k,'') for k in RAW_FIELDS})
    return out

def plot_motor_response(rows,path,title,ylabel):
    if not rows:return
    t=[float(r['t_s']) for r in rows]
    target=[float(r['target']) for r in rows]
    actual=[float(r['actual']) for r in rows]
    err=[float(r['error']) for r in rows]
    fig,ax=plt.subplots(figsize=(11,5.6),dpi=180)
    ax.plot(t,target,label='Setpoint',linewidth=1.8)
    ax.plot(t,actual,label='Response',linewidth=1.6)
    ax.plot(t,err,label='Error',linewidth=1.0,alpha=.75)
    ax.set_xlabel('Time (s)');ax.set_ylabel(ylabel)
    ax.grid(True,alpha=.28);ax.legend(loc='best');fig.tight_layout()
    fig.savefig(path,dpi=300,bbox_inches='tight')
    plt.close(fig)

def write_result(round_id,var_id,res,accepted,best_score):
    d=OUTROOT/f'uji{round_id}';d.mkdir(parents=True,exist_ok=True)
    lp=d/f'variasi{round_id}_{var_id}_left.csv';rp=d/f'variasi{round_id}_{var_id}_right.csv';cp=d/f'variasi{round_id}_{var_id}_combined.csv'
    lpng=d/f'variasi{round_id}_{var_id}_left.png';rpng=d/f'variasi{round_id}_{var_id}_right.png'
    with lp.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=RAW_FIELDS);w.writeheader();w.writerows(csv_safe_rows(res['left_rows']))
    with rp.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=RAW_FIELDS);w.writeheader();w.writerows(csv_safe_rows(res['right_rows']))
    cfields=list(res['combined'][0].keys()) if res['combined'] else ['t_s']
    with cp.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=cfields);w.writeheader();w.writerows(res['combined'])
    plot_motor_response(res['left_rows'],lpng,f'LEFT response uji{round_id} variasi{var_id}','Position (deg)')
    plot_motor_response(res['right_rows'],rpng,f'RIGHT response uji{round_id} variasi{var_id}','Speed (ERPM)')
    L,R=res['left'],res['right'];p=res['profile'];g=res['gains']
    row=dict(round=round_id,variation=var_id,result='ABORT' if res['abort'] else 'PASS',accepted=int(accepted),score=res['score'],best_score=best_score,
      left_score=L.score,right_score=R.score,left_steady_error=L.steady_error,right_steady_error=R.steady_error,left_steady_std=L.steady_std,right_steady_std=R.steady_std,left_rmse=L.rmse,right_rmse=R.rmse,left_iae=L.iae,right_iae=R.iae,left_itae=L.itae,right_itae=R.itae,left_rise90=L.rise90,right_rise90=R.rise90,left_settling=L.settling,right_settling=R.settling,left_overshoot=L.overshoot,right_overshoot=R.overshoot,left_oscillation=L.oscillation,right_oscillation=R.oscillation,left_peak_current=L.peak_current,right_peak_current=R.peak_current,left_rms_current=L.rms_current,right_rms_current=R.rms_current,left_peak_duty=L.peak_duty,right_peak_duty=R.peak_duty,min_vbus=min(L.min_vbus,R.min_vbus),max_temp=max(L.max_temp,R.max_temp),fault_left=res['fault_l'],fault_right=res['fault_r'],phase_trip_delta_left=res['phase_dl'],phase_trip_delta_right=res['phase_dr'],dc_trip_delta_left=res['dc_dl'],dc_trip_delta_right=res['dc_dr'],isr_max_cycles=p.get('total_max',''),isr_deadline_miss=max(0,p.get('deadline_miss',0)-res.get('profile0',{}).get('deadline_miss',0)),isr_miss_rate=(max(0,p.get('deadline_miss',0)-res.get('profile0',{}).get('deadline_miss',0))/max(1,p.get('steady_isr_count',0)-res.get('profile0',{}).get('steady_isr_count',0))),dma_pending_exit=max(0,p.get('dma_tc_pending_exit',0)-res.get('profile0',{}).get('dma_tc_pending_exit',0)),skp=g['skp'],ski=g['ski'],skd=g['skd'],pkp=g['pkp'],pki=g['pki'],pkd=g['pkd'],note=res['abort'],left_csv=str(lp),right_csv=str(rp),combined_csv=str(cp),left_png=str(lpng),right_png=str(rpng))
    exists=SUMMARY.exists()
    with SUMMARY.open('a',newline='') as f:
        w=csv.DictWriter(f,fieldnames=SUMMARY_FIELDS); 
        if not exists:w.writeheader()
        w.writerow(row)
    return row

def candidate_from(best,steps,var_id):
    if var_id==1:return quantized(best.copy())
    key=PRIMARY[var_id-1];direction=DIRECTION[var_id-1]
    cand=best.copy()
    # Primary coordinate move + tiny dither of every other coefficient, so every variation is a genuinely different 6-gain set.
    keys=['skp','ski','skd','pkp','pki','pkd']
    for j,k in enumerate(keys):
        d=steps[k]*(direction if k==key else (0.12 if ((var_id+j)&1)==0 else -0.12))
        lo,hi=BOUNDS[k];cand[k]=clamp(cand[k]+d,lo,hi)
    return quantized(cand)

def quality_pass(row):
    # Final constants must be good dynamically AND realtime-safe. A low PID
    # score alone is not enough to persist gains.
    return (
        row['result']=='PASS' and int(row['fault_left'])==0 and int(row['fault_right'])==0 and
        abs(float(row['left_steady_error']))<=3.0 and abs(float(row['right_steady_error']))<=250.0 and
        float(row['left_steady_std'])<=1.5 and float(row['right_steady_std'])<=180.0 and
        float(row['left_overshoot'])<=12.0 and float(row['right_overshoot'])<=12.0 and
        float(row['left_rise90'])<=1.2 and float(row['right_rise90'])<=1.2 and
        float(row['left_settling'])<=1.8 and float(row['right_settling'])<=1.8 and
        float(row['left_oscillation'])<=0.60 and float(row['right_oscillation'])<=0.60 and
        float(row['left_peak_current'])<=CURRENT_ABORT_A and float(row['right_peak_current'])<=CURRENT_ABORT_A and
        float(row['min_vbus'])>=VBUS_MIN and float(row['max_temp'])<=75.0 and
        int(row['phase_trip_delta_left'])==0 and int(row['phase_trip_delta_right'])==0 and
        int(row['dc_trip_delta_left'])==0 and int(row['dc_trip_delta_right'])==0 and
        int(row['isr_deadline_miss'])==0 and float(row['isr_miss_rate'])==0.0
    )

def late_round_quality(history):
    # Require qualified accepted winners in three high-excitation rounds, not a
    # lucky single final run. Round 8..10 include 8k ERPM and near-full steering.
    for rd in (8,9,10):
        rr=[r for r in history if int(r['round'])==rd and int(r['accepted'])==1 and quality_pass(r)]
        if not rr:return False
    return True

def main():
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('--round',type=int);ap.add_argument('--variation',type=int);ap.add_argument('--fresh',action='store_true')
    a=ap.parse_args()
    if a.fresh:
        for p in (SUMMARY,STATE,BESTJSON):
            try:p.unlink()
            except FileNotFoundError:pass
    camp=Campaign();best=START.copy();steps=STEP0.copy();history=[]
    try:
      if STATE.exists() and not a.fresh:
        st=json.loads(STATE.read_text());best=st.get('best_gains',best);steps=st.get('steps',steps)
      rounds=[a.round] if a.round else list(range(1,11))
      for rd in rounds:
        if rd not in ROUND_PROFILES:continue
        round_best=best.copy();round_best_score=float('inf');round_best_row=None
        vars_=[a.variation] if a.variation else list(range(1,11))
        for var in vars_:
            cand=candidate_from(round_best,steps,var)
            print('BEGIN',f'uji{rd}',f'variasi{rd}_{var}',json.dumps(cand,sort_keys=True),flush=True)
            try:res=camp.run_variation(rd,var,cand)
            except Exception as e:
                res=dict(gains=cand,left_rows=[],right_rows=[],combined=[],left=SegmentMetric(),right=SegmentMetric(),abort=f'exception {type(e).__name__}:{e}',score=1999.0,phase_dl=0,phase_dr=0,dc_dl=0,dc_dr=0,profile={},profile0={},fault_l=0,fault_r=0)
                camp.stop_all()
            improved=(not res['abort']) and (round_best_score==float('inf') or res['score']<round_best_score*0.995)
            if improved:
                round_best=cand.copy();round_best_score=res['score']
                # modest expansion around a successful direction
                for k in steps:steps[k]=min(STEP0[k]*1.5,steps[k]*1.05)
            else:
                # reject regression; next candidate is generated from accepted best with a finer search radius
                key=PRIMARY[var-1];steps[key]=max(STEP0[key]*0.18,steps[key]*0.62)
            row=write_result(rd,var,res,improved,round_best_score if math.isfinite(round_best_score) else res['score'])
            if improved:round_best_row=row
            history.append(row)
            best=round_best.copy()
            STATE.write_text(json.dumps({'last_round':rd,'last_variation':var,'best_gains':best,'steps':steps,'round_best_score':round_best_score},indent=2,sort_keys=True)+'\n')
            print('RESULT',f'uji{rd}',f'var{var}',row['result'],'accepted',int(improved),'score',round(float(row['score']),4),'best',round(float(row['best_score']),4),
                  'Lerr',round(float(row['left_steady_error']),3),'Rerr',round(float(row['right_steady_error']),1),'Losc',round(float(row['left_oscillation']),3),'Rosc',round(float(row['right_oscillation']),3),
                  'Ipk',round(float(row['left_peak_current']),2),round(float(row['right_peak_current']),2),'ISR',row['isr_max_cycles'],'miss%',round(100*float(row['isr_miss_rate']),3),flush=True)
            # return to center/zero between variations; failed candidates never become next baseline
            try:camp.apply(round_best,store=False);camp.preposition(180.0,1.8);camp.stop_all()
            except Exception:camp.stop_all()
            time.sleep(.10)
        best=round_best.copy()
        print('ROUND_BEST',rd,json.dumps(best,sort_keys=True),'score',round_best_score,flush=True)
      # Re-apply final best. Persist only if the full final round produced a genuinely qualified winner.
      final_rows=[r for r in history if int(r['round'])==10 and int(r['accepted'])==1]
      final=min(final_rows,key=lambda r:float(r['score'])) if final_rows else (history[-1] if history else None)
      persisted=False
      late_ok=late_round_quality(history)
      if final and quality_pass(final) and late_ok and not a.round and not a.variation:
          camp.apply(best,store=True);persisted=True
      else:
          camp.apply(best,store=False)
      report={'best_gains':best,'persisted':persisted,'quality_pass':bool(final and quality_pass(final)),'late_round_quality':late_ok,'final_best_row':final,'summary_csv':str(SUMMARY),'state':str(STATE)}
      BESTJSON.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
      print('FINAL',json.dumps(report,sort_keys=True),flush=True)
    finally:
      camp.stop_all();camp.link.close()

if __name__=='__main__':main()
