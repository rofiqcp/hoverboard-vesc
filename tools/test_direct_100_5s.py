#!/usr/bin/env python3
import csv, sys, threading, time, statistics
from datetime import datetime
sys.path.insert(0, "/home/otomasi2/forclift/hoverboard-vesc/tools")
from vesc_dual import VescDual
from vesc_tool import enc_setpoint

PORT="/dev/ttyUSB0"
BAUD=921600
TARGET=-100.0
DURATION=8.0
CMD_HZ=50.0
SAMPLE_HZ=20.0
stamp=datetime.now().strftime("%Y%m%d_%H%M%S")
OUT=f"/home/otomasi2/forclift/log/direct_100erpm_{stamp}.csv"

link=VescDual(PORT, BAUD, timeout=0.12)
rows=[]
done=threading.Event()
t0=time.monotonic()
deadline=t0+DURATION

def sender():
    pkt=enc_setpoint("rpm", TARGET)
    zero=enc_setpoint("current", 0.0)
    next_tx=t0
    while True:
        now=time.monotonic()
        if now>=deadline:
            break
        link.send_no_reply(pkt, False)
        link.send_no_reply(pkt, True)
        next_tx += 1.0/CMD_HZ
        time.sleep(max(0.0, next_tx-time.monotonic()))
    for _ in range(3):
        link.send_no_reply(zero, False)
        link.send_no_reply(zero, True)
        time.sleep(0.015)
    done.set()

th=threading.Thread(target=sender,daemon=True)
th.start()
next_sample=t0
while not done.is_set() or time.monotonic() < deadline+0.6:
    now=time.monotonic()
    phase="RUN" if now<deadline else "STOP"
    try:
        l,r=link.values_pair()
        rows.append({
            "t_s":now-t0,"phase":phase,
            "left_erpm":l.rpm,"right_erpm":r.rpm,
            "left_iq_a":l.iq,"right_iq_a":r.iq,
            "left_id_a":l.id,"right_id_a":r.id,
            "left_imot_a":l.current_motor,"right_imot_a":r.current_motor,
            "left_iin_a":l.current_in,"right_iin_a":r.current_in,
            "left_duty":l.duty,"right_duty":r.duty,
            "left_vin_v":l.vin,"right_vin_v":r.vin,
            "left_fault":l.fault,"right_fault":r.fault,
        })
    except Exception as e:
        rows.append({"t_s":now-t0,"phase":phase,"error":repr(e)})
    next_sample += 1.0/SAMPLE_HZ
    time.sleep(max(0.0,next_sample-time.monotonic()))
th.join(timeout=1.0)
fields=sorted({k for row in rows for k in row})
with open(OUT,"w",newline="",encoding="utf-8") as f:
    w=csv.DictWriter(f,fieldnames=fields)
    w.writeheader(); w.writerows(rows)
run=[x for x in rows if x.get("phase")=="RUN" and "left_erpm" in x]
print("CSV",OUT)
print("samples",len(rows),"run",len(run))
if run:
    for side in ("left","right"):
        e=[float(x[f"{side}_erpm"]) for x in run]
        iq=[float(x[f"{side}_iq_a"]) for x in run]
        print(side,
              "erpm med/min/max",round(statistics.median(e),1),round(min(e),1),round(max(e),1),
              "zero%",round(100*sum(abs(v)<1 for v in e)/len(e),1),
              "Iq min/max",round(min(iq),2),round(max(iq),2))
print("duration_command_s",round(DURATION,3))
link.close()
