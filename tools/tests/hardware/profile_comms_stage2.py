#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse, time
from vesc_common import u32_delta
from vesc_dual import VescDual, parse_fw

PROFILE_REV=0x00030000


def main():
    ap=argparse.ArgumentParser(description='Read-only Stage-2 communications/ISR stress acceptance')
    ap.add_argument('port',nargs='?',default='auto')
    ap.add_argument('--baud',type=int,default=921600)
    ap.add_argument('--seconds',type=float,default=5.0)
    ap.add_argument('--pair-hz',type=float,default=25.0,help='LEFT+RIGHT selective telemetry pairs per second')
    a=ap.parse_args()
    link=VescDual(a.port,a.baud,timeout=1.2)
    try:
        print('FW',parse_fw(link.fw(False)))
        h0=link.comms_health()
        p0=link.isr_profile(reset=True)
        start=time.monotonic(); deadline=start+max(0.5,a.seconds)
        period=1.0/max(1.0,a.pair_hz); next_t=start; pairs=0
        while time.monotonic()<deadline:
            link.values(False); link.values(True); pairs+=1
            next_t+=period
            rem=next_t-time.monotonic()
            if rem>0: time.sleep(rem)
        elapsed=time.monotonic()-start
        h1=link.comms_health(); p1=link.isr_profile(False)
        dp={k:u32_delta(p0[k],p1[k]) for k in ('deadline_miss','dma_tc_pending_exit','irq_entry','irq_exit')}
        dh={k:u32_delta(h0[k],h1[k]) for k in ('rx_queue_drop','tx_queue_drop','tx_start_fail','uart_rx_error','uart_rx_restart','uart_forced_recovery')}
        freq=(dp['irq_entry']/elapsed) if elapsed>0 else 0.0
        achieved=pairs/elapsed if elapsed>0 else 0.0
        checks={
            'profile_revision':p1['profile_revision']==PROFILE_REV,
            'irq_balance':dp['irq_entry']==dp['irq_exit'],
            'isr_frequency':15800.0<=freq<=16200.0,
            'deadline_miss':dp['deadline_miss']==0,
            'dma_backlog':dp['dma_tc_pending_exit']==0,
            'rx_drop':dh['rx_queue_drop']==0,
            'tx_drop':dh['tx_queue_drop']==0,
            'tx_start':dh['tx_start_fail']==0,
            'uart_error':dh['uart_rx_error']==0,
            'uart_restart':dh['uart_rx_restart']==0,
            'uart_forced_recovery':dh['uart_forced_recovery']==0,
            'queues_drained':h1['pending_count']==0 and h1['tx_count']<=1,
        }
        print(f'pair_rate={achieved:.2f} Hz pairs={pairs} elapsed={elapsed:.3f}s ISR={freq:.2f} Hz')
        print('delta_isr',dp)
        print('delta_comms',dh)
        print('health',{k:h1[k] for k in ('rx_queue_highwater','tx_queue_highwater','process_gap_max_ms','pending_count','tx_count','tx_active','main_vesc_max_cycles')})
        print('checks',checks)
        if not all(checks.values()): raise SystemExit('COMMS_ISR_STAGE2_HW_FAIL')
        print('COMMS_ISR_STAGE2_HW_PASS')
    finally:
        link.close()

if __name__=='__main__': main()
