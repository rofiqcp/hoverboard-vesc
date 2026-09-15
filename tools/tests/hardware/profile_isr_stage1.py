#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse, time
from vesc_common import u32_delta
from vesc_dual import VescDual, parse_fw
CPU_HZ=64_000_000
PWM_HZ=16_000
PROFILE_REV=0x00030000


def main():
    ap=argparse.ArgumentParser(description='Read-only Stage-1 ISR profiler acceptance')
    ap.add_argument('port',nargs='?',default='auto')
    ap.add_argument('--baud',type=int,default=921600)
    ap.add_argument('--seconds',type=float,default=2.0)
    a=ap.parse_args()
    link=VescDual(a.port,a.baud,timeout=1.5)
    try:
        print('FW',parse_fw(link.fw(False)))
        base=link.isr_profile(reset=True)
        time.sleep(max(0.1,a.seconds))
        p=link.isr_profile(reset=False)
        de=u32_delta(base['irq_entry'],p['irq_entry'])
        dx=u32_delta(base['irq_exit'],p['irq_exit'])
        dc=u32_delta(base['snapshot_dwt'],p['snapshot_dwt'])
        freq=(de*CPU_HZ/dc) if dc else 0.0
        active=max(1,min(6,p['active_slot_count'])); slots=[p[f'slot{i}_count'] for i in range(active)]
        dslots=[p[f'detail_slot{i}_count'] for i in range(active)]
        steady=p['steady_isr_count']
        expected_detail=steady/31.0 if steady else 0.0
        checks={
            'revision': p['profile_revision']==PROFILE_REV,
            'entry_exit': de==dx,
            'frequency': 15_800.0<=freq<=16_200.0,
            'slot_sequence': p['slot_sequence_errors']==0,
            'slot_balance': bool(slots) and max(slots)-min(slots)<=1,
            'slot_total': sum(slots)==steady,
            'detail_walk': all(x>0 for x in dslots),
            'detail_ratio': abs(p['detail_sample_count']-expected_detail)<=2.0,
            'deadline': p['deadline_miss']==0 and p['total_max']<CPU_HZ//PWM_HZ,
            'dma_backlog': u32_delta(base['dma_tc_pending_exit'],p['dma_tc_pending_exit'])==0,
        }
        print(f"ISR freq={freq:.2f} Hz entry={de} exit={dx} max={p['total_max']}/{CPU_HZ//PWM_HZ} cycles")
        print('slots',slots,'detail_slots',dslots,'detail',p['detail_sample_count'],f'expected~{expected_detail:.1f}')
        print('stages', {k:p[k] for k in ('pre_gate','pre_offset','pre_protect','left_control','right_control','sensor','pll','current','regulator','id_pi','current_circle','iq_pi','decouple_limit','svpwm','fast_hold_svpwm','duty_mag','speed_pid','position_pid')})
        print('checks',checks)
        if not all(checks.values()): raise SystemExit('ISR_PROFILE_STAGE1_HW_FAIL')
        print('ISR_PROFILE_STAGE1_HW_PASS')
    finally:
        link.close()

if __name__=='__main__': main()
