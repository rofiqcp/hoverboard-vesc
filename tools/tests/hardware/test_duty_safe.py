#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse, time, statistics
from vesc_dual import VescDual, COMM_SET_DUTY
from vesc_debug import send_one, release_one

def run(port='/dev/ttyUSB0'):
    link = VescDual(port,921600, timeout=0.30)
    link.require_platform_compatible(True)
    try:
        for duty in (0.05, 0.10, 0.20):
            before = link.diag(False)
            vals = []
            reason = ''
            try:
                t0 = time.monotonic()
                while time.monotonic() - t0 < 0.35:
                    send_one(link, 'left', COMM_SET_DUTY, round(duty * 100000))
                    v = link.values(False)
                    vals.append(v)
                    if v.fault or abs(v.current_motor) > 2.6 or abs(v.rpm) > 1800:
                        reason = f'abort fault={v.fault} I={v.current_motor:.2f} erpm={v.rpm:.0f}'
                        break
                    time.sleep(0.012)
            finally:
                release_one(link, 'left')
                time.sleep(0.12)
            after = link.diag(False)
            im = [x.current_motor for x in vals]
            iq = [x.iq for x in vals]
            er = [x.rpm for x in vals]
            du = [x.duty for x in vals]
            if vals:
                print(
                    f'duty={duty:.2f} n={len(vals)} actual={du[-1]*100:.1f}% '
                    f'peakDuty={max(map(abs,du))*100:.1f}% erpm={er[-1]:.0f} '
                    f'peakERPM={max(map(abs,er)):.0f} Imean={statistics.mean(im):.3f}A '
                    f'Istd={statistics.pstdev(im):.3f}A Ipeak={max(map(abs,im)):.3f}A '
                    f'IqStd={statistics.pstdev(iq):.3f}A hallBad+={after.hall_invalid-before.hall_invalid} '
                    f'seqReject+={after.hall_sequence_rejects-before.hall_sequence_rejects} '
                    f'periodReject+={after.hall_period_rejects-before.hall_period_rejects} '
                    f'qdrop+={after.rx_queue_drops-before.rx_queue_drops} {reason}'
                )
            if reason:
                return 2
        return 0
    finally:
        link.close()

def main():
    ap = argparse.ArgumentParser(description='Low-duty guarded hardware smoke test.')
    ap.add_argument('--port', default='auto')
    ap.add_argument('--arm', action='store_true', help='required to send motor duty commands')
    a = ap.parse_args()
    if not a.arm:
        ap.error('motor actuation requires --arm')
    return run(a.port)

if __name__ == '__main__':
    raise SystemExit(main())
