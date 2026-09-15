#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse
from vesc_dual import VescDual
from vesc_debug import hall_table_valid


def dist200(a: int, b: int) -> int:
    if a >= 200 or b >= 200:
        return 200
    d = abs(int(a) - int(b))
    return min(d, 200 - d)


def test_motor(link: VescDual, name: str, right: bool,
               amps: float, repeats: int, tolerance: int) -> int:
    reference = None
    tables = []
    for run in range(1, repeats + 1):
        before = link.diag(right)
        trip0 = before.current_trips
        print(f"{name} detect run {run}/{repeats}: {amps:.2f} A")
        ok, table = link.detect_hall(amps, right)
        after = link.diag(right)
        valid, why = hall_table_valid(table)
        print(f"  table={table} protocol={ok} valid={valid} ({why}) "
              f"store_verify={after.hall_store_ok} trips={trip0}->{after.current_trips}")
        if (not ok or not valid or after.fault or
                after.current_trips != trip0 or not after.hall_store_ok or
                after.hall_table != table):
            print(f"{name} HALL_REPEAT_FAIL run={run}")
            return 2
        if reference is None:
            reference = table[:]
        else:
            delta = [dist200(table[h], reference[h]) for h in range(1, 7)]
            print(f"  delta_to_run1={delta}")
            if max(delta) > tolerance:
                print(f"{name} HALL_REPEAT_FAIL tolerance={tolerance}/200")
                return 2
        tables.append(table[:])

    final = link.diag(right)
    if final.hall_table != tables[-1] or not final.hall_store_ok:
        print(f"{name} HALL_FINAL_EEPROM_VERIFY_FAIL")
        return 2
    print(f"{name} HALL_REPEAT_PASS repeats={repeats} final={final.hall_table}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="Repeatable VESC Hall detect + EEPROM verification")
    p.add_argument("port", nargs="?", default="auto")
    p.add_argument("--motor", choices=("left", "right", "both"), default="both")
    p.add_argument("--amps", type=float, default=1.0)
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--tolerance", type=int, default=2,
                   help="maximum circular table delta in 0..199 units")
    p.add_argument("--arm", action="store_true")
    a = p.parse_args()
    if not a.arm:
        raise SystemExit("add --arm only with wheels mechanically free")
    if a.repeats < 3:
        raise SystemExit("repeats must be >=3")
    if not 0 <= a.tolerance <= 10:
        raise SystemExit("tolerance must be 0..10")

    link = VescDual(a.port,921600, timeout=25.0)
    link.require_platform_compatible(True)
    try:
        rc = 0
        if a.motor in ("left", "both"):
            rc |= test_motor(link, "LEFT", False, a.amps, a.repeats, a.tolerance)
        if a.motor in ("right", "both"):
            rc |= test_motor(link, "RIGHT", True, a.amps, a.repeats, a.tolerance)
        print("HALL_REPEAT_ALL:", "PASS" if rc == 0 else "FAIL")
        return rc
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
