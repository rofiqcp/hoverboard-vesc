#!/usr/bin/env python3
import sys
print("AUTOCATCH_DISABLED: production firmware must remain reachable by normal SWD; no automated connect-under-reset is permitted.", file=sys.stderr)
raise SystemExit(2)
