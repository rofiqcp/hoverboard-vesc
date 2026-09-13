Import('env')
from pathlib import Path
import sys
root=Path(env['PROJECT_DIR'])
sys.path.insert(0,str(root/'tools'))
from build_identity import compute
x=compute(root)
env.Append(CPPDEFINES=[('F103_BUILD_ID32',f"0x{x['build_id32']:08X}u"),('F103_GIT_SHA_HI32',f"0x{x['git_hi32']:08X}u"),('F103_GIT_SHA_LO16',f"0x{x['git_lo16']:04X}u")])
print(f"F103_BUILD_ID git={x['git_sha12']} source=0x{x['build_id32']:08X}")
