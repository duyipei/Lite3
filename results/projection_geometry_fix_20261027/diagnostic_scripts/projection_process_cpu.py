import os,time,json,sys
from pathlib import Path
pids=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  args=path.read_bytes().split(b'\0')
  if any(a.endswith(b'/voa_costmap_bridge.py') for a in args):pids.append(int(path.parent.name))
 except (OSError,ValueError):pass
if len(pids)!=1:raise RuntimeError('expected one bridge: '+repr(pids))
pid=pids[0]
def ticks():
 s=Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
 return int(s[11])+int(s[12])
a=ticks();t=time.monotonic();time.sleep(10);elapsed=time.monotonic()-t
report={'pid':pid,'window_s':elapsed,'single_core_cpu_percent':100*(ticks()-a)/os.sysconf('SC_CLK_TCK')/elapsed,'threads':len(list(Path('/proc/%d/task'%pid).iterdir()))}
Path(sys.argv[1]).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
