#!/usr/bin/python3
"""Feed recorded manual RC frames through the installed mapping without ROS or devices."""
import json
import sys
from pathlib import Path

from analyze import load

control = Path(__file__).resolve().parents[1]/"install/kaboat_control/lib/kaboat_control"
sys.path.insert(0,str(control))
from supervisor import Supervisor

cfg=json.loads((control/"control_config.json").read_text())
s=Supervisor(cfg)
inside=[]
outside=[]
bad=0
for folder in sys.argv[1:]:
    for t,ch in load(Path(folder)).get("/rc_channels",[]):
        if len(ch)<6 or ch[4]<1400 or not 800<ch[5]<1200:
            continue
        offset=max(abs(ch[0]-992),abs(ch[1]-992))
        s.set_rc(ch,t)
        if 0<offset<=65:
            inside.append(offset)
            bad+=int(not s.neutral() or (s.v,s.w)!=(0.,0.))
        if 66<=offset<=80:
            outside.append(offset)
            bad+=int(s.neutral() or (s.v,s.w)==(0.,0.))
print("실제 프레임 → 설치 감독기 오프라인 매핑:",
      "중심 이외 내부",len(inside),"관측 편차", (min(inside),max(inside)) if inside else None,
      "경계 밖",len(outside),"불일치",bad)
sys.exit(int(bad>0 or not inside or not outside))
