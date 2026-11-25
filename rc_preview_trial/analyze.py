#!/usr/bin/python3
"""Offline analysis. All timings are bag receive times."""
import argparse
import bisect
import json
import sqlite3
import subprocess
import heapq
from deadband import AGES, DeadbandHold
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def verify_closed(dbfile):
    users = subprocess.run(["fuser", str(dbfile)], capture_output=True)
    if users.returncode != 1 or users.stdout.strip() or users.stderr.strip():
        raise RuntimeError(f"사용 중이거나 점유 확인 실패: {dbfile}")


def deadband_only(path):
    data = load(path)
    hold = DeadbandHold()
    def stream(k):
        for t,value in data.get(k,[]):
            yield t,k,value
    streams = [stream(k) for k in AGES]
    for t,k,v in heapq.merge(*streams, key=lambda item:item[0]):
        hold.update(k,t,v)
        if hold.result:
            break
    print("중립 데드밴드", hold.result or "판정 보류", f"연속 {hold.elapsed:.3f}초",
          f"판정 구간 PREVIEW {hold.samples}개")
    return hold.result=="PASS"


def load(path):
    topics = {}
    for dbfile in sorted(path.glob("*.db3")):
        verify_closed(dbfile)
        with sqlite3.connect(dbfile.resolve().as_uri()+"?mode=ro&immutable=1",uri=True) as db:
            for i,name,typ in db.execute("select id,name,type from topics"):
                if name == "/rosout":
                    continue
                cls = get_message(typ)
                samples = topics.setdefault(name,[])
                for t,b in db.execute("select timestamp,data from messages where topic_id=? order by timestamp,id",(i,)):
                    msg = deserialize_message(b,cls)
                    if name.endswith("cmd_vel"):
                        value = (msg.linear.x,msg.angular.z)
                    elif name.endswith("motor_status"):
                        value = json.loads(msg.data)
                    elif name in ("/boat/motor_preview","/rc_channels"):
                        value = list(msg.data)
                    else:
                        value = msg.data
                    samples.append((t/1e9,value))
    return {k:sorted(v,key=lambda row:row[0]) for k,v in topics.items()}


def axis(raw):
    x = max(-1,min(1,(raw-992)/(819 if raw>=992 else 818)))
    return (1 if x>=0 else -1)*max(0,(abs(x)-.08)/.92)


def target(v,w):
    l,r = v/.30-w/.60,v/.30+w/.60
    scale=max(1,abs(l)/.15,abs(r)/.15)
    high=.04 if abs(v)<1e-8 and abs(w)>1e-8 else .15
    return [max(-.04,min(high,x/scale)) for x in (l,r)]


def analyze(path):
    data=load(path)
    print("메시지 수", {k:len(v) for k,v in data.items()})
    index={k:[t for t,_ in rows] for k,rows in data.items()}
    def latest(k,t,age):
        i=bisect.bisect_right(index.get(k,[]),t)-1
        if i>=0 and t-index[k][i]<=age:
            return data[k][i][1]
    names=["전진","후진","좌회전","우회전","복합 조향","중립 데드밴드"]
    outcomes={k:[] for k in names}
    prev=None
    last_kind=None
    stable=None
    ramp=[]
    zero=[None,None]
    last_sign=[None,None]
    dwell=[[],[]]
    for t,p in data.get("/boat/motor_preview",[]):
        m=latest("/boat/motor_status",t,1.)
        rc=latest("/rc_channels",t,.30)
        cmd=latest("/boat/cmd_vel",t,.12)
        lq=latest("/rc_uplink_lq",t,1.)
        if not (m and m.get("backend")=="preview" and m.get("armed") and rc and len(rc)>=6
                and rc[4]>1400 and 800<rc[5]<1200 and cmd and lq and 0<lq<=100
                and latest("/boat/estop",t,.15) is False and len(p)==4):
            prev=last_kind=stable=None
            zero=[None,None];last_sign=[None,None]
            continue
        if prev and t-prev[0]>.12:
            prev=last_kind=stable=None
            zero=[None,None];last_sign=[None,None]
        dt=t-prev[0] if prev else 0
        v,w=cmd
        x,y=axis(rc[1]),-axis(rc[0])
        rc_v=x*(.03 if x>=0 else .009)
        kind=None
        if abs(v-rc_v)<.0008 and abs(w-y*.06)<.002:
            if abs(y)<.01 and abs(x)>.25:
                kind="전진" if x>0 else "후진"
            elif abs(x)<.01 and abs(y)>.25:
                kind="좌회전" if y>0 else "우회전"
            elif x>.25 and abs(y)>.25:
                kind="복합 조향"
            elif x==y==0 and 15<=max(abs(rc[0]-992),abs(rc[1]-992))<=64:
                kind="중립 데드밴드"
        if kind!=last_kind or prev is None or (prev and max(abs(v-prev[2][0]),abs(w-prev[2][1]))>.002):
            stable=t
        if kind and stable is not None and t-stable>=1.2:
            goal=target(v,w)
            ok=all(abs(p[i]-goal[i])<.003 and abs(p[2+i]-(1500+400*goal[i]))<1.3 for i in (0,1))
            if kind=="중립 데드밴드":
                ok = cmd==(0.,0.) and p==[0.,0.,1500.,1500.]
            outcomes[kind].append((ok,p[2:]))
        for i in (0,1):
            sign=(p[i]>1e-7)-(p[i]<-1e-7)
            if prev and sign and p[i]!=prev[1][i] and dt>0:
                ramp.append((abs(p[i]-prev[1][i]),dt))
            if sign==0:
                if zero[i] is None: zero[i]=t
            else:
                if last_sign[i] and sign!=last_sign[i]:
                    dwell[i].append(0 if zero[i] is None else t-zero[i])
                last_sign[i]=sign
                zero[i]=None
        prev=(t,p,cmd);last_kind=kind
    for kind,rows in outcomes.items():
        status="판정 보류" if len(rows)<3 else ("PASS" if all(v for v,_ in rows) else "FAIL")
        print(kind,status,"안정 표본",len(rows),"불일치",sum(not ok for ok,_ in rows),
              "목표 µs 관측 예",rows[-1][1] if rows else "-")
    # Controller dt caps at 100 ms. Receive timestamps have no per-message clock alignment.
    overs=sum(delta>.030001 for delta,dt in ramp)
    late=sum(delta>.30*dt+.003 for delta,dt in ramp) # 10 ms receive-time allowance
    print("슬루", "판정 보류" if not ramp or late and not overs else ("FAIL/누락 확인" if overs else "PASS(수신시각 허용차 10ms)"),
          "변화 표본",len(ramp),"0.030 초과",overs,"시간기준 초과",late)
    for i,d in enumerate(dwell):
        print("CH",i,"방향 반전 대기", "판정 보류" if not d else
              ("PASS(수신시각)" if min(d)>=.40 else "FAIL" if min(d)==0 else "판정 보류(발행시각 확인)"),
              "관측 중립 유지 초", [round(x,3) for x in d])


if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("bag",type=Path)
    ap.add_argument("--deadband-only",action="store_true")
    ap.add_argument("--partial",action="store_true",help="종료된 불완전 bag의 저장 표본만 분석")
    args=ap.parse_args()
    if not (args.bag/"metadata.yaml").is_file():
        if not args.partial or not list(args.bag.glob("*.db3")):
            ap.error("종료된 bag 필요")
        for dbfile in args.bag.glob("*.db3"):
            verify_closed(dbfile)
            with sqlite3.connect(dbfile.resolve().as_uri()+"?mode=ro&immutable=1",uri=True) as db:
                if db.execute("pragma integrity_check").fetchall()!=[("ok",)]:
                    ap.error("DB 무결성 오류")
        print("부분 기록: SQLite 무결성 확인, 저장된 구간에 한해 분석")

    if args.deadband_only:
        raise SystemExit(0 if deadband_only(args.bag) else 1)
    analyze(args.bag)
