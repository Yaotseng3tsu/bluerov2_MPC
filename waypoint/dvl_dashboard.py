#!/usr/bin/env python3
"""DVL A50 实时仪表盘 + 数据记录（集成脚本）。

后台连接 Water Linked DVL A50（JSON/TCP 16171）：
  - 一边把 velocity + position_local 写入 CSV（同 dvl_traj_log 格式，存 data/dvl_data）；
  - 一边跑一个本地网页，实时显示 XY 轨迹 / 速度 / 高度 / yaw / 底锁状态。

页面纯原生 JS + canvas，不依赖任何外部库/CDN，现场无网也能用。

用法：
    python dvl_dashboard.py --tag live1 --reset
    然后浏览器打开   http://localhost:8080
    Ctrl+C 停止（CSV 自动保存 + 打印汇总）

参数：
    --ip 192.168.2.95   DVL 地址
    --http-port 8080    网页端口
    --tag / --outdir    CSV 标签 / 目录
    --reset             开始前把 position_local 归零
"""
import argparse
import csv
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

# ---- 共享状态（append-only，靠索引增量拉取）----
VEL = []   # {t, vx, vy, vz, alt, fom, valid}
POS = []   # {t, x, y, z, yaw}
STATE = {"n_vel": 0, "n_valid": 0, "n_pos": 0, "t0": time.time(),
         "connected": False, "last_alt": None, "last_fom": None,
         "last_yaw": None, "last_valid": False}
LOCK = threading.Lock()

VEL_FIELDS = ["wall_time", "time_of_validity", "time_of_transmission", "time",
              "velocity_valid", "vx", "vy", "vz", "altitude", "fom", "status",
              "cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz"]
POS_FIELDS = ["wall_time", "ts", "x", "y", "z", "roll", "pitch", "yaw", "std", "status"]


def cov_flat(cov):
    try:
        return [cov[0][0], cov[0][1], cov[0][2], cov[1][1], cov[1][2], cov[2][2]]
    except Exception:
        return [None] * 6


def dvl_reader(args, stop):
    """后台线程：连 DVL，解析，写 CSV，更新共享状态。"""
    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    vel_path = os.path.join(args.outdir, f"dvl_vel_{args.tag}_{stamp}.csv")
    pos_path = os.path.join(args.outdir, f"dvl_pos_{args.tag}_{stamp}.csv")
    STATE["vel_path"] = vel_path
    STATE["pos_path"] = pos_path
    print(f"[*] 速度 CSV : {vel_path}")
    print(f"[*] 位置 CSV : {pos_path}")

    while not stop.is_set():
        try:
            sock = socket.create_connection((args.ip, args.port), timeout=5)
        except OSError as exc:
            print(f"[warn] 连不上 DVL（{exc}），3s 后重试…")
            STATE["connected"] = False
            time.sleep(3)
            continue
        sock.settimeout(2.0)
        STATE["connected"] = True
        print(f"[*] 已连接 DVL {args.ip}:{args.port}")
        if args.reset:
            try:
                sock.sendall(b'{"command":"reset_dead_reckoning"}\n')
                print("[*] 已发送 reset_dead_reckoning")
            except OSError:
                pass
            args.reset = False  # 只发一次

        fv = open(vel_path, "a", newline="", encoding="utf-8")
        fp = open(pos_path, "a", newline="", encoding="utf-8")
        wv = csv.DictWriter(fv, fieldnames=VEL_FIELDS)
        wp = csv.DictWriter(fp, fieldnames=POS_FIELDS)
        if os.path.getsize(vel_path) == 0:
            wv.writeheader()
        if os.path.getsize(pos_path) == 0:
            wp.writeheader()

        buf = b""
        try:
            while not stop.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    STATE["connected"] = False
                    break
                if not chunk:
                    STATE["connected"] = False
                    break
                STATE["connected"] = True
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        m = json.loads(line.decode("utf-8", "ignore"))
                    except json.JSONDecodeError:
                        continue
                    now = time.time() - STATE["t0"]
                    wt = f"{time.time():.3f}"
                    ty = m.get("type")

                    if ty == "velocity" or "velocity_valid" in m:
                        valid = bool(m.get("velocity_valid"))
                        cxx, cxy, cxz, cyy, cyz, czz = cov_flat(m.get("covariance"))
                        wv.writerow({
                            "wall_time": wt,
                            "time_of_validity": m.get("time_of_validity"),
                            "time_of_transmission": m.get("time_of_transmission"),
                            "time": m.get("time"), "velocity_valid": valid,
                            "vx": m.get("vx"), "vy": m.get("vy"), "vz": m.get("vz"),
                            "altitude": m.get("altitude"), "fom": m.get("fom"),
                            "status": m.get("status"),
                            "cov_xx": cxx, "cov_xy": cxy, "cov_xz": cxz,
                            "cov_yy": cyy, "cov_yz": cyz, "cov_zz": czz})
                        fv.flush()
                        with LOCK:
                            VEL.append({"t": round(now, 3),
                                        "vx": m.get("vx"), "vy": m.get("vy"),
                                        "vz": m.get("vz"), "alt": m.get("altitude"),
                                        "fom": m.get("fom"), "valid": valid})
                            STATE["n_vel"] += 1
                            if valid:
                                STATE["n_valid"] += 1
                            STATE["last_alt"] = m.get("altitude")
                            STATE["last_fom"] = m.get("fom")
                            STATE["last_valid"] = valid

                    elif ty == "position_local":
                        wp.writerow({
                            "wall_time": wt, "ts": m.get("ts"),
                            "x": m.get("x"), "y": m.get("y"), "z": m.get("z"),
                            "roll": m.get("roll"), "pitch": m.get("pitch"),
                            "yaw": m.get("yaw"), "std": m.get("std"),
                            "status": m.get("status")})
                        fp.flush()
                        with LOCK:
                            POS.append({"t": round(now, 3), "x": m.get("x"),
                                        "y": m.get("y"), "z": m.get("z"),
                                        "yaw": m.get("yaw")})
                            STATE["n_pos"] += 1
                            STATE["last_yaw"] = m.get("yaw")
        finally:
            sock.close(); fv.close(); fp.close()
    print("[*] DVL 读取线程结束")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # 静音

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if self.path.startswith("/api"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            vs = int(q.get("vsince", [0])[0])
            ps = int(q.get("psince", [0])[0])
            with LOCK:
                vel_new = VEL[vs:]
                pos_new = POS[ps:]
                elapsed = max(time.time() - STATE["t0"], 1e-6)
                stats = {
                    "connected": STATE["connected"],
                    "n_vel": STATE["n_vel"], "n_valid": STATE["n_valid"],
                    "n_pos": STATE["n_pos"],
                    "valid_pct": (100 * STATE["n_valid"] / STATE["n_vel"]) if STATE["n_vel"] else 0,
                    "rate": STATE["n_vel"] / elapsed,
                    "elapsed": elapsed,
                    "last_alt": STATE["last_alt"], "last_fom": STATE["last_fom"],
                    "last_yaw": STATE["last_yaw"], "last_valid": STATE["last_valid"],
                }
            body = json.dumps({"vel": vel_new, "pos": pos_new,
                               "vtotal": vs + len(vel_new),
                               "ptotal": ps + len(pos_new),
                               "stats": stats}).encode("utf-8")
            self._send(200, body, "application/json")
            return
        self._send(404, b"not found", "text/plain")


PAGE = r"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>DVL A50 实时仪表盘</title>
<style>
:root{--bg:#0f1720;--panel:#182430;--fg:#e6edf3;--mut:#8b98a5;--ok:#2ecc71;--bad:#e74c3c;--acc:#3498db;}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,"Segoe UI",sans-serif;background:var(--bg);color:var(--fg)}
header{padding:10px 16px;background:var(--panel);display:flex;align-items:center;gap:16px;flex-wrap:wrap}
h1{font-size:16px;margin:0}
.badge{padding:3px 10px;border-radius:12px;font-size:13px;font-weight:600}
.stat{display:flex;flex-direction:column;font-size:12px;color:var(--mut)}
.stat b{font-size:16px;color:var(--fg)}
main{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}
.card{background:var(--panel);border-radius:10px;padding:10px}
.card h2{font-size:13px;margin:0 0 6px;color:var(--mut);font-weight:600}
canvas{width:100%;height:260px;display:block;background:#0b1219;border-radius:6px}
#traj{height:360px}
.full{grid-column:1/3}
@media(max-width:760px){main{grid-template-columns:1fr}.full{grid-column:1}}
</style></head>
<body>
<header>
  <h1>DVL A50 实时仪表盘</h1>
  <span id="conn" class="badge">连接中…</span>
  <span id="lock" class="badge">--</span>
  <div class="stat">高度<b id="alt">-- m</b></div>
  <div class="stat">有效率<b id="pct">--%</b></div>
  <div class="stat">FOM<b id="fom">--</b></div>
  <div class="stat">Yaw<b id="yaw">-- °</b></div>
  <div class="stat">频率<b id="rate">-- Hz</b></div>
  <div class="stat">时长<b id="dur">-- s</b></div>
</header>
<main>
  <div class="card full"><h2>XY 轨迹（position_local，绿=起点 红=当前）</h2><canvas id="traj"></canvas></div>
  <div class="card"><h2>Body 速度 vx/vy/vz（最近 30s）</h2><canvas id="vel"></canvas></div>
  <div class="card"><h2>高度 altitude（最近 30s）</h2><canvas id="altc"></canvas></div>
</main>
<script>
let vsince=0, psince=0;
const pos=[], vel=[];
function fit(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;
  c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return x;}
function draw(){
  // 轨迹
  let c=document.getElementById('traj'),x=fit(c),W=c.clientWidth,H=c.clientHeight;
  x.clearRect(0,0,W,H);
  if(pos.length>1){
    let xs=pos.map(p=>p.x),ys=pos.map(p=>p.y);
    let minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys);
    let cx=(minx+maxx)/2,cy=(miny+maxy)/2,span=Math.max(maxx-minx,maxy-miny,1)*1.2;
    const sc=Math.min(W,H)/span;
    const TX=v=>W/2+(v-cx)*sc, TY=v=>H/2-(v-cy)*sc;
    // 网格
    x.strokeStyle='#20303f';x.lineWidth=1;
    for(let g=-5;g<=5;g++){let gx=TX(cx+g*span/6);x.beginPath();x.moveTo(gx,0);x.lineTo(gx,H);x.stroke();
      let gy=TY(cy+g*span/6);x.beginPath();x.moveTo(0,gy);x.lineTo(W,gy);x.stroke();}
    // 路径
    x.strokeStyle='#3498db';x.lineWidth=2;x.beginPath();
    pos.forEach((p,i)=>{i?x.lineTo(TX(p.x),TY(p.y)):x.moveTo(TX(p.x),TY(p.y));});x.stroke();
    // 起点/当前
    let s=pos[0],e=pos[pos.length-1];
    x.fillStyle='#2ecc71';x.beginPath();x.arc(TX(s.x),TY(s.y),6,0,7);x.fill();
    x.fillStyle='#e74c3c';x.beginPath();x.arc(TX(e.x),TY(e.y),6,0,7);x.fill();
    // 航向箭头
    if(e.yaw!=null){let a=e.yaw*Math.PI/180,L=22;
      x.strokeStyle='#e74c3c';x.lineWidth=2;x.beginPath();
      x.moveTo(TX(e.x),TY(e.y));x.lineTo(TX(e.x)+L*Math.cos(a),TY(e.y)-L*Math.sin(a));x.stroke();}
    // 比例尺文字
    x.fillStyle='#8b98a5';x.font='12px sans-serif';
    x.fillText('span≈'+span.toFixed(1)+' m',8,16);
  } else {x.fillStyle='#8b98a5';x.font='14px sans-serif';x.fillText('等待 position_local 数据…',12,24);}

  // 速度时序（最近30s）
  drawSeries('vel', vel, 30, [['vx','#3498db'],['vy','#e67e22'],['vz','#2ecc71']]);
  // 高度
  drawSeries('altc', vel.filter(d=>d.valid), 30, [['alt','#2ecc71']], true);
}
function drawSeries(id, data, win, series, autoOnly){
  let c=document.getElementById(id),x=fit(c),W=c.clientWidth,H=c.clientHeight;
  x.clearRect(0,0,W,H);
  if(!data.length){return;}
  let tmax=data[data.length-1].t, tmin=tmax-win;
  let d=data.filter(p=>p.t>=tmin);
  let vals=[];series.forEach(([k])=>d.forEach(p=>{if(p[k]!=null)vals.push(p[k]);}));
  if(!vals.length)return;
  let lo=Math.min(...vals),hi=Math.max(...vals);if(lo===hi){lo-=0.1;hi+=0.1;}
  if(!autoOnly){let m=Math.max(Math.abs(lo),Math.abs(hi));lo=-m;hi=m;}
  const pad=8;
  const TX=t=>pad+(t-tmin)/(win)*(W-2*pad);
  const TY=v=>H-pad-(v-lo)/(hi-lo)*(H-2*pad);
  // 零线
  x.strokeStyle='#2b3d4d';x.lineWidth=1;let zy=TY(0);x.beginPath();x.moveTo(0,zy);x.lineTo(W,zy);x.stroke();
  series.forEach(([k,col])=>{x.strokeStyle=col;x.lineWidth=1.6;x.beginPath();let started=false;
    d.forEach(p=>{if(p[k]==null)return;let X=TX(p.t),Y=TY(p[k]);started?x.lineTo(X,Y):x.moveTo(X,Y);started=true;});x.stroke();});
  // 图例 + 当前值
  x.font='12px sans-serif';let lx=8;
  series.forEach(([k,col])=>{let last=d.length?d[d.length-1][k]:null;
    x.fillStyle=col;x.fillText(k+(last!=null?('='+last.toFixed(3)):''),lx,14);lx+=90;});
  x.fillStyle='#8b98a5';x.fillText('['+lo.toFixed(2)+','+hi.toFixed(2)+']',W-90,14);
}
async function poll(){
  try{
    let r=await fetch('/api?vsince='+vsince+'&psince='+psince);
    let j=await r.json();
    j.vel.forEach(d=>vel.push(d)); j.pos.forEach(d=>pos.push(d));
    vsince=j.vtotal; psince=j.ptotal;
    if(vel.length>20000)vel.splice(0,vel.length-20000);
    let s=j.stats;
    let conn=document.getElementById('conn');
    conn.textContent=s.connected?'已连接':'断开';
    conn.style.background=s.connected?'#1e5e3a':'#5e1e1e';
    let lk=document.getElementById('lock');
    lk.textContent=s.last_valid?'底锁 LOCK ✅':'无底锁';
    lk.style.background=s.last_valid?'#1e5e3a':'#5e3a1e';
    document.getElementById('alt').textContent=(s.last_alt!=null?s.last_alt.toFixed(2):'--')+' m';
    document.getElementById('pct').textContent=s.valid_pct.toFixed(0)+'%';
    document.getElementById('fom').textContent=(s.last_fom!=null?s.last_fom.toFixed(4):'--');
    document.getElementById('yaw').textContent=(s.last_yaw!=null?s.last_yaw.toFixed(1):'--')+' °';
    document.getElementById('rate').textContent=s.rate.toFixed(1)+' Hz';
    document.getElementById('dur').textContent=s.elapsed.toFixed(0)+' s';
    draw();
  }catch(e){document.getElementById('conn').textContent='页面断开';}
}
setInterval(poll,300);poll();
window.addEventListener('resize',draw);
</script>
</body></html>"""


def main():
    ap = argparse.ArgumentParser(description="DVL A50 实时仪表盘 + 记录")
    ap.add_argument("--ip", default="192.168.2.95")
    ap.add_argument("--port", type=int, default=16171)
    ap.add_argument("--http-port", type=int, default=8080, dest="http_port")
    ap.add_argument("--tag", default="live")
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data", "dvl_data"))
    ap.add_argument("--reset", action="store_true")
    args = ap.parse_args()

    STATE["t0"] = time.time()
    stop = threading.Event()
    th = threading.Thread(target=dvl_reader, args=(args, stop), daemon=True)
    th.start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    url = f"http://localhost:{args.http_port}"
    print("=" * 56)
    print(f"  仪表盘已启动 →  {url}")
    print("  浏览器打开上面地址即可实时观测；Ctrl+C 停止")
    print("=" * 56)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] 停止中…")
    finally:
        stop.set()
        srv.shutdown()
        with LOCK:
            print("=" * 56)
            print("汇总")
            print(f"  时长        : {time.time()-STATE['t0']:.1f} s")
            print(f"  velocity    : {STATE['n_vel']}（有效 {STATE['n_valid']}）")
            print(f"  position    : {STATE['n_pos']}")
            print(f"  速度 CSV    : {STATE.get('vel_path')}")
            print(f"  位置 CSV    : {STATE.get('pos_path')}")


if __name__ == "__main__":
    main()
