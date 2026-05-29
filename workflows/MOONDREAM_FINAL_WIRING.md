# Moondream — Final Wiring & Verify (post-download)

Run this **after** the rover has finished the autonomous Moondream download
(`~/moondream2_local/.vendor_done` exists — see
[`BRINGUP_STATUS_AND_PLAN.md`](BRINGUP_STATUS_AND_PLAN.md) §5). Goal: point the
bridge at the local patched snapshot and prove Moondream loads + answers on the
Jetson GPU. **~2 minutes. No rover movement.**

Prereqs: laptop on a network that can reach the rover; rover powered on with the
stack/bridge available. Replace `<ROVER_IP>` throughout (rediscover per
[`ROVER_BOOTSTRAP.md`](ROVER_BOOTSTRAP.md) §0 — e.g.
`arp -an | grep 54:ef:33:9e:ea:7f`). User is `agilex`, hostname `master`.

---

## 0. Confirm reachable + download finished
```bash
ssh agilex@<ROVER_IP> 'hostname; ls -la ~/moondream2_local/.vendor_done 2>/dev/null \
  && echo "DOWNLOAD COMPLETE" || echo "NOT DONE — wait for the autonomous downloader"'
```
Stop here if it says NOT DONE.

## 1. Deploy the merged client to the rover
`perception/moondream_client.py` (with `MOONDREAM_MODEL_PATH` support) is on
`main` but the rover’s `~/RoverMind` is an rsync copy, so push it. From the Mac:
```bash
cd /Users/n.chinlue/code/RoverMind && git checkout main && git pull
rsync -av -e "ssh -o ConnectTimeout=10" \
  --exclude '.git' --exclude '.venv' --exclude '__pycache__' \
  --exclude '*.pyc' --exclude '.pytest_cache' \
  ~/code/RoverMind/ agilex@<ROVER_IP>:~/RoverMind/
```
(`--symlink-install` makes the Python change live; no rebuild needed.)

## 2. Point the env at the local snapshot
Add `MOONDREAM_MODEL_PATH` to `~/rm_env.sh` (idempotent — only adds once):
```bash
ssh agilex@<ROVER_IP> 'grep -q MOONDREAM_MODEL_PATH ~/rm_env.sh \
  || echo "export MOONDREAM_MODEL_PATH=\$HOME/moondream2_local" >> ~/rm_env.sh; \
  grep MOONDREAM_MODEL_PATH ~/rm_env.sh'
```

## 3. GPU verify — Moondream loads + answers (standalone, no ROS needed)
This is the real proof the py3.8 patch + local snapshot work on the GPU. First
load is slow (model → GPU). Use a long timeout (~300 s).
```bash
ssh agilex@<ROVER_IP> 'bash -lc "
  source ~/rm_env.sh >/dev/null 2>&1
  cd ~/RoverMind && PYTHONPATH=~/RoverMind python3.8 - <<PY
from PIL import Image
from perception.moondream_client import MoondreamClient, resolve_model_source
print(\"source:\", resolve_model_source())
m = MoondreamClient(device=\"cuda\")
print(\"ANSWER:\", m.ask(Image.new(\"RGB\", (64,64), (128,128,128)), \"What color is this image?\"))
PY"'
```
**Pass:** prints the local path from `resolve_model_source()` and an `ANSWER:`
string (no traceback). **Fail:** any `TypeError: 'type' object is not
subscriptable` → a `.py` in `~/moondream2_local` wasn’t patched (re-run
`python3.8 ~/vendor_moondream.py` to re-patch); CUDA OOM → check the GPU is free.

## 4. Restart the bridge under the updated env
So the bridge process inherits `MOONDREAM_MODEL_PATH` (it builds `MoondreamClient`
lazily on the first `capture_and_analyze`). Assumes the ROS stack is up; if not,
bring it up first (`ROVER_BOOTSTRAP.md` / `LIMO_WORKFLOW.md` §2d).
```bash
ssh agilex@<ROVER_IP> 'bash -lc "
  source ~/rm_env.sh
  pkill -f bridge/bridge_server.py 2>/dev/null; sleep 1
  cd ~/RoverMind
  setsid bash -c \"PYTHONPATH=~/RoverMind python3.8 bridge/bridge_server.py --bind 127.0.0.1:9000\" \
    > ~/rovermind_logs/bridge.log 2>&1 < /dev/null &
  sleep 3; tail -2 ~/rovermind_logs/bridge.log; ss -ltn | grep 9000
"'
```
Expect `[bridge] listening on tcp://127.0.0.1:9000` + a LISTEN on 9000.

## 5. Done → vision is unblocked
Step 3 passing means **Gate 3.4 is unblocked**. Remaining gates (all
permission-gated, need a clear/safe area + a spotter):
- **Gate 2.3** — `execute_command` drive (no Moondream needed).
- **Gate 3.4** — `capture_and_analyze` with the **real camera** (needs the full
  ROS stack up so the bridge can grab color+depth frames). First call loads
  Moondream (5–15 s); then sub-second.
- **Gate 4.3** — `run_agent.py "drive to the <target>"` end-to-end from the Mac
  (needs the SSH tunnel `-L 9000:localhost:9000` + `OPENAI_API_KEY`, now auto-loaded
  from `.env`).

See the smoke-test runbook `docs/superpowers/runbooks/py38-bridge-rover-smoke.md`
for the gate-by-gate pass/fail criteria.
