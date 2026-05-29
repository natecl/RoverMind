# RoverMind Bring-up — Current State & Plan

Living status doc for getting RoverMind running end-to-end on the LIMO rover.
Companions: [`../LIMO_WORKFLOW.md`](../LIMO_WORKFLOW.md) (the canonical
every-session workflow), [`ROVER_BOOTSTRAP.md`](ROVER_BOOTSTRAP.md) (fresh-rover
bootstrap + DDS/QoS preflight gotchas), and the gate-by-gate smoke-test runbook
`docs/superpowers/runbooks/py38-bridge-rover-smoke.md` (on disk only — `docs/`
is gitignored).

_Last updated: 2026-05-28._

---

## 1. Architecture recap (why things are split)
- **Mac (Python 3.10 venv)** runs the LangGraph agent (`scripts/run_agent.py`).
- **Rover (Python 3.8)** runs ROS 2 Foxy, the safety stack, and the **bridge**
  (`bridge/bridge_server.py`) which owns rclpy + Moondream.
- Mac ↔ rover talk over a **JSON-RPC bridge on an SSH tunnel** (`-L 9000:localhost:9000`),
  **not** ROS-over-network. The agent never imports rclpy/Moondream.

## 2. Current state (what works)
- ✅ **SSH + key auth** to `agilex@<rover-ip>` (IP changes per session; rover
  hostname `master`).
- ✅ **Fresh-rover bootstrap** done: repo rsync’d, `~/rovermind_ws` symlinks,
  `colcon build`. Packages discoverable. (See `ROVER_BOOTSTRAP.md`.)
- ✅ **Two preflight blockers fixed & merged to `main`** (PRs #6–#8):
  - DDS: stale Husarnet CycloneDDS profile replaced with a loopback profile
    (`~/rm_cyclonedds.xml`, referenced from `~/rm_env.sh`) so local data flows.
  - AEB `/scan` QoS → `qos_profile_sensor_data` (BEST_EFFORT); the brake now
    sees the LiDAR instead of blind-braking. Guarded by `tests/test_aeb_qos.py`.
- ✅ **Stack brought up & verified** (no driving): limo drivers, safety+AEB
  (AEB correctly braked on a real 0.27 m obstacle), bridge `ping → pong`,
  `/scan /imu /odom` all delivering, fresh participants can join.
- ✅ **Mac prereqs done**: `.venv` (3.10), deps installed, `OPENAI_API_KEY`
  auto-loaded from a gitignored `.env` (PR #8), SSH tunnel verified.

## 3. Smoke-test gates (runbook status)
| Gate | What | Status |
|---|---|---|
| 1.5 | bridge `ping` | ✅ pass |
| 2.3 | `execute_command` physically drives ~30 cm | ⏳ pending — needs clear/safe area + go-ahead |
| 3.4 | `capture_and_analyze` (Moondream vision) | 🔴 blocked on Moondream install (see §4) |
| 4.3 | full agent `run_agent.py` autonomous drive | ⏳ pending — needs 2.3 + 3.4 |

Driving gates are **permission-gated**: never run `execute_command` /
`run_agent.py` without explicit go-ahead and the LIMO_WORKFLOW.md §2e go/no-go
checks (AEB on, open floor clear of drop-offs, clear front arc, a human ready to
Ctrl-C / lift).

## 4. Moondream vision — the Python 3.8 blocker
**Problem:** the Jetson’s CUDA `torch` is a **py3.8** wheel, but Moondream2’s
`trust_remote_code` modeling files use py3.9+ builtin-generic annotations
(`tuple[int, int]` evaluated at import, no `from __future__ import annotations`),
so they fail to import on py3.8. Every `.query`-capable revision has this. py3.9
exists on the rover but has **no CUDA torch**, so there’s no easy GPU path there.

**Chosen fix (Option A — patch for py3.8, keep GPU):**
1. Vendor a local snapshot at `~/moondream2_local` and **prepend
   `from __future__ import annotations`** to its `.py` files (defers the
   annotations → py3.8-safe). Done by `~/vendor_moondream.py` on the rover.
2. `perception/moondream_client.py` now resolves the model source from the
   `MOONDREAM_MODEL_PATH` env var (local patched snapshot) and falls back to the
   hub id+revision otherwise. Pure helper `resolve_model_source()` is unit-tested
   in `tests/test_moondream_source.py` (Mac-runnable).
   - ⚠️ **Uncommitted** as of this writing: `perception/moondream_client.py` (M)
     and `tests/test_moondream_source.py` (new). Commit + rsync to the rover as
     part of §6.
3. Deps already installed on the rover (torch untouched): `transformers==4.46.3`
   (last py3.8-compatible), `accelerate`, `einops`, plus `hf_transfer`.

**Download reality:** the ~3.7 GB weights are slow over the iPhone hotspot.
`hf_transfer` (6 MB/s) **keeps hanging on this rover**; the plain downloader is
reliable+resumable but ~0.5 MB/s (~1.5–2 h). So we download **autonomously** on
the rover (see §5) rather than babysit it.

## 5. PLAN — autonomous, unattended Moondream download
Goal: the rover downloads + patches Moondream on its own (e.g. over school wifi),
surviving stalls, network drops, and reboots — no laptop or phone tethered.

**Requirements:** rover powered on + has internet. The download needs the rover
online only; it does **not** need the laptop on the same network (the laptop is
only needed later for the bridge wiring/verify in §6). School wifi works for the
download even if the laptop can’t join it — watch for captive portals (a headless
rover can’t click through one; register the rover MAC `54:ef:33:9e:ea:7f` or log
in from its screen).

**Mechanism (run once on the rover’s terminal):** a self-healing loop
`~/moondream_fetch_loop.sh` + a `@reboot` cron entry:
- single-instance (`flock`), waits for internet, runs `~/vendor_moondream.py`,
- **stall watchdog**: if `~/moondream2_local` stops growing for 180 s, kill &
  retry (works around the `hf_transfer`/connection hangs; uses the plain
  downloader, `HF_HUB_ENABLE_HF_TRANSFER=0`),
- retries through drops, **stops permanently** once download+patch succeed,
  marked by `~/moondream2_local/.vendor_done`.

Full standalone runbook (run on the rover's terminal):
[`MOONDREAM_DOWNLOAD_ON_ROVER.md`](MOONDREAM_DOWNLOAD_ON_ROVER.md). The exact
copy-paste commands are also in the **Appendix** below. Status check on the
rover:
```bash
tail -n 20 ~/moondream_fetch.log
du -sh ~/moondream2_local              # → ~3.7 GB when done
ls ~/moondream2_local/.vendor_done 2>/dev/null && echo FINISHED || echo "downloading"
```

## 6. Remaining steps after the download finishes (~2 min, needs laptop)
Full copy-paste runbook: [`MOONDREAM_FINAL_WIRING.md`](MOONDREAM_FINAL_WIRING.md).
Do these when the laptop can reach the rover (back on the shared hotspot):
1. **Commit + deploy the client change**: commit `perception/moondream_client.py`
   + `tests/test_moondream_source.py`; rsync the repo to the rover (`--symlink-install`
   makes the py change live, no rebuild).
2. **Point the bridge at the local model**: add to `~/rm_env.sh` on the rover:
   `export MOONDREAM_MODEL_PATH=$HOME/moondream2_local`.
3. **Restart the bridge** under the updated env (so it picks up the env var; it
   constructs `MoondreamClient` lazily on first `capture_and_analyze`).
4. **GPU verify**: construct `MoondreamClient(device="cuda")` + a trivial
   `.ask(dummy_img, "...")` returns a string → unblocks Gate 3.4.

## 7. Then: the remaining gates
- **Gate 2.3** — `execute_command` drive (no Moondream needed): clear/open area,
  go-ahead, spotter ready. First real motion.
- **Gate 3.4** — `capture_and_analyze` with the real camera.
- **Gate 4.3** — `run_agent.py "drive to the <target>"` end-to-end.

## 8. Handy rover facts
- IP changes per session; rediscover from the Mac via `arp -an | grep 54:ef:33:9e:ea:7f`,
  `~/.ssh/config`, `~/.ssh/known_hosts` (see `ROVER_BOOTSTRAP.md` §0).
- Non-interactive SSH skips the rover’s `~/.bashrc` ROS prompt → source
  `~/rm_env.sh` (sets `ROS_DOMAIN_ID=2`, RMW cyclonedds, the loopback DDS profile,
  and the three workspace overlays).
- Launch nodes detached (no `tmux`/`screen` on the rover): `setsid bash -c "<cmd>" > log 2>&1 < /dev/null &`.
- Logs live in `~/rovermind_logs/` (limo, safety, bridge) and `~/moondream_fetch.log`.

---

## Appendix — autonomous downloader commands (paste once on the rover terminal)

```bash
# 1) (Re)create the download+patch script
cat > ~/vendor_moondream.py <<'EOF'
"""Download Moondream2 (2025-06-21) into ~/moondream2_local and patch its
remote code for Python 3.8 (prepend `from __future__ import annotations`)."""
import os, glob
from huggingface_hub import snapshot_download

DEST = os.path.expanduser("~/moondream2_local")
print("downloading snapshot (resumes from cache)...", flush=True)
snapshot_download(repo_id="vikhyatk/moondream2", revision="2025-06-21", local_dir=DEST)

FUTURE = "from __future__ import annotations\n"
patched = []
for py in glob.glob(os.path.join(DEST, "**", "*.py"), recursive=True):
    with open(py, "r", encoding="utf-8") as f:
        src = f.read()
    if any(l.strip() == "from __future__ import annotations" for l in src.splitlines()):
        continue
    with open(py, "w", encoding="utf-8") as f:
        f.write(FUTURE + src)
    patched.append(os.path.basename(py))
print("patched:", patched, flush=True)
print("OK", flush=True)
EOF

# 2) Self-healing loop: waits for net, downloads, kills+retries on stall, stops when done
cat > ~/moondream_fetch_loop.sh <<'EOF'
#!/bin/bash
set -u
DEST="$HOME/moondream2_local"; DONE="$DEST/.vendor_done"; LOG="$HOME/moondream_fetch.log"
exec 9>"$HOME/.moondream_fetch.lock"; flock -n 9 || exit 0   # single instance
log(){ echo "[$(date '+%F %T')] $*"; }
[ -f "$DONE" ] && { log "already done"; exit 0; }
export HF_HUB_ENABLE_HF_TRANSFER=0 HF_HUB_DOWNLOAD_TIMEOUT=30
pkill -9 -f vendor_moondream.py 2>/dev/null; sleep 1
while [ ! -f "$DONE" ]; do
  if ! python3.8 -c "import socket; socket.setdefaulttimeout(8); socket.create_connection(('huggingface.co',443))" 2>/dev/null; then
    log "waiting for internet..."; sleep 30; continue
  fi
  log "internet ok; download attempt"
  ( cd "$HOME" && python3.8 vendor_moondream.py ) >> "$LOG" 2>&1 &
  VPID=$!; last=-1; stall=0
  while kill -0 "$VPID" 2>/dev/null; do
    sleep 30
    cur=$(du -sb "$DEST" 2>/dev/null | cut -f1); cur=${cur:-0}
    if [ "$cur" -gt "$last" ]; then last=$cur; stall=0
    else stall=$((stall+30)); log "no growth ${stall}s (${cur} bytes)"
      [ "$stall" -ge 180 ] && { log "stalled; killing to retry"; kill -9 "$VPID" 2>/dev/null; break; }
    fi
  done
  if wait "$VPID" 2>/dev/null; then touch "$DONE"; log "DONE (downloaded+patched)"; break; fi
  log "attempt ended without success; retry in 15s"; sleep 15
done
log "complete"
EOF
chmod +x ~/moondream_fetch_loop.sh

# 3) Auto-start on every reboot (no sudo needed)
( crontab -l 2>/dev/null | grep -v moondream_fetch_loop; \
  echo "@reboot /bin/bash $HOME/moondream_fetch_loop.sh >> $HOME/moondream_fetch.log 2>&1" ) | crontab -

# 4) Start it now, detached (survives logout / closing the terminal)
setsid bash ~/moondream_fetch_loop.sh >> ~/moondream_fetch.log 2>&1 < /dev/null &
echo "autonomous downloader started."
```
