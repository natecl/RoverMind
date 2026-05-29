# Download Moondream onto the Rover (run on the rover's terminal)

How to get the Moondream2 model onto the rover **autonomously** — you run these
**directly on the rover's own terminal** (screen/keyboard or any shell on the
rover). It survives stalls, wifi drops, and reboots, and stops itself when done,
so you can connect the rover to a stable wifi and walk away. No laptop needed for
the download.

When it finishes, do the bridge wiring + GPU test from
[`MOONDREAM_FINAL_WIRING.md`](MOONDREAM_FINAL_WIRING.md).

---

## Why this is needed / what it does
The ~3.7 GB weights download slowly over a phone hotspot and `hf_transfer` keeps
hanging on this rover, so a one-shot download is unreliable. These steps install
a small **self-healing loop** that: waits for internet → downloads with the plain
(reliable) HF downloader → kills & retries if it stalls → patches the model's
`.py` files for Python 3.8 → stops once complete. A `@reboot` cron entry restarts
it automatically if the rover reboots.

## Requirements
- Rover **powered on** and **has internet** (any wifi that reaches the internet —
  it does **not** need to be the same network as your laptop).
- School/eduroam wifi works, **but** watch for a **captive portal**: a headless
  rover can't click a browser login. Either log in from the rover's own screen,
  or have IT register the rover's MAC (`54:ef:33:9e:ea:7f`).

---

## Step 1 (one-time, only if deps are missing) — install py3.8 deps, keep Jetson torch
Skip if `python3.8 -c "import transformers, accelerate, einops, hf_transfer"`
prints nothing/exits 0. The constraints file pins the Jetson's CUDA build so pip
**never reinstalls torch/torchvision/numpy/pillow**:
```bash
cat > ~/pip-constraints.txt <<'EOF'
torch==2.1.0a0+41361538.nv23.06
torchvision==0.16.1
numpy==1.23.4
pillow==10.4.0
EOF
# Confirm pip won't touch torch et al, THEN install:
python3.8 -m pip install --dry-run -c ~/pip-constraints.txt "transformers==4.46.3" accelerate einops hf_transfer
python3.8 -m pip install        -c ~/pip-constraints.txt "transformers==4.46.3" accelerate einops hf_transfer
```
(`transformers==4.46.3` is the last release that supports Python 3.8.)

## Step 2 — set up + start the autonomous downloader (paste once)
```bash
# (a) download+patch script
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

# (b) self-healing loop: waits for net, downloads, kills+retries on stall, stops when done
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

# (c) auto-start on every reboot (no sudo)
( crontab -l 2>/dev/null | grep -v moondream_fetch_loop; \
  echo "@reboot /bin/bash $HOME/moondream_fetch_loop.sh >> $HOME/moondream_fetch.log 2>&1" ) | crontab -

# (d) start it now, detached (survives logout / closing the terminal)
setsid bash ~/moondream_fetch_loop.sh >> ~/moondream_fetch.log 2>&1 < /dev/null &
echo "autonomous downloader started."
```

## Step 3 — connect the rover to a stable wifi and leave
Connect the rover to the wifi (from its screen/network settings). The downloader
sits "waiting for internet..." until it's online, then downloads on its own.

## Step 4 — check status (anytime, on the rover)
```bash
tail -n 20 ~/moondream_fetch.log     # what it's doing
du -sh ~/moondream2_local            # grows toward ~3.7 GB
ls ~/moondream2_local/.vendor_done 2>/dev/null && echo FINISHED || echo "still downloading"
```
When `.vendor_done` exists, it's fully **downloaded and patched**.

---

## When it's done
Ping the assistant (or follow [`MOONDREAM_FINAL_WIRING.md`](MOONDREAM_FINAL_WIRING.md))
to point the bridge at `~/moondream2_local`, GPU-verify, and restart the bridge —
which unblocks the vision gate (3.4). Background/context:
[`BRINGUP_STATUS_AND_PLAN.md`](BRINGUP_STATUS_AND_PLAN.md).
