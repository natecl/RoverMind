# Next Gates — resume here (vision is done)

Self-contained runbook to pick up bring-up in a fresh session. Companions:
[`BRINGUP_STATUS_AND_PLAN.md`](BRINGUP_STATUS_AND_PLAN.md) (full context),
[`ROVER_BOOTSTRAP.md`](ROVER_BOOTSTRAP.md) (fresh-rover repo/stack + DDS/QoS),
[`MOONDREAM_FINAL_WIRING.md`](MOONDREAM_FINAL_WIRING.md) (bridge env for Moondream),
and the gate-by-gate criteria in `docs/superpowers/runbooks/py38-bridge-rover-smoke.md`
(on disk only — `docs/` is gitignored).

_Last updated: 2026-05-29._

---

## What's DONE
- ✅ **New rover** (MAC `54:ef:33:9e:e7:71`, hostname `master`) bootstrapped: repo
  rsynced to `~/RoverMind`, `~/rovermind_ws` built (`colcon build --symlink-install`,
  2 pkgs), AEB `/scan` QoS fix present.
- ✅ **Moondream vision works on the GPU** (Gate 3.4 proven *standalone*): model at
  `~/moondream2_local` (3.85 GB `model.safetensors`, py3.8-patched `.py`,
  `.vendor_done`). `transformers==4.46.3`/`accelerate`/`einops` installed; CUDA torch
  2.1 untouched. Verified: `MoondreamClient(device="cuda").ask(...)` → real answer.
- ✅ Fix committed: `perception/moondream_client.py` `enable_gqa` SDPA shim (torch 2.1
  compat) + `tests/test_sdpa_gqa_shim.py`.

## Connecting to THE rover (there are 2–3 AgileX rovers around!)
IP changes per session/network. **Always match the exact MAC** `54:ef:33:9e:e7:71`;
do NOT use `…e7:73` or `…ea:7f` (other rovers). On the Mac:
```bash
for i in $(seq 1 254); do ping -c1 -W200 <subnet>.$i >/dev/null 2>&1 & done; wait
arp -an | grep -i "54:ef:33:9e:e7:71"     # -> our rover's current IP
```
Last seen: `192.168.137.36`. Confirm: `ssh agilex@<ip> 'cat /sys/class/net/wlan0/address; ls ~/moondream2_local/.vendor_done'`.

## Gotchas learned this session (apply before the gates)
1. **Wifi power-save drops SSH/sustained transfers.** After any reboot run on the
   rover: `sudo iw dev wlan0 set power_save off` (not persistent).
2. **8 GB Jetson is memory-tight.** Moondream alone ≈ 4 GB on the GPU (unified mem).
   Running the **full ROS stack + bridge + Moondream together may OOM** during
   `capture_and_analyze`. Mitigate: run headless (stop the desktop GUI to free
   ~1.5 GB) and/or watch `free -h`. A reboot gives the cleanest slate (~5+ GB free).
3. **`rm_env.sh` / `rm_cyclonedds.xml` do NOT exist on this rover yet** — they're not
   in the repo; reconstruct per `ROVER_BOOTSTRAP.md` §2.5/§8 before bringing up the
   stack. Note: this rover's `~/.bashrc` already sets a loopback `CYCLONEDDS_URI` but
   **no `MaxAutoParticipantIndex`** → watch for "Failed to find a free participant
   index" with ~10 nodes; if it appears, use the `rm_cyclonedds.xml` profile
   (loopback + `<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>`).
4. Non-interactive SSH skips `~/.bashrc` ROS prompt → source overlays manually
   (`/opt/ros/foxy/setup.bash`, `~/limo_ros2_ws/install/setup.bash`,
   `~/rovermind_ws/install/setup.bash`) or `~/rm_env.sh` once it exists.
5. Launch nodes detached (no tmux/screen): `setsid bash -c "<cmd>" > log 2>&1 < /dev/null &`.

## Remaining gates — ALL permission-gated (clear/safe area + spotter, AEB on, human ready to Ctrl-C/lift)
Run gate-by-gate per `py38-bridge-rover-smoke.md`. Do not skip ahead.

### Pre: wire the bridge for Moondream + bring up the stack
- Add to `~/rm_env.sh` (create it first): `export MOONDREAM_MODEL_PATH=$HOME/moondream2_local`.
- Bring up: limo drivers → safety+AEB → bridge (`bridge/bridge_server.py --bind 127.0.0.1:9000`),
  all sourcing the env so the bridge inherits `MOONDREAM_MODEL_PATH`.
- From the Mac, open the tunnel: `ssh -L 9000:localhost:9000 agilex@<ip>`.

### Gate 1.5 — bridge ping
`python3 -c "from bridge.client import BridgeClient; ... print(c.ping())"` → `pong`.

### Gate 2.3 — `execute_command` drives ~30 cm  ⚠️ first real motion
`c.execute_command(heading_deg=0.0, distance_m=0.3)` → `success=True` AND rover
physically moves. (No Moondream needed.)

### Gate 3.4 — `capture_and_analyze` with the REAL camera
Needs the full ROS stack up (bridge grabs color+depth). First call loads Moondream
(5–15 s, **watch for OOM** — see gotcha #2), then sub-second. `c.capture_and_analyze('<target>')`
→ a `SceneObservation` with plausible `found`/`direction`/`distance`.

### Gate 4.3 — full agent end-to-end (from the Mac)
`source .venv/bin/activate && python scripts/run_agent.py "drive to the <target>"`
(needs the SSH tunnel + `OPENAI_API_KEY`, auto-loaded from `.env`). Pass = terminal
`status: arrived` (or coherent `lost`/`aborted`) and the rover meaningfully approaches.

## After all gates pass
Per the smoke runbook: the branch is ready; address any minor cleanups in
`docs/superpowers/plans/2026-05-26-py38-bridge.md`.
