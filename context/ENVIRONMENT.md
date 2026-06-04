# Environment — rover identity & networks

> Last verified: 2026-06-04

Connection facts split into **durable** (the rover's MAC, the network profiles) and
**volatile** (the IP, which changes every session). **Never hardcode an IP in prose** — always
rediscover it from the MAC. The MAC→IP lookup also tells you *which network you're on*.

**Known viable rover IPs** — auto-maintained by scripts/rover_connect.sh (newest row appended; a
repeat IP just refreshes its date). The live MAC lookup picks the active rover; if Wi-Fi discovery
fails, the script falls back to pinging these known IPs for the matching MAC, newest first.

<!-- ROVER_CONNECT:BEGIN -->
| Last seen | Profile | IP | MAC |
|-----------|---------|-----|-----|
| 2026-06-04 | iPhone hotspot | `172.20.10.7` | `54:ef:33:9e:e7:71` |
<!-- ROVER_CONNECT:END -->

## Rovers (stable — MAC does not change per network)

| Rover | MAC | user@host | Notes |
|-------|-----|-----------|-------|
| **LIMO (ours)** | `54:ef:33:9e:e7:71` | `agilex@master` | the one to use; Python 3.8.10 |
| old LIMO | `54:ef:33:9e:ea:7f` | — | **do NOT use** — previous rover, may still be powered on the same hotspot |
| other AgileX | `54:ef:33:9e:e7:73` | — | **do NOT use** (another rover nearby) |

`sudo` on the rover is **not** passwordless (password is set on the device). Key auth needs a
one-time `ssh-copy-id agilex@<ip>` on a fresh rover.

## Network profiles (the IP is volatile; the subnet + applicability are the durable keys)

| Profile | When it applies | Subnet | Reach the rover via |
|---------|-----------------|--------|---------------------|
| **iPhone hotspot** | Default in the field. iPhone Personal Hotspot, both Mac + rover joined. | `172.20.10.0/24` | Wi-Fi IP from arp (below) → SSH + tunnel `:9000` |
| **Windows/Android hotspot** | Alternate peer-to-peer hotspot. | `192.168.137.0/24` | Wi-Fi IP from arp → SSH + tunnel `:9000` |
| **USB tether** | Wi-Fi unreachable; rover cabled to the Mac with the supplied USB cable. | `192.168.55.0/24` | Fixed `192.168.55.1` (Jetson USB-ethernet fallback) |
| **Corporate / guest Wi-Fi** | — | varies | **Does not work** — client isolation blocks peer-to-peer even when both show "connected". Switch to a hotspot. |

> Big downloads (e.g. the 3.7 GB Moondream pull) **stall on the iPhone hotspot** — use a stable
> Wi-Fi (e.g. school/home) for those, then return to the hotspot to drive.

## Selecting the active profile

The easy path — **run the connect helper**, which discovers the IP, identifies the profile,
records it in this file, and prints the tunnel command:

```bash
scripts/rover_connect.sh                 # uses the default MAC above
scripts/rover_connect.sh <other-mac>     # for a different rover
```

Manual equivalent:

```bash
# On the Mac. 1) Populate arp across the known hotspot subnets, then find the rover by MAC:
for net in 172.20.10 192.168.137; do for i in $(seq 1 254); do ping -c1 -W300 $net.$i >/dev/null 2>&1 & done; done; wait
arp -an | grep -i "54:ef:33:9e:e7:71"     # -> the rover's current IP

# 2) Match that IP's subnet to the table above -> that's your active profile.
#    172.20.10.x -> iPhone | 192.168.137.x -> Win/Android | 192.168.55.1 -> USB tether
# 3) Open the tunnel + confirm identity:
ssh -L 9000:localhost:9000 agilex@<ip>
ssh agilex@<ip> 'hostname; cat /sys/class/net/wlan0/address'   # expect: master / 54:ef:33:9e:e7:71
```

If arp finds nothing: the rover isn't on this network — check the hotspot, try the USB tether
(`192.168.55.1`), or read the IP locally on the rover (`hostname -I`).

## Wi-Fi to this rover is flaky (RF issue) — known workarounds

- Power-save off after boot: `sudo iw dev wlan0 set power_save off` (may revert; re-apply).
- Use SSH connection multiplexing (one persistent master, reused) so drops don't tear down work.
- Keep launch commands instant; run rover processes detached (`setsid … & disown`).

Full bring-up procedure lives in `AGENT_WORKFLOW.md` / `LIMO_WORKFLOW.md` (which point here for
the connection step). Cross-ref: memory `[[project_new_rover_2026_05]]`.
