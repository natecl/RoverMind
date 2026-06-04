#!/usr/bin/env bash
# Discover the LIMO rover on the current network by its (stable) MAC, identify which network
# profile is active, record the IP in context/ENVIRONMENT.md as a growing list of viable IPs,
# and print the SSH tunnel command.
#
# Selection is keyed on the MAC: live arp discovery picks the active IP; if Wi-Fi turns up
# nothing, we fall back to the known IPs already recorded for that MAC (newest first) and use
# the first one that pings. The ENVIRONMENT.md table is append-only (a repeat IP just refreshes
# its "Last seen" date), so committing it accumulates a useful history instead of churning.
#
# Usage:
#   scripts/rover_connect.sh                 # default rover MAC
#   scripts/rover_connect.sh <mac>           # a different rover
#   scripts/rover_connect.sh <mac> --open    # also open the SSH tunnel
set -euo pipefail

MAC="${1:-54:ef:33:9e:e7:71}"
OPEN="${2:-}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/context/ENVIRONMENT.md"
SUBNETS=(172.20.10 192.168.137)   # iPhone hotspot, Windows/Android hotspot

profile_for_ip() {
  case "$1" in
    172.20.10.*)   echo "iPhone hotspot" ;;
    192.168.137.*) echo "Windows/Android hotspot" ;;
    192.168.55.*)  echo "USB tether" ;;
    *)             echo "unknown" ;;
  esac
}

echo "Looking for rover $MAC ..."

# 1) Live discovery: populate the arp cache with a quick parallel ping sweep, then match by MAC.
for net in "${SUBNETS[@]}"; do
  for i in $(seq 1 254); do ping -c1 -W300 "$net.$i" >/dev/null 2>&1 & done
done
wait
IP="$(arp -an | grep -i "$MAC" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1 || true)"
SRC="live"

# 2) Fixed USB-ethernet tether fallback.
if [ -z "$IP" ] && ping -c1 -W300 192.168.55.1 >/dev/null 2>&1; then
  IP="192.168.55.1"; SRC="usb"
fi

# 3) MAC-keyed fallback: try the known IPs already recorded for this MAC, newest first.
if [ -z "$IP" ] && [ -f "$ENV_FILE" ]; then
  while read -r cand; do
    [ -z "$cand" ] && continue
    if ping -c1 -W300 "$cand" >/dev/null 2>&1; then IP="$cand"; SRC="known-list"; break; fi
  done < <(grep -iF "$MAC" "$ENV_FILE" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | awk '!seen[$0]++' | tail -r)
fi

if [ -z "$IP" ]; then
  echo "Rover ($MAC) not found live, on the USB tether, or among known IPs." >&2
  echo "Check the hotspot, try the USB cable, or read 'hostname -I' on the rover." >&2
  exit 1
fi

PROFILE="$(profile_for_ip "$IP")"
DATE="$(date +%Y-%m-%d)"
echo "Found: $IP  (profile: $PROFILE, via: $SRC)"
echo "Tunnel: ssh -L 9000:localhost:9000 agilex@$IP"

# Append the IP to the table (or refresh the date of an existing IP+MAC row). Data rows are the
# only lines inside the block that contain backticks, so headers/separator pass through untouched.
if [ -f "$ENV_FILE" ]; then
  awk -v ip="$IP" -v mac="$MAC" -v profile="$PROFILE" -v date="$DATE" '
    /<!-- ROVER_CONNECT:BEGIN -->/ { print; inblock=1; updated=0; next }
    /<!-- ROVER_CONNECT:END -->/ {
      if (!updated) printf "| %s | %s | `%s` | `%s` |\n", date, profile, ip, mac
      print; inblock=0; next
    }
    inblock && $0 ~ /`/ {
      if (index($0, "`" ip "`") && index($0, "`" mac "`")) {
        printf "| %s | %s | `%s` | `%s` |\n", date, profile, ip, mac
        updated=1; next
      }
      print; next
    }
    { print }
  ' "$ENV_FILE" > "$ENV_FILE.tmp" && mv "$ENV_FILE.tmp" "$ENV_FILE"
  echo "Recorded $IP in $ENV_FILE (known-IP list)."
fi

if [ "$OPEN" = "--open" ]; then
  exec ssh -L 9000:localhost:9000 "agilex@$IP"
fi
