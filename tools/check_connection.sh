#!/usr/bin/env bash
set -euo pipefail

interface="${1:-enp99s0}"
robot_ip="${2:-192.168.123.161}"

if ! ip link show dev "${interface}" >/dev/null 2>&1; then
    echo "ERROR: interface not found: ${interface}" >&2
    exit 2
fi

echo "Interface:"
ip -brief link show dev "${interface}"
ip -brief address show dev "${interface}"

carrier="$(cat "/sys/class/net/${interface}/carrier" 2>/dev/null || echo 0)"
if [[ "${carrier}" != "1" ]]; then
    echo "ERROR: ${interface} has no carrier. Connect the Ethernet cable first." >&2
    exit 3
fi

if ! ip -4 address show dev "${interface}" | grep -q '192\.168\.123\.'; then
    echo "ERROR: ${interface} has no 192.168.123.x address." >&2
    echo "Configure a static address such as 192.168.123.222/24." >&2
    exit 4
fi

if ! ping -I "${interface}" -c 2 -W 1 "${robot_ip}"; then
    echo "ERROR: robot ${robot_ip} is unreachable through ${interface}." >&2
    exit 5
fi

echo "OK: network link and robot ping are ready. DDS state subscription can be tested next."
