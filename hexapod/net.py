"""Which addresses this machine is reachable at.

Used by the CLI banner and by the status panel, which shows the operator the
address to type into a laptop. A VPN usually owns the default route, so
following the route to the internet finds the tunnel, not the LAN the robot is
on; interfaces are ranked physical first, tunnels last instead.
"""

from __future__ import annotations

import subprocess
import sys
from typing import List, Optional, Tuple

TUNNEL_PREFIXES = ("utun", "tun", "tap", "wg", "ppp", "tailscale", "zt",
                   "docker", "br-", "veth", "virbr", "awdl", "llw", "bridge")
PHYSICAL_PREFIXES = ("en", "eth", "wlan", "wl", "wlp", "enp", "eno")


def is_tunnel(interface: str) -> bool:
    return interface.startswith(TUNNEL_PREFIXES)


def interface_addresses() -> List[Tuple[str, str]]:
    """(interface, ipv4) for every configured interface, most likely LAN first."""
    found = []
    try:
        if sys.platform.startswith("linux"):
            output = subprocess.run(
                ["ip", "-4", "-o", "addr", "show", "scope", "global"],
                capture_output=True, text=True, timeout=2, check=False,
            ).stdout
            for line in output.splitlines():
                parts = line.split()
                if len(parts) >= 4 and parts[2] == "inet":
                    found.append((parts[1], parts[3].split("/")[0]))
        else:
            output = subprocess.run(["ifconfig"], capture_output=True, text=True, timeout=2, check=False).stdout
            interface = ""
            for line in output.splitlines():
                if line and not line[0].isspace():
                    interface = line.split(":")[0]
                    continue
                parts = line.split()
                if len(parts) >= 2 and parts[0] == "inet":
                    found.append((interface, parts[1]))
    except (OSError, subprocess.SubprocessError):
        return []

    usable = [(name, address) for name, address in found if not address.startswith(("127.", "169.254."))]

    def rank(entry: Tuple[str, str]) -> int:
        name = entry[0]
        if is_tunnel(name):
            return 2
        if name.startswith(PHYSICAL_PREFIXES):
            return 0
        return 1

    return sorted(usable, key=rank)


def lan_address() -> Optional[str]:
    """Best guess at the address a laptop on the same network should use."""
    addresses = interface_addresses()
    return addresses[0][1] if addresses else None
