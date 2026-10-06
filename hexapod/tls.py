"""Self-signed TLS so the laptop camera works on a LAN address.

Browsers treat http://localhost as a secure context and allow getUserMedia.
http://192.168.x.x is not, so Hand dies with a blocked-camera error. A
self-signed cert on the same port makes the origin https and the camera works
after the first warning.
"""

from __future__ import annotations

import logging
import subprocess
from pathlib import Path
from typing import List, Tuple

from .net import interface_addresses

log = logging.getLogger(__name__)

CERT_DAYS = 365
CERT_CN = "hexapod"
TLS_HOME = Path.home() / ".hexapod" / "tls"


def san_parts() -> List[str]:
    """DNS and IP names the cert must cover. Regenerated when the LAN address changes."""

    parts = ["DNS:localhost", "IP:127.0.0.1"]
    for _, address in interface_addresses():
        parts.append(f"IP:{address}")
    return list(dict.fromkeys(parts))


def tls_paths(root: Path = TLS_HOME) -> Tuple[Path, Path, Path]:
    return root / "cert.pem", root / "key.pem", root / "sans.txt"


def ensure_pair(root: Path = TLS_HOME) -> Tuple[Path, Path]:
    """Return cert and key, minting them if missing, stale, or the LAN set moved."""

    cert, key, stamp = tls_paths(root)
    wanted = "\n".join(san_parts()) + "\n"
    if cert.is_file() and key.is_file() and stamp.is_file() and stamp.read_text() == wanted:
        return cert, key
    root.mkdir(parents=True, exist_ok=True)
    _mint(cert, key, wanted)
    stamp.write_text(wanted)
    log.info("tls cert for %s", ", ".join(san_parts()))
    return cert, key


def _mint(cert: Path, key: Path, san_text: str) -> None:
    san = san_text.strip().replace("\n", ",")
    try:
        subprocess.run(
            [
                "openssl", "req", "-x509", "-newkey", "rsa:2048", "-sha256",
                "-days", str(CERT_DAYS), "-nodes",
                "-keyout", str(key), "-out", str(cert),
                "-subj", f"/CN={CERT_CN}",
                "-addext", f"subjectAltName={san}",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("openssl is required to mint the self-signed cert") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(exc.stderr.strip() or "openssl failed to mint a cert") from exc
