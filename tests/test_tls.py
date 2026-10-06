from pathlib import Path

from hexapod.tls import ensure_pair, san_parts


def test_san_covers_localhost():
    parts = san_parts()
    assert "DNS:localhost" in parts
    assert "IP:127.0.0.1" in parts


def test_ensure_pair_writes_pem(tmp_path: Path):
    cert, key = ensure_pair(tmp_path)
    assert cert.is_file() and "BEGIN CERTIFICATE" in cert.read_text()
    assert key.is_file() and "BEGIN" in key.read_text()
    again, _ = ensure_pair(tmp_path)
    assert again == cert
