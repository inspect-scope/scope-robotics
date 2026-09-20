#!/usr/bin/env bash
# Install the server as a system service and the panel as a user service. Run on the Pi as the robot user:
#
#   deploy/install.sh
#
# Re-run after moving the repo or changing a unit file. Idempotent.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
USER_NAME="$(id -un)"

if [ ! -x "$REPO/.venv/bin/hexapod" ]; then
  echo "no $REPO/.venv/bin/hexapod. Create it first:" >&2
  echo "  python3 -m venv --system-site-packages .venv && .venv/bin/pip install -e ." >&2
  exit 1
fi

# The server needs the serial port, i2c and the camera.
sudo usermod -aG dialout,i2c,video "$USER_NAME"

sed "s|@USER@|$USER_NAME|g; s|@REPO@|$REPO|g" "$HERE/hexapod.service" \
  | sudo tee /etc/systemd/system/hexapod.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now hexapod.service
echo "hexapod.service: $(systemctl is-active hexapod.service)"

# The kiosk runs in the desktop session, so it is a user unit. Lingering lets it
# start at boot without anyone logging in over ssh first.
mkdir -p "$HOME/.config/systemd/user"
sed "s|@REPO@|$REPO|g" "$HERE/hexapod-kiosk.service" > "$HOME/.config/systemd/user/hexapod-kiosk.service"
systemctl --user daemon-reload
systemctl --user enable --now hexapod-kiosk.service || true
sudo loginctl enable-linger "$USER_NAME"
echo "hexapod-kiosk.service: $(systemctl --user is-active hexapod-kiosk.service || true)"

echo
echo "status page:  http://localhost:8000/status"
echo "logs:         journalctl -u hexapod -f"
echo "kiosk logs:   journalctl --user -u hexapod-kiosk -f"
