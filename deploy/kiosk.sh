#!/usr/bin/env bash
# Show /status full screen on the panel. Waits for the display and the server first.
#
#   HEXAPOD_STATUS_URL=http://localhost:8000/status deploy/kiosk.sh
#
# Runs as the desktop user. Works from a systemd user unit or from the
# compositor's autostart file; it finds the Wayland socket or X display itself.
set -u

URL="${HEXAPOD_STATUS_URL:-http://localhost:8000/status}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

# Wait for a display. Bookworm runs labwc or wayfire on Wayland; older images use X.
for _ in $(seq 1 60); do
  if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ]; then break; fi
  sock="$(ls "$XDG_RUNTIME_DIR"/wayland-[0-9]* 2>/dev/null | head -n1)"
  if [ -n "$sock" ]; then export WAYLAND_DISPLAY="$(basename "$sock")"; break; fi
  if [ -S /tmp/.X11-unix/X0 ]; then export DISPLAY="${DISPLAY:-:0}"; break; fi
  sleep 1
done

# Wait for the server; the kiosk is useless before it.
until curl -fs -o /dev/null "$URL"; do sleep 1; done

BROWSER="$(command -v chromium-browser || command -v chromium || true)"
if [ -z "$BROWSER" ]; then
  echo "kiosk.sh: chromium not found; sudo apt install chromium-browser" >&2
  exit 1
fi

exec "$BROWSER" \
  --kiosk "$URL" \
  --noerrdialogs --disable-infobars --no-first-run --incognito \
  --disable-session-crashed-bubble --disable-features=TranslateUI \
  --check-for-update-interval=31536000 --hide-scrollbars \
  --window-position=0,0 --window-size=320,480 --force-device-scale-factor=1
