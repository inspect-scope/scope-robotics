#!/usr/bin/env bash
# Show /status full screen on the panel. Waits for the display and the server first.
#
#   HEXAPOD_STATUS_URL=https://localhost:8000/status deploy/kiosk.sh
#   HEXAPOD_PANEL_TRANSFORM=270 deploy/kiosk.sh   # panel mounted the other way up
#
# Runs as the desktop user. Works from a systemd user unit or from the
# compositor's autostart file; it finds the Wayland socket or X display itself.
set -u

URL="${HEXAPOD_STATUS_URL:-https://localhost:8000/status}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

# Wait for a display. Bookworm runs labwc or wayfire on Wayland; older images use X.
for _ in $(seq 1 60); do
  if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ]; then break; fi
  sock="$(ls "$XDG_RUNTIME_DIR"/wayland-[0-9]* 2>/dev/null | head -n1)"
  if [ -n "$sock" ]; then export WAYLAND_DISPLAY="$(basename "$sock")"; break; fi
  if [ -S /tmp/.X11-unix/X0 ]; then export DISPLAY="${DISPLAY:-:0}"; break; fi
  sleep 1
done

# The panel is a 320x480 ST7796S mounted on its side, so the output has to be
# rotated before Chromium sizes itself to it. Harmless if wlr-randr is missing
# or the output is named something else on your image.
if [ -n "${WAYLAND_DISPLAY:-}" ] && command -v wlr-randr >/dev/null; then
  wlr-randr --output "${HEXAPOD_PANEL_OUTPUT:-SPI-1}" \
            --transform "${HEXAPOD_PANEL_TRANSFORM:-90}" || \
    echo "kiosk.sh: could not rotate the panel; the page will be cut off" >&2
fi

# Wait for the server; the kiosk is useless before it.
until curl -kfs -o /dev/null "$URL"; do sleep 1; done

BROWSER="$(command -v chromium-browser || command -v chromium || true)"
if [ -z "$BROWSER" ]; then
  echo "kiosk.sh: chromium not found; sudo apt install chromium-browser" >&2
  exit 1
fi

# 0.96 is 480/500. Chromium will not make a window narrower than 500 CSS px, so
# on this 480 px panel the right 20 px falls off the glass at scale 1. Scaling
# the whole window by 480/500 lands it exactly on the screen.
exec "$BROWSER" \
  --kiosk "$URL" \
  --ignore-certificate-errors \
  --noerrdialogs --disable-infobars --no-first-run --incognito \
  --password-store=basic \
  --disable-session-crashed-bubble --disable-features=TranslateUI \
  --check-for-update-interval=31536000 --hide-scrollbars \
  --window-position=0,0 --window-size="${HEXAPOD_PANEL_SIZE:-480,320}" \
  --force-device-scale-factor="${HEXAPOD_PANEL_DSF:-0.96}"
