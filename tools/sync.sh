#!/usr/bin/env bash
# Mirror the working tree to the Pi over ssh, then optionally run a command there.
#
#   tools/sync.sh                            # sync only
#   tools/sync.sh .venv/bin/hexapod check    # sync, then run that on the Pi
#   tools/sync.sh sudo systemctl restart hexapod
#   HEXAPOD_HOST=hexapod@192.168.11.253 tools/sync.sh
#
# The Pi's .venv, .git and caches are excluded, so they survive --delete.
set -euo pipefail

HOST="${HEXAPOD_HOST:-hexapod@hexapod.local}"
DEST="${HEXAPOD_PATH:-hexapod}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

rsync -az --delete --itemize-changes \
  --exclude '.git/' \
  --exclude '.venv/' \
  --exclude '__pycache__/' \
  --exclude '.pytest_cache/' \
  --exclude '*.egg-info/' \
  --exclude 'assets/' \
  --exclude '.DS_Store' \
  "$ROOT/" "$HOST:$DEST/"

echo "synced -> $HOST:$DEST"

if [ "$#" -gt 0 ]; then
  echo "--- running on $HOST: $* ---"
  # -t so anything that prompts, sudo most of all, has a terminal to prompt on.
  ssh -t "$HOST" "cd $DEST && $*"
fi
