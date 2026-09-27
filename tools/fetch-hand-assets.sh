#!/usr/bin/env bash
# MediaPipe Hands wasm + landmarker. The JS glue is vendored; these two are
# gitignored because they are large.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VISION="https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.32"
TASK="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
DEST="$ROOT/hexapod/static/vendor/mediapipe"

mkdir -p "$DEST/wasm"
curl -fsSL "$VISION/wasm/vision_wasm_internal.wasm" -o "$DEST/wasm/vision_wasm_internal.wasm"
curl -fsSL "$TASK" -o "$DEST/hand_landmarker.task"
echo "wrote $DEST/wasm/vision_wasm_internal.wasm and $DEST/hand_landmarker.task"
