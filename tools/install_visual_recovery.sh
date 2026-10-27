#!/usr/bin/env bash
# Run on Jetson. Install optional recovery assets without replacing system Torch.
set -Eeuo pipefail
PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DEPENDENCIES=$PROJECT/tools/pnp_lab/deps
CHECKPOINT_DIR=$HOME/.cache/torch/hub/checkpoints
CHECKPOINT=$CHECKPOINT_DIR/loftr_outdoor.ckpt
EXPECTED=21f5bec5968178e8bc8b7633441836fe5de4f47d861dd2cd7dc38e271b0479ec
URL=https://huggingface.co/kornia/loftr/resolve/0fb3b456cf176746709c38fa9cf80695a1c82dac/loftr_outdoor.ckpt
python3 -c 'import torch, cv2; print("Torch:", torch.__version__, "CUDA:", torch.cuda.is_available())'
mkdir -p "$DEPENDENCIES" "$CHECKPOINT_DIR"
if ! PYTHONPATH="$DEPENDENCIES${PYTHONPATH:+:$PYTHONPATH}" python3 -c \
    'import kornia; assert kornia.__version__ == "0.6.12"' 2>/dev/null; then
  python3 -m pip install --disable-pip-version-check --no-deps \
    --target "$DEPENDENCIES" kornia==0.6.12
fi
if ! printf '%s  %s\n' "$EXPECTED" "$CHECKPOINT" | sha256sum --check --status; then
  TEMP_CHECKPOINT=$(mktemp "$CHECKPOINT_DIR/loftr-download.XXXXXXXX")
  trap 'rm -f -- "$TEMP_CHECKPOINT"' EXIT
  curl --fail --location --retry 2 --connect-timeout 10 --max-time 180 \
    --output "$TEMP_CHECKPOINT" "$URL"
  printf '%s  %s\n' "$EXPECTED" "$TEMP_CHECKPOINT" | sha256sum --check
  mv -- "$TEMP_CHECKPOINT" "$CHECKPOINT"
  trap - EXIT
fi
echo 'PASS: local LoFTR recovery assets ready; no robot processes started.'
