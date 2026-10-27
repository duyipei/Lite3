#!/usr/bin/env bash
# Run on Jetson after copying third_party/{accelerated_features,xfeat_wheels}.
# Never install/replace Torch, NumPy, OpenCV or the legacy Kornia environment.
set -Eeuo pipefail
PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ $(uname -m) == aarch64 ]] || { echo 'This offline wheel set targets Jetson aarch64.' >&2; exit 1; }
python3 -c 'import torch, cv2; assert torch.cuda.is_available(); print("Using",torch.__version__,cv2.__version__)'
python3 - "$PROJECT" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1])
for folder,filename in [('accelerated_features','runtime_manifest.json'),('xfeat_wheels','manifest.json')]:
 p=root/'third_party'/folder
 for name,digest in json.loads((p/filename).read_text())['sha256'].items():
  if hashlib.sha256((p/name).read_bytes()).hexdigest()!=digest:
   raise RuntimeError('asset checksum mismatch: '+name)
PY
python3 -m pip install --no-deps --no-index --disable-pip-version-check \
  --find-links "$PROJECT/third_party/xfeat_wheels" \
  --target "$PROJECT/third_party/xfeat_deps" \
  kornia==0.7.2 kornia-rs==0.1.3 packaging==24.0 tqdm==4.66.5
PYTHONPATH="$PROJECT/third_party/xfeat_deps" python3 -c \
  'import kornia; from kornia.feature.lightglue import LightGlue; assert kornia.__version__=="0.7.2"; print("XFeat dependencies ready")'
