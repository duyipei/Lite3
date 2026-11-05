#!/usr/bin/env bash
# Developer-only offline checks. No ROS process or network access.
set -euo pipefail
T22_CHECK_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
T22_PROJECT_DIR=$(cd -- "$T22_CHECK_DIR/../.." && pwd)
cd -- "$T22_PROJECT_DIR"
export PYTHONDONTWRITEBYTECODE=1
python3 -B - <<'PY'
import ast
from pathlib import Path
paths = [Path('scripts/leg_odom_tf_bridge.py'), Path('scripts/voa_costmap_bridge.py'),
         Path('scripts/lite3_t22_probe.py')]
paths += list(Path('tools/t22').glob('*.py'))
paths += list(Path('tests').glob('*t22*.py'))
for path in paths:
    ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))
print('Python 3.8 syntax: {} files passed'.format(len(paths)))
PY
bash -n tools/t22/check_t22.sh tools/t22/collect_t22.sh
python3 -B -m unittest discover -s tests -p 'test_t22_*.py' -v
(
    cd tests
    python3 -B -m unittest -v test_voa_projection_freshness \
        test_voa_costmap_bridge.GeometryTests test_voa_costmap_bridge.RuntimeTests \
        test_factory_voa_mode.FactoryGateTests test_visual_camera_startup test_visual_latency
)
git diff --check
