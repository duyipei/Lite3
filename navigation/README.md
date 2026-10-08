# Site fusion visual relocalization and multi-waypoint patrol

This overlay deliberately keeps navigation output away from the real motion
topic until an operator completes the physical safety checks.

## Artifacts

- `site_fusion_20260909_001/localization_map.pcd`: 10 cm PCD for NDT localization.
- `site_fusion_20260909_001/patrol_map.yaml`: conservative 2D corridor map.
- `site_fusion_20260909_001/mission_draft.yaml`: 43-waypoint out-and-back draft. It is disabled.
- `site_fusion_20260909_001/nav2_safe.yaml`: low-speed Nav2 parameters.
- `site_fusion_20260909_001_area_v4_review/`: expanded LiDAR-observed
  traversability map. It is a review candidate and does not replace the
  conservative corridor map. Gray cells remain unknown and are not
  traversable.
- Detailed procedure: `docs/2026-09-09_visual_relocalization_patrol.md`.
- Full on-site visual initialization and RViz click-to-navigate procedure:
  `docs/2026-09-10_visual_relocalization_rviz_goal_field_test.md`.
- Indoor two-stage visual initial relocalization (start-area priority plus
  global keyframe fallback):
  `docs/2026-09-14_visual_initial_relocalization.md`.

The expanded candidate can be selected without changing the scripts:

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh
```

Its `navigation_area_comparison.png` overlays the previous corridor in blue
on the newly accepted free area in pale green. The candidate keeps
`mission_draft.yaml` disabled until an on-site A-to-B test has passed.

## Terminal sequence

1. Localization only:

   `bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh`

2. Start the RGB visual localizer in a second terminal:

   `bash /home/ysc/lite3_patrol_project/navigation/start_visual_localizer.sh`

3. Start the guarded visual initialization bridge:

   `bash /home/ysc/lite3_patrol_project/navigation/start_visual_initialization.sh`

   It remains blocked until the run-specific `visual_initialization.yaml` contains a
   verified visual localizer topic and visual-map to LiDAR-map transform.
   Request initialization with `/patrol/request_visual_initialization`. It
   publishes `/patrol/localization_ready=true` only after stable visual input
   and matching NDT odometry. Manual `/initialpose` remains a fallback, not the
   normal patrol startup path.

4. Isolated Nav2 (still cannot move the robot):

   `bash /home/ysc/lite3_patrol_project/navigation/start_nav_isolated.sh`

5. Validate the mission without sending goals:

   `bash /home/ysc/lite3_patrol_project/navigation/mission_dry_run.sh`

6. Only after the on-site safety checkpoint, start the gate. It starts
   DISARMED and emits only zero commands:

   `bash /home/ysc/lite3_patrol_project/navigation/start_gate_disarmed.sh`

Do not arm or execute `mission_draft.yaml` until the map overlay, current
visual-plus-NDT pose, emergency stop, and fenced test area have been reviewed.
The velocity gate also requires `/patrol/localization_ready=true`. The mission
requires both `enabled: true` and
`review_required: false`, plus the runner's `--execute` flag.
