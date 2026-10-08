# Expanded navigation map v4

This directory is an independent review candidate. It does not replace
`site_fusion_20260909_001`.

- White: LiDAR-observed, locally ground-supported, connected free space.
- Black: obstacle evidence, including the map builder's 0.15 m pre-inflation.
- Gray: unknown; Nav2 is configured not to plan through it.
- Red in `patrol_map_preview.png`: the physically recorded route.

Offline validation passed for all 43 existing waypoint centers and their
0.8 x 0.6 m padded footprints. The mission remains disabled. Before enabling
new off-route points, inspect them in RViz and execute only a nearby A-to-B
low-speed test with an operator holding the emergency stop.
