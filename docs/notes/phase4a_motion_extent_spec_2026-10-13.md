# Phase 4A — minimal motion-history correction

Evidence: final6 accepts target-owned partial boxes at frames 1283/1284. Top remains near y=222 while bottom contracts; `_accept` writes their centers into history. At 1292 the correct candidate passes both appearance thresholds but fails the vertical motion gate because inferred vy=-58 px/s. Production diagnostic and hashes are preserved under `phase4a_final6_d3_neural_diagnosis_20261013`.

Keep tracker, candidate validation, time source, timeout, reference policy, bbox output and controller interfaces unchanged. No new model/dependency/state machine.

Within `_accept`, append a dirty crop to trusted motion history only when its top/bottom changes are consistent with translation relative to the last trusted bbox. Compare extent change to bbox-scale jitter plus common edge displacement. An asymmetric top/bottom change during crowding freezes history; it must not teach vertical velocity. A clean crop retains existing update behavior. Appearance references remain governed by the stricter clean-crop rule. The actual confirmed detection box remains the output; the guard does not synthesize a full-body bbox for gimbal.

Tests: same physical target retains appearance while bottom/top truncation changes its measured centroid; after a short ambiguous gap a fresh full-body observation with a new TID must confirm twice and resume the target. Also verify that coherent vertical translation under moderate limb overlap updates motion history. Existing wrong-person vertical/appearance/motion and dirty-gallery tests must continue passing.

D3 is now development/regression. Before opening outputs from a new active guard, select and raw-only label a later crossing as event-held-out (same capture limitation stated), or a fresh capture. Compare actual YOLO/DeepSORT replay and timing; cached observations are development evidence only. Independent human review remains the user's selected responsibility; Antigravity stays stopped.
