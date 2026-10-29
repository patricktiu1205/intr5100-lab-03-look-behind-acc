# Lab III Controller Comparison

The current version is the selected lab candidate. Further tuning is deferred to report discussion. Its synthetic results generally improve tracking while retaining useful disturbance attenuation; these are not physical test results.

| Measure | Original front only | Previous look behind | Current look behind |
| --- | ---: | ---: | ---: |
| 0.04 Hz front-gap RMSE cu | 9.77 | 25.76 | 23.43 |
| 0.04 Hz leader-to-follower gain | 2.016 | 0.878 | 0.633 |
| Stopping-test front-gap RMSE cu | 38.01 | 71.22 | 45.89 |

In a new 0.9 s leader-packet freeze, the previous version makes two stale-data stop decisions; the current version makes none. Gap RMSE falls from 49.67 to 41.50 cu. The current version combines bounded short-gap prediction, fading rear feedback, rear-spacing washout and jointly selected gains.

![Current comparison](<../results/Disturbance Comparison.png>)

## Limits for report discussion

The local nominal peak leader-to-follower gain rises slightly from 1.032 to 1.055, although the major front-only resonance remains suppressed. Performance is not uniformly better: a 0.45 s known-age case has gain above one, some transient ratios worsen, and a prolonged rear-telemetry freeze still causes rear overlap in both versions. Scale mismatch also remains a limitation. No full string-stability or physical safety claim is established.

[Report Discussion Notes](<Report Discussion Notes.md>) records these limitations without requiring more algorithm development before the lab.

## Evidence

- 27 deterministic logic/telemetry tests passed.
- 35 actual TA GUI active cycles passed in an offline temporary deployment, including bypass and front-stop checks; no network connections or callback-error fallbacks.
- 20 regression cases and 25 new validation cases compare four configurations: original front-only, previous look-behind, current look-behind and a matched current front-only ablation. The ablation disables every rear-dependent command term.
- Front headway and initial conditions/inputs remain matched. The original front-only baseline is not independently optimized for attenuation.
- The prior evaluation cases became development data for this refinement. The 25 new cases were not used for parameter selection.
- Full metrics and failed cases are retained in `review/refinement_comparison.json`. Model matrices/curves are in `review/linear_response.json`. Selection evidence is in `review/improvement_selection.json`.
- Numerical refinement checks are in `review/numerical_refinement.json`. Reception age and hidden transport latency are tested separately; the controller cannot directly know an unreported source timestamp.
- Metrics after simulated overlap are not physically meaningful; failed cases should not be ranked by their subsequent averages.

The previous implementation and evidence remain under `archive/v2/`. Only `student_controller3.py` is intended for submission.
