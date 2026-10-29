# Lab III Controller Design

## Current tested version

The single-file submission implements equilibrium-referenced Leading Cruise Control with front priority, bounded short-gap prediction and confidence-weighted rear feedback. Supporting files and earlier Git repositories are unchanged. Tuning is complete for this version; unresolved improvements are recorded in [Report Discussion Notes](<Report Discussion Notes.md>).

## Feedback law

Let v0, v1 and v2 be leader, student and follower coordinate speeds, and sF and sR be route-based bumper gaps. Vehicle length is 250 coordinate units. The front target remains dF = 300 + 1.8v1 in every matched comparison.

The current front reference is rF = v0 + 0.15(sF − dF) + 0.50(v0 − v1). A 6 s filter gives a leader-speed reference vRef. With qRef = clip(vRef/γ, 0, 0.85 × 70), the public default IDM equilibrium is sRRef = (250 + 1.2qRef) / sqrt[1 − (qRef/70)⁴]. This is not a new fit to experimental data.

Let eR = sR − sRRef. A bounded 60 s low-pass estimate removes persistent rear-spacing bias. The dynamic rear correction is clip[−0.35(eR − bias) − 1.5(v2 − vRef), −30, 30], followed by a 0.35 s filter. Front headroom, rear approach and telemetry confidence gate this optional term. The front-only ablation disables both rear feedback and the rear command floor, so rear state cannot influence its command.

The desired coordinate-speed reference divided by β gives the returned speed command. Commands are limited to 0–70, with ordinary increase/decrease limits of 6/12 command units per second. Protective constraints may override those smoothing limits. γ = 2.8 and β = 2.8 remain separate, uncalibrated telemetry and command-response hypotheses. Route distances are not rescaled.

## Telemetry handling and constraints

Timestamped observations are propagated to a common receiver time for at most 1.2 s after a previously trusted observation. Leader and rear prediction use bounded filtered speed trends. Own-motion prediction replays recent requested commands through a nominal 0.5 s first-order response. Fresh observations override prediction. Cold observations older than 0.6 s, malformed states and exhausted front/own prediction horizons cause zero output.

Rear feedback fades with age and is disabled when invalid or outside the common straight. Timestamp regressions and large route-position innovations do not overwrite newer accepted observations. History/run changes and section transitions reset controller memory. No integral speed state is accumulated.

Front protections retain a 120-cu reserve, positive-closing TTC stop below 1.5 s, and a conditional stopping-distance cap assuming 45 cu/s² braking and 0.5 s reaction allowance. A rear approach command floor is applied only up to the front cap; incompatible bounds are explicitly diagnosed. No further acceleration is permitted from front data older than 0.6 s. These are model-dependent guards, not a physical safety certificate. TA-owned sensor and spacing shields remain unchanged.

Reception age cannot reveal all unknown transport delay. Requested commands may differ from actual actuator commands after a TA shield override. Both facts limit prediction accuracy. Long rear loss still fails in a documented stress test.

## Selection and evidence

The earlier evaluation cases were explicitly reused as development cases for this refinement. A fixed 324-point joint-gain/washout grid was screened by local response and tracking cost, then checked in nonlinear development scenarios. The chosen gains were frozen before 25 newly defined validation cases. Local peak-gain bounds permit a small regression relative to the previous version; the actual regression is reported rather than described as strict string stability.

The [comparison](<Disturbance Comparison.md>) retains the original front-only controller, the previous look-behind version, the current version and a matched current front-only ablation. All use the same desired front spacing and paired initial conditions/inputs. Twenty earlier cases are regression tests; the 25 new cases were not used for this selection. They remain synthetic model checks, not independent experiments.

Twenty-seven deterministic checks and 35 actual TA GUI active cycles pass with network connections prohibited. The current sources, frozen parameters and numerical evidence are retained under `tests/` and `review/`. The previous tested version is preserved separately under `archive/v2/`.

## Paper connections

[Horn 2013](https://people.csail.mit.edu/bkph/articles/Suppressing_Traffic_Flow%20Instabilities_IEEE_ITS_2013.pdf) motivates bilateral information. [Wang et al.](https://arxiv.org/html/2012.04313v2), equation 27, motivates rear spacing and speed feedback relative to equilibrium. [Li et al. 2023](https://arxiv.org/html/2309.01625v1), section III, motivates checking head-to-tail disturbance propagation. The present speed-command interface, washout, predictor, gains and guards are engineering adaptations; the papers’ stability conclusions are not inherited.
