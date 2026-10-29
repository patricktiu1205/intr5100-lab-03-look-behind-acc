# Lab III Disturbance Comparison

The revised look-behind controller reduces the main modeled disturbance resonance and improves several independent sinusoidal tests relative to the matched front-only ablation. It trades away front-gap tracking accuracy and does not establish string stability or physical safety. All results below are synthetic; no experimental record was changed.

## Main results

| Measure | Front only | Revised look behind |
| --- | ---: | ---: |
| Peak sampled local head-to-tail gain, lag 0.5 s | 2.337 | 1.032 |
| Held-out 0.015 Hz fitted head-to-tail gain | 1.135 | 0.882 |
| Held-out 0.04 Hz fitted head-to-tail gain | 2.016 | 0.878 |
| Front-gap target RMSE in the 0.04 Hz test, cu | 9.77 | 25.76 |

Gain below one means attenuation for that measured response. The local peak is still above one, so the local model does not meet a full frequency-domain string-stability condition. All eight examined no-delay local models are asymptotically stable; their pole radii below one are not a certificate for quantization, switching, delay or the physical vehicle.

![Matched synthetic comparison](<results/Disturbance Comparison.png>)

The front-only and revised tests use identical front targets, initial conditions, leader input, supplied following IDM and protections. The front-only controller is an ablation with the same front gains, not a separately optimized algorithm. The sine amplitude is four reported-speed units unless indicated otherwise. After an 80 s settling interval, four complete periods are fitted using sine, cosine and intercept terms. The continuous-frequency model and integer-command simulation are separate sources of evidence.

## Every held-out case

The first six rows use fitted sinusoidal amplitude gain. Other sinusoidal rows use the same fitted gain. Pulse, stop, gap, plant, scale and dropout rows use demeaned tail/leader speed standard-deviation ratios. These metrics are labeled by case and should not be pooled into one ranking.

| Case | Front only gain or ratio | Look behind gain or ratio | Front RMSE cu baseline / revised | Revised minimum rear gap cu |
| --- | ---: | ---: | ---: | ---: |
| sine 0.015 | 1.135 | 0.882 | 9.2 / 27.4 | 284.2 |
| sine 0.04 | 2.016 | 0.878 | 9.8 / 25.8 | 273.0 |
| sine 0.1 | 0.000 | 0.000 | 9.6 / 16.2 | 303.4 |
| sine 0.22 | 0.000 | 0.000 | 9.2 / 10.9 | 310.4 |
| delayed sine | 0.301 | 0.808 | 8.2 / 30.5 | 259.1 |
| small amplitude | 0.001 | 0.000 | 5.8 / 14.3 | 301.9 |
| pulse | 1.104 | 1.047 | 24.2 / 42.2 | 204.8 |
| stop | 1.284 | 1.087 | 38.0 / 71.2 | 131.9 |
| tight rear | 1.198 | 1.048 | 24.2 / 44.6 | 180.0 |
| small front gap | 1.113 | 1.107 | 44.3 / 73.3 | 202.7 |
| large front gap | 1.122 | 0.980 | 68.7 / 103.6 | 203.3 |
| fast plant | 1.058 | 0.955 | 27.2 / 37.1 | 235.0 |
| slow plant | 1.464 | 1.148 | 17.8 / 75.8 | 168.3 |
| gain low | 0.004 | 0.001 | 285.5 / 519.8 | 315.3 |
| gain high | 3.934 | 3.756 | 180.5 / 214.0 | 71.7 |
| scale low | 4.306 | 4.277 | 180.3 / 211.7 | 69.7 |
| scale high | 0.002 | 0.006 | 183.2 / 435.8 | 315.3 |
| 020 period sine | 1.326 | 0.858 | 9.6 / 20.0 | 285.8 |
| 020 period pulse | 1.081 | 1.000 | 23.8 / 39.7 | 218.2 |
| rear dropout | 1.892 | 1.725 | 24.2 / 71.5 | -166.1 |

## Failures and interpretation

- The delayed 0.06 Hz case worsens: fitted gain rises from 0.301 to 0.808. Improvement is not uniform.
- Front-gap RMSE increases across this suite. At 0.04 Hz it rises from 9.77 to 25.76 cu. The attenuation gain is a tracking tradeoff, not a free improvement.
- Some high-frequency or small-amplitude cases have nearly zero tail response because integer command deadbands hold the simulated follower speed nearly constant. This is not evidence for a linear zero transfer gain. The unquantized local analysis exposes the remaining resonances.
- Command-gain-high and telemetry-scale-low cases enter the rear shield band, with revised minimum gaps about 71.7 and 69.7 cu. Both controllers are sensitive to scale error.
- Command-gain-low and telemetry-scale-high cases produce very large rear gaps and low following mean speed. Their small variation ratios reflect loss of formation, not successful damping. Full mean speeds and maximum gaps remain in the JSON evidence.
- A 10 s frozen rear-telemetry stress case causes simulated rear overlap in BOTH designs: minimum rear gap is −167.8 cu for front-only and −166.1 cu for revised control. The controller disables stale rear feedback, but the unchanged following IDM still operates on stale received state. Neither design is demonstrated collision-safe under that fault. The simulation continues after the first overlap solely to retain the full failure record; subsequent trajectories are not physically meaningful.

## Verification

Sixteen deterministic logic tests pass. The actual unmodified TA GUI loop passes 35 active cycles, rear-IDM bypass, student bypass and front stopping, with network connection attempts prohibited and no callback-error fallback. These establish software compatibility, not experimental performance.

Halving the synthetic integration step from 0.05 to 0.025 s changes the checked attenuation metrics by at most 1.01% across two sinusoidal cases and one stopping case for both controllers. The 0.15 s controller sampling interval and integer commands are held fixed. A separate finite-difference step check confirms consistency of the local Jacobian.

## Files and reproduction

- `student_controller3.py`: current single-file submission; default rear weight 1.0.
- `tests/compare_disturbance.py`: fixed development grid and held-out nonlinear evaluation.
- `tests/check_linear_response.py`: local sampled model, poles and frequency curves.
- `tests/build_comparison.py`: numerical refinement, paired traces and plot.
- `review/gain_selection.json`: every development candidate and selection rule.
- `review/disturbance_comparison.json`: full held-out metrics, including failed cases.
- `review/linear_response.json`: local matrices, poles and frequency samples.
- `review/numerical_refinement.json`: coarse/fine integration comparison.

Use the shared `intr5100` environment and the commands in [README](README.md). Only submit `student_controller3.py`. A physical front-only comparison can use a copy with only the module-level instantiation changed to `LookBehindController(rear_weight=0.0)`; all other front settings and protections must remain matched.
