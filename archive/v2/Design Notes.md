# Lab III Controller Design

## Revised algorithm

The submission implements equilibrium-referenced Leading Cruise Control in desired-speed form. The front-only component is unchanged; the rear component now damps deviations from a slowly varying leader-speed reference instead of chasing the follower’s relative speed. Offline gain selection explicitly targets the leader-to-follower response.

Wang et al. [2], equation 27, adds rear spacing and rear speed errors relative to an equilibrium. The demonstrated gains in that paper can be negative for both errors. This provides a more specific design basis than the initial bilateral draft, whose positive rear-to-own relative-speed term could reinforce disturbances. Our scaling, speed-command interface, filters and protections differ from the paper. Its stability conclusions do not transfer to this controller.

Horn [1] motivates using both adjacent vehicles. Li et al. [3], section III, motivates checking the complete mixed-platoon transfer response rather than judging the controlled vehicle alone. That paper uses LQR and an OVM; we instead retain the supplied IDM and select a small structured gain set offline. No online optimizer or Riccati solver is required.

## Equations and selected parameters

Positions and gaps use route coordinate units, not metres. Let v0, v1 and v2 denote leader, student and follower coordinate speeds. Front and rear bumper gaps sF and sR subtract the supplied 250-cu vehicle length.

The unchanged front policy is

- dF = 300 + 1.8 v1.
- rF = v0 + 0.10(sF − dF) + 0.25(v0 − v1).

The leader reference vRef is low-pass filtered with a 6 s time constant and initialized to the current leader speed on activation. In reported-speed units, qRef = clip(vRef/γ, 0, 0.85 × 70). The rear equilibrium reference uses the **public default** supplied IDM:

sRRef = (250 + 1.2 qRef) / sqrt[1 − (qRef/70)⁴].

This is the zero-acceleration, equal-speed equilibrium of that supplied implementation, not a fit to experimental records. If TA IDM parameters change, these reference constants require review; supporting files are never edited.

The rear correction is

rR = clip[−0.20(sR − sRRef) − 1.00(v2 − vRef), −30, 30].

A 0.35 s filter smooths the rear term. A front-headroom gate fades it to zero at the 120-cu reserve and reaches one at the front target. Extra slowing from rear feedback is also suppressed when the rear gap is small or the follower is closing. This rear guard limits only the optional correction; necessary front braking retains priority. Invalid, stale or bypassed rear data immediately clear the rear filter.

The desired command is (rF + gated rR)/β. The default rear weight is now 1.0; the matched front-only ablation uses zero. Commands remain bounded to 0–70, with normal increase/decrease limits of 6/12 command units per second. These limits do not measure physical acceleration or comfort.

γ = 2.8 converts reported speed to coordinate speed; β = 2.8 converts desired coordinate speed to command. They remain **independent uncalibrated hypotheses**. Positions and bumper gaps receive no extra scaling. The published papers’ metre-based gains were not copied into these coordinate units.

## Protections and state management

The front reserve is 120 cu. TTC uses positive closing speed v1 − v0; opening gaps have no finite closing TTC. A reserve breach or TTC below 1.5 s stops the command. A conditional stopping-distance cap assumes 45 cu/s² braking and 0.5 s reaction allowance:

vSafe = max(0, −bτ + sqrt[(bτ)² + v0² + 2b max(sProtected − 120, 0)]).

The command is capped by vSafe/β after rate limiting. This is a model-dependent guard, not a safety proof. The TA-owned sensor and spacing shields remain unchanged. Those shields cannot guarantee safety under stale follower data and actuator/model mismatch.

The controller rejects invalid/stale front states and missing geometry with zero output. Rear faults disable only rear feedback. History is read without mutation. Memory resets on history replacement/shrinkage, rewound history time, long callback gaps and selected-section transitions. There is no integrator to wind up during bypass or saturation. The required callback signature and float speed-command return are unchanged.

## Gain selection and comparison

A fixed 60-candidate grid varied rear gap gain, rear speed gain and leader-reference time constant. Selection used only six development sinusoidal cases: 0.025, 0.075 and 0.15 Hz, each with assumed plant lag 0.5 or 0.8 s. The objective combined worst and mean fitted tail gains with worst total speed-variation ratio. Feasibility constraints rejected negative/small gaps, large spacing drift, excessive mean-speed bias and poor front tracking.

The selected gains were frozen before evaluating 20 separate comparison cases. These are held out from this parameter search, not independent experimental data. The comparison is a rear-feedback ablation with identical front gains, headway, initial states, leader inputs, following IDM and shields. It does not establish superiority over an independently optimized front-only controller.

The local sampled-data analysis differentiates the actual controller and supplied IDM around reported speed 40. It includes controller filters and an assumed held-command first-order plant. Quantization, active caps, delays and switching are excluded from this local model. The nonlinear simulations include integer commands, switching protections, delays and scale stress. Supporting code is used read-only.

See [Disturbance Comparison](<Disturbance Comparison.md>) for gains, spacing tradeoffs, numerical checks and failures. The initial draft and its original synthetic evidence are retained in `review/archive/`, clearly separate from the current results.

## References

1. B. K. P. Horn. Suppressing Traffic Flow Instabilities. IEEE ITSC, 2013. [Author manuscript](https://people.csail.mit.edu/bkph/articles/Suppressing_Traffic_Flow%20Instabilities_IEEE_ITS_2013.pdf).
2. J. Wang, Y. Zheng, C. Chen, Q. Xu and K. Li. Leading Cruise Control in Mixed Traffic Flow: System Modeling, Controllability, and String Stability. [Author preprint revised 2021](https://arxiv.org/html/2012.04313v2), equations 27–31 and section V-A; DOI 10.1109/TITS.2021.3118021.
3. S. Li et al. Information Flow Topology in Mixed Traffic: A Comparative Study between Looking Ahead and Looking Behind. IEEE ITSC, 2023. [Author preprint](https://arxiv.org/html/2309.01625v1), section III. The study uses LQR; the submission does not claim to reproduce its controller or guarantees.
