# Lab III Report Discussion Notes

These points describe pre-lab synthetic tests. They must remain distinct from the physical Lab III results when the report is written.

## Tracking and disturbance attenuation

Equilibrium-referenced rear feedback reduces the main modeled disturbance resonance. The current version also uses a slow rear-spacing washout filter and joint feedback tuning to reduce excessive front-gap error. In the 0.04 Hz comparison, gap RMSE decreases from 25.76 to 23.43 coordinate units and leader-to-follower amplitude gain decreases from 0.878 to 0.633. The stopping test reduces gap RMSE from 71.22 to 45.89 coordinate units. These results support an engineering tradeoff, not a universally optimal controller.

The improvement is not uniform. The 0.015 Hz test has slightly higher gap RMSE, and the local peak gain increases from about 1.032 to 1.055. Both remain substantially below the original front-only resonance, but a peak above one does not establish full string stability. Some transient speed-variation ratios also worsen. Future work could consider frequency-weighted tuning and a more explicit tracking-versus-attenuation objective.

## Telemetry limitations

Bounded prediction and confidence-weighted rear feedback avoid unnecessary full stops during short gaps. In one new 0.9 s leader-packet freeze, the earlier controller issues two stale-data stop decisions; the current controller issues none. The corresponding gap RMSE falls from 49.67 to 41.50 coordinate units. Prediction is restricted to a finite horizon, and cold stale data are not treated as a reliable initial condition.

The platform timestamps received packets, rather than providing a guaranteed measurement-generation timestamp. The controller can measure silence since reception but cannot identify all hidden transport latency. Simulations distinguish these two cases. One 0.45 s known-age case improves tracking while its disturbance gain rises above one. This limits any general delay-robustness claim.

Prolonged rear telemetry loss remains unresolved: both versions exhibit rear overlap in the 10 s freeze stress test. The unchanged following IDM continues to act on stale state. Altering the controlled middle vehicle alone cannot guarantee safe behavior of that follower. Post-overlap trajectories are not physically meaningful and must not be ranked as successful tests. Future improvements belong in the report discussion and, if later authorized, in a coordinated telemetry watchdog or revised experimental protocol.

## Model assumptions and experimental interpretation

Telemetry conversion and command-response gain remain separate 2.8 hypotheses. The own-motion predictor uses a nominal 0.5 s response and requested commands, not actuator acknowledgements. Shield overrides and scale mismatch can invalidate that prediction. Neither measured comfort nor fuel savings can be inferred from the command traces alone.

Keep the current implementation fixed for the planned comparison. Report the actual parameters, activation/bypass intervals, telemetry faults and observed outcomes. Discuss unresolved limitations openly rather than presenting the synthetic checks as physical validation or seeking a perfect algorithm before the lab.
