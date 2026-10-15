# PID and MPC AEB comparison

**Source:** deterministic_1d_model

**Status:** DIAGNOSTICS ONLY — COMPARISON INVALID

Measured outputs of the simplified longitudinal model. These are not CARLA or physical vehicle results.

Invalidity reasons: pid:aborted_run, pid:no_stop_after_aeb, pid:gap_overlap_proxy, mpc:aborted_run, mpc:no_stop_after_aeb, mpc:gap_overlap_proxy

| Metric | PID | MPC | MPC − PID |
|---|---:|---:|---:|
| Samples | 161 | 164 | — |
| Observed duration (s) | 8.0000 | 8.1500 | — |
| Speed at AEB (mph) | 50.0000 | 50.0000 | — |
| Gap at AEB (m) | 44.5428 | 44.5428 | — |
| Minimum gap (m) | -0.1067 | -0.3317 | unavailable |
| Minimum finite TTC (s) | 0.0000 | 0.0000 | unavailable |
| AEB to stop threshold (s) | unavailable | unavailable | unavailable |
| AEB to stop threshold (m) | unavailable | unavailable | unavailable |
| Peak deceleration, finite difference (m/s²) | 7.7955 | 7.9927 | unavailable |
| Peak absolute jerk, finite difference (m/s³) | 45.3550 | 45.3550 | unavailable |
| Gap ≤ 0 proxy | True | True | — |
| Collision sensor contact | unavailable | unavailable | — |
| Final phase | ABORTED | ABORTED | — |
| Outcome | aborted | aborted | — |

Stop threshold: **1 mph**. Distance integrates ego speed from first AEB.

- **PID events (s):** FCW 4.5000; threat 3.0500; AEB 5.3000; stop unavailable. Controller runtime (ms): unavailable.
- **MPC events (s):** FCW 4.5000; threat 3.0500; AEB 5.3000; stop unavailable. Controller runtime (ms): unavailable.

No overall winner: gap, response and comfort trade off. Deltas require complete, matched, documented runs.

## Measurement limits

- Model traces are controller experiments, not CARLA or road validation.
- Gap <= 0 is an overlap proxy; contact requires separate sensor evidence.
- TTC assumes constant relative motion; nonclosing TTC is unavailable.
- Finite differences are unsmoothed and depend on sampling and speed quantization.
- Stop is the first sample at or below the speed threshold, not exact zero; distance uses trapezoids.
- One paired run does not establish statistical significance.

## Traces

![PID/MPC traces](comparison.png)

[Download SVG](comparison.svg)
