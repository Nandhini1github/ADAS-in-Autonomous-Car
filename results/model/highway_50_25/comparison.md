# PID and MPC AEB comparison

**Source:** deterministic_1d_model

**Status:** VALID PAIRED COMPARISON

Measured outputs of the simplified longitudinal model. These are not CARLA or physical vehicle results.

| Metric | PID | MPC | MPC − PID |
|---|---:|---:|---:|
| Samples | 198 | 188 | — |
| Observed duration (s) | 9.8500 | 9.3500 | — |
| Speed at AEB (mph) | 50.0000 | 50.0000 | — |
| Gap at AEB (m) | 44.5428 | 44.5428 | — |
| Minimum gap (m) | 2.5445 | 8.2619 | 5.7174 |
| Minimum finite TTC (s) | 0.8806 | 1.4447 | 0.5641 |
| AEB to stop threshold (s) | 3.5500 | 3.0500 | -0.5000 |
| AEB to stop threshold (m) | 41.9949 | 36.2654 | -5.7295 |
| Peak deceleration, finite difference (m/s²) | 6.5693 | 8.0000 | 1.4307 |
| Peak absolute jerk, finite difference (m/s³) | 85.4729 | 57.9508 | -27.5221 |
| Gap ≤ 0 proxy | False | False | — |
| Collision sensor contact | unavailable | unavailable | — |
| Final phase | COMPLETE | COMPLETE | — |
| Outcome | completed_with_positive_gap | completed_with_positive_gap | — |

Stop threshold: **1 mph**. Distance integrates ego speed from first AEB.

- **PID events (s):** FCW 4.5000; threat 3.0500; AEB 5.3000; stop 8.8500. Controller runtime (ms): unavailable.
- **MPC events (s):** FCW 4.5000; threat 3.0500; AEB 5.3000; stop 8.3500. Controller runtime (ms): unavailable.

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
