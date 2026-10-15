# PID and MPC AEB comparison

**Source:** deterministic_1d_model

**Status:** VALID PAIRED COMPARISON

Measured outputs of the simplified longitudinal model. These are not CARLA or physical vehicle results.

| Metric | PID | MPC | MPC − PID |
|---|---:|---:|---:|
| Samples | 204 | 203 | — |
| Observed duration (s) | 10.1500 | 10.1000 | — |
| Speed at AEB (mph) | 25.0000 | 25.0000 | — |
| Gap at AEB (m) | 22.0791 | 22.0791 | — |
| Minimum gap (m) | 10.6039 | 11.5229 | 0.9190 |
| Minimum finite TTC (s) | 1.8201 | 1.8201 | 0.0000 |
| AEB to stop threshold (s) | 1.8500 | 1.8000 | -0.0500 |
| AEB to stop threshold (m) | 11.4722 | 10.5357 | -0.9365 |
| Peak deceleration, finite difference (m/s²) | 6.5693 | 7.9858 | 1.4165 |
| Peak absolute jerk, finite difference (m/s³) | 78.6328 | 45.3550 | -33.2779 |
| Gap ≤ 0 proxy | False | False | — |
| Collision sensor contact | unavailable | unavailable | — |
| Final phase | COMPLETE | COMPLETE | — |
| Outcome | completed_with_positive_gap | completed_with_positive_gap | — |

Stop threshold: **1 mph**. Distance integrates ego speed from first AEB.

- **PID events (s):** FCW 6.3000; threat 3.0500; AEB 7.3000; stop 9.1500. Controller runtime (ms): unavailable.
- **MPC events (s):** FCW 6.3000; threat 3.0500; AEB 7.3000; stop 9.1000. Controller runtime (ms): unavailable.

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
