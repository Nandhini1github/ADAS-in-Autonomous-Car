# PID versus MPC: longitudinal model results

These are measured outputs of the repository's deterministic 1-D model, not CARLA or road measurements.
Each pair uses identical settings and observations up to AEB activation. Six cases are a parameter sweep, not statistical repetitions.

Both controllers fail the reduced-grip and delayed-brake stress cases with their current nominal models. These failures are included below.

| Case | Controller | Min gap (m) | AEB → stop (s) | Stop distance (m) | Comparison |
|---|---|---:|---:|---:|---|
| [urban_15_10](urban_15_10/comparison.md) | PID | 8.661 | 1.150 | 4.585 | valid |
| [urban_15_10](urban_15_10/comparison.md) | MPC | 8.898 | 1.100 | 4.333 | valid |
| [urban_25_15](urban_25_15/comparison.md) | PID | 10.604 | 1.850 | 11.472 | valid |
| [urban_25_15](urban_25_15/comparison.md) | MPC | 11.523 | 1.800 | 10.536 | valid |
| [arterial_35_20](arterial_35_20/comparison.md) | PID | 9.435 | 2.500 | 21.389 | valid |
| [arterial_35_20](arterial_35_20/comparison.md) | MPC | 11.762 | 2.350 | 19.046 | valid |
| [highway_50_25](highway_50_25/comparison.md) | PID | 2.544 | 3.550 | 41.995 | valid |
| [highway_50_25](highway_50_25/comparison.md) | MPC | 8.262 | 3.050 | 36.265 | valid |
| [reduced_grip_35_20](reduced_grip_35_20/comparison.md) | PID | -0.003 | N/A | N/A | invalid / overlap |
| [reduced_grip_35_20](reduced_grip_35_20/comparison.md) | MPC | -0.112 | N/A | N/A | invalid / overlap |
| [delayed_brake_50_25](delayed_brake_50_25/comparison.md) | PID | -0.107 | N/A | N/A | invalid / overlap |
| [delayed_brake_50_25](delayed_brake_50_25/comparison.md) | MPC | -0.332 | N/A | N/A | invalid / overlap |

## Reproduce

```bash
python3 scripts/run_model_benchmark.py --allow-stress-failures
```

Without `--allow-stress-failures`, the documented overlap cases cause a nonzero exit code. They remain invalid comparisons with either option.
The plant includes brake delay, an actuator time constant, and grip scaling. Controllers retain their nominal model in the stress cases.
See [methodology](../../docs/comparison_methodology.md) and [model assumptions](../../docs/model_benchmark.md).

## Example: 50/25 mph model traces

![50/25 mph PID and MPC model comparison](highway_50_25/comparison.png)
