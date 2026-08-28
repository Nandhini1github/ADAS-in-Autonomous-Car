# Baseline CSV Analysis

Source: the uploaded `town10_aeb_pid.csv` from the 15 mph validation run.

| Metric | Observed value |
|---|---:|
| Samples | 414 |
| Duration | 26.545 s |
| Logged phases | `CRUISE` only |
| Maximum ego speed | 16.361 mph |
| Maximum target speed | 11.252 mph |
| Minimum gap | 14.685 m |
| Minimum finite TTC | 7.555 s |
| Maximum absolute steering command | 0.0313 |
| FCW / AEB / braking samples | 0 / 0 / 0 |
| Final ego / target speed | 15.601 / 11.252 mph |

The CSV supports retaining conservative lateral tuning: steering remained far below
the configured 0.18 limit. It does **not** validate AEB PID or MPC gains because the
target never entered a braking phase, TTC never crossed either threshold, and the
ego never requested brake. The new state machine therefore requires stable speed
hold before an explicit target hard-brake phase. AEB tuning must be revisited only
after new PID and MPC logs contain actual AEB activation.
