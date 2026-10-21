# Scenario 2: rear camera AEB during a child crossing

Scenario 2 reverses a Lincoln MKZ from a curbside parking space in
`Town10HD_Opt`. After the car has moved backward 1 m, a child leaves the
sidewalk and crosses the road behind it. Rear camera observations feed an
AEB supervisor that applies full braking and holds the car stopped while
the child finishes crossing.

**Validated in the packaged CARLA 0.10.0 simulator on 21 October 2026.**
The [recorded run](<../AEB scenario 2/>) completed with zero collision contacts,
the child crossed the road, and the car remained stopped. See the
[video](<../AEB scenario 2/rear_aeb.mp4>),
[CSV](<../AEB scenario 2/rear_aeb.csv>), and
[metadata](<../AEB scenario 2/metadata.json>).

| Recorded event or measurement | Result |
|---|---|
| Child begins crossing | `2.50 s` |
| AEB activates for predicted crossing | `3.95 s` |
| Reverse speed at AEB | `1.322 m/s` (`4.76 km/h`) |
| Camera-estimated clearance at AEB | `2.299 m` |
| First speed at or below `0.08 m/s` | `4.20 s` |
| Child crosses reverse centerline | `5.55 s` |
| Child finishes / scenario completes | `17.35 s` |
| Minimum evaluated rear clearance during lateral overlap | `1.965 m` |
| Measured walking speed during crossing | `1.25 m/s` |
| Collision sensor contacts | `0` |
| CSV rows / video frames | `347 / 347`, at `20 Hz` |

Clearance during overlap uses projected actor bounding boxes for independent
evaluation. It is separate from the RGB/depth clearance that triggers braking.
The stop threshold is not exact zero speed; the final sample is stationary
with full braking. Video frame zero matches the first CSV sample at `0.05 s`.
The source and recorded artifact hashes were verified. The host client used
OpenCV `4.6.0`; the simulator ran from its existing Docker package. Unit tests
also cover decoding, threat geometry, reverse gating, frame errors, and failure
cleanup. This is one scene demonstration, not general perception validation.

## Selected parking space

The scene uses an open section of the residential curbside parking strip on
**Road 8, shoulder lane +3**, between existing parked vehicles. An overhead
RGB inspection confirmed the parked row and available space. The local
OpenDrive describes this strip as a shoulder, not a `parking` lane.

| Element | Configured CARLA world pose or value |
|---|---|
| Map | `Town10HD_Opt` |
| Ego | `(x, y, z) = (7.5, 127.55, 0.5) m`, yaw `180°` |
| Vehicle | `vehicle.lincoln.mkz` |
| Child start | `(15.0, 123.8, 0.6) m`, yaw `90°` |
| Child destination | `(15.0, 142.5, 0.6) m` |
| Child blueprint | `walker.pedestrian.0048`; runtime `age=child` required |
| Requested reverse speed | `1.4 m/s`, approximately `5.0 km/h` |
| Desired child speed | `1.25 m/s`; release-specific command scaling described below |
| Crossing trigger | Ego reverse displacement reaches `1.0 m` |
| Simulation step | `0.05 s`, synchronous, `ClearNoon` |

At the ego location the 2 m shoulder spans approximately
`y=126.54..128.54 m`; its center is near `y=127.54 m`. The curb and sidewalk
are on the lower-Y side. The child starts on sidewalk lane +5, walks toward
increasing Y, crosses the parking strip, and continues across the roadway.
The ego faces decreasing X and reverses toward increasing X with zero
steering. Spawn Z values are initial clearances; CARLA physics settles the
actors before observations are recorded.

The runner requires the selected placement to be free. A blocked spawn fails
explicitly instead of selecting another location. It records the settled
poses and runtime road/lane classification in `metadata.json`.

The installed simulator exposes child blueprints `.0048` and `.0049`. The
[official UE5 pedestrian catalogue](https://carla-ue5.readthedocs.io/en/latest/catalogue_pedestrians/)
also identifies those models as children. The runner checks the configured
blueprint's age attribute before spawning it.

### CARLA 0.10.0 compatibility

Reverse commands use `reverse=True`, `gear=-1`, and
`manual_gear_shift=False`, allowing the automatic transmission to engage
reverse. During inspection of this shipped vehicle, enabling manual gear
shift caused the gear to alternate with neutral. AEB is gated by the observed
reverse flag and negative gear; the CSV records that gate in `reverse_gear`.

After settling, the runner assigns a single **0.1 m/s reverse velocity** to
wake the stationary Chaos physics body. This initialization is recorded in
metadata; subsequent motion follows CARLA physics and vehicle control
commands. CARLA 0.10's sleeping-body behavior and the use of a small velocity
assignment to wake it are also documented by the
[Autoware CARLA interface](https://github.com/autowarefoundation/autoware_universe/blob/main/simulator/autoware_carla_interface/README.md).

The configured `walker_speed_scale=20.48` compensates for the shipped
`walker.pedestrian.0048` movement implementation. CARLA 0.10.0 disables the
assignment that would align the character movement cap with the controller's
maximum, while still dividing movement input by that maximum. See the tagged
[WalkerController.cpp](https://github.com/carla-simulator/carla/blob/0.10.0/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Walker/WalkerController.cpp)
and its [4096 cm/s constant](https://github.com/carla-simulator/carla/blob/0.10.0/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Walker/WalkerController.h).
The installed child has a 200 cm/s movement cap, giving the correction
`4096 / 200 = 20.48`.

For a desired **1.25 m/s**, the runner therefore passes **25.6** as
`WalkerControl.speed`. That command value is not the measured walking speed.
Metadata records both `requested_speed_mps` and `walker_control_speed`;
`measured_child_speed_mps` records actual actor speed in the CSV. This
correction is specific to the selected blueprint and shipped release. For a
runtime with corrected speed handling, set `walker_speed_scale=1.0` and verify
the measured speed before interpreting crossing timing.

## Camera observations and braking

The default `rgb_depth` perception mode uses two rear sensors with the same
pose and calibration:

| Sensor | Role |
|---|---|
| `sensor.camera.rgb` | YOLOX-s person detection, rear video, and saved braking frame |
| `sensor.camera.depth` | Depth within detected RGB boxes, used to estimate rear clearance and lateral position |

Both use **640 × 360 pixels**, **110° horizontal field of view**, and the
**20 Hz** simulation cadence. They are rigidly attached on the vehicle
centerline, **1 m above the ego origin**, **0.05 m behind the rear bounding-box
bumper**, facing backward at relative yaw **180°**. Lens distortion is
disabled for the pinhole projection. Each control update requires matching
simulation frame numbers from all active cameras; a missing or skipped frame
terminates the run.

The RGB detector runs the OpenCV Zoo **YOLOX-s** ONNX model through OpenCV DNN
on the CPU. It filters predictions to COCO's `person` class, associates
detections across frames, and samples a coherent depth region within each RGB
box. The camera calibration and depth yield rear clearance, lateral position,
and estimated width. Neither detection nor range estimation queries the
child's actor pose, velocity, or bounding box. The detector does not classify
age; the scenario's child identity comes from its verified blueprint.

**Distance comes from CARLA's simulated depth camera**, not from monocular
RGB distance estimation. Bounding-box error, occlusion, depth-region
selection, missed detections, and track association remain limitations.
The 20 Hz rate is simulation time: synchronous execution waits for inference
and does not establish a 20 Hz wall-clock throughput guarantee.

The CARLA 0.10.0 shipping build has a known omission of pedestrians in
semantic/instance segmentation output. The reports and planned release fix
are documented in [CARLA issue #8731](https://github.com/carla-simulator/carla/issues/8731)
and [CARLA issue #9014](https://github.com/carla-simulator/carla/issues/9014).
This is why the default uses RGB detection. An optional `instance_depth` mode
retains a synthetic label-and-depth baseline for renderer versions whose
pedestrian labels have been verified. That mode adds a co-located
`sensor.camera.instance_segmentation`; RGB supplies its review image.

The AEB supervisor combines these detections with measured ego reverse speed
and the reverse-gear gate. Consecutive image detections estimate lateral
motion and predict a crossing into the car's rear corridor. A threat in that
corridor triggers braking when either the TTC threshold or the stopping
distance threshold is reached:

```text
stopping threshold = stop margin + speed × reaction time
                     + speed² / (2 × assumed braking deceleration)
```

| Supervisor setting | Default |
|---|---|
| Lateral margin beyond ego width | `0.25 m` |
| Stop margin | `0.8 m` |
| Reaction allowance | `0.25 s` |
| Assumed braking deceleration | `4.0 m/s²` |
| TTC trigger | `1.8 s` |
| Crossing prediction horizon | Up to `2.0 s` |

A speed PID regulates normal reverse motion. Once AEB triggers, the runner
commands zero throttle and full brake, and keeps AEB latched. A completed
scenario requires AEB activation, a stop at or below **0.08 m/s** held for
**1 s**, the child crossing the reverse centerline and reaching its
destination, and no collision sensor contacts or cleanup errors.

Child actor state controls the crossing choreography and independent outcome
measurements. It does not supply pedestrian detections or emergency braking
demand. Ego state supplies speed, travel limits, and motion checks. The run
aborts on contact, unexpected forward/lateral/rotational motion, or **8 m**
of reverse travel; it also has a **25 s** simulation timeout. These limits
support a controlled simulator demonstration, not a validated vehicle safety
system. A successful run demonstrates one scene, not general detector
accuracy or performance with physical depth sensors.

## Run with the packaged simulator

Use the installed **CARLA 0.10.0 packaged server and matching Python API**.
No CARLA source checkout or source build is required. The client environment
needs **NumPy, OpenCV (`opencv-python-headless`), and Pillow**, plus **FFmpeg** for
video. The matching CARLA wheel is installed separately. In a prepared local
Python environment, install the Scenario 2 optional dependencies with:

```bash
python -m pip install -e '.[scenario2]'
```

The [OpenCV Zoo model documentation](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox)
recommends OpenCV 4.8 or newer. The project Dockerfile installs the
distribution's `python3-opencv` package for the default RGB detector. Rebuild
an older client image to pick up this dependency.

Start the existing simulator and controller container if needed:

```bash
bash scripts/docker.sh simulator
bash scripts/docker.sh start
```

With the default `desktop-linux` controller container, run from the repository
root:

```bash
bash scripts/docker.sh exec python scripts/download_rear_detector.py
bash scripts/docker.sh exec python scripts/run_rear_camera_aeb.py \
  --host host.docker.internal --port 2020
```

For a local Python environment with the matching CARLA API installed:

```bash
python scripts/download_rear_detector.py
python scripts/run_rear_camera_aeb.py --host 127.0.0.1 --port 2020
```

The downloader fetches the approximately **36 MB**
`object_detection_yolox_2022nov.onnx` model into `.runtime/models/`. It checks
SHA-256 before installing or reusing the file:

```text
c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063
```

The model comes from
[OpenCV Zoo's YOLOX directory](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox),
whose files are licensed under Apache 2.0. The downloader rejects an existing
file with a different hash. The weights are not committed to this repository.

The default configuration is
[`config/town10_rear_aeb.json`](../config/town10_rear_aeb.json). Use `--config`
to select another configuration, `--output-dir` to choose a new output
directory, `--model` to select a YOLOX-s ONNX file, or `--no-video` to omit the
overview camera and MP4. The config's `perception` value defaults to
`rgb_depth`; use `instance_depth` only with
verified pedestrian segmentation. An explicitly selected output directory
must not already exist.

The runner requires an **idle server**: no active vehicle, walker, or sensor
actors and no other synchronous client. The server must already have
`Town10HD_Opt` loaded; add `--load-map` to permit loading it when the idle
server has a different map. Cleanup removes the scenario's actors and sensors
and restores the saved weather and world settings. A map loaded with
`--load-map` remains loaded afterward.

## Recorded artifacts

By default, each run writes a new `results/scenario_2_<UTC timestamp>/`
directory. Exit code zero means the metadata reports a completed run;
otherwise inspect its outcome and error fields.

| Artifact | Contents |
|---|---|
| `config.json` | Configuration used for this run |
| `rear_aeb.csv` | Dedicated Scenario 2 observations and applied commands |
| `metadata.json` | Outcome, event times, contact events, versions, poses, camera/perception settings, startup initialization, source/artifact hashes, and cleanup status |
| `rear_aeb.mp4` | Rear RGB and external overview side by side, with detections and telemetry; omitted with `--no-video` |
| `rear_at_brake.png` | Rear RGB frame at first AEB activation |
| `instance_at_brake.png` | Unconverted instance image from that same frame, when using `instance_depth` |
| `depth_at_brake.png` | Unconverted encoded depth image from that same frame |
| `overview_at_brake.png` | External RGB view at AEB activation; omitted with `--no-video` |

Braking images exist only when AEB activates. Encoded depth PNGs and optional
instance PNGs retain sensor channel data and need their corresponding decoders
for analysis.

CSV columns include simulation `frame` and `time_s`, scenario `phase`, signed
and reverse speed in **m/s**, reverse travel in **m**, visible pedestrian
count, `measured_child_speed_mps`, camera-derived `rear_clearance_m` and
`lateral_m`, `ttc_s`, AEB status
and reason, and throttle/brake commands on **0..1** scales. Crossing flags
and collision count describe outcome progress. Columns prefixed `gt_` are
independent simulator-state measurements, not perception inputs. Missing
detections produce empty observation cells rather than zero range.

This schema is separate from Scenario 1's 12-column PID/MPC traces. Do not
pass `rear_aeb.csv` to `compare_aeb_results.py`; Scenario 2 currently records
a single reverse AEB policy.

Implementation: [runner](../src/town10_adas/rear_runner.py),
[RGB detector](../src/town10_adas/rear_rgb.py),
[AEB supervisor and optional instance perception](../src/town10_adas/rear_perception.py),
[video recorder](../src/town10_adas/rear_video.py), and
[CLI entry point](../scripts/run_rear_camera_aeb.py).
