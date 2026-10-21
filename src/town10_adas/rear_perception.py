"""Synthetic rear-camera perception and a latched reverse AEB supervisor.

This is a simulator baseline using CARLA's rendered instance labels and depth,
not an RGB detector or a deployable safety system. Actor positions/velocities
are deliberately absent from this module. NumPy is needed only for decoding.

Both cameras must have the same pinhole calibration (lens_k=lens_kcube=0),
frame and pose: yaw=180, pitch=roll=0, centered laterally, just behind the rear
bumper. CARLA depth is axial z-buffer depth, encoded in raw BGRA bytes; see
https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera and
https://carla.readthedocs.io/en/latest/ref_sensors/#instance-segmentation-camera.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Integral
from typing import Any, Iterable


@dataclass(frozen=True)
class CameraPedestrian:
    """Visible pedestrian surface, in ego coordinates (+y is vehicle right).

    Positive clearance is behind the rear bumper. The opaque instance ID is
    derived from image channels, and must never be treated as a CARLA actor ID.
    Bounding box right/bottom coordinates are exclusive.
    """

    instance_id: int
    clearance_m: float
    lateral_m: float
    width_m: float
    bbox_xyxy: tuple[int, int, int, int] = (0, 0, 0, 0)
    pixel_count: int = 0


def _positive(name: str, value: float, *, zero: bool = False) -> None:
    if not math.isfinite(value) or value < 0.0 or (not zero and value == 0.0):
        qualifier = "nonnegative" if zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")


def _image_shape(image: Any) -> tuple[int, int]:
    for name in ("frame", "width", "height"):
        value = getattr(image, name, None)
        if not isinstance(value, Integral) or isinstance(value, bool):
            raise ValueError(f"image {name} must be an integer")
        if value < (0 if name == "frame" else 1):
            raise ValueError(f"invalid image {name}")
    return image.height, image.width


def detect_pedestrians(
    instance_image: Any,
    depth_image: Any,
    fov_deg: float,
    camera_to_bumper_m: float = 0.05,
    min_pixels: int = 8,
    max_range_m: float = 15.0,
    pedestrian_tag: int = 12,
) -> list[CameraPedestrian]:
    """Decode aligned, unconverted CARLA instance/depth images.

    ``camera_to_bumper_m`` is the positive distance the camera extends behind
    the bumper. Current CARLA uses tag 12 for pedestrians; older installations
    require an explicit tag override. IDs combine green/blue as G*256+B.
    Pixel support and range filtering reject tiny fragments and invalid depth.
    The tenth percentile depth is a robust estimate of the nearest surface.
    """
    _positive("fov_deg", fov_deg)
    if fov_deg >= 180.0:
        raise ValueError("fov_deg must be less than 180")
    _positive("camera_to_bumper_m", camera_to_bumper_m, zero=True)
    _positive("max_range_m", max_range_m)
    if max_range_m >= 1000.0 or camera_to_bumper_m >= max_range_m:
        raise ValueError("range must exceed camera offset and be below 1000 m")
    if not isinstance(min_pixels, Integral) or isinstance(min_pixels, bool) or min_pixels < 1:
        raise ValueError("min_pixels must be a positive integer")
    if (not isinstance(pedestrian_tag, Integral) or isinstance(pedestrian_tag, bool)
            or not 0 <= pedestrian_tag <= 255):
        raise ValueError("pedestrian_tag must be a byte")
    shape = _image_shape(instance_image)
    if shape != _image_shape(depth_image):
        raise ValueError("instance/depth image dimensions do not match")
    if instance_image.frame != depth_image.frame:
        raise ValueError("instance/depth image frames do not match")
    for img in (instance_image, depth_image):
        if hasattr(img, "fov") and (
            not math.isfinite(img.fov) or not math.isclose(img.fov, fov_deg, abs_tol=1e-5)
        ):
            raise ValueError("image field of view does not match calibration")
    if hasattr(instance_image, "timestamp") and hasattr(depth_image, "timestamp"):
        timestamps = instance_image.timestamp, depth_image.timestamp
        if not all(math.isfinite(t) for t in timestamps) or not math.isclose(
            *timestamps, rel_tol=0.0, abs_tol=1e-6
        ):
            raise ValueError("instance/depth image timestamps do not match")

    import numpy as np

    arrays = []
    for img in (instance_image, depth_image):
        raw = np.frombuffer(img.raw_data, dtype=np.uint8)
        if raw.size != shape[0] * shape[1] * 4:
            raise ValueError("image raw_data must contain width*height*4 BGRA bytes")
        arrays.append(raw.reshape(*shape, 4))
    instance, depth = arrays
    semantic = instance[:, :, 2]
    ids = instance[:, :, 1].astype(np.uint16) * 256 + instance[:, :, 0]
    axial_m = (depth[:, :, 2].astype(np.float64)
               + depth[:, :, 1].astype(np.float64) * 256.0
               + depth[:, :, 0].astype(np.float64) * 65536.0) * (1000.0 / 16777215.0)
    valid = ((semantic == pedestrian_tag) & (axial_m > 0.0)
             & (axial_m + camera_to_bumper_m <= max_range_m))
    focal_px = shape[1] / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    detections = []
    for instance_id in np.unique(ids[valid]):
        rows, cols = np.nonzero(valid & (ids == instance_id))
        if len(rows) < min_pixels:
            continue
        distances = axial_m[rows, cols]
        nearest_m = float(np.percentile(distances, 10.0))
        # At yaw 180 degrees, image-right is ego-left. Pixel-center sampling.
        lateral = -(cols + 0.5 - shape[1] / 2.0) * distances / focal_px
        left, right = float(lateral.min()), float(lateral.max())
        detections.append(CameraPedestrian(
            instance_id=int(instance_id),
            clearance_m=nearest_m + camera_to_bumper_m,
            lateral_m=(left + right) / 2.0,
            width_m=right - left + nearest_m / focal_px,
            bbox_xyxy=(int(cols.min()), int(rows.min()), int(cols.max()) + 1, int(rows.max()) + 1),
            pixel_count=len(rows),
        ))
    return sorted(detections, key=lambda detection: detection.clearance_m)


@dataclass(frozen=True)
class RearAEBDecision:
    brake: float
    active: bool
    reason: str
    target_instance_id: int | None
    clearance_m: float | None
    ttc_s: float | None
    stopping_distance_m: float


class RearAEB:
    """Brake on rear corridor threats; hold the stop until explicit reset.

    Supply a nonnegative reverse speed magnitude and a reverse/gear gate.
    Lateral motion is estimated only from successive image detections. A
    small first-observation corridor expansion avoids waiting for a velocity
    estimate when a pedestrian first appears very close to the vehicle path.
    This straight-reverse model assumes zero steering and a level camera.
    """

    def __init__(
        self,
        ego_half_width_m: float = 1.0,
        lateral_margin_m: float = 0.25,
        stop_margin_m: float = 0.8,
        reaction_s: float = 0.25,
        braking_decel_mps2: float = 4.0,
        ttc_threshold_s: float = 1.8,
        prediction_horizon_s: float = 2.0,
        near_corridor_margin_m: float = 0.25,
        track_timeout_s: float = 0.5,
    ) -> None:
        for name, value in (
            ("ego_half_width_m", ego_half_width_m),
            ("braking_decel_mps2", braking_decel_mps2),
            ("ttc_threshold_s", ttc_threshold_s),
            ("prediction_horizon_s", prediction_horizon_s),
            ("track_timeout_s", track_timeout_s),
        ):
            _positive(name, value)
        for name, value in (
            ("lateral_margin_m", lateral_margin_m), ("stop_margin_m", stop_margin_m),
            ("reaction_s", reaction_s), ("near_corridor_margin_m", near_corridor_margin_m),
        ):
            _positive(name, value, zero=True)
        self.ego_half_width_m = ego_half_width_m
        self.lateral_margin_m = lateral_margin_m
        self.stop_margin_m = stop_margin_m
        self.reaction_s = reaction_s
        self.braking_decel_mps2 = braking_decel_mps2
        self.ttc_threshold_s = ttc_threshold_s
        self.prediction_horizon_s = prediction_horizon_s
        self.near_corridor_margin_m = near_corridor_margin_m
        self.track_timeout_s = track_timeout_s
        self.reset()

    def reset(self) -> None:
        self._tracks: dict[int, tuple[float, float]] = {}
        self._last_time_s: float | None = None
        self._latched = False
        self._target_id: int | None = None

    @property
    def latched(self) -> bool:
        return self._latched

    def update(
        self,
        detections: Iterable[CameraPedestrian],
        *,
        speed_mps: float,
        reverse: bool,
        time_s: float,
    ) -> RearAEBDecision:
        _positive("speed_mps", speed_mps, zero=True)
        _positive("time_s", time_s, zero=True)
        if not isinstance(reverse, bool):
            raise ValueError("reverse must be a boolean gear/direction gate")
        if self._last_time_s is not None and time_s <= self._last_time_s:
            raise ValueError("observation time must strictly increase")
        observations = list(detections)
        seen = set()
        for detection in observations:
            if (not isinstance(detection.instance_id, Integral)
                    or isinstance(detection.instance_id, bool) or detection.instance_id < 0
                    or detection.instance_id in seen):
                raise ValueError("detection instance IDs must be unique nonnegative integers")
            seen.add(detection.instance_id)
            if not all(math.isfinite(value) for value in (
                detection.clearance_m, detection.lateral_m, detection.width_m
            )):
                raise ValueError("detection geometry must be finite")
            _positive("detection width_m", detection.width_m)
        self._last_time_s = time_s
        stopping_m = (self.stop_margin_m + speed_mps * self.reaction_s
                      + speed_mps ** 2 / (2.0 * self.braking_decel_mps2))
        if not reverse:
            self._tracks.clear()
            return RearAEBDecision(0.0, False, "not_reversing", None, None, None, stopping_m)

        self._tracks = {
            key: value for key, value in self._tracks.items()
            if time_s - value[0] <= self.track_timeout_s
        }
        candidates: list[tuple[CameraPedestrian, float | None, str]] = []
        for detection in observations:
            previous = self._tracks.get(detection.instance_id)
            self._tracks[detection.instance_id] = (time_s, detection.lateral_m)
            if detection.clearance_m < 0.0:
                continue
            ttc_s = detection.clearance_m / speed_mps if speed_mps > 1e-6 else None
            corridor_m = self.ego_half_width_m + self.lateral_margin_m + detection.width_m / 2.0
            overlaps = abs(detection.lateral_m) <= corridor_m
            reason = "rear_corridor"
            if not overlaps and previous is not None:
                lateral_velocity = (detection.lateral_m - previous[1]) / (time_s - previous[0])
                horizon_s = min(self.prediction_horizon_s, ttc_s) if ttc_s is not None else 0.0
                predicted_y = detection.lateral_m + lateral_velocity * horizon_s
                overlaps = (min(detection.lateral_m, predicted_y) <= corridor_m
                            and max(detection.lateral_m, predicted_y) >= -corridor_m)
                reason = "predicted_crossing"
            elif not overlaps and previous is None:
                overlaps = abs(detection.lateral_m) <= corridor_m + self.near_corridor_margin_m
                reason = "near_corridor"
            urgent = (detection.clearance_m <= stopping_m
                      or (ttc_s is not None and ttc_s <= self.ttc_threshold_s))
            if overlaps and urgent:
                candidates.append((detection, ttc_s, reason))

        if candidates and not self._latched:
            target, ttc_s, reason = min(candidates, key=lambda candidate: candidate[0].clearance_m)
            self._latched = True
            self._target_id = target.instance_id
            return RearAEBDecision(1.0, True, reason, target.instance_id,
                                   target.clearance_m, ttc_s, stopping_m)
        if self._latched:
            target = next((item for item in observations if item.instance_id == self._target_id), None)
            clearance = target.clearance_m if target is not None else None
            ttc_s = clearance / speed_mps if clearance is not None and speed_mps > 1e-6 else None
            return RearAEBDecision(1.0, True, "latched", self._target_id, clearance, ttc_s, stopping_m)
        return RearAEBDecision(0.0, False, "clear", None, None, None, stopping_m)
