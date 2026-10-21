"""Rear RGB person detection with YOLOX-s and aligned simulated depth.

The OpenCV Zoo 2022nov YOLOX model uses RGB float pixels in [0, 255], a
top-left 640-square letterbox, and un-decoded stride-grid outputs. Reference:
https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox

This baseline has no actor-state inputs. COCO's person class does not identify
age. Range comes from a coherent depth cluster in the middle of each RGB box;
occluders, inaccurate boxes, and detector misses remain limitations. Association
is one-to-one but can switch identity when people overlap or move abruptly.
The rear cameras must be aligned, undistorted, level, and face yaw 180 degrees.
"""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
from typing import Any

from .rear_perception import CameraPedestrian, _image_shape, _positive


def _iou(first, second) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    union = ((first[2] - first[0]) * (first[3] - first[1])
             + (second[2] - second[0]) * (second[3] - second[1]) - intersection)
    return intersection / union if union > 0.0 else 0.0


def decode_yolox_persons(
    output: Any, image_width: int, image_height: int, scale: float,
    confidence: float = 0.35, iou_threshold: float = 0.5,
) -> list[tuple[tuple[int, int, int, int], float]]:
    """Decode the Zoo model, filter COCO person, and suppress duplicate boxes.

    Returns source-image boxes with exclusive right/bottom coordinates and
    objectness-times-class scores. A different model output layout is an error.
    """
    import numpy as np

    _positive("letterbox scale", scale)
    if not 0.0 < confidence < 1.0 or not 0.0 < iou_threshold < 1.0:
        raise ValueError("confidence and IoU thresholds must be between zero and one")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    raw = np.asarray(output)
    if raw.shape != (1, 8400, 85):
        raise ValueError(f"expected YOLOX output (1,8400,85), got {raw.shape}")
    predictions = raw[0]
    scores = predictions[:, 4] * predictions[:, 5]
    # Reject rows whose winning class is not person, even if person is runner-up.
    mask = ((scores >= confidence) & (predictions[:, 5:].argmax(axis=1) == 0)
            & np.isfinite(predictions).all(axis=1))
    indices = np.flatnonzero(mask)
    grids, strides = [], []
    for stride in (8, 16, 32):
        side = 640 // stride
        xx, yy = np.meshgrid(np.arange(side), np.arange(side))
        grids.append(np.stack((xx, yy), axis=-1).reshape(-1, 2))
        strides.append(np.full(side * side, stride))
    grid = np.concatenate(grids)[indices]
    stride = np.concatenate(strides)[indices, None]
    selected = predictions[indices]
    centers = (selected[:, :2] + grid) * stride
    # Very large exponent values indicate corrupt output, not a real object.
    with np.errstate(over="ignore", invalid="ignore"):
        sizes = np.exp(selected[:, 2:4]) * stride
    boxes = np.concatenate((centers - sizes / 2.0, centers + sizes / 2.0), axis=1) / scale
    candidates = []
    for index, box in zip(indices, boxes):
        if not np.isfinite(box).all():
            continue
        x0 = max(0, min(image_width, math.floor(float(box[0]))))
        y0 = max(0, min(image_height, math.floor(float(box[1]))))
        x1 = max(0, min(image_width, math.ceil(float(box[2]))))
        y1 = max(0, min(image_height, math.ceil(float(box[3]))))
        if x1 - x0 < 2 or y1 - y0 < 4:
            continue
        candidates.append(((x0, y0, x1, y1), float(scores[index])))
    candidates.sort(key=lambda item: (-item[1], item[0]))
    kept = []
    for candidate in candidates:
        if all(_iou(candidate[0], previous[0]) <= iou_threshold for previous in kept):
            kept.append(candidate)
    return kept


def pedestrian_from_depth(
    bbox: tuple[int, int, int, int], axial_depth: Any, *, fov_deg: float,
    camera_to_bumper_m: float, max_range_m: float,
) -> CameraPedestrian | None:
    """Estimate range from central-body depth support, not a box's background.

    Use the 20th percentile as a foreground seed, then retain a coherent depth
    band around it. A minimum 10% body-region support rejects isolated pixels.
    The full RGB box width conservatively covers limbs outside the body sample.
    This is an imperfect RGB-box/depth heuristic, not semantic segmentation.
    """
    import numpy as np

    image_height, image_width = axial_depth.shape
    x0, y0, x1, y1 = bbox
    if not (0 <= x0 < x1 <= image_width and 0 <= y0 < y1 <= image_height):
        raise ValueError("detection box must lie inside the depth image")
    width, height = x1 - x0, y1 - y0
    # Exclude box edges, ground below feet, and empty space around the head.
    sx0, sx1 = x0 + int(width * 0.2), x0 + max(int(width * 0.8), 1)
    sy0, sy1 = y0 + int(height * 0.15), y0 + max(int(height * 0.75), 1)
    region = axial_depth[sy0:sy1, sx0:sx1]
    valid = (np.isfinite(region) & (region > 0.0)
             & (region + camera_to_bumper_m <= max_range_m))
    required = max(8, math.ceil(region.size * 0.1))
    values = region[valid]
    if values.size < required:
        return None
    seed_m = float(np.percentile(values, 20.0))
    cluster = valid & (np.abs(region - seed_m) <= max(0.25, seed_m * 0.08))
    rows, cols = np.nonzero(cluster)
    if rows.size < required:
        return None
    depths = region[cluster]
    depth_m = float(np.percentile(depths, 10.0))
    focal_px = image_width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    lateral = -(cols + sx0 + 0.5 - image_width / 2.0) * depths / focal_px
    return CameraPedestrian(
        instance_id=0, clearance_m=depth_m + camera_to_bumper_m,
        lateral_m=float(np.median(lateral)), width_m=width * depth_m / focal_px,
        bbox_xyxy=bbox, pixel_count=int(rows.size),
    )


class _PersonTracker:
    """Deterministic one-to-one association, expiring after ten camera frames."""

    def __init__(self, max_gap_frames: int = 10) -> None:
        self.max_gap_frames = max_gap_frames
        self.tracks: dict[int, tuple[int, CameraPedestrian]] = {}
        self.next_id = 1

    def assign(self, detections: list[CameraPedestrian], frame: int) -> list[CameraPedestrian]:
        self.tracks = {key: value for key, value in self.tracks.items()
                       if frame - value[0] <= self.max_gap_frames}
        pairs = []
        for index, detection in enumerate(detections):
            for track_id, (old_frame, old) in self.tracks.items():
                distance = math.hypot(detection.lateral_m - old.lateral_m,
                                      detection.clearance_m - old.clearance_m)
                overlap = _iou(detection.bbox_xyxy, old.bbox_xyxy)
                first, second = detection.bbox_xyxy, old.bbox_xyxy
                center_distance = math.hypot((first[0] + first[2] - second[0] - second[2]) / 2,
                                             (first[1] + first[3] - second[1] - second[3]) / 2)
                box_scale = max(first[2] - first[0], second[2] - second[0])
                if distance <= 0.8 + 0.15 * (frame - old_frame) and (
                    overlap > 0.0 or center_distance <= box_scale * 1.5
                ):
                    pairs.append((distance + (1.0 - overlap) * 0.2, track_id, index))
        matches, used = {}, set()
        for _, track_id, index in sorted(pairs):
            if index not in matches and track_id not in used:
                matches[index] = track_id
                used.add(track_id)
        output = []
        for index, detection in enumerate(detections):
            track_id = matches.get(index)
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
            detection = replace(detection, instance_id=track_id)
            self.tracks[track_id] = frame, detection
            output.append(detection)
        return sorted(output, key=lambda detection: detection.clearance_m)


class RearRGBDetector:
    """YOLOX COCO person detector; CARLA images and depth are the only inputs."""

    def __init__(self, model_path: str | Path, fov_deg: float,
                 camera_to_bumper_m: float = 0.05, max_range_m: float = 15.0,
                 confidence: float = 0.35) -> None:
        _positive("fov_deg", fov_deg)
        _positive("camera_to_bumper_m", camera_to_bumper_m, zero=True)
        _positive("max_range_m", max_range_m)
        if fov_deg >= 180.0 or not camera_to_bumper_m < max_range_m < 1000.0:
            raise ValueError("invalid rear camera calibration")
        if not math.isfinite(confidence) or not 0.0 < confidence < 1.0:
            raise ValueError("confidence must be between zero and one")
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLOX model missing: {model_path}; run scripts/download_rear_detector.py")
        import cv2

        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.fov_deg = fov_deg
        self.camera_to_bumper_m = camera_to_bumper_m
        self.max_range_m = max_range_m
        self.confidence = confidence
        self.tracker = _PersonTracker()
        self.last_frame: int | None = None

    def detect(self, rgb_image: Any, depth_image: Any) -> list[CameraPedestrian]:
        import cv2
        import numpy as np

        shape = _image_shape(rgb_image)
        if shape != _image_shape(depth_image) or rgb_image.frame != depth_image.frame:
            raise ValueError("RGB and depth frames and dimensions must match")
        if self.last_frame is not None and rgb_image.frame <= self.last_frame:
            raise ValueError("RGB observation frame must strictly increase")
        for image in (rgb_image, depth_image):
            if hasattr(image, "fov") and (not math.isfinite(image.fov)
                    or not math.isclose(image.fov, self.fov_deg, abs_tol=1e-5)):
                raise ValueError("camera field of view differs from calibration")
        if hasattr(rgb_image, "timestamp") and hasattr(depth_image, "timestamp"):
            timestamps = rgb_image.timestamp, depth_image.timestamp
            if not all(math.isfinite(t) for t in timestamps) or not math.isclose(
                *timestamps, rel_tol=0.0, abs_tol=1e-6
            ):
                raise ValueError("RGB and depth timestamps must match")
        arrays = []
        for image in (rgb_image, depth_image):
            data = np.frombuffer(image.raw_data, dtype=np.uint8)
            if data.size != shape[0] * shape[1] * 4:
                raise ValueError("camera payload must contain width*height*4 BGRA bytes")
            arrays.append(data.reshape(*shape, 4))
        rgb = arrays[0][:, :, :3][:, :, ::-1]
        scale = min(640.0 / shape[1], 640.0 / shape[0])
        resized_width, resized_height = int(shape[1] * scale), int(shape[0] * scale)
        padded = np.full((640, 640, 3), 114.0, dtype=np.float32)
        padded[:resized_height, :resized_width] = cv2.resize(rgb, (resized_width, resized_height))
        blob = np.ascontiguousarray(padded.transpose(2, 0, 1)[None])
        self.net.setInput(blob)
        output = self.net.forward()
        boxes = decode_yolox_persons(output, shape[1], shape[0], scale, self.confidence)
        depth = arrays[1]
        axial = (depth[:, :, 2].astype(np.float64) + depth[:, :, 1].astype(np.float64) * 256.0
                 + depth[:, :, 0].astype(np.float64) * 65536.0) * (1000.0 / 16777215.0)
        detections = []
        for box, _score in boxes:
            detection = pedestrian_from_depth(box, axial, fov_deg=self.fov_deg,
                camera_to_bumper_m=self.camera_to_bumper_m, max_range_m=self.max_range_m)
            if detection is not None:
                detections.append(detection)
        self.last_frame = rgb_image.frame
        return self.tracker.assign(detections, rgb_image.frame)
