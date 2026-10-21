"""Frame-aligned rear AEB video from camera images owned by the scenario runner."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from fractions import Fraction
import math
import os
from pathlib import Path
import selectors
import shutil
import subprocess
import tempfile
import time
from types import SimpleNamespace
from typing import Any


class RearVideoRecorder:
    """Encode one rear-camera frame per observation, optionally beside an overview.

    ``width`` and ``height`` describe each input camera. With ``overview=True``
    the output is twice as wide, and every call to ``write`` must supply an
    overview image from the same simulation frame. Sensors remain owned by the
    caller. ``brake`` telemetry is normalized to [0, 1].
    """

    def __init__(self, output_path: str | Path, width: int, height: int, dt: float,
                 *, overview: bool = True, encoder_timeout_s: float = 30.0) -> None:
        if any(type(value) is not int or value <= 0 or value % 2 for value in (width, height)):
            raise ValueError("video width and height must be positive even integers")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("video dt must be positive and finite")
        if not math.isfinite(encoder_timeout_s) or encoder_timeout_s <= 0:
            raise ValueError("encoder timeout must be positive and finite")
        if not isinstance(overview, bool):
            raise ValueError("overview must be boolean")
        self.output_path = Path(output_path)
        if self.output_path.suffix.lower() != ".mp4":
            raise ValueError("video output must have an .mp4 extension")
        if self.output_path.exists():
            raise FileExistsError(f"video output already exists: {self.output_path}")
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg is required to record rear AEB video")
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError as exc:
            raise RuntimeError("Pillow is required to draw rear AEB video telemetry") from exc

        self._Image, self._ImageDraw = Image, ImageDraw
        self.camera_width, self.height = width, height
        self.overview = overview
        self.width = width * (2 if overview else 1)
        self.dt = float(dt)
        self._rate = 1 / Fraction(str(dt)).limit_denominator(1_000_000)
        self.fps = float(self._rate)
        self.frame_count = 0
        self.first_frame: int | None = None
        self.last_frame: int | None = None
        self._closed = False
        self._failed = False
        self._finalized = False
        self._timeout = encoder_timeout_s
        scale = min(width / 960.0, height / 540.0)
        self._padding = max(3, round(12 * scale))
        self._line = max(10, round(25 * scale))
        self._font = self._load_font(ImageFont, "DejaVuSans.ttf", max(8, round(18 * scale)))
        self._bold = self._load_font(ImageFont, "DejaVuSans-Bold.ttf", max(8, round(20 * scale)))
        self._selector = selectors.DefaultSelector()
        self._stderr = tempfile.TemporaryFile(mode="w+b")
        self._process: subprocess.Popen[bytes] | None = None
        try:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self._process = subprocess.Popen([
                ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
                "-f", "rawvideo", "-pixel_format", "bgra",
                "-video_size", f"{self.width}x{height}",
                "-framerate", f"{self._rate.numerator}/{self._rate.denominator}",
                "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "veryfast",
                "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(self.output_path),
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self._stderr, bufsize=0)
            assert self._process.stdin is not None
            os.set_blocking(self._process.stdin.fileno(), False)
            self._selector.register(self._process.stdin, selectors.EVENT_WRITE)
        except BaseException:
            self._failed = True
            try:
                self.close()
            except Exception:
                pass
            raise

    @staticmethod
    def _load_font(image_font: Any, name: str, size: int) -> Any:
        for path in (name, f"/usr/share/fonts/truetype/dejavu/{name}"):
            try:
                return image_font.truetype(path, size)
            except OSError:
                pass
        try:
            return image_font.load_default(size=size)
        except TypeError:
            return image_font.load_default()

    @property
    def metadata(self) -> dict[str, Any]:
        return {
            "path": str(self.output_path), "fps": self.fps, "width": self.width,
            "height": self.height, "camera_width": self.camera_width,
            "frame_count": self.frame_count, "first_frame": self.first_frame,
            "last_frame": self.last_frame, "telemetry_overlay": True,
            "layout": "rear_and_overview" if self.overview else "rear_only",
            "finalized": self._finalized,
        }

    def _validate_image(self, image: Any, name: str, frame: int) -> None:
        if image is None:
            raise ValueError(f"{name} image is required")
        if image.frame != frame:
            raise ValueError(f"{name} image frame {image.frame} does not match rear frame {frame}")
        if image.width != self.camera_width or image.height != self.height:
            raise ValueError(f"{name} image dimensions do not match configured camera size")
        if len(image.raw_data) != self.camera_width * self.height * 4:
            raise ValueError(f"{name} image has an invalid BGRA byte length")

    @staticmethod
    def _number(telemetry: Mapping[str, Any], key: str) -> float | None:
        try:
            value = float(telemetry[key])
        except (KeyError, ValueError, TypeError):
            return None
        return value if math.isfinite(value) else None

    @staticmethod
    def _active(telemetry: Mapping[str, Any], key: str) -> bool:
        value = telemetry.get(key, False)
        return str(value).strip().lower() in {"true", "yes", "1", "1.0"}

    def _render(self, image: Any, telemetry: Mapping[str, Any], detections: Sequence[Any],
                overview_image: Any) -> Any:
        canvas = self._Image.new("RGB", (self.width, self.height))
        canvas.paste(self._Image.frombytes("RGB", (self.camera_width, self.height),
                                         bytes(image.raw_data), "raw", "BGRX"), (0, 0))
        if self.overview:
            canvas.paste(self._Image.frombytes("RGB", (self.camera_width, self.height),
                                             bytes(overview_image.raw_data), "raw", "BGRX"),
                         (self.camera_width, 0))
        draw = self._ImageDraw.Draw(canvas, "RGBA")
        padding, line = self._padding, self._line
        active = self._active(telemetry, "aeb_active")
        white, muted = (245, 248, 255, 255), (193, 209, 225, 255)
        accent = (255, 87, 87, 255) if active else (93, 230, 183, 255)
        panel = (8, 16, 28, 220)

        for detection in detections:
            box = detection.bbox_xyxy
            if len(box) != 4 or any(not math.isfinite(float(value)) for value in box):
                raise ValueError("pedestrian bounding boxes must contain four finite coordinates")
            x1, y1, x2, y2 = (float(value) for value in box)
            if x2 < x1 or y2 < y1:
                raise ValueError("pedestrian bounding box corners are reversed")
            x1, x2 = (max(0, min(self.camera_width - 1, round(value))) for value in (x1, x2))
            y1, y2 = (max(0, min(self.height - 1, round(value))) for value in (y1, y2))
            draw.rectangle((x1, y1, x2, y2), outline=accent, width=max(1, round(self.camera_width / 320)))
            draw.text((x1 + 3, max(padding + line, y1 - line)), "PEDESTRIAN", font=self._font, fill=accent)

        def text(x: int, y: int, value: str, *, bold: bool = False, color: Any = white) -> None:
            draw.text((x, y), value, font=self._bold if bold else self._font, fill=color)

        def value(key: str, suffix: str = "", digits: int = 2) -> str:
            number = self._number(telemetry, key)
            return "--" if number is None else f"{number:.{digits}f}{suffix}"

        title_height = line + 2 * padding
        draw.rectangle((0, 0, self.width, title_height), fill=panel)
        text(padding, padding, "SCENARIO 2 | REAR CAMERA AEB", bold=True)
        if self.overview:
            draw.line((self.camera_width, 0, self.camera_width, self.height), fill=(37, 55, 73, 255), width=2)
            text(self.camera_width + padding, padding, "PARKING OVERVIEW", bold=True)
        time_label = "SIM " + value("time_s", " s")
        time_width = draw.textlength(time_label, font=self._font)
        text(int(self.width - padding - time_width), padding, time_label, color=muted)

        bottom = self.height - (3 * line + 2 * padding)
        draw.rectangle((0, bottom, self.width, self.height), fill=panel)
        second = self.camera_width + padding if self.overview else self.width // 2 + padding
        speed = self._number(telemetry, "reverse_speed_mps")
        speed_label = "--" if speed is None else f"{speed:.2f} m/s ({speed * 3.6:.1f} km/h)"
        phase = str(telemetry.get("phase", "--"))
        text(padding, bottom + padding, "PHASE " + phase, bold=True)
        text(second, bottom + padding, "REVERSE " + speed_label)
        text(padding, bottom + padding + line, "REAR CLEARANCE " + value("rear_clearance_m", " m"))
        brake = self._number(telemetry, "brake")
        brake_label = "--" if brake is None else f"{brake * 100:.0f}%"
        text(second, bottom + padding + line, f"BRAKE {brake_label}   TTC {value('ttc_s', ' s')}")
        visible = self._active(telemetry, "pedestrian_visible")
        text(padding, bottom + padding + 2 * line,
             "PEDESTRIAN VISIBLE" if visible else "PEDESTRIAN NOT VISIBLE", color=accent if visible else muted)
        text(second, bottom + padding + 2 * line,
             "AEB ACTIVE" if active else "AEB STANDBY", bold=True, color=accent if active else muted)
        return SimpleNamespace(width=self.width, height=self.height, frame=image.frame,
                               raw_data=canvas.convert("RGBA").tobytes("raw", "BGRA"))

    def _encoder_error(self) -> str:
        self._stderr.seek(0)
        return self._stderr.read().decode("utf-8", errors="replace").strip()

    def write(self, image: Any, telemetry: Mapping[str, Any], detections: Sequence[Any] = (),
              *, overview_image: Any = None) -> Any:
        """Encode and return composited BGRA pixels for exactly one observation."""
        if self._closed:
            raise RuntimeError("rear video recorder is closed")
        if self._failed:
            raise RuntimeError("rear video recorder failed; close it before starting another recording")
        frame = image.frame
        if type(frame) is not int or frame < 0:
            raise ValueError("camera frame must be a nonnegative integer")
        if self.last_frame is not None and frame != self.last_frame + 1:
            raise ValueError(f"video requires consecutive frames; got {frame} after {self.last_frame}")
        self._validate_image(image, "rear", frame)
        if self.overview:
            self._validate_image(overview_image, "overview", frame)
        elif overview_image is not None:
            raise ValueError("overview image supplied to a rear-only video recorder")
        rendered = self._render(image, telemetry, detections, overview_image)
        assert self._process is not None and self._process.stdin is not None
        data = memoryview(rendered.raw_data)
        deadline = time.monotonic() + self._timeout
        try:
            while data:
                if self._process.poll() is not None:
                    raise RuntimeError(f"ffmpeg exited during rear video recording: {self._encoder_error()}")
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not self._selector.select(timeout=remaining):
                    raise TimeoutError(f"ffmpeg did not accept rear camera frame {frame} before the deadline")
                try:
                    written = os.write(self._process.stdin.fileno(), data)
                except BlockingIOError:
                    continue
                except BrokenPipeError as exc:
                    raise RuntimeError(f"ffmpeg closed its video input: {self._encoder_error()}") from exc
                if written <= 0:
                    raise RuntimeError("ffmpeg accepted no rear video data")
                data = data[written:]
        except BaseException:
            self._failed = True
            raise
        if self.first_frame is None:
            self.first_frame = frame
        self.last_frame = frame
        self.frame_count += 1
        return rendered

    def close(self) -> None:
        """Finalize MP4 with a bounded encoder wait; never destroy caller sensors."""
        if self._closed:
            return
        self._closed = True
        errors: list[str] = []
        try:
            self._selector.close()
            if self._process is not None:
                if self._process.stdin is not None:
                    try:
                        self._process.stdin.close()
                    except OSError as exc:
                        errors.append(f"could not close video encoder input: {exc}")
                try:
                    returncode = self._process.wait(timeout=self._timeout)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    try:
                        self._process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        errors.append("ffmpeg did not exit after being killed")
                    errors.append("ffmpeg timed out finalizing rear video")
                else:
                    if returncode != 0:
                        errors.append(f"ffmpeg exited with status {returncode}: {self._encoder_error()}")
        finally:
            self._stderr.close()
        self._finalized = not errors and not self._failed and self.frame_count > 0
        if errors:
            self._failed = True
            raise RuntimeError("; ".join(errors))

    def __enter__(self) -> "RearVideoRecorder":
        return self

    def __exit__(self, exc_type: Any, _exc: Any, _traceback: Any) -> None:
        try:
            self.close()
        except Exception:
            if exc_type is None:
                raise
