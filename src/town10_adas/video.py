"""Record frame-aligned CARLA RGB video without numpy or OpenCV."""

from __future__ import annotations

from collections.abc import Mapping
from fractions import Fraction
import math
import os
from pathlib import Path
import queue
import selectors
import shutil
import subprocess
import tempfile
import time
from types import SimpleNamespace
from typing import Any


class _TelemetryOverlay:
    """Lazy Pillow renderer; cache fonts and leave the driving view unobstructed."""

    def __init__(self, width: int, height: int) -> None:
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError as exc:
            raise RuntimeError("Pillow is required when video telemetry overlay is enabled") from exc
        self.Image = Image
        self.ImageDraw = ImageDraw
        self.width, self.height = width, height
        scale = min(width / 960.0, height / 540.0)
        self.margin = max(2, round(10 * scale))
        self.padding = max(2, round(12 * scale))
        self.line_height = max(10, round(27 * scale))
        font_size = max(8, round(20 * scale))
        self.font = self._font(ImageFont, "DejaVuSans.ttf", font_size)
        self.bold = self._font(ImageFont, "DejaVuSans-Bold.ttf", font_size)

    @staticmethod
    def _font(image_font: Any, name: str, size: int) -> Any:
        for path in (name, f"/usr/share/fonts/truetype/dejavu/{name}"):
            try:
                return image_font.truetype(path, size)
            except OSError:
                pass
        # Pillow 10+ provides a scalable bundled font, including the em dash.
        try:
            return image_font.load_default(size=size)
        except TypeError:
            return image_font.load_default()

    @staticmethod
    def _number(sample: Mapping[str, Any], key: str) -> float | None:
        try:
            value = float(sample.get(key, ""))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def render(self, image: Any, sample: Mapping[str, Any], controller: str | None) -> Any:
        canvas = self.Image.frombytes(
            "RGB", (self.width, self.height), bytes(image.raw_data), "raw", "BGRX"
        )
        draw = self.ImageDraw.Draw(canvas, "RGBA")
        margin, padding, line = self.margin, self.padding, self.line_height
        top = margin
        bottom = self.height - margin - 3 * line - 2 * padding
        panel = (7, 13, 23, 205)
        draw.rectangle((margin, top, self.width - margin, top + 2 * line + 2 * padding), fill=panel)
        draw.rectangle((margin, bottom, self.width - margin, self.height - margin), fill=panel)
        left = margin + padding
        right = self.width - margin - padding
        middle = self.width // 2 + padding
        white = (244, 247, 252, 255)
        muted = (193, 209, 225, 255)
        red = (255, 91, 91, 255)

        def label(x: int, y: int, value: str, *, bold: bool = False, color: Any = white) -> None:
            font = self.bold if bold else self.font
            try:
                draw.text((x, y), value, font=font, fill=color)
            except UnicodeEncodeError:
                draw.text((x, y), value.replace("—", "-"), font=font, fill=color)

        def formatted(key: str, suffix: str = "", digits: int = 1) -> str:
            value = self._number(sample, key)
            return "—" if value is None else f"{value:.{digits}f}{suffix}"

        def speed(key: str) -> str:
            value = self._number(sample, key)
            return "—" if value is None else f"{value:.1f} mph / {value * 1.609344:.1f} km/h"

        time_label = f"SIM {formatted('time_s', ' s', 2)}"
        time_width = draw.textlength(time_label, font=self.font)
        label(left, top + padding, f"{str(controller or 'ADAS').upper()} CONTROLLER", bold=True)
        label(int(right - time_width), top + padding, time_label, color=muted)
        label(left, top + padding + line, f"EGO  {speed('ego_speed_mph')}")
        label(middle, top + padding + line, f"LEAD  {speed('target_speed_mph')}")

        phase = str(sample.get("phase", "—")).upper()
        label(left, bottom + padding, f"PHASE  {phase}", bold=True)
        label(middle, bottom + padding,
              f"GAP {formatted('distance_m', ' m')}    TTC {formatted('ttc_s', ' s')}")
        label(left, bottom + padding + line, f"EGO BRAKE  {formatted('brake_percent', '%', 0)}")
        label(middle, bottom + padding + line,
              f"LEAD BRAKE  {formatted('target_brake_percent', '%', 0)}")
        lead_brake = self._number(sample, "target_brake_percent") or 0.0
        lead_braking = lead_brake > 0 and phase in {"THREAT", "AEB", "COMPLETE", "ABORTED"}
        active_value = str(sample.get("aeb_active", "0")).strip().lower()
        aeb_active = active_value in {"true", "yes"} or (self._number(sample, "aeb_active") or 0) > 0
        label(left, bottom + padding + 2 * line,
              "LEAD BRAKING" if lead_braking else "LEAD CRUISING" if phase in {"ACCELERATE", "HOLD"} else "LEAD BRAKE RELEASED",
              bold=lead_braking, color=red if lead_braking else muted)
        label(middle, bottom + padding + 2 * line,
              "AEB ACTIVE" if aeb_active else "AEB STANDBY",
              bold=aeb_active, color=red if aeb_active else muted)
        return SimpleNamespace(
            width=image.width, height=image.height, frame=image.frame,
            raw_data=canvas.convert("RGBA").tobytes("raw", "BGRA"),
        )


class CarlaVideoRecorder:
    """Own a chase camera and encode one image for each captured simulation tick.

    Create this after spawning the ego vehicle, then call ``capture(snapshot.frame)``
    after each synchronous world tick. Always call ``close()`` before destroying
    the ego vehicle or restoring asynchronous world settings. Video time advances
    by ``fixed_delta_seconds`` for every frame, independent of wall-clock speed.
    """

    def __init__(
        self,
        world: Any,
        ego: Any,
        output_path: str | Path,
        *,
        fixed_delta_seconds: float,
        width: int = 960,
        height: int = 540,
        frame_timeout_s: float = 15.0,
        encoder_timeout_s: float = 30.0,
    ) -> None:
        if not math.isfinite(fixed_delta_seconds) or fixed_delta_seconds <= 0:
            raise ValueError("video requires a positive finite fixed_delta_seconds")
        if any(not isinstance(value, int) or value <= 0 or value % 2 for value in (width, height)):
            raise ValueError("video width and height must be positive even integers")
        if any(not math.isfinite(value) or value <= 0 for value in (frame_timeout_s, encoder_timeout_s)):
            raise ValueError("video timeouts must be positive and finite")
        settings = world.get_settings()
        if not settings.synchronous_mode or not math.isclose(
            settings.fixed_delta_seconds or 0.0, fixed_delta_seconds, rel_tol=1e-6, abs_tol=1e-9
        ):
            raise ValueError("video requires synchronous CARLA with the configured fixed timestep")
        if getattr(settings, "no_rendering_mode", False):
            raise ValueError("CARLA no_rendering_mode must be disabled to record RGB video")
        self.output_path = Path(output_path)
        if self.output_path.suffix.lower() != ".mp4":
            raise ValueError("video output must have an .mp4 extension")
        if self.output_path.exists():
            raise FileExistsError(f"video output already exists: {self.output_path}")
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg is required to record CARLA video")

        import carla

        self.width = width
        self.height = height
        frame_rate = 1 / Fraction(str(fixed_delta_seconds)).limit_denominator(1_000_000)
        self.fps = float(frame_rate)
        self.frame_count = 0
        self.first_frame: int | None = None
        self.last_frame: int | None = None
        self._overlay: _TelemetryOverlay | None = None
        self._frame_timeout_s = frame_timeout_s
        self._encoder_timeout_s = encoder_timeout_s
        self._frames: queue.Queue[Any] = queue.Queue(maxsize=8)
        self._closed = False
        self._camera: Any = None
        self._process: subprocess.Popen[bytes] | None = None
        self._selector = selectors.DefaultSelector()
        self._stderr = tempfile.TemporaryFile(mode="w+b")
        try:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self._process = subprocess.Popen(
                [
                    ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
                    "-f", "rawvideo", "-pixel_format", "bgra",
                    "-video_size", f"{width}x{height}",
                    "-framerate", f"{frame_rate.numerator}/{frame_rate.denominator}",
                    "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "veryfast",
                    "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                    str(self.output_path),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
                bufsize=0,
            )
            assert self._process.stdin is not None
            os.set_blocking(self._process.stdin.fileno(), False)
            self._selector.register(self._process.stdin, selectors.EVENT_WRITE)
            blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
            for name, value in {
                "image_size_x": str(width), "image_size_y": str(height),
                "fov": "90", "sensor_tick": "0.0",
            }.items():
                blueprint.set_attribute(name, value)
            self._camera = world.spawn_actor(
                blueprint,
                carla.Transform(carla.Location(x=-7.0, z=3.5), carla.Rotation(pitch=-15.0)),
                attach_to=ego,
                attachment_type=carla.AttachmentType.Rigid,
            )
            self._camera.listen(self._receive_image)
        except BaseException:
            try:
                self.close()
            except Exception:
                pass
            raise

    @property
    def metadata(self) -> dict[str, Any]:
        return {
            "path": str(self.output_path), "fps": self.fps,
            "width": self.width, "height": self.height, "frame_count": self.frame_count,
            "first_frame": self.first_frame, "last_frame": self.last_frame,
            "telemetry_overlay": self._overlay is not None,
        }

    def _receive_image(self, image: Any) -> None:
        if self._closed:
            return
        try:
            self._frames.put_nowait(image)
        except queue.Full:
            # Keep memory bounded during optional startup ticks. A missing frame
            # during recording is detected by capture(), never silently skipped.
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frames.put_nowait(image)
            except queue.Full:
                pass

    def _encoder_error(self) -> str:
        self._stderr.seek(0)
        return self._stderr.read().decode("utf-8", errors="replace").strip()

    def capture(
        self, frame: int, *, telemetry: Mapping[str, Any] | None = None,
        controller: str | None = None,
    ) -> Any:
        """Encode this frame and return its image, optionally with a telemetry HUD.

        Telemetry must describe the requested frame. The returned BGRA pixels are
        exactly those sent to ffmpeg, so a live preview can show the same overlay.
        Pillow is loaded only when telemetry is supplied.
        """
        if self._closed:
            raise RuntimeError("video recorder is closed")
        if self.last_frame is not None and frame != self.last_frame + 1:
            raise ValueError(f"video requires consecutive frames; got {frame} after {self.last_frame}")
        assert self._process is not None and self._process.stdin is not None
        deadline = time.monotonic() + self._frame_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"CARLA RGB camera did not deliver frame {frame}")
            try:
                image = self._frames.get(timeout=remaining)
            except queue.Empty as exc:
                raise TimeoutError(f"CARLA RGB camera did not deliver frame {frame}") from exc
            if image.frame < frame:
                continue
            if image.frame > frame:
                raise RuntimeError(f"CARLA RGB camera skipped requested frame {frame}; received {image.frame}")
            break
        if image.width != self.width or image.height != self.height:
            raise RuntimeError("CARLA camera dimensions changed during recording")
        data = memoryview(image.raw_data)
        if len(data) != self.width * self.height * 4:
            raise RuntimeError("CARLA RGB camera returned an unexpected BGRA frame size")
        if telemetry is not None:
            if self._overlay is None:
                self._overlay = _TelemetryOverlay(self.width, self.height)
            image = self._overlay.render(image, telemetry, controller)
            data = memoryview(image.raw_data)
        deadline = time.monotonic() + self._encoder_timeout_s
        while data:
            if self._process.poll() is not None:
                raise RuntimeError(f"ffmpeg exited during video recording: {self._encoder_error()}")
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self._selector.select(timeout=remaining):
                raise TimeoutError(f"ffmpeg did not accept CARLA frame {frame} before the deadline")
            try:
                written = os.write(self._process.stdin.fileno(), data)
            except BlockingIOError:
                continue
            except BrokenPipeError as exc:
                raise RuntimeError(f"ffmpeg closed its video input: {self._encoder_error()}") from exc
            if written <= 0:
                raise RuntimeError("ffmpeg accepted no video data")
            data = data[written:]
        self.frame_count += 1
        if self.first_frame is None:
            self.first_frame = frame
        self.last_frame = frame
        return image

    def close(self) -> None:
        """Stop/destroy the camera and finalize the MP4, with bounded waits."""
        if self._closed:
            return
        self._closed = True
        errors: list[str] = []
        try:
            if self._camera is not None:
                try:
                    self._camera.stop()
                except Exception as exc:
                    errors.append(f"could not stop video camera: {exc}")
                try:
                    if self._camera.is_alive:
                        self._camera.destroy()
                except Exception as exc:
                    errors.append(f"could not destroy video camera: {exc}")
            self._selector.close()
            if self._process is not None:
                if self._process.stdin is not None:
                    self._process.stdin.close()
                try:
                    returncode = self._process.wait(timeout=self._encoder_timeout_s)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=5.0)
                    errors.append("ffmpeg timed out while finalizing video")
                else:
                    if returncode != 0:
                        errors.append(f"ffmpeg exited with status {returncode}: {self._encoder_error()}")
        finally:
            self._stderr.close()
        if errors:
            raise RuntimeError("; ".join(errors))

    def __enter__(self) -> "CarlaVideoRecorder":
        return self

    def __exit__(self, exc_type: Any, _exc: Any, _traceback: Any) -> None:
        try:
            self.close()
        except Exception:
            if exc_type is None:
                raise
