#!/usr/bin/env python3
"""Watch a fresh CARLA PID/MPC pair, review staged recordings, then explicitly save."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import mimetypes
from pathlib import Path
import re
import secrets
import shutil
import subprocess
import sys
import threading
import time
import traceback
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.config import load_config

ASSETS = frozenset({
    "town10_aeb_pid.mp4", "town10_aeb_mpc.mp4",
    "town10_aeb_pid.csv", "town10_aeb_mpc.csv",
    "town10_aeb_pid.metadata.json", "town10_aeb_mpc.metadata.json",
    "comparison/comparison.png", "comparison/comparison.svg",
    "comparison/comparison.json", "comparison/comparison.md", "comparison/summary.csv",
    "preview_config.json", "preview.log",
})

PAGE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>CARLA · PID / MPC preview</title><style>
:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#10141c;color:#e4eaf4}
body{max-width:1280px;margin:0 auto;padding:24px}h1{margin-bottom:8px;font-size:28px}h2{font-size:21px}
p{line-height:1.55;color:#b8c5d9}button,a.button{border:0;border-radius:7px;padding:12px 18px;background:#609eff;color:#071326;font:600 15px system-ui;cursor:pointer}button:disabled{opacity:.4;cursor:default}.bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:18px 0}.card{background:#1b2230;border:1px solid #303c51;border-radius:12px;padding:18px;margin:18px 0}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}video,img{width:100%;background:#090d14;border-radius:8px}#live{max-height:530px;object-fit:contain}#metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-top:15px}.metric{padding:10px;background:#101722;border-radius:6px}.metric span{display:block;color:#9dacc3;font-size:12px}.metric strong{display:block;font-size:19px;margin-top:4px}.muted{font-size:13px;color:#9dacc3}.good{color:#70e2a4}.bad{color:#ffad9e}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px;color:#ffbaa8}code{overflow-wrap:anywhere}table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:9px;border-bottom:1px solid #303c51}a{color:#95c1ff}.links{display:flex;gap:16px;flex-wrap:wrap}.events{display:flex;gap:12px;flex-wrap:wrap;margin:12px 0}.event{padding:12px 16px;border-radius:7px;font-weight:700;background:#6b3419;color:#ffe5b2}.event.aeb{background:#8a2434;color:#fff0f1}#scenario{font-size:16px;line-height:1.7}.hidden{display:none!important}@media(max-width:720px){.grid{grid-template-columns:1fr}#metrics{grid-template-columns:1fr 1fr}body{padding:14px}}
</style></head><body>
<h1>CARLA · PID / MPC comparison</h1>
<p>Watch each controller live, then review both recordings and graphs. Preview files are staged temporarily; press <strong>Save results</strong> after your review to copy them into the results folder.</p>
<div class="bar"><button id="start">Start fresh comparison</button><strong id="status">Connecting…</strong></div>
<p class="muted" id="config"></p><div class="card"><h2>Scenario</h2><p id="scenario"></p></div><div class="card"><h2 id="liveTitle">Live camera</h2><div class="events" aria-live="polite"><div id="leadEvent" class="event hidden">Lead vehicle braking</div><div id="aebEvent" class="event aeb hidden">Ego AEB active</div></div><p id="waiting">Ready when you are. Start runs PID followed by MPC using the same route and settings.</p><img id="live" class="hidden" alt="Live CARLA chase camera"><div id="metrics"></div></div>
<div id="review" class="hidden"><div class="card"><h2>Review both recordings</h2><p>Play both together from the start, or use each video's controls to inspect individual moments.</p><div class="bar"><button id="playBoth">Play both from start</button><button id="pauseBoth">Pause both</button></div><div class="grid"><div><h3>PID</h3><video id="pid" controls preload="metadata"></video></div><div><h3>MPC</h3><video id="mpc" controls preload="metadata"></video></div></div></div>
<div class="card"><h2>Comparison</h2><p id="validity"></p><table><thead><tr><th>Metric</th><th>PID</th><th>MPC</th></tr></thead><tbody id="summary"></tbody></table><img id="plot" class="hidden" alt="PID and MPC speed, gap, braking and time-to-collision graphs"><p class="links" id="downloads"></p></div>
<div class="card"><h2>Save after review</h2><p id="saveHelp">The videos, CSV traces, graph data and comparison plots are ready for your review.</p><button id="save">Save results</button><p id="saved" class="good"></p></div></div>
<div id="errors" class="card hidden"><h2>Run details</h2><pre id="errorText"></pre></div><p class="muted" id="stage"></p>
<script>
const token=__TOKEN__;let currentRun='',loadedReview='',frameBusy=false,lastFrame=-1;
const $=id=>document.getElementById(id);
async function action(path){const response=await fetch(path,{method:'POST',headers:{'X-Preview-Token':token}});const result=await response.json();if(!response.ok)throw Error(result.error||'Request failed');return result;}
$('start').onclick=async()=>{$('start').disabled=true;try{await action('/start');await poll();}catch(e){alert(e.message);}};
$('save').onclick=async()=>{$('save').disabled=true;try{await action('/save');await poll();}catch(e){alert(e.message);$('save').disabled=false;}};
$('playBoth').onclick=()=>{for(const id of ['pid','mpc']){$(id).currentTime=0;$(id).play().catch(e=>{$('status').textContent=e.message;});}};
$('pauseBoth').onclick=()=>{for(const id of ['pid','mpc'])$(id).pause();};
const labels=[['time_s','Time','s'],['phase','Phase',''],['ego_speed_mph','Ego speed','mph'],['target_speed_mph','Lead speed','mph'],['distance_m','Gap','m'],['ttc_s','TTC','s'],['brake_percent','Ego brake','%'],['aeb_active','AEB',''],['target_brake_percent','Lead brake','%'],['target_accel_mps2','Lead acceleration','m/s²'],['ego_accel_mps2','Ego acceleration','m/s²'],['lane_error_m','Lane error','m']];
function metrics(sample){$('metrics').replaceChildren();for(const [key,label,unit] of labels){const box=document.createElement('div');box.className='metric';const title=document.createElement('span');title.textContent=label;const value=document.createElement('strong');let v=sample[key]??'—';if(key==='aeb_active')v=sample[key]===undefined?'—':Number(v)?'ACTIVE':'Off';value.textContent=v+(unit?' '+unit:'');if((key==='ego_speed_mph'||key==='target_speed_mph')&&Number.isFinite(Number(sample[key])))value.textContent+=' / '+(Number(sample[key])*1.609344).toFixed(1)+' km/h';box.append(title,value);$('metrics').append(box);}}
function review(s){if(loadedReview===s.run_id)return;loadedReview=s.run_id;const base='/assets/'+encodeURIComponent(s.run_id)+'/';for(const id of ['pid','mpc']){const file='town10_aeb_'+id+'.mp4';if(s.assets.includes(file))$(id).src=base+file;else $(id).removeAttribute('src');}const report=s.report||{};$('validity').textContent=report.comparison_valid?'Valid paired comparison · provenance and initial traces match.':'Diagnostics only · '+(report.invalid_reasons||['Comparison validation unavailable']).join(', ');$('validity').className=report.comparison_valid?'good':'bad';$('summary').replaceChildren();for(const [key,label] of [['minimum_gap_m','Minimum gap (m)'],['stop_time_from_aeb_s','Stop time after AEB (s)'],['stop_distance_from_aeb_m','Stopping distance after AEB (m)'],['peak_deceleration_fd_mps2','Peak deceleration (m/s²)'],['peak_absolute_jerk_fd_mps3','Peak absolute jerk (m/s³)']]){const row=document.createElement('tr');for(const value of [label,report.runs?.pid?.[key],report.runs?.mpc?.[key]]){const cell=document.createElement('td');cell.textContent=value==null?'Unavailable':typeof value==='number'?value.toFixed(3):value;row.append(cell);}$('summary').append(row);}if(s.assets.includes('comparison/comparison.png')){$('plot').src=base+'comparison/comparison.png';$('plot').classList.remove('hidden');}else $('plot').classList.add('hidden');$('downloads').replaceChildren();for(const [file,label] of [['town10_aeb_pid.csv','PID CSV'],['town10_aeb_mpc.csv','MPC CSV'],['comparison/summary.csv','Summary CSV for graphs'],['comparison/comparison.json','Comparison details']]){if(s.assets.includes(file)){const a=document.createElement('a');a.href=base+file;a.textContent=label;a.target='_blank';$('downloads').append(a);}}$('save').textContent=report.comparison_valid?'Save results':'Save diagnostic outputs';}
function scenario(config){const speed=v=>v+' mph ('+(v*1.609344).toFixed(1)+' km/h)';$('scenario').textContent='Ego: '+speed(config.ego_speed_mph)+'. Lead: '+speed(config.target_speed_mph)+'. Nominal initial gap: '+config.initial_gap_m+' m. The lead vehicle applies '+(config.target_brake*100).toFixed(0)+'% braking after both vehicles hold their target speeds for '+config.stable_hold_s+' s. '+(config.start_at_cruise_speed?'Vehicles begin at cruise speed through a one-time initialization; normal vehicle physics runs afterward, with cruise gears held.':'Vehicles accelerate from rest.')+' An RGB camera records the scene, and a collision sensor records contacts. AEB uses CARLA ground-truth positions, speeds and acceleration; radar and lidar perception are not used.';}
function events(sample,status){const live=status==='running';const threat=['THREAT','AEB'].includes(sample.phase);$('leadEvent').classList.toggle('hidden',!(live&&threat&&(sample.target_brake_percent===undefined||Number(sample.target_brake_percent)>0)));$('aebEvent').classList.toggle('hidden',!(live&&Number(sample.aeb_active)===1));}
async function poll(){try{const response=await fetch('/state',{cache:'no-store'});if(!response.ok)throw Error('Preview server is unavailable');const s=await response.json();if(s.run_id!==currentRun){currentRun=s.run_id;loadedReview='';lastFrame=-1;$('live').classList.add('hidden');for(const id of ['pid','mpc'])$(id).pause();}$('status').textContent=s.message;scenario(s.config);events(s.telemetry||{},s.status);$('config').textContent=s.config.map+' · spawn '+s.config.spawn_index+' · '+s.config.ego_speed_mph+' mph ego / '+s.config.target_speed_mph+' mph lead · CARLA '+s.config.host+':'+s.config.port;$('start').disabled=['running','comparing','saving'].includes(s.status);$('liveTitle').textContent=s.controller?s.controller.toUpperCase()+' · live camera':'Live camera';$('waiting').classList.toggle('hidden',s.frame_number>0);metrics(s.telemetry||{});$('stage').textContent=s.stage?'Preview staging: '+s.stage:'';$('errors').classList.toggle('hidden',!s.error);$('errorText').textContent=s.error||'';const done=['completed','saved','saving'].includes(s.status);$('review').classList.toggle('hidden',!done);if(done)review(s);$('save').disabled=s.status!=='completed';$('saved').textContent=s.saved_directory?'Saved to '+s.saved_directory:'';if(s.frame_number>lastFrame&&!frameBusy){frameBusy=true;const img=$('live');img.onload=()=>{frameBusy=false;img.classList.remove('hidden');};img.onerror=()=>{frameBusy=false;};img.src='/frame.jpg?run='+encodeURIComponent(s.run_id)+'&frame='+s.frame_number;lastFrame=s.frame_number;}}catch(e){$('status').textContent=e.message;}}
poll();setInterval(poll,300);
</script></body></html>'''


def stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")


class Preview:
    def __init__(self, *, root: Path = ROOT, config: str | Path = "config/town10_adas.json",
                 host="host.docker.internal", port=2020, spawn_index=105,
                 review_stage: str | Path | None = None):
        self.root = root.resolve()
        self.config_path = Path(config)
        if not self.config_path.is_absolute():
            self.config_path = self.root / self.config_path
        self.config = load_config(self.config_path)
        self.config = replace(self.config,
            carla=replace(self.config.carla, host=host, port=port, timeout_s=60.0),
            scenario=replace(self.config.scenario, retain_actors_s=0.0))
        if not self.config.carla.synchronous_mode:
            raise ValueError("live comparison requires synchronous_mode=true")
        self.spawn_index = spawn_index
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.Lock()
        self.stages: dict[str, Path] = {}
        self.jpeg: bytes | None = None
        self.worker: threading.Thread | None = None
        self.state = dict(status="idle", message="Ready to start", run_id="", controller=None,
                          frame_number=0, telemetry={}, report=None, assets=[], stage=None,
                          saved_directory=None, error=None)
        if review_stage is not None:
            self.load_review_stage(review_stage)

    def snapshot(self):
        with self.lock:
            return self.state.copy() | {"config": {
                "host": self.config.carla.host, "port": self.config.carla.port,
                "map": self.config.carla.map, "spawn_index": self.spawn_index,
                "ego_speed_mph": self.config.scenario.ego_speed_mph,
                "target_speed_mph": self.config.scenario.target_speed_mph,
                "initial_gap_m": self.config.scenario.initial_gap_m,
                "stable_hold_s": self.config.scenario.stable_hold_s,
                "target_brake": self.config.scenario.target_brake,
                "start_at_cruise_speed": getattr(self.config.scenario, "start_at_cruise_speed", False),
                "fixed_delta_seconds": self.config.carla.fixed_delta_seconds}}

    def load_review_stage(self, path: str | Path):
        """Expose a completed temporary run without starting CARLA or copying results."""
        stage = Path(path)
        if not stage.is_absolute():
            stage = self.root / stage
        stage = stage.resolve()
        runtime = (self.root / ".runtime").resolve()
        if not stage.is_relative_to(runtime) or stage == runtime or not stage.is_dir():
            raise ValueError("review stage must be a run directory under this repository's .runtime")
        required = {f"town10_aeb_{controller}.{extension}" for controller in ("pid", "mpc")
                    for extension in ("csv", "metadata.json", "mp4")}
        required.add("comparison/comparison.json")
        missing = sorted(name for name in required if not (stage / name).is_file())
        if missing:
            raise ValueError("review stage is incomplete: " + ", ".join(missing))
        expected = asdict(self.config.scenario)
        expected.pop("retain_actors_s", None)
        for controller in ("pid", "mpc"):
            metadata = json.loads((stage / f"town10_aeb_{controller}.metadata.json").read_text())
            shared = metadata.get("shared_settings", {})
            observed = dict(shared.get("config", {}).get("scenario", {}))
            observed.pop("retain_actors_s", None)
            if observed != expected or shared.get("route", {}).get("spawn_index") != self.spawn_index:
                raise ValueError("review stage scenario/spawn does not match the selected configuration")
        report = json.loads((stage / "comparison/comparison.json").read_text())
        if report.get("source") != "carla" or not isinstance(report.get("comparison_valid"), bool):
            raise ValueError("review stage must contain a CARLA comparison report")
        run_id = stage.name
        with self.lock:
            self.stages[run_id] = stage
            self.state.update(status="completed", run_id=run_id, stage=str(stage), report=report,
                assets=sorted(name for name in ASSETS if (stage / name).is_file()),
                message="Ready to review · " + ("valid paired comparison" if report["comparison_valid"]
                                                else "diagnostics only; comparison invalid"))

    def start(self):
        with self.lock:
            if self.state["status"] in {"running", "comparing", "saving"}:
                raise ValueError("A comparison or save is already running")
            run_id = "preview_" + stamp() + "_" + secrets.token_hex(3)
            stage = self.root / ".runtime" / run_id
            stage.mkdir(parents=True, exist_ok=False)
            self.stages[run_id] = stage
            self.jpeg = None
            self.state = dict(status="running", message="Connecting to CARLA for PID…",
                              run_id=run_id, controller="pid", frame_number=0, telemetry={},
                              report=None, assets=[], stage=str(stage), saved_directory=None, error=None)
            self.worker = threading.Thread(target=self._run_pair, args=(stage,), daemon=False,
                                           name="carla-preview")
            self.worker.start()
            return {"run_id": run_id, "stage": str(stage)}

    def _update(self, **values):
        with self.lock:
            self.state.update(values)

    def _run_pair(self, stage: Path):
        # Import CARLA and Pillow only in the worker, keeping startup errors visible in the UI.
        try:
            from PIL import Image
            from town10_adas.carla_runner import run
            config = replace(self.config, logging=replace(self.config.logging,
                                                         output_directory=str(stage)))
            (stage / "preview_config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
            outcomes = {}
            for controller in ("pid", "mpc"):
                self._update(controller=controller, telemetry={},
                             message=f"Reloading CARLA world for {controller.upper()}…")
                clock_origin = None
                def publish(image, sample):
                    nonlocal clock_origin
                    simulation_time = float(sample["time_s"])
                    if clock_origin is None:
                        clock_origin = time.monotonic() - simulation_time
                    remaining = clock_origin + simulation_time - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                    rgb = Image.frombytes("RGBA", (image.width, image.height),
                                          bytes(image.raw_data), "raw", "BGRA").convert("RGB")
                    stream = io.BytesIO()
                    rgb.save(stream, format="JPEG", quality=80)
                    with self.lock:
                        self.jpeg = stream.getvalue()
                        self.state.update(telemetry=dict(sample),
                            frame_number=self.state["frame_number"] + 1,
                            message=f"{controller.upper()} live · {sample['phase']} · {simulation_time:.2f} s")
                output = stage / f"town10_aeb_{controller}.csv"
                outcomes[controller] = run(config, controller, output, self.spawn_index,
                    video_path=output.with_suffix(".mp4"), reload_world=True,
                    video_overlay=True, preview_callback=publish)
                with (stage / "preview.log").open("a", encoding="utf-8") as log:
                    log.write(f"{controller.upper()} runner return code: {outcomes[controller]}\n")
            self._update(status="comparing", controller=None,
                         message="Preparing graphs and checking comparison validity…")
            completed = subprocess.run([sys.executable, str(self.root / "scripts/compare_aeb_results.py"),
                "--pid", str(stage / "town10_aeb_pid.csv"),
                "--mpc", str(stage / "town10_aeb_mpc.csv"),
                "--output-dir", str(stage / "comparison")],
                cwd=self.root, capture_output=True, text=True, timeout=180)
            with (stage / "preview.log").open("a", encoding="utf-8") as log:
                log.write(completed.stdout + completed.stderr)
            report_path = stage / "comparison/comparison.json"
            if not report_path.is_file():
                raise RuntimeError(f"Comparison could not produce a report (exit {completed.returncode}):\n"
                                   + completed.stdout + completed.stderr)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            if completed.returncode not in (0, 2):
                raise RuntimeError(f"Comparison command failed (exit {completed.returncode}):\n"
                                   + completed.stdout + completed.stderr)
            if any(outcomes.values()) and report.get("comparison_valid"):
                raise RuntimeError("A scenario did not complete but its report claims validity")
            assets = sorted(name for name in ASSETS if (stage / name).is_file())
            self._update(status="completed", report=report, assets=assets,
                message="Ready to review · " + ("valid paired comparison" if report.get("comparison_valid")
                                                else "diagnostics only; comparison invalid"),
                error=completed.stderr.strip() or None)
        except BaseException:
            detail = traceback.format_exc()
            try:
                with (stage / "preview.log").open("a", encoding="utf-8") as log:
                    log.write(detail)
            except OSError:
                pass
            self._update(status="error", message="Preview failed · staged files retained",
                         controller=None, error=detail)
            print(detail, file=sys.stderr, flush=True)

    def save(self):
        with self.lock:
            if self.state["status"] != "completed":
                raise ValueError("Finish and review a comparison before saving")
            stage = self.stages[self.state["run_id"]]
            destination = self.root / "results" / ("run_" + stamp() + "_" + secrets.token_hex(3))
            self.state.update(status="saving", message="Copying reviewed outputs to results…")
        try:
            # Original trace/video metadata and stage references are preserved verbatim.
            shutil.copytree(stage, destination)
        except Exception:
            self._update(status="completed", message="Save failed · preview is still available",
                         error=traceback.format_exc())
            raise
        self._update(status="saved", saved_directory=str(destination),
                     message="Reviewed outputs saved")
        return {"saved_directory": str(destination)}


class PreviewServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, preview: Preview):
        self.preview = preview
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server: PreviewServer
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):
        if not self.path.startswith(("/state", "/frame.jpg")):
            super().log_message(format, *args)

    def _headers(self, status, content_type, length, **headers):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        for name, value in headers.items():
            self.send_header(name.replace("_", "-"), str(value))
        self.end_headers()

    def _bytes(self, status, content_type, payload, **headers):
        self._headers(status, content_type, len(payload), **headers)
        if self.command != "HEAD":
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def _json(self, status, value):
        self._bytes(status, "application/json; charset=utf-8",
                    json.dumps(value, allow_nan=False).encode("utf-8"))

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        path = unquote(urlsplit(self.path).path)
        preview = self.server.preview
        if path == "/":
            self._bytes(200, "text/html; charset=utf-8",
                        PAGE.replace("__TOKEN__", json.dumps(preview.token)).encode("utf-8"))
        elif path == "/state":
            self._json(200, preview.snapshot())
        elif path == "/frame.jpg":
            with preview.lock:
                jpeg = preview.jpeg
            if jpeg:
                self._bytes(200, "image/jpeg", jpeg)
            else:
                self._json(404, {"error": "No camera frame yet"})
        elif path.startswith("/assets/"):
            parts = path.split("/", 3)
            if len(parts) != 4 or parts[3] not in ASSETS:
                self._json(404, {"error": "Unknown asset"})
                return
            with preview.lock:
                stage = preview.stages.get(parts[2])
                active = preview.state["run_id"] == parts[2] and preview.state["status"] in {"running", "comparing"}
            if stage is None or active:
                self._json(404, {"error": "Recording is not ready"})
                return
            file = stage / parts[3]
            if not file.is_file() or file.is_symlink() or not file.resolve().is_relative_to(stage.resolve()):
                self._json(404, {"error": "Asset unavailable"})
                return
            self._file(file)
        else:
            self._json(404, {"error": "Not found"})

    def _file(self, path):
        size = path.stat().st_size
        start, end = 0, size - 1
        requested = self.headers.get("Range")
        status = 200
        if requested:
            match = re.fullmatch(r"bytes=(\d*)-(\d*)", requested.strip())
            if match is None or not any(match.groups()) or size == 0:
                self._bytes(416, "text/plain", b"Invalid byte range", Content_Range=f"bytes */{size}")
                return
            first, last = match.groups()
            if first:
                start = int(first)
                end = min(int(last), size - 1) if last else size - 1
            else:
                count = int(last)
                start = max(0, size - count)
            if start > end or start >= size or (not first and int(last) == 0):
                self._bytes(416, "text/plain", b"Range unavailable", Content_Range=f"bytes */{size}")
                return
            status = 206
        length = max(0, end - start + 1)
        extra = {"Accept_Ranges": "bytes"}
        if status == 206:
            extra["Content_Range"] = f"bytes {start}-{end}/{size}"
        self._headers(status, mimetypes.guess_type(path.name)[0] or "application/octet-stream", length, **extra)
        if self.command == "HEAD":
            return
        try:
            with path.open("rb") as stream:
                stream.seek(start)
                while length:
                    chunk = stream.read(min(length, 256 * 1024))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    length -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        preview = self.server.preview
        # The unguessable session token is embedded only in our same-origin page.
        # No CORS headers are provided, so other websites cannot read/use it.
        token = self.headers.get("X-Preview-Token", "")
        if not secrets.compare_digest(token, preview.token):
            self.close_connection = True
            self._json(403, {"error": "Invalid preview session token; reload the page"})
            return
        # These actions do not accept a body; close rather than leave bytes on a keepalive socket.
        if self.headers.get("Transfer-Encoding") or self.headers.get("Content-Length", "0") != "0":
            self.close_connection = True
            self._json(400, {"error": "Request body is not supported"})
            return
        path = urlsplit(self.path).path
        try:
            if path == "/start":
                self._json(202, preview.start())
            elif path == "/save":
                self._json(200, preview.save())
            else:
                self._json(404, {"error": "Not found"})
        except ValueError as exc:
            self._json(409, {"error": str(exc)})
        except Exception as exc:
            self._json(500, {"error": f"{type(exc).__name__}: {exc}"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/town10_adas.json"),
                        help="Scenario configuration, relative to the repository or absolute")
    parser.add_argument("--review-stage", type=Path,
                        help="Review a completed .runtime run immediately without starting CARLA")
    parser.add_argument("--host", default="host.docker.internal", help="CARLA RPC host")
    parser.add_argument("--port", type=int, default=2020, help="CARLA RPC port")
    parser.add_argument("--listen-host", default="127.0.0.1",
                        help="Preview HTTP bind address (default: localhost only)")
    parser.add_argument("--listen-port", type=int, default=8765, help="Preview HTTP port")
    parser.add_argument("--spawn-index", type=int, default=105)
    args = parser.parse_args()
    if not args.host.strip() or not args.listen_host.strip() or not 1 <= args.port <= 65535 or not 1 <= args.listen_port <= 65535:
        parser.error("hosts must be nonempty and ports must be between 1 and 65535")
    if args.spawn_index < 0:
        parser.error("spawn index must be nonnegative")
    preview = Preview(config=args.config, host=args.host, port=args.port,
                      spawn_index=args.spawn_index, review_stage=args.review_stage)
    server = PreviewServer((args.listen_host, args.listen_port), preview)
    print(f"CARLA preview ready at http://localhost:{args.listen_port} · {preview.snapshot()['message']}", flush=True)
    print(f"HTTP bind: {args.listen_host}:{args.listen_port}. Final results require the Save results button.", flush=True)
    if args.listen_host == "0.0.0.0":
        print("Publish this container port only on 127.0.0.1.", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("Stopping HTTP preview. Any active comparison finishes its cleanup before exit.", flush=True)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
