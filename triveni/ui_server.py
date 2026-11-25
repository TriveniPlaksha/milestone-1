# -*- coding: utf-8 -*-
"""ui_server.py - Visual UI for the TRIVENI Core Guidance & Safety Emulator.

Thin HTTP wrapper around the EXISTING backend.  It does not re-implement any
safety logic: every decision comes from triveni.safety.safety_engine.evaluate(),
inputs are built with the existing simulators / models, and every decision is
written with the existing EventLogger.

Uses only the Python standard library (plus PyYAML, already required).

Usage:
    python ui_server.py              # http://127.0.0.1:8000
    python ui_server.py --port 8080
"""
from __future__ import annotations

import argparse
import json
import sys
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional

_ROOT = Path(__file__).parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import yaml

from triveni.models.crosswalk_map import CrosswalkMap
from triveni.models.system_health import SystemHealth
from triveni.safety.safety_engine import evaluate
from triveni.logging.event_logger import EventLogger
from triveni.simulator.signal_simulator import (
    CROSSWALK_IDS,
    INTERSECTION_ID,
    TWO_PHASE_CLEARANCE_A_MS,
    TWO_PHASE_CLEARANCE_B_MS,
    TWO_PHASE_CYCLE_MS,
    TWO_PHASE_EW_GREEN_MS,
    TWO_PHASE_NS_GREEN_MS,
    make_signal,
    two_phase_signals,
)
from triveni.simulator.pedestrian_simulator import make_localization

# Routing (source -> destination) reuses the existing CLI tool's logic.
import triveni_query as tq

# ---------------------------------------------------------------------------
# Paths / startup state
# ---------------------------------------------------------------------------
_CONFIG_PATH = _ROOT / "config" / "triveni.yaml"
_MAP_PATH = _ROOT / "data" / "intersection_demo.json"
_LOG_PATH = _ROOT / "logs" / "ui_decision_log.jsonl"
_UI_DIR = _ROOT / "ui"

with open(_CONFIG_PATH, "r", encoding="utf-8") as _f:
    BASE_CFG: Dict[str, Any] = yaml.safe_load(_f)

with open(_MAP_PATH, "r", encoding="utf-8") as _f:
    MAP_JSON: Dict[str, Any] = json.load(_f)

CMAP = CrosswalkMap.from_dict(MAP_JSON)
LOGGER = EventLogger(_LOG_PATH)   # mode "w": each server run starts a fresh log


# ---------------------------------------------------------------------------
# Request -> backend objects
# ---------------------------------------------------------------------------

def _phase_label(now_ms: int) -> str:
    t = now_ms % TWO_PHASE_CYCLE_MS
    a = TWO_PHASE_EW_GREEN_MS
    b = a + TWO_PHASE_CLEARANCE_A_MS
    c = b + TWO_PHASE_NS_GREEN_MS
    if t < a:
        return "Phase A - E-W vehicles GREEN"
    if t < b:
        return "Clearance A"
    if t < c:
        return "Phase B - N-S vehicles GREEN"
    return "Clearance B"


def _opt_int(v: Any) -> Optional[int]:
    if v is None or v == "":
        return None
    return int(v)


def _build_signals(now_ms: int, spec: Dict[str, Any]) -> Dict[str, Any]:
    """Build per-crosswalk SignalStates.

    spec[cw_id] = {mode: AUTO|WALK|DONT_WALK|CLEARANCE|MISSING,
                   age_ms, remaining_ms (None = auto), health, wrong_intersection}
    AUTO takes state/remaining from the existing two_phase_signals() simulator.
    """
    auto = two_phase_signals(now_ms, cycle_start_ms=0)
    out: Dict[str, Any] = {}
    for cw_id in CROSSWALK_IDS:
        s = spec.get(cw_id, {}) or {}
        mode = s.get("mode", "AUTO")
        if mode == "MISSING":
            continue
        state = auto[cw_id].signal_state if mode == "AUTO" else mode
        remaining = _opt_int(s.get("remaining_ms"))
        if remaining is None and mode == "AUTO":
            remaining = auto[cw_id].remaining_time_ms
        if s.get("omit_remaining"):
            remaining = None
        out[cw_id] = make_signal(
            cw_id,
            state,
            now_ms,
            remaining_time_ms=remaining,
            source_age_ms=int(s.get("age_ms", 50)),
            receive_age_ms=int(s.get("receive_age_ms", 10)),
            source_health=s.get("health", "OK"),
            intersection_id=("INT_WRONG_INTERSECTION"
                             if s.get("wrong_intersection") else INTERSECTION_ID),
        )
    return out


def _build_localization(now_ms: int, spec: Dict[str, Any]):
    if not spec.get("present", True):
        return None
    return make_localization(
        float(spec.get("x", 5.5)),
        float(spec.get("y", 9.5)),
        now_ms,
        heading_deg=float(spec.get("heading", 180.0)),
        position_uncertainty_m=float(spec.get("uncertainty", 0.75)),
        health=spec.get("health", "OK"),
        intersection_id=("INT_WRONG_INTERSECTION"
                         if spec.get("wrong_intersection") else INTERSECTION_ID),
        age_ms=int(spec.get("age_ms", 35)),
    )


def _build_health(spec: Dict[str, Any]) -> Optional[SystemHealth]:
    if not spec.get("present", True):
        return None
    return SystemHealth(
        edge_service=spec.get("edge_service", "OK"),
        signal_input=spec.get("signal_input", "OK"),
        localization_input=spec.get("localization_input", "OK"),
        map_loaded=bool(spec.get("map_loaded", True)),
        clock_status=spec.get("clock_status", "OK"),
    )


def _build_config(overrides: Dict[str, Any]) -> Dict[str, Any]:
    cfg = dict(BASE_CFG)
    for k, v in (overrides or {}).items():
        if k in cfg and v is not None and v != "":
            cfg[k] = v
    return cfg


def handle_evaluate(body: Dict[str, Any]) -> Dict[str, Any]:
    now_ms = int(body.get("now_ms", 0))
    signals = _build_signals(now_ms, body.get("signals", {}))
    loc = _build_localization(now_ms, body.get("localization", {}))
    health = _build_health(body.get("health", {}))
    cfg = _build_config(body.get("config", {}))
    intended = body.get("intended_crosswalk_id") or None

    result = evaluate(signals, CMAP, loc, health, now_ms,
                      intended_crosswalk_id=intended, config=cfg)
    if body.get("log", True):
        LOGGER.log(result.decision, result.evidence)

    d = result.decision
    return {
        "decision": {
            "decision": d.decision,
            "reason_codes": d.reason_codes,
            "intersection_id": d.intersection_id,
            "crosswalk_id": d.crosswalk_id,
            "decision_timestamp_ms": d.decision_timestamp_ms,
        },
        "evidence": result.evidence,
        "signals": {
            cw: {
                "state": s.signal_state,
                "remaining_ms": s.remaining_time_ms,
                "age_ms": now_ms - s.source_timestamp_ms,
                "health": s.source_health,
                "intersection_id": s.intersection_id,
            }
            for cw, s in signals.items()
        },
        "phase": _phase_label(now_ms),
        "cycle_t": now_ms % TWO_PHASE_CYCLE_MS,
        # Routing guidance exactly as triveni_query computes it (_phase_timing)
        "timing": {
            cw: dict(zip(("status", "walk_remaining_ms", "wait_ms"), tq._phase_timing(cw, now_ms)),
                     phase_label=tq.CROSSWALK_DATA[cw]["phase_label"])
            for cw in CROSSWALK_IDS
        },
    }


def handle_init() -> Dict[str, Any]:
    return {
        "map": MAP_JSON,
        "config": BASE_CFG,
        "crosswalk_ids": CROSSWALK_IDS,
        "intersection_id": INTERSECTION_ID,
        "cycle": {
            "total": TWO_PHASE_CYCLE_MS,
            "a": TWO_PHASE_EW_GREEN_MS,
            "clr_a": TWO_PHASE_CLEARANCE_A_MS,
            "b": TWO_PHASE_NS_GREEN_MS,
            "clr_b": TWO_PHASE_CLEARANCE_B_MS,
        },
        "log_path": str(_LOG_PATH.resolve()),
        "named_locations": tq.NAMED_LOCATIONS,
    }


def handle_route(body: Dict[str, Any]) -> Dict[str, Any]:
    """Plan source -> destination using triveni_query's routing helpers."""
    src_txt = str(body.get("source", "")).strip()
    dst_txt = str(body.get("destination", "")).strip()
    src = tq._parse_location(src_txt)
    dst = tq._parse_location(dst_txt)
    if src is None:
        return {"error": "Could not understand source {!r}. Use a name (CW_NORTH, NE, ...) or 'x y'.".format(src_txt)}
    if dst is None:
        return {"error": "Could not understand destination {!r}. Use a name (CW_NORTH_FAR, SW, ...) or 'x y'.".format(dst_txt)}
    for label, (x, y) in (("Source", src), ("Destination", dst)):
        if tq._is_in_road(x, y):
            nn, nx, ny = tq._nearest_safe(x, y)
            return {"error": "{} ({}, {}) is inside the vehicle road zone. Nearest safe sidewalk: {} ({}, {}).".format(
                label, x, y, nn, nx, ny)}

    sx, sy = src
    dx, dy = dst
    steps = []
    cur = (sx, sy)
    for cw_id, from_q, to_q in tq._find_route(sx, sy, dx, dy):
        entry = tq.CROSSWALK_DATA[cw_id]["sides"][from_q]
        exit_ = tq.CROSSWALK_DATA[cw_id]["sides"][to_q]
        d1, m1 = tq._compass(cur[0], cur[1], entry[0], entry[1])
        steps.append({"type": "walk", "to": entry,
                      "label": ("Walk {} {:.1f} m to {} curb ({} side)".format(d1, m1, cw_id, from_q)
                                if m1 > 0 else "Already at {} curb ({} side)".format(cw_id, from_q))})
        steps.append({"type": "wait", "cw": cw_id, "at": entry,
                      "label": "Wait at {} until TRIVENI says WALK".format(cw_id)})
        d2, m2 = tq._compass(entry[0], entry[1], exit_[0], exit_[1])
        steps.append({"type": "cross", "cw": cw_id, "to": exit_,
                      "label": "Cross {} heading {} ({:.1f} m) to {} side".format(cw_id, d2, m2, to_q)})
        cur = exit_
    d3, m3 = tq._compass(cur[0], cur[1], dx, dy)
    steps.append({"type": "walk", "to": [dx, dy],
                  "label": "Walk {} {:.1f} m to destination".format(d3, m3) if m3 > 0 else "Arrive at destination"})
    return {
        "source": {"label": src_txt.upper(), "x": sx, "y": sy},
        "destination": {"label": dst_txt.upper(), "x": dx, "y": dy},
        "steps": steps,
    }


# ---------------------------------------------------------------------------
# HTTP plumbing
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # keep console quiet
        pass

    def _send(self, code: int, payload: Any, ctype: str = "application/json") -> None:
        data = (json.dumps(payload, default=str).encode("utf-8")
                if ctype == "application/json" else payload)
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, (_UI_DIR / "index.html").read_bytes(), "text/html")
        elif path == "/api/init":
            self._send(200, handle_init())
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
            if path == "/api/evaluate":
                self._send(200, handle_evaluate(body))
            elif path == "/api/route":
                self._send(200, handle_route(body))
            else:
                self._send(404, {"error": "not found"})
        except (ValueError, TypeError) as exc:
            # Model validation rejected the input (e.g. negative uncertainty)
            self._send(400, {"error": str(exc)})


def main() -> None:
    ap = argparse.ArgumentParser(description="TRIVENI visual UI")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    url = "http://127.0.0.1:{}".format(args.port)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print("  TRIVENI UI running at {}".format(url))
    print("  Decisions logged to {}".format(_LOG_PATH.resolve()))
    print("  Ctrl+C to stop.")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  Stopped.")


if __name__ == "__main__":
    main()
