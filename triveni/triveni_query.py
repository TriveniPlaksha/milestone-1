# -*- coding: utf-8 -*-
"""triveni_query.py - Interactive TRIVENI pedestrian crossing guidance tool.

Two modes of operation:
  1. SINGLE LOCATION  : enter a crosswalk name or coordinates
     -> shows current signal state, predicate checklist, WALK/WAIT decision

  2. SOURCE -> DESTINATION : enter two locations separated by " to " or "->"
     -> full routing: which crosswalk, direction to walk, when to wait,
        how long until WALK, direction across, direction to destination

Input formats
-------------
  Single:
    CW_NORTH                 crosswalk name (entry zone)
    5.5 9.5                  x y coordinates
    5.5 9.5 180              x y heading

  Routing:
    CW_NORTH to CW_NORTH_FAR
    5.5 9.5 to -5.5 9.5
    NE to SW
    CW_NORTH to -9.5 -5.5

Named locations
---------------
  Crosswalk curb (where you wait to cross):
    CW_NORTH   ->  ( 5.5,  9.5)  NE curb  - Phase A: WALK
    CW_EAST    ->  ( 9.5, -5.5)  SE curb  - Phase B: WALK
    CW_SOUTH   ->  (-5.5, -9.5)  SW curb  - Phase A: WALK
    CW_WEST    ->  (-9.5,  5.5)  NW curb  - Phase B: WALK
  Far-side arrival (after crossing):
    CW_NORTH_FAR  (-5.5,  9.5)  NW side
    CW_EAST_FAR   ( 9.5,  5.5)  NE side
    CW_SOUTH_FAR  ( 5.5, -9.5)  SE side
    CW_WEST_FAR   (-9.5, -5.5)  SW side
  Corner shortcuts:
    NE (9.5,9.5)  NW (-9.5,9.5)  SE (9.5,-9.5)  SW (-9.5,-9.5)

Road zone
---------
  The vehicle road is the central area: roughly |x|<8 AND |y|<8.
  Source and destination should be on the SIDEWALK (outside the road zone).
  TRIVENI will warn if you enter a road-zone coordinate.

Signal cycle (two-phase India-style)
--------------------------------------
  Phase A   t=  0..44999 ms  E-W vehicles GREEN  CW_NORTH & CW_SOUTH: WALK
  Clr A     t=45000..49999
  Phase B   t=50000..94999 ms  N-S vehicles GREEN  CW_EAST & CW_WEST: WALK
  Clr B     t=95000..99999
  Cycle = 100,000 ms

Commands
--------
  time <ms>  set now_ms (e.g. time 60000)
  next       advance 5000 ms
  phase      show current phase
  map        show intersection map
  quit/q     exit
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

_ROOT = Path(__file__).parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import yaml

from triveni.models.crosswalk_map import CrosswalkMap
from triveni.models.system_health import SystemHealth
from triveni.safety.safety_engine import evaluate
from triveni.logging.event_logger import EventLogger
from triveni.simulator.signal_simulator import (
    two_phase_signals,
    TWO_PHASE_CYCLE_MS,
    TWO_PHASE_EW_GREEN_MS,
)
from triveni.simulator.pedestrian_simulator import make_localization

# ---------------------------------------------------------------------------
# Paths / config
# ---------------------------------------------------------------------------
_CONFIG_PATH = _ROOT / "config" / "triveni.yaml"
_MAP_PATH    = _ROOT / "data"   / "intersection_demo.json"
_LOG_PATH    = _ROOT / "logs"   / "decision_log.jsonl"

with open(_CONFIG_PATH, "r", encoding="utf-8") as _f:
    CFG = yaml.safe_load(_f)

CMAP = CrosswalkMap.from_json(_MAP_PATH)
LOGGER = EventLogger(_LOG_PATH)

GOOD_HEALTH = SystemHealth(
    edge_service="OK", signal_input="OK", localization_input="OK",
    map_loaded=True, clock_status="OK",
)

# ---------------------------------------------------------------------------
# Intersection data
# ---------------------------------------------------------------------------
# Each crosswalk:
#   sides       : dict of quadrant -> (x, y) curb-centre coordinates
#   walk_start  : ms in cycle when WALK begins
#   walk_end    : ms in cycle when WALK ends (CLEARANCE begins)
#   phase_label : human-readable phase name
# ---------------------------------------------------------------------------
CROSSWALK_DATA = {
    "CW_NORTH": {
        "sides": {
            "NE": ( 5.5,  9.5),   # entry zone centre  x=[3,8]  y=[8,11]
            "NW": (-5.5,  9.5),   # exit zone centre   x=[-8,-3] y=[8,11]
        },
        "walk_start": 0,
        "walk_end":   45_000,
        "phase_label": "Phase A (E-W vehicles GREEN, t=0-44999 ms)",
    },
    "CW_EAST": {
        "sides": {
            "SE": ( 9.5, -5.5),   # entry zone centre  x=[8,11] y=[-8,-3]
            "NE": ( 9.5,  5.5),   # exit zone centre   x=[8,11] y=[3,8]
        },
        "walk_start": 50_000,
        "walk_end":   95_000,
        "phase_label": "Phase B (N-S vehicles GREEN, t=50000-94999 ms)",
    },
    "CW_SOUTH": {
        "sides": {
            "SW": (-5.5, -9.5),   # entry zone centre  x=[-8,-3] y=[-11,-8]
            "SE": ( 5.5, -9.5),   # exit zone centre   x=[3,8]  y=[-11,-8]
        },
        "walk_start": 0,
        "walk_end":   45_000,
        "phase_label": "Phase A (E-W vehicles GREEN, t=0-44999 ms)",
    },
    "CW_WEST": {
        "sides": {
            "NW": (-9.5,  5.5),   # entry zone centre  x=[-11,-8] y=[3,8]
            "SW": (-9.5, -5.5),   # exit zone centre   x=[-11,-8] y=[-8,-3]
        },
        "walk_start": 50_000,
        "walk_end":   95_000,
        "phase_label": "Phase B (N-S vehicles GREEN, t=50000-94999 ms)",
    },
}

# Named sidewalk locations users can type
NAMED_LOCATIONS: dict[str, tuple[float, float]] = {
    # Crosswalk entry curbs (where you WAIT to cross)
    "CW_NORTH":       ( 5.5,  9.5),
    "CW_EAST":        ( 9.5, -5.5),
    "CW_SOUTH":       (-5.5, -9.5),
    "CW_WEST":        (-9.5,  5.5),
    # Entry aliases
    "CW_NORTH_ENTRY": ( 5.5,  9.5),
    "CW_EAST_ENTRY":  ( 9.5, -5.5),
    "CW_SOUTH_ENTRY": (-5.5, -9.5),
    "CW_WEST_ENTRY":  (-9.5,  5.5),
    # Far-side / exit curbs (where you arrive after crossing)
    "CW_NORTH_FAR":   (-5.5,  9.5),
    "CW_EAST_FAR":    ( 9.5,  5.5),
    "CW_SOUTH_FAR":   ( 5.5, -9.5),
    "CW_WEST_FAR":    (-9.5, -5.5),
    # Exit aliases (same as FAR)
    "CW_NORTH_EXIT":  (-5.5,  9.5),
    "CW_EAST_EXIT":   ( 9.5,  5.5),
    "CW_SOUTH_EXIT":  ( 5.5, -9.5),
    "CW_WEST_EXIT":   (-9.5, -5.5),
    # Corner shortcuts (sidewalk positions near intersection corners)
    "NE": ( 9.5,  9.5),
    "NW": (-9.5,  9.5),
    "SE": ( 9.5, -9.5),
    "SW": (-9.5, -9.5),
}

# Single-crosswalk connections between adjacent quadrants
# CW_NORTH: NE <-> NW  (east-west crossing on the north side)
# CW_EAST:  SE <-> NE  (north-south crossing on the east side)
# CW_SOUTH: SW <-> SE  (east-west crossing on the south side)
# CW_WEST:  NW <-> SW  (north-south crossing on the west side)
#
# Quad layout:
#   NW ---[CW_NORTH]--- NE
#   |                    |
# [CW_WEST]          [CW_EAST]
#   |                    |
#   SW ---[CW_SOUTH]--- SE

QUAD_CONNECTIONS: dict[frozenset, str] = {
    frozenset(["NE", "NW"]): "CW_NORTH",
    frozenset(["NE", "SE"]): "CW_EAST",
    frozenset(["SW", "SE"]): "CW_SOUTH",
    frozenset(["NW", "SW"]): "CW_WEST",
}

# Two-crossing routes for diagonal movements
# Format: (src_quad, dst_quad) -> [(cw_id, from_quad, to_quad), ...]
TWO_STEP: dict[tuple, list] = {
    ("NE", "SW"): [("CW_NORTH", "NE", "NW"), ("CW_WEST",  "NW", "SW")],
    ("SW", "NE"): [("CW_WEST",  "SW", "NW"), ("CW_NORTH", "NW", "NE")],
    ("NW", "SE"): [("CW_NORTH", "NW", "NE"), ("CW_EAST",  "NE", "SE")],
    ("SE", "NW"): [("CW_EAST",  "SE", "NE"), ("CW_NORTH", "NE", "NW")],
}


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _get_quadrant(x: float, y: float) -> str:
    if x >= 0 and y >= 0: return "NE"
    if x <  0 and y >= 0: return "NW"
    if x >= 0 and y <  0: return "SE"
    return "SW"


def _compass(x1: float, y1: float, x2: float, y2: float) -> tuple[str, float]:
    """Return (cardinal_direction, distance_m) from (x1,y1) to (x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.05:
        return "already here", 0.0
    angle = math.degrees(math.atan2(dy, dx))  # 0=East, 90=North, 180=West
    if   -22.5 <= angle <   22.5: return "East",       dist
    elif  22.5 <= angle <   67.5: return "North-East",  dist
    elif  67.5 <= angle <  112.5: return "North",       dist
    elif 112.5 <= angle <  157.5: return "North-West",  dist
    elif angle >= 157.5 or angle < -157.5: return "West", dist
    elif -157.5 <= angle < -112.5: return "South-West",  dist
    elif -112.5 <= angle <  -67.5: return "South",       dist
    else:                           return "South-East",  dist


def _is_in_road(x: float, y: float) -> bool:
    """True if (x,y) is inside the vehicle road/intersection area."""
    # Road area: within the |x|<8, |y|<8 central box
    # Entry/exit zones start at y=8 or x=8 (boundary), so |x|<8 AND |y|<8 is road only
    return abs(x) < 7.5 and abs(y) < 7.5


def _nearest_safe(x: float, y: float) -> tuple[str, float, float]:
    """Return the nearest named sidewalk location to (x,y)."""
    best_name, best_x, best_y, best_d = "CW_NORTH", 5.5, 9.5, float("inf")
    for name, (nx, ny) in NAMED_LOCATIONS.items():
        d = math.sqrt((x - nx) ** 2 + (y - ny) ** 2)
        if d < best_d:
            best_d, best_name, best_x, best_y = d, name, nx, ny
    return best_name, best_x, best_y


# ---------------------------------------------------------------------------
# Phase / signal timing
# ---------------------------------------------------------------------------

def _phase_timing(cw_id: str, now_ms: int) -> tuple[str, int, int]:
    """
    Returns (status, walk_remaining_ms, wait_until_walk_ms)
      status            : "WALK" | "CLEARANCE" | "DONT_WALK"
      walk_remaining_ms : ms left in WALK phase (0 if not WALK)
      wait_until_walk_ms: ms until next WALK (0 if currently WALK)
    """
    t  = now_ms % TWO_PHASE_CYCLE_MS
    ws = CROSSWALK_DATA[cw_id]["walk_start"]   # 0 or 50000
    we = CROSSWALK_DATA[cw_id]["walk_end"]     # 45000 or 95000
    ce = we + 5_000                            # clearance end (50000 or 100000)

    if ws <= t < we:
        # Currently in WALK phase
        return "WALK", we - t, 0

    elif we <= t < ce:
        # Clearance phase - DO NOT start crossing
        # Next WALK: at ws of next cycle
        wait = (TWO_PHASE_CYCLE_MS - t) + ws
        return "CLEARANCE", 0, wait

    else:
        # DONT_WALK
        if t < ws:
            wait = ws - t                           # Phase A crosswalks during Phase B
        else:
            wait = TWO_PHASE_CYCLE_MS - t + ws     # Phase B crosswalks during Phase A
        return "DONT_WALK", 0, wait


def _phase_label_str(now_ms: int) -> str:
    t = now_ms % TWO_PHASE_CYCLE_MS
    if   t < 45_000: return f"Phase A (E-W GREEN, t={t})  CW_NORTH & CW_SOUTH: WALK"
    elif t < 50_000: return f"Clearance A (t={t})         CW_NORTH & CW_SOUTH: CLEARANCE"
    elif t < 95_000: return f"Phase B (N-S GREEN, t={t})  CW_EAST & CW_WEST: WALK"
    else:            return f"Clearance B (t={t})          CW_EAST & CW_WEST: CLEARANCE"


# ---------------------------------------------------------------------------
# Route finder
# ---------------------------------------------------------------------------

def _find_route(
    sx: float, sy: float, dx: float, dy: float
) -> list[tuple[str, str, str]]:
    """
    Returns list of (cw_id, from_quad, to_quad) crossing steps.
    Empty list means same quadrant (no road crossing needed).
    """
    sq = _get_quadrant(sx, sy)
    dq = _get_quadrant(dx, dy)

    if sq == dq:
        return []

    key = frozenset([sq, dq])
    if key in QUAD_CONNECTIONS:
        cw_id = QUAD_CONNECTIONS[key]
        return [(cw_id, sq, dq)]

    route_key = (sq, dq)
    if route_key in TWO_STEP:
        return TWO_STEP[route_key]

    return []  # shouldn't happen


# ---------------------------------------------------------------------------
# Input parsing
# ---------------------------------------------------------------------------

def _parse_location(text: str) -> tuple[float, float] | None:
    """Parse a location string -> (x, y) or None."""
    t = text.strip().upper().replace(",", " ")
    if t in NAMED_LOCATIONS:
        return NAMED_LOCATIONS[t]
    parts = t.split()
    if len(parts) >= 2:
        try:
            return float(parts[0]), float(parts[1])
        except ValueError:
            pass
    return None


def _parse_routing(raw: str) -> tuple | None:
    """
    Parse "SRC to DST" or "SRC -> DST".
    Returns (src_label, sx, sy, dst_label, dx, dy) or None.
    """
    for sep in [" -> ", "->", " TO ", " to "]:
        lower = raw.lower()
        sep_lower = sep.lower().strip()
        # find separator case-insensitively
        idx = lower.find(sep_lower)
        if idx != -1:
            left  = raw[:idx].strip()
            right = raw[idx + len(sep_lower):].strip()
            src = _parse_location(left)
            dst = _parse_location(right)
            if src and dst:
                return left.upper(), src[0], src[1], right.upper(), dst[0], dst[1]
    return None


def _parse_single(raw: str) -> tuple[float, float, float] | None:
    """
    Parse a single location query -> (x, y, heading_deg) or None.
    """
    t = raw.strip().upper()
    # Crosswalk name
    _PRESETS = {
        "CW_NORTH": ( 5.5,  9.5, 180.0),
        "CW_EAST":  ( 9.5, -5.5,  90.0),
        "CW_SOUTH": (-5.5, -9.5,   0.0),
        "CW_WEST":  (-9.5,  5.5, 270.0),
    }
    if t in _PRESETS:
        return _PRESETS[t]
    if t in NAMED_LOCATIONS:
        x, y = NAMED_LOCATIONS[t]
        return x, y, 180.0
    parts = t.replace(",", " ").split()
    if len(parts) == 2:
        try:
            x, y = float(parts[0]), float(parts[1])
            # Guess heading from quadrant
            heading = {("NE"): 180.0, ("NW"): 0.0, ("SE"): 90.0, ("SW"): 270.0}.get(
                _get_quadrant(x, y), 180.0)
            return x, y, heading
        except ValueError:
            pass
    if len(parts) == 3:
        try:
            return float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            pass
    return None


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def _sep(char: str = "-", width: int = 65) -> None:
    print(char * width)


def _pf(flag: bool) -> str:
    return "[PASS]" if flag else "[FAIL]"


# ---------------------------------------------------------------------------
# Mode 1: Single location query
# ---------------------------------------------------------------------------

def _run_single_query(x: float, y: float, heading: float, label: str, now_ms: int) -> None:
    """Evaluate one point and print full predicate checklist + decision."""
    if _is_in_road(x, y):
        nn, nx, ny = _nearest_safe(x, y)
        print()
        print(f"  [REJECTED] ({x}, {y}) is inside the vehicle road zone.")
        print(f"  (0,0) is the centre of the intersection - not a valid pedestrian position.")
        print(f"  Nearest safe sidewalk: {nn} at ({nx}, {ny})")
        print(f"  Please use a sidewalk position (outside the road area).")
        print()
        return

    signals = two_phase_signals(now_ms, cycle_start_ms=0)
    loc     = make_localization(x, y, now_ms, heading_deg=heading,
                                position_uncertainty_m=0.75)
    result  = evaluate(signals, CMAP, loc, GOOD_HEALTH, now_ms, config=CFG)
    LOGGER.log(result.decision, result.evidence)

    ev = result.evidence
    d  = result.decision
    cw = ev.get("established_crosswalk_id") or "-"
    sig_state = ev.get("signal_state") or "-"
    rem = ev.get("remaining_time_ms")
    rem_str = f"{rem} ms ({rem/1000:.0f}s)" if rem is not None else "-"

    _sep("=")
    print(f"  SIGNAL QUERY : {label}")
    print(f"  Time  : now_ms={now_ms}  |  {_phase_label_str(now_ms)}")
    _sep()
    print(f"  Position         : ({x}, {y})  heading={heading} deg")
    print(f"  Established CW   : {cw}")
    print(f"  Signal state     : {sig_state}   remaining={rem_str}")
    _sep()
    print("  PREDICATE CHECKLIST")
    print(f"    {_pf(ev['Sv'])}  Sv  Signal WALK & fresh    state={sig_state}  age={ev.get('signal_age_ms')} ms")
    print(f"    {_pf(ev['Cv'])}  Cv  Crosswalk established  cw={cw}")
    print(f"    {_pf(ev['Lv'])}  Lv  Localization valid     uncertainty={ev.get('position_uncertainty_m')} m  age={ev.get('localization_age_ms')} ms")
    print(f"    {_pf(ev['Fv'])}  Fv  Information fresh      sig={ev.get('signal_age_ms')} ms  loc={ev.get('localization_age_ms')} ms")
    print(f"    {_pf(ev['Hv'])}  Hv  System healthy")
    print(f"    {_pf(ev['Tv'])}  Tv  Timing sufficient      remaining={rem_str}")
    _sep()

    banner = {
        "WALK":        "[WALK]        Safe to cross!",
        "WAIT":        "[WAIT]        Signal not WALK. Wait at the curb.",
        "UNAVAILABLE": "[UNAVAILABLE] Cannot determine safe crossing.",
        "FAULT":       "[FAULT]       System error detected.",
    }.get(d.decision, f"[{d.decision}]")

    print(f"\n  DECISION >> {banner}")
    print(f"  Reason   :  {d.reason_codes}")
    _sep("=")
    print()


# ---------------------------------------------------------------------------
# Mode 2: Source -> Destination routing
# ---------------------------------------------------------------------------

def _run_routing(
    src_label: str, sx: float, sy: float,
    dst_label: str, dx: float, dy: float,
    now_ms: int,
) -> None:
    """Full routing guidance from source to destination."""

    # Road zone rejection — invalid source or destination
    for label, x, y in [(src_label, sx, sy), (dst_label, dx, dy)]:
        if _is_in_road(x, y):
            nn, nx, ny = _nearest_safe(x, y)
            print()
            print(f"  [REJECTED] {label} ({x}, {y}) is inside the vehicle road zone.")
            print(f"  (0,0) is the centre of the intersection - not a valid pedestrian position.")
            print(f"  Nearest safe sidewalk: {nn} at ({nx}, {ny})")
            print(f"  Please use a sidewalk position (outside the road area).")
            print()
            return

    route = _find_route(sx, sy, dx, dy)
    sq    = _get_quadrant(sx, sy)
    dq    = _get_quadrant(dx, dy)

    _sep("=")
    print(f"  ROUTING GUIDANCE")
    print(f"  FROM : {src_label}  ({sx}, {sy})  [{sq} side]")
    print(f"  TO   : {dst_label}  ({dx}, {dy})  [{dq} side]")
    print(f"  TIME : now_ms={now_ms}  |  {_phase_label_str(now_ms)}")
    _sep()

    # ---- SAME SIDE: no crossing needed ----
    if not route:
        direction, dist = _compass(sx, sy, dx, dy)
        print(f"  No road crossing needed - both on the {sq} side.")
        print()
        if direction == "already here":
            print(f"  You are already at the destination!")
        else:
            print(f"  Walk {direction} for {dist:.1f} m to reach destination.")
        _sep("=")
        print()
        return

    # ---- CROSSING ROUTE ----
    num_crossings = len(route)
    cur_x, cur_y  = sx, sy

    if num_crossings == 1:
        print(f"  Route: 1 road crossing needed")
    else:
        print(f"  Route: {num_crossings} road crossing(s) needed")
        print(f"  [NOTE] Signal phase will change by the time you reach crossing 2.")
        print(f"         Showing guidance for the FIRST crossing only.")
        print(f"         Re-query after completing it for updated signal info.")
    print()

    # Only resolve the FIRST crossing with signal guidance
    cw_id, from_quad, to_quad = route[0]
    entry_x, entry_y = CROSSWALK_DATA[cw_id]["sides"][from_quad]
    exit_x,  exit_y  = CROSSWALK_DATA[cw_id]["sides"][to_quad]
    status, walk_rem, wait_ms = _phase_timing(cw_id, now_ms)
    phase_lbl = CROSSWALK_DATA[cw_id]["phase_label"]

    _sep("-")
    print(f"  STEP 1  |  Cross {cw_id}")
    _sep("-")

    # a) Walk to entry curb
    dir_to_entry, dist_to_entry = _compass(cur_x, cur_y, entry_x, entry_y)
    print(f"  a) Walk to {cw_id} curb")
    if dist_to_entry > 0.3:
        print(f"       Direction : {dir_to_entry}")
        print(f"       Distance  : {dist_to_entry:.1f} m")
        print(f"       Curb at   : ({entry_x}, {entry_y})  [{from_quad} side]")
    else:
        print(f"       You are already at the curb ({entry_x}, {entry_y})")
    print()

    # b) Wait / Walk decision
    print(f"  b) Signal status for {cw_id}")
    print(f"       Walks during  : {phase_lbl}")
    if status == "WALK":
        print(f"       Status        : WALK")
        print(f"       >> GO NOW!  {walk_rem/1000:.0f} s remaining in this phase")
        if walk_rem < 10_000:
            print(f"       [!] Only {walk_rem/1000:.0f}s left - start crossing immediately!")
    elif status == "CLEARANCE":
        print(f"       Status        : CLEARANCE  (peds finishing crossing - do not start)")
        print(f"       >> WAIT at curb.  Next WALK in {wait_ms/1000:.0f} s")
    else:
        m, s = divmod(wait_ms // 1000, 60)
        print(f"       Status        : DONT WALK")
        if m > 0:
            print(f"       >> WAIT at curb.  Next WALK in {m} min {s} s  ({wait_ms/1000:.0f} s total)")
        else:
            print(f"       >> WAIT at curb.  Next WALK in {wait_ms/1000:.0f} s")
    print()

    # c) Cross direction
    dir_across, dist_across = _compass(entry_x, entry_y, exit_x, exit_y)
    print(f"  c) Cross the road")
    print(f"       Direction  : {dir_across}")
    print(f"       Distance   : {dist_across:.1f} m")
    print(f"       Arrive at  : ({exit_x}, {exit_y})  [{to_quad} side]")
    print()

    # ---- NEXT CROSSING PREVIEW (if multi-crossing route) ----
    if num_crossings > 1:
        _sep("-")
        print(f"  NEXT CROSSING (after step 1)")
        _sep("-")
        for i, (next_cw, next_from, next_to) in enumerate(route[1:], start=2):
            print(f"  Step {i}: Cross {next_cw}  ({next_from} -> {next_to})")
        print(f"  Then walk to: {dst_label}  ({dx}, {dy})")
        print()
        print(f"  >> Re-query after crossing {cw_id} for live signal status.")
        print()
    else:
        # Single crossing — show final walk to destination
        cur_x, cur_y = exit_x, exit_y
        _sep("-")
        print(f"  STEP 2  |  Walk to destination")
        _sep("-")
        dir_final, dist_final = _compass(cur_x, cur_y, dx, dy)
        if dir_final == "already here":
            print(f"  You have arrived at {dst_label} ({dx}, {dy})!")
        else:
            print(f"  From  : ({cur_x}, {cur_y})")
            print(f"  Walk  : {dir_final} for {dist_final:.1f} m")
            print(f"  To    : {dst_label}  ({dx}, {dy})")
        print()

    _sep("=")
    print()


# ---------------------------------------------------------------------------
# Intersection ASCII map
# ---------------------------------------------------------------------------

def _print_map(now_ms: int) -> None:
    t = now_ms % TWO_PHASE_CYCLE_MS
    a_walk = "WALK " if t < 45_000 else "     "
    b_walk = "WALK " if 50_000 <= t < 95_000 else "     "

    print()
    print("  Intersection Map  -  INT_KEMPS_CORNER  (Kemps Corner, Mumbai)")
    print("  Origin (0,0) = intersection centre. +x=East +y=North. Units=metres.")
    print()
    print(f"  Phase A crosswalks (CW_NORTH, CW_SOUTH) : {a_walk}  (t=0-44999 ms)")
    print(f"  Phase B crosswalks (CW_EAST,  CW_WEST)  : {b_walk}  (t=50000-94999 ms)")
    print()
    print("         -12   -9.5  -5.5   0    5.5   9.5   12     (x)")
    print("    12  |  NW corner          NE corner             |")
    print("        |                                           |")
    print("    9.5 |  CW_NORTH_FAR      CW_NORTH(entry)       |  <- Phase A WALK")
    print("        |  (-5.5,9.5)        (5.5,9.5)             |")
    print("    8   +----[CW_NORTH  crosswalk]------------------+")
    print("        |          road  (vehicle area)             |")
    print("   5.5  | CW_WEST(entry)                CW_EAST_FAR|  <- Phase B WALK")
    print("        | (-9.5,5.5)                   (9.5,5.5)   |")
    print("        |  8   +--------intersection--------+ 8     |")
    print("        |      |      (0,0)                |        |")
    print("   -5.5 | CW_WEST_FAR               CW_EAST(entry) |  <- Phase B WALK")
    print("        | (-9.5,-5.5)                (9.5,-5.5)    |")
    print("   -8   +----[CW_SOUTH crosswalk]------------------+")
    print("        |                                           |")
    print("   -9.5 |  CW_SOUTH(entry)   CW_SOUTH_FAR         |  <- Phase A WALK")
    print("        |  (-5.5,-9.5)       (5.5,-9.5)            |")
    print("   -12  |  SW corner          SE corner             |")
    print()
    print("  Valid sidewalk positions (source/destination examples):")
    print("    CW_NORTH  ( 5.5,  9.5)   CW_EAST   ( 9.5, -5.5)")
    print("    CW_SOUTH  (-5.5, -9.5)   CW_WEST   (-9.5,  5.5)")
    print("    NE        ( 9.5,  9.5)   NW        (-9.5,  9.5)")
    print("    SE        ( 9.5, -9.5)   SW        (-9.5, -9.5)")
    print()


# ---------------------------------------------------------------------------
# Main interactive loop
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="TRIVENI pedestrian crossing guidance tool"
    )
    parser.add_argument(
        "--time", type=int, default=10_000,
        help="Starting simulated time in ms (default: 10000 = Phase A active)"
    )
    args = parser.parse_args()
    now_ms = args.time

    print()
    print("  ================================================")
    print("  TRIVENI Pedestrian Crossing Guidance Tool")
    print("  Two-phase India-style signal model")
    print("  ================================================")
    print()
    print(f"  >> now_ms={now_ms}  |  {_phase_label_str(now_ms)}")
    print()
    print("  SINGLE QUERY  : enter crosswalk name or coordinates")
    print("    CW_NORTH  |  CW_EAST  |  CW_SOUTH  |  CW_WEST")
    print("    5.5 9.5   |  5.5 9.5 180")
    print()
    print("  ROUTING QUERY : enter source -> destination")
    print("    CW_NORTH to CW_NORTH_FAR")
    print("    5.5 9.5 to -5.5 9.5")
    print("    NE to SW         (diagonal - 2 crossings)")
    print("    CW_NORTH to SW")
    print()
    print("  COMMANDS : time <ms>  next  phase  map  quit")
    print()

    while True:
        try:
            raw = input(f"  [now_ms={now_ms}] Query: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  Bye!")
            break

        if not raw:
            continue
        cmd = raw.lower().strip()

        # --- built-in commands ---
        if cmd in ("quit", "q", "exit"):
            print("  Bye!")
            break

        if cmd == "phase":
            print(f"\n  Current: {_phase_label_str(now_ms)}\n")
            continue

        if cmd == "map":
            _print_map(now_ms)
            continue

        if cmd == "next":
            now_ms += 5_000
            print(f"\n  >> Advanced to now_ms={now_ms}  |  {_phase_label_str(now_ms)}\n")
            continue

        if cmd.startswith("time "):
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    now_ms = int(parts[1])
                    print(f"\n  Jumped to now_ms={now_ms}  |  {_phase_label_str(now_ms)}\n")
                except ValueError:
                    print("  Usage: time <ms>  e.g.  time 60000\n")
            else:
                print("  Usage: time <ms>  e.g.  time 60000\n")
            continue

        # --- routing query: "X to Y" or "X -> Y" ---
        routing = _parse_routing(raw)
        if routing:
            src_label, sx, sy, dst_label, dx, dy = routing
            _run_routing(src_label, sx, sy, dst_label, dx, dy, now_ms)
            continue

        # --- single location query ---
        single = _parse_single(raw)
        if single:
            x, y, heading = single
            label = raw.upper() if raw.upper() in NAMED_LOCATIONS or raw.upper() in {
                "CW_NORTH", "CW_EAST", "CW_SOUTH", "CW_WEST"} else f"({x}, {y})"
            _run_single_query(x, y, heading, label, now_ms)
            continue

        print(
            "  Could not parse input.\n"
            "  Try: CW_NORTH  |  5.5 9.5  |  CW_NORTH to CW_NORTH_FAR  |  NE to SW\n"
        )


if __name__ == "__main__":
    main()
