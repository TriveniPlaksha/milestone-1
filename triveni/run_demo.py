# -*- coding: utf-8 -*-
"""run_demo.py - TRIVENI Core Guidance & Safety Emulator - Live Demos A-E.

Usage:
    python run_demo.py              # all demos in sequence
    python run_demo.py --pause      # pause for Enter between demos (for screen recording)
    python run_demo.py --slow       # add 0.2 s display delay per tick in Demo E

Signal model: two-phase India-style (no separate pedestrian-only phase).
  Phase A  E-W vehicles GREEN  ->  CW_NORTH & CW_SOUTH: WALK
                                   (peds cross the N-S road, moving East<->West)
  Phase B  N-S vehicles GREEN  ->  CW_EAST  & CW_WEST:  WALK
                                   (peds cross the E-W road, moving North<->South)

Cycle = 100 000 ms  (45 s Phase A + 5 s Clearance + 45 s Phase B + 5 s Clearance).

At the end the path to logs/decision_log.jsonl is printed.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Ensure package is importable when run from project root
_PROJECT_ROOT = Path(__file__).parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import yaml

from triveni.models.crosswalk_map import CrosswalkMap
from triveni.models.system_health import SystemHealth
from triveni.safety.safety_engine import evaluate, EvaluationResult
from triveni.logging.event_logger import EventLogger
from triveni.simulator.signal_simulator import (
    make_stale_signal,
    two_phase_signals,
    TWO_PHASE_CYCLE_MS,
    TWO_PHASE_EW_GREEN_MS,
)
from triveni.simulator.pedestrian_simulator import (
    make_localization,
    preset_near_correct_crosswalk,
    preset_between_two_crosswalks,
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_ROOT = _PROJECT_ROOT
_CONFIG_PATH = _ROOT / "config" / "triveni.yaml"
_MAP_PATH = _ROOT / "data" / "intersection_demo.json"
_LOG_PATH = _ROOT / "logs" / "decision_log.jsonl"

# ---------------------------------------------------------------------------
# Load config and map once at startup
# ---------------------------------------------------------------------------
with open(_CONFIG_PATH, "r", encoding="utf-8") as _f:
    CFG = yaml.safe_load(_f)

CMAP = CrosswalkMap.from_json(_MAP_PATH)

GOOD_HEALTH = SystemHealth(
    edge_service="OK", signal_input="OK", localization_input="OK",
    map_loaded=True, clock_status="OK",
)

LOGGER = EventLogger(_LOG_PATH)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pass_fail(flag: bool) -> str:
    return "[PASS]" if flag else "[FAIL]"


def _sep(char: str = "-", width: int = 68) -> None:
    print(char * width)


def _pause(do_pause: bool) -> None:
    if do_pause:
        input("\n  [Press Enter to continue...]\n")


def _narrate(title: str, story: str, inputs: dict, result: EvaluationResult) -> None:
    """Print the narrated demo output for one scenario."""
    ev = result.evidence
    _sep("=")
    print("  " + title)
    _sep("-")
    print("  " + story)
    _sep("-")
    print("  INPUTS")
    for k, v in inputs.items():
        print("    {}: {}".format(k, v))
    _sep("-")
    print("  PREDICATE CHECKLIST")
    print("    {} Sv  Signal valid and WALK       state={}  age={} ms".format(
        _pass_fail(ev["Sv"]), ev.get("signal_state"), ev.get("signal_age_ms")))
    print("    {} Cv  Crosswalk established       cw={}".format(
        _pass_fail(ev["Cv"]), ev.get("established_crosswalk_id")))
    print("    {} Lv  Localization acceptable     uncertainty={} m  age={} ms".format(
        _pass_fail(ev["Lv"]), ev.get("position_uncertainty_m"), ev.get("localization_age_ms")))
    print("    {} Fv  Information fresh           sig_age={} ms  loc_age={} ms".format(
        _pass_fail(ev["Fv"]), ev.get("signal_age_ms"), ev.get("localization_age_ms")))
    print("    {} Hv  System healthy              signal_src={}  loc={}".format(
        _pass_fail(ev["Hv"]), ev.get("signal_source_health"), ev.get("localization_health")))
    print("    {} Tv  Timing satisfied            remaining={} ms".format(
        _pass_fail(ev["Tv"]), ev.get("remaining_time_ms")))
    _sep("-")
    d = result.decision
    print("  FINAL DECISION: {}  |  reason_codes: {}".format(d.decision, d.reason_codes))
    _sep("=")
    print()


# ---------------------------------------------------------------------------
# Demo A: Valid crossing
# Phase A active (E-W vehicles GREEN); CW_NORTH & CW_SOUTH are WALK.
# Pedestrian at CW_NORTH entry crosses parallel to E-W traffic.
# ---------------------------------------------------------------------------

def demo_a(pause: bool) -> None:
    # cycle position t=10000 -> Phase A; CW_NORTH remaining = 45000-10000 = 35000 ms
    now_ms = 10_000
    signals = two_phase_signals(now_ms, cycle_start_ms=0)
    loc = preset_near_correct_crosswalk(now_ms)
    result = evaluate(signals, CMAP, loc, GOOD_HEALTH, now_ms, config=CFG)
    LOGGER.log(result.decision, result.evidence)
    ev = result.evidence
    _narrate(
        title="DEMO A - Valid Crossing  [Phase A: E-W vehicles GREEN, CW_NORTH WALK]",
        story=(
            "E-W vehicles are green (Phase A). Pedestrian at CW_NORTH curb (5.5, 9.5)\n"
            "  crosses the N-S road — parallel to the E-W vehicle green arm.\n"
            "  Signal WALK, 35 s remaining. Data fresh, system healthy -> TRIVENI says WALK."
        ),
        inputs={
            "Signal phase":    "A — E-W vehicles GREEN  (cycle t=10 000 ms)",
            "CW_NORTH signal": "WALK  age={} ms  remaining={} ms".format(
                ev.get("signal_age_ms"), ev.get("remaining_time_ms")),
            "CW_EAST signal":  "DONT_WALK  (N-S crossing waits for Phase B)",
            "Pedestrian":      "(5.5, 9.5)  uncertainty=0.75 m  zone=CW_NORTH entry",
            "System health":   "ALL OK",
        },
        result=result,
    )
    _pause(pause)


# ---------------------------------------------------------------------------
# Demo B: Ambiguous crosswalk
# Phase A active; pedestrian at the NE corner shared by two zone polygons.
# ---------------------------------------------------------------------------

def demo_b(pause: bool) -> None:
    now_ms = 11_000
    signals = two_phase_signals(now_ms, cycle_start_ms=0)
    loc = preset_between_two_crosswalks(now_ms)   # (8.0, 8.0)
    result = evaluate(signals, CMAP, loc, GOOD_HEALTH, now_ms, config=CFG)
    LOGGER.log(result.decision, result.evidence)
    ev = result.evidence
    _narrate(
        title="DEMO B - Ambiguous Crosswalk  [Phase A: CW_NORTH WALK, ped at corner]",
        story=(
            "Phase A active (CW_NORTH WALK). Pedestrian stands at (8.0, 8.0) —\n"
            "  NE corner shared by CW_NORTH entry zone and CW_EAST exit zone.\n"
            "  TRIVENI cannot determine intended crosswalk -> CROSSWALK_AMBIGUOUS -> UNAVAILABLE."
        ),
        inputs={
            "Signal phase":    "A — E-W vehicles GREEN  (cycle t=11 000 ms)",
            "CW_NORTH signal": "WALK  age={} ms".format(ev.get("signal_age_ms")),
            "Pedestrian":      "(8.0, 8.0)  uncertainty=0.75 m  zone=AMBIGUOUS",
            "System health":   "ALL OK",
        },
        result=result,
    )
    _pause(pause)


# ---------------------------------------------------------------------------
# Demo C: Stale signal
# Phase A active; CW_NORTH signal data is 900 ms old (above 500 ms threshold).
# ---------------------------------------------------------------------------

def demo_c(pause: bool) -> None:
    now_ms = 12_000
    # Build two-phase signals, then override CW_NORTH with a stale reading
    signals = dict(two_phase_signals(now_ms, cycle_start_ms=0))
    signals["CW_NORTH"] = make_stale_signal(
        "CW_NORTH", "WALK", now_ms, stale_age_ms=900, remaining_time_ms=33_000,
    )
    loc = preset_near_correct_crosswalk(now_ms)
    result = evaluate(signals, CMAP, loc, GOOD_HEALTH, now_ms, config=CFG)
    LOGGER.log(result.decision, result.evidence)
    ev = result.evidence
    _narrate(
        title="DEMO C - Stale Signal  [Phase A: CW_NORTH data 900 ms old]",
        story=(
            "Phase A active but CW_NORTH signal data is 900 ms old — above the\n"
            "  500 ms freshness threshold. TRIVENI rejects stale data even when\n"
            "  signal says WALK -> SIGNAL_STALE -> UNAVAILABLE."
        ),
        inputs={
            "Signal phase":    "A — E-W vehicles GREEN  (cycle t=12 000 ms)",
            "CW_NORTH signal": "WALK  age={} ms  (threshold=500 ms)".format(
                ev.get("signal_age_ms")),
            "Pedestrian":      "(5.5, 9.5)  uncertainty=0.75 m",
            "System health":   "ALL OK",
        },
        result=result,
    )
    _pause(pause)


# ---------------------------------------------------------------------------
# Demo D: DON'T WALK
# Phase B active (N-S vehicles GREEN); CW_NORTH shows DONT_WALK.
# Pedestrian at CW_NORTH entry must wait for the next Phase A window.
# ---------------------------------------------------------------------------

def demo_d(pause: bool) -> None:
    # cycle position t=60000 -> Phase B; CW_NORTH DONT_WALK, CW_EAST & CW_WEST WALK
    now_ms = 60_000
    signals = two_phase_signals(now_ms, cycle_start_ms=0)
    loc = preset_near_correct_crosswalk(now_ms)
    result = evaluate(signals, CMAP, loc, GOOD_HEALTH, now_ms, config=CFG)
    LOGGER.log(result.decision, result.evidence)
    ev = result.evidence
    _narrate(
        title="DEMO D - DON'T WALK  [Phase B: N-S vehicles GREEN, CW_NORTH DONT_WALK]",
        story=(
            "N-S vehicles are now green (Phase B). CW_EAST & CW_WEST are WALK;\n"
            "  CW_NORTH is DONT_WALK. Pedestrian at CW_NORTH entry must wait for\n"
            "  Phase A. Signal predicate Sv fails -> SIGNAL_NOT_WALK -> WAIT."
        ),
        inputs={
            "Signal phase":    "B — N-S vehicles GREEN  (cycle t=60 000 ms)",
            "CW_NORTH signal": "DONT_WALK  age={} ms".format(ev.get("signal_age_ms")),
            "CW_EAST signal":  "WALK  (E-W crossing open in Phase B)",
            "Pedestrian":      "(5.5, 9.5)  uncertainty=0.75 m  zone=CW_NORTH entry",
            "System health":   "ALL OK",
        },
        result=result,
    )
    _pause(pause)


# ---------------------------------------------------------------------------
# Demo E: Journey Replay – two-phase cycle, Phase B -> Clearance -> Phase A
#
# now_ms: 90 000 to 120 000 (1 000 ms steps, 31 ticks)
# cycle_start_ms = 0
#
# Timeline (cycle_t = now_ms % TWO_PHASE_CYCLE_MS):
#   90 000 - 94 999  Phase B  (N-S green)   CW_NORTH: DONT_WALK  -> WAIT
#   95 000 - 99 999  Clearance B            CW_NORTH: DONT_WALK  -> WAIT
#  100 000 - 102 999 Phase A starts         CW_NORTH: WALK       -> WALK (ped at curb)
#  103 000 - 107 000 Phase A, ped walking   CW_NORTH: WALK       -> WALK (in entry zone)
#  108 000 - 120 000 Ped mid-crossing       CW_NORTH: WALK       -> UNAVAILABLE
#
# Pedestrian script:
#   Stays at (5.5, 9.5) until t=102 999, then walks west at 0.8 m/s:
#     x = 5.5 - 0.8 * (now_ms - 103 000) / 1000
#     y = 9.5, heading=180, uncertainty=0.75 m
#
# NOTE: mid-crossing guidance is NOT defined in Milestone 01.
# Once ped leaves the CW_NORTH entry zone (x < 2.25 approx.) the engine
# outputs UNAVAILABLE / CROSSWALK_NOT_ESTABLISHED — expected behaviour.
# ---------------------------------------------------------------------------

def demo_e(pause: bool, slow: bool = False) -> None:
    """Journey replay: two-phase cycle crossing the Phase B -> Phase A boundary."""
    _sep("=")
    print("  DEMO E - JOURNEY REPLAY  (two-phase cycle, 90 000 ms -> 120 000 ms)")
    print("  Phase B (N-S green) -> Clearance B -> Phase A (E-W green) -> Ped walks")
    _sep("=")

    START_MS        = 90_000
    END_MS          = 120_000
    PED_MOVE_START  = 103_000   # ped starts walking 3 s after Phase A begins
    PED_Y           = 9.5
    PED_SPEED_MPS   = 0.8       # m/s westward (decreasing x)

    def _ped_pos(now_ms: int) -> tuple:
        if now_ms < PED_MOVE_START:
            return 5.5, PED_Y
        elapsed_s = (now_ms - PED_MOVE_START) / 1000.0
        return 5.5 - PED_SPEED_MPS * elapsed_s, PED_Y

    def _stage(now_ms: int) -> str:
        ct = now_ms % TWO_PHASE_CYCLE_MS
        if ct >= 95_000:
            return "Clearance B — CW_NORTH DONT_WALK, ped waiting at curb"
        if ct >= 50_000:
            return "Phase B — N-S vehicles GREEN, CW_NORTH DONT_WALK, ped waiting"
        if now_ms < PED_MOVE_START:
            return "Phase A — E-W vehicles GREEN, CW_NORTH WALK, ped at curb"
        return "Phase A — CW_NORTH WALK, ped walking west (UNAVAILABLE once off entry zone)"

    prev_stage = ""
    tick_count = 0
    decision_counts: dict = {}
    false_walks = 0

    for now_ms in range(START_MS, END_MS + 1, 1_000):
        sigs = two_phase_signals(now_ms, cycle_start_ms=0)
        x, y = _ped_pos(now_ms)
        loc = make_localization(x, y, now_ms, age_ms=30, heading_deg=180.0)
        result = evaluate(sigs, CMAP, loc, GOOD_HEALTH, now_ms, config=CFG)
        LOGGER.log(result.decision, result.evidence)

        d = result.decision
        ev = result.evidence
        decision_counts[d.decision] = decision_counts.get(d.decision, 0) + 1
        tick_count += 1

        if d.decision == "WALK":
            if not (ev["Sv"] and ev["Cv"] and ev["Lv"] and ev["Fv"] and ev["Hv"] and ev["Tv"]):
                false_walks += 1

        stage = _stage(now_ms)
        if stage != prev_stage:
            _sep("-")
            print("  >>  " + stage)
            _sep("-")
            prev_stage = stage

        cycle_t = now_ms % TWO_PHASE_CYCLE_MS
        print(
            "  t={:7d}  ct={:6d}  x={:6.2f}  sig={:<10}  rem={:<7}  -> {:<12}  {}".format(
                now_ms, cycle_t, x,
                sigs["CW_NORTH"].signal_state,
                str(sigs["CW_NORTH"].remaining_time_ms),
                d.decision, d.reason_codes[0],
            )
        )

        if slow:
            time.sleep(0.2)

    _sep("=")
    print("  JOURNEY REPLAY SUMMARY")
    _sep("-")
    print("  Total ticks : {}".format(tick_count))
    for dec, cnt in sorted(decision_counts.items()):
        print("  {:<12}: {}".format(dec, cnt))
    print("  False WALK events: {}  <- must be 0".format(false_walks))
    _sep("=")
    print()
    if pause:
        _pause(pause)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="TRIVENI live demo")
    parser.add_argument("--pause", action="store_true",
                        help="Wait for Enter between demos (for screen recording).")
    parser.add_argument("--slow", action="store_true",
                        help="Add 0.2 s display delay per tick in Demo E (for screen recording).")
    args = parser.parse_args()

    print()
    print("  TRIVENI Core Guidance & Safety Emulator v0.1 - Milestone 01")
    print("  Signal model: two-phase India-style (peds parallel to green vehicle arm)")
    print("  Demos A-E")
    print()

    demo_a(args.pause)
    demo_b(args.pause)
    demo_c(args.pause)
    demo_d(args.pause)
    # demo_e(args.pause, slow=args.slow)  # Demo E commented out for now

    print("  Log written to: {}".format(_LOG_PATH.resolve()))
    print()


if __name__ == "__main__":
    main()
