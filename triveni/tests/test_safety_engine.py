"""tests/test_safety_engine.py – TRIVENI automated test suite.

Covers all 10 mandatory scenarios (spec §10).  Every test has a one-line
plain-English docstring: situation → expected decision and why.

Signal model: two-phase India-style (no separate pedestrian-only phase).
  Phase A  E-W vehicles GREEN  →  CW_NORTH & CW_SOUTH: WALK
  Phase B  N-S vehicles GREEN  →  CW_EAST  & CW_WEST:  WALK
  Cycle = 100 000 ms  (45 s + 5 s clearance + 45 s + 5 s clearance)
"""
from __future__ import annotations

from typing import Any, Dict

import pytest

from triveni.models.crosswalk_map import CrosswalkMap
from triveni.models.guidance_decision import GuidanceDecision
from triveni.models.signal_state import SignalState
from triveni.models.system_health import SystemHealth
from triveni.safety.safety_engine import DEFAULT_CONFIG, evaluate
from triveni.simulator.pedestrian_simulator import (
    preset_between_two_crosswalks,
    preset_high_uncertainty,
    preset_near_correct_crosswalk,
    preset_wrong_intersection_id,
)
from triveni.simulator.signal_simulator import (
    make_all_signals,
    make_stale_signal,
)


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

NOW_MS = 100_000
MAP_ID = "INT_KEMPS_CORNER"

_MAP_DATA = {
    "intersection_id": MAP_ID,
    "crosswalks": [
        {"crosswalk_id": "CW_NORTH", "entry_zone": [[3,8],[8,8],[8,11],[3,11]],
         "exit_zone": [[-8,8],[-3,8],[-3,11],[-8,11]], "heading_deg": 180},
        {"crosswalk_id": "CW_EAST",  "entry_zone": [[8,-8],[11,-8],[11,-3],[8,-3]],
         "exit_zone": [[8,3],[11,3],[11,8],[8,8]], "heading_deg": 90},
        {"crosswalk_id": "CW_SOUTH", "entry_zone": [[-8,-11],[-3,-11],[-3,-8],[-8,-8]],
         "exit_zone": [[3,-11],[8,-11],[8,-8],[3,-8]], "heading_deg": 0},
        {"crosswalk_id": "CW_WEST",  "entry_zone": [[-11,3],[-8,3],[-8,8],[-11,8]],
         "exit_zone": [[-11,-8],[-8,-8],[-8,-3],[-11,-3]], "heading_deg": 270},
    ],
}

@pytest.fixture()
def cmap() -> CrosswalkMap:
    return CrosswalkMap.from_dict(_MAP_DATA)

@pytest.fixture()
def good_health() -> SystemHealth:
    return SystemHealth(
        edge_service="OK", signal_input="OK", localization_input="OK",
        map_loaded=True, clock_status="OK",
    )

@pytest.fixture()
def cfg() -> Dict[str, Any]:
    return dict(DEFAULT_CONFIG)


def _walk_signals(cw_id: str, now_ms: int = NOW_MS) -> Dict[str, SignalState]:
    """All signals DONT_WALK except cw_id which is WALK (18 s remaining)."""
    return make_all_signals(
        now_ms,
        states={cw_id: "WALK"},
        remaining_times={cw_id: 18_000},
    )


# ---------------------------------------------------------------------------
# Mandatory scenario 1 – WALK: all predicates valid
# ---------------------------------------------------------------------------

def test_s1_valid_walk(cmap, good_health, cfg):
    """WALK + correct crosswalk + good localization + fresh data → WALK (all mandatory predicates valid)."""
    signals = _walk_signals("CW_NORTH")
    loc = preset_near_correct_crosswalk(NOW_MS)
    result = evaluate(signals, cmap, loc, good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "WALK"
    assert "ALL_REQUIRED_CONDITIONS_VALID" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 2 – DONT_WALK signal → WAIT
# ---------------------------------------------------------------------------

def test_s2_dont_walk_signal(cmap, good_health, cfg):
    """DONT_WALK signal → WAIT (signal does not permit crossing)."""
    signals = make_all_signals(NOW_MS, states={"CW_NORTH": "DONT_WALK"})
    loc = preset_near_correct_crosswalk(NOW_MS)
    result = evaluate(signals, cmap, loc, good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "WAIT"
    assert "SIGNAL_NOT_WALK" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 3 – Signal missing → UNAVAILABLE
# ---------------------------------------------------------------------------

def test_s3_signal_missing(cmap, good_health, cfg):
    """Signal missing for established crosswalk → UNAVAILABLE (no trustworthy signal)."""
    # Pass empty signal dict
    result = evaluate({}, cmap, preset_near_correct_crosswalk(NOW_MS), good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "UNAVAILABLE"
    assert "SIGNAL_MISSING" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 4 – Signal stale → UNAVAILABLE
# ---------------------------------------------------------------------------

def test_s4_signal_stale(cmap, good_health, cfg):
    """Signal age > freshness threshold → UNAVAILABLE (freshness violated)."""
    stale = make_stale_signal("CW_NORTH", "WALK", NOW_MS, stale_age_ms=900,
                              remaining_time_ms=18_000)
    signals = {"CW_NORTH": stale}
    loc = preset_near_correct_crosswalk(NOW_MS)
    result = evaluate(signals, cmap, loc, good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "UNAVAILABLE"
    assert "SIGNAL_STALE" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 5 – Localization missing → UNAVAILABLE
# ---------------------------------------------------------------------------

def test_s5_localization_missing(cmap, good_health, cfg):
    """Localization is None → UNAVAILABLE (position cannot be established)."""
    signals = _walk_signals("CW_NORTH")
    result = evaluate(signals, cmap, None, good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "UNAVAILABLE"
    assert "LOCALIZATION_MISSING" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 6 – Localization uncertainty too high → UNAVAILABLE
# ---------------------------------------------------------------------------

def test_s6_localization_uncertainty_too_high(cmap, good_health, cfg):
    """Uncertainty > max_position_uncertainty_m → UNAVAILABLE (confidence insufficient)."""
    signals = _walk_signals("CW_NORTH")
    loc = preset_high_uncertainty(NOW_MS)   # 2.0 m > 1.5 threshold
    result = evaluate(signals, cmap, loc, good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "UNAVAILABLE"
    assert "LOCALIZATION_UNCERTAINTY_TOO_HIGH" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 7 – Wrong intersection ID → FAULT or UNAVAILABLE
# ---------------------------------------------------------------------------

def test_s7_wrong_intersection(cmap, good_health, cfg):
    """Wrong intersection ID in localization → FAULT or UNAVAILABLE (wrong intersection)."""
    signals = _walk_signals("CW_NORTH")
    loc = preset_wrong_intersection_id(NOW_MS)
    result = evaluate(signals, cmap, loc, good_health, NOW_MS, config=cfg)
    assert result.decision.decision in ("FAULT", "UNAVAILABLE")
    assert "WRONG_INTERSECTION" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 8 – Crosswalk ambiguous → UNAVAILABLE
# ---------------------------------------------------------------------------

def test_s8_crosswalk_ambiguous(cmap, good_health, cfg):
    """Pedestrian between two crosswalks → UNAVAILABLE (intended crossing not established)."""
    signals = _walk_signals("CW_NORTH")
    loc = preset_between_two_crosswalks(NOW_MS)   # (8.0, 8.0)
    result = evaluate(signals, cmap, loc, good_health, NOW_MS, config=cfg)
    assert result.decision.decision == "UNAVAILABLE"
    assert "CROSSWALK_AMBIGUOUS" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 9 – Critical component unhealthy → FAULT
# ---------------------------------------------------------------------------

def test_s9_system_unhealthy(cmap, cfg):
    """FAULT in SystemHealth → FAULT (system health invalid)."""
    bad_health = SystemHealth(
        edge_service="FAULT", signal_input="OK", localization_input="OK",
        map_loaded=True, clock_status="OK",
    )
    signals = _walk_signals("CW_NORTH")
    loc = preset_near_correct_crosswalk(NOW_MS)
    result = evaluate(signals, cmap, loc, bad_health, NOW_MS, config=cfg)
    assert result.decision.decision == "FAULT"
    assert "SYSTEM_UNHEALTHY" in result.decision.reason_codes


# ---------------------------------------------------------------------------
# Mandatory scenario 10 – WALK changes to DONT_WALK → leaves WALK immediately
# ---------------------------------------------------------------------------

def test_s10_walk_then_dont_walk(cmap, good_health, cfg):
    """Two consecutive ticks: tick-1 WALK → WALK; tick-2 DONT_WALK → WAIT immediately."""
    loc = preset_near_correct_crosswalk(NOW_MS)

    # Tick 1 – signal is WALK
    sigs1 = _walk_signals("CW_NORTH", now_ms=NOW_MS)
    r1 = evaluate(sigs1, cmap, loc, good_health, NOW_MS, config=cfg)
    assert r1.decision.decision == "WALK"

    # Tick 2 – signal flips to DONT_WALK (no engine memory / hysteresis)
    now2 = NOW_MS + 1000
    sigs2 = make_all_signals(now2, states={"CW_NORTH": "DONT_WALK"})
    loc2 = preset_near_correct_crosswalk(now2)
    r2 = evaluate(sigs2, cmap, loc2, good_health, now2, config=cfg)
    assert r2.decision.decision == "WAIT"
    assert "SIGNAL_NOT_WALK" in r2.decision.reason_codes
