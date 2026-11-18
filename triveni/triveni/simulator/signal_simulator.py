"""signal_simulator.py - Synthetic traffic-signal simulator.

SPaT (Signal Phase and Timing) Primer
--------------------------------------
SPaT describes *which phase* the pedestrian signal is in right now and
*how long* until it changes.  TRIVENI maps SPaT data to its internal model:

    Physical display                 signal_state field
    -------------------------------- ------------------
    Pedestrian green man (WALK)   -> WALK
    Flashing hand (clearance)     -> CLEARANCE
    Red man (DON'T WALK)          -> DONT_WALK
    Time left in current phase    -> remaining_time_ms
    Time controller produced data -> source_timestamp_ms
    Time TRIVENI received it      -> receive_timestamp_ms

Signal age = now_ms - source_timestamp_ms.
A signal is stale if its age > signal_freshness_ms (config threshold).

Default timestamps used by all builders:
    source_timestamp_ms  = now_ms - 50   (signal was produced 50 ms ago)
    receive_timestamp_ms = now_ms - 10   (TRIVENI received it 10 ms ago)

Only the **pedestrian signal** of each crosswalk is used.
Nothing here is connected to a real signal controller.
WALK alone does NOT mean "cross" - all six safety predicates must hold.

Two-Phase Signal Cycle (two_phase_signals) — India Style
---------------------------------------------------------
No separate pedestrian-only phase.  Pedestrians walk PARALLEL to (and
simultaneously with) the green vehicle arm.  This matches many Indian
urban intersections where left turns use a free-left slip road.

Phase A  E-W vehicles GREEN  →  CW_NORTH & CW_SOUTH: WALK
                                 (peds cross the N-S road, moving East↔West)
Phase B  N-S vehicles GREEN  →  CW_EAST  & CW_WEST:  WALK
                                 (peds cross the E-W road, moving North↔South)

TWO_PHASE_CYCLE_MS = TWO_PHASE_EW_GREEN_MS + TWO_PHASE_CLEARANCE_A_MS
                   + TWO_PHASE_NS_GREEN_MS + TWO_PHASE_CLEARANCE_B_MS
                   = 100 000 ms

    t in cycle       Phase          CW_NORTH  CW_SOUTH  CW_EAST  CW_WEST
    ---------------- -------------- --------  --------  -------  -------
    0     - 44 999   Phase A E-W ↔  WALK      WALK      DONT_W   DONT_W
    45 000- 49 999   Clearance A    CLEARANCE CLEARANCE DONT_W   DONT_W
    50 000- 94 999   Phase B N-S ↕  DONT_W    DONT_W    WALK     WALK
    95 000- 99 999   Clearance B    DONT_W    DONT_W    CLEARANC CLEARANC

remaining_time_ms:
    WALK      -> ms left in the current WALK phase for this crosswalk
    CLEARANCE -> ms left in the CLEARANCE phase for this crosswalk
    DONT_WALK -> ms until the next WALK phase begins for this crosswalk

All simulators are deterministic: no wall-clock, no time.time(), no sleep,
no threads.  Everything is computed purely from the now_ms argument.
"""
from __future__ import annotations

from typing import Dict, Optional

from triveni.models.signal_state import SignalState


INTERSECTION_ID = "INT_KEMPS_CORNER"
CROSSWALK_IDS = ["CW_NORTH", "CW_EAST", "CW_SOUTH", "CW_WEST"]

# ---------------------------------------------------------------------------
# Two-phase cycle constants — India-style (no separate pedestrian phase)
# ---------------------------------------------------------------------------
TWO_PHASE_EW_GREEN_MS: int = 45_000    # Phase A: E-W vehicles green; CW_NORTH & CW_SOUTH WALK
TWO_PHASE_CLEARANCE_A_MS: int = 5_000  # Clearance after Phase A
TWO_PHASE_NS_GREEN_MS: int = 45_000    # Phase B: N-S vehicles green; CW_EAST  & CW_WEST  WALK
TWO_PHASE_CLEARANCE_B_MS: int = 5_000  # Clearance after Phase B
TWO_PHASE_CYCLE_MS: int = (
    TWO_PHASE_EW_GREEN_MS + TWO_PHASE_CLEARANCE_A_MS
    + TWO_PHASE_NS_GREEN_MS + TWO_PHASE_CLEARANCE_B_MS
)  # = 100 000 ms

# Crosswalks whose pedestrians WALK in each vehicle phase
_TWO_PHASE_A_WALK = frozenset({"CW_NORTH", "CW_SOUTH"})  # E-W GREEN → peds cross E↔W
_TWO_PHASE_B_WALK = frozenset({"CW_EAST",  "CW_WEST" })  # N-S GREEN → peds cross N↔S


# ---------------------------------------------------------------------------
# Default timestamp helpers
# ---------------------------------------------------------------------------

def _default_timestamps(now_ms: int) -> tuple[int, int]:
    """Return (source_timestamp_ms, receive_timestamp_ms) defaults."""
    return now_ms - 50, now_ms - 10


# ---------------------------------------------------------------------------
# Single-crosswalk static builders
# ---------------------------------------------------------------------------

def make_signal(
    crosswalk_id: str,
    signal_state: str,
    now_ms: int,
    *,
    remaining_time_ms: Optional[int] = None,
    source_age_ms: int = 50,
    receive_age_ms: int = 10,
    source_health: str = "OK",
    intersection_id: str = INTERSECTION_ID,
) -> SignalState:
    """Build a single SignalState with explicit timing.

    Parameters
    ----------
    crosswalk_id    : target crosswalk
    signal_state    : WALK | DONT_WALK | CLEARANCE (or DON'T_WALK alias)
    now_ms          : current simulated time in ms
    remaining_time_ms : ms left in the current phase (None = omit)
    source_age_ms   : source_timestamp_ms = now_ms - source_age_ms  (default 50)
    receive_age_ms  : receive_timestamp_ms = now_ms - receive_age_ms (default 10)
    source_health   : OK | DEGRADED | FAULT
    intersection_id : override if needed (e.g. wrong intersection tests)
    """
    source_ts = now_ms - source_age_ms
    receive_ts = now_ms - receive_age_ms
    return SignalState(
        intersection_id=intersection_id,
        crosswalk_id=crosswalk_id,
        signal_state=signal_state,
        remaining_time_ms=remaining_time_ms,
        source_timestamp_ms=source_ts,
        receive_timestamp_ms=receive_ts,
        source_health=source_health,
    )


def make_stale_signal(
    crosswalk_id: str,
    signal_state: str,
    now_ms: int,
    stale_age_ms: int = 900,
    *,
    remaining_time_ms: Optional[int] = None,
    source_health: str = "OK",
) -> SignalState:
    """Build a SignalState whose age exceeds the typical freshness threshold."""
    return make_signal(
        crosswalk_id=crosswalk_id,
        signal_state=signal_state,
        now_ms=now_ms,
        remaining_time_ms=remaining_time_ms,
        source_age_ms=stale_age_ms,
        source_health=source_health,
    )


def make_delayed_signal(
    crosswalk_id: str,
    signal_state: str,
    now_ms: int,
    delay_ms: int = 900,
    *,
    remaining_time_ms: Optional[int] = None,
) -> SignalState:
    """Build a SignalState delayed in transit (same effect as stale)."""
    return make_signal(
        crosswalk_id=crosswalk_id,
        signal_state=signal_state,
        now_ms=now_ms,
        remaining_time_ms=remaining_time_ms,
        source_age_ms=delay_ms,
    )


def make_signal_with_contradictory_timestamp(
    crosswalk_id: str,
    signal_state: str,
    now_ms: int,
    *,
    remaining_time_ms: Optional[int] = None,
) -> SignalState:
    """Build a SignalState where source_timestamp is in the future (contradictory).

    Produces age < 0, detected as SIGNAL_TIMESTAMP_INVALID by the engine.
    """
    future_source_ts = now_ms + 5_000
    receive_ts = future_source_ts + 10
    return SignalState(
        intersection_id=INTERSECTION_ID,
        crosswalk_id=crosswalk_id,
        signal_state=signal_state,
        remaining_time_ms=remaining_time_ms,
        source_timestamp_ms=future_source_ts,
        receive_timestamp_ms=receive_ts,
        source_health="OK",
    )


# ---------------------------------------------------------------------------
# Full-intersection static helper
# ---------------------------------------------------------------------------

def make_all_signals(
    now_ms: int,
    states: Optional[Dict[str, str]] = None,
    remaining_times: Optional[Dict[str, int]] = None,
    source_age_ms: int = 50,
) -> Dict[str, SignalState]:
    """Build a dict of SignalState for all four crosswalks.

    Parameters
    ----------
    now_ms          : current simulated time
    states          : mapping crosswalk_id -> signal_state.
                      Defaults to DONT_WALK for any crosswalk not listed.
    remaining_times : mapping crosswalk_id -> remaining_time_ms (optional per crosswalk)
    source_age_ms   : age applied to all signals
    """
    states = states or {}
    remaining_times = remaining_times or {}
    result: Dict[str, SignalState] = {}
    for cw_id in CROSSWALK_IDS:
        state = states.get(cw_id, "DONT_WALK")
        result[cw_id] = make_signal(
            crosswalk_id=cw_id,
            signal_state=state,
            now_ms=now_ms,
            remaining_time_ms=remaining_times.get(cw_id),
            source_age_ms=source_age_ms,
        )
    return result


# ---------------------------------------------------------------------------
# Two-phase coordinated signal simulation (India style)
# ---------------------------------------------------------------------------

def two_phase_signals(
    now_ms: int,
    cycle_start_ms: int = 0,
) -> Dict[str, SignalState]:
    """Return all four per-crosswalk SignalStates for one tick of the two-phase cycle.

    India-style two-phase signal: pedestrians walk PARALLEL to (and at the same
    time as) the green vehicle phase.  No separate all-red pedestrian phase.
    This matches intersections where left turns use a free-left slip road.

    Phase A  (t = 0 … TWO_PHASE_EW_GREEN_MS-1)
        E-W vehicles GREEN.
        CW_NORTH & CW_SOUTH → WALK  (peds cross E↔W, parallel to E-W vehicles)
        CW_EAST  & CW_WEST  → DONT_WALK

    Clearance A  (t = TWO_PHASE_EW_GREEN_MS … +TWO_PHASE_CLEARANCE_A_MS-1)
        CW_NORTH & CW_SOUTH → CLEARANCE
        CW_EAST  & CW_WEST  → DONT_WALK

    Phase B  (t = 50 000 … +TWO_PHASE_NS_GREEN_MS-1)
        N-S vehicles GREEN.
        CW_EAST  & CW_WEST  → WALK  (peds cross N↔S, parallel to N-S vehicles)
        CW_NORTH & CW_SOUTH → DONT_WALK

    Clearance B  (t = 95 000 … TWO_PHASE_CYCLE_MS-1)
        CW_EAST  & CW_WEST  → CLEARANCE
        CW_NORTH & CW_SOUTH → DONT_WALK

    remaining_time_ms semantics
    ---------------------------
    WALK      : ms left in this crosswalk's current WALK window.
    CLEARANCE : ms left in this crosswalk's CLEARANCE window.
    DONT_WALK : ms until this crosswalk's NEXT WALK window begins.

    Parameters
    ----------
    now_ms          : current simulated time in ms
    cycle_start_ms  : epoch-ms when the current cycle started (default 0)

    Returns
    -------
    Dict mapping crosswalk_id -> SignalState for all four crosswalks.
    """
    t = (now_ms - cycle_start_ms) % TWO_PHASE_CYCLE_MS

    # Cumulative phase boundary offsets
    _a_end     = TWO_PHASE_EW_GREEN_MS                          # 45 000
    _a_clr_end = _a_end + TWO_PHASE_CLEARANCE_A_MS              # 50 000
    _b_end     = _a_clr_end + TWO_PHASE_NS_GREEN_MS             # 95 000
    # _b_clr_end = TWO_PHASE_CYCLE_MS                           # 100 000

    src_ts     = now_ms - 50
    receive_ts = now_ms - 10

    result: Dict[str, SignalState] = {}

    for cw_id in CROSSWALK_IDS:
        is_phase_a_cw = cw_id in _TWO_PHASE_A_WALK  # True for CW_NORTH, CW_SOUTH

        if t < _a_end:
            # ── Phase A: E-W vehicles GREEN ────────────────────────────────
            if is_phase_a_cw:
                state     = "WALK"
                remaining = _a_end - t            # ms left in Phase A WALK
            else:
                state     = "DONT_WALK"
                remaining = _a_clr_end - t        # ms until Phase B WALK starts

        elif t < _a_clr_end:
            # ── Clearance A ────────────────────────────────────────────────
            if is_phase_a_cw:
                state     = "CLEARANCE"
                remaining = _a_clr_end - t        # ms left in clearance
            else:
                state     = "DONT_WALK"
                remaining = _a_clr_end - t        # Phase B starts right after

        elif t < _b_end:
            # ── Phase B: N-S vehicles GREEN ────────────────────────────────
            if is_phase_a_cw:
                state     = "DONT_WALK"
                remaining = TWO_PHASE_CYCLE_MS - t  # ms until next Phase A (cycle wrap)
            else:
                state     = "WALK"
                remaining = _b_end - t            # ms left in Phase B WALK

        else:
            # ── Clearance B ────────────────────────────────────────────────
            if is_phase_a_cw:
                state     = "DONT_WALK"
                remaining = TWO_PHASE_CYCLE_MS - t  # Phase A starts at next cycle
            else:
                state     = "CLEARANCE"
                remaining = TWO_PHASE_CYCLE_MS - t  # ms left in clearance

        result[cw_id] = SignalState(
            intersection_id=INTERSECTION_ID,
            crosswalk_id=cw_id,
            signal_state=state,
            remaining_time_ms=remaining,
            source_timestamp_ms=src_ts,
            receive_timestamp_ms=receive_ts,
            source_health="OK",
        )

    return result
