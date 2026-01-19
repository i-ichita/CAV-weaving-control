# -*- coding: utf-8 -*-
"""
weaving_v11/safety.py (v1.0 / 2025-12-26)

安全ロジック一元化モジュール
================================

目的:
    全ての安全関連計算 (TTC, RSS, AEB) を単一の、ドキュメント化された、
    テスト可能なモジュールに統合。コード重複を排除し、
    システム全体で一貫した安全動作を保証。

設計哲学:
    - 単一の真実: 全ての安全閾値と数式をここで定義
    - 型安全性: 全ての公開APIに完全な型アノテーション
    - テスト容易性: 可能な限り純粋関数、副作用最小化
    - Apollo準拠: Apollo Safety ManagerとUN R157標準に基づく

参照:
    - Apollo: modules/planning/tasks/deciders/speed_decider/speed_decider.cc
    - Apollo: modules/control/common/control_gflags.cc (安全パラメータ)
    - UN R157: 自動車線保持システム (ALKS) 規則
    - ISO 22179: 先進緊急ブレーキシステム (AEBS)
    - RSS: 責任感応型安全モデル (Shalev-Shwartz et al., 2017)

作者: CAV Weaving Control Team
================================================================================
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


# ============================================================================
# Safety Constants (Centralized Definition)
# ============================================================================

@dataclass(frozen=True)
class SafetyConstants:
    """
    Immutable safety parameters used throughout the system.
    
    All values are based on:
    - UN R157 (ALKS) regulation
    - Apollo production defaults
    - RSS theoretical foundations
    - Empirical validation from simulation
    """
    
    # === Vehicle Physical Parameters ===
    L_VEHICLE: float = 5.0          # [m] Vehicle length (Apollo Lincoln MKZ)
    VEHICLE_WIDTH: float = 1.8      # [m] Vehicle width
    
    # === Braking Capabilities (UN R157 Compliant) ===
    A_MAX_BRAKE: float = -6.0       # [m/s²] Maximum emergency braking (UN R157)
    A_COMFORT_BRAKE: float = -3.5   # [m/s²] Comfortable braking limit
    A_STRONG_BRAKE: float = -5.0    # [m/s²] Strong but not emergency braking
    A_GENTLE_BRAKE: float = -1.5    # [m/s²] Gentle deceleration (CAV planning)
    
    # === TTC (Time-to-Collision) Thresholds ===
    TTC_CRITICAL: float = 1.0       # [s] Absolute emergency - max braking
    TTC_URGENT: float = 1.5         # [s] Urgent - strong braking
    TTC_WARNING: float = 2.0        # [s] Warning - moderate braking
    TTC_CAUTION: float = 4.0        # [s] Caution - gentle adjustment (was 2.0->3.0 for stability)
    TTC_SAFE: float = 5.0           # [s] Safe - normal operation
    
    # === RSS Parameters ===
    RSS_REACTION_TIME: float = 0.5  # [s] Driver/system reaction time
    RSS_REACTION_TIME_CAV: float = 0.15  # [s] CAV reaction time (V2V)
    RSS_MIN_GAP: float = 2.0        # [m] Minimum standstill gap
    RSS_MIN_GAP_CAV: float = 2.5    # [m] Minimum gap for CAV-CAV (stable baseline)
    RSS_SAFETY_MARGIN: float = 2.0  # [m] Additional safety buffer
    
    # === Gap Thresholds ===
    GAP_CRITICAL: float = 1.0       # [m] Extreme proximity - QP override
    GAP_OVERLAP: float = 0.0        # [m] Physical overlap detected
    GAP_MIN_FRONT: float = 20.0     # [m] Minimum front gap for LC
    GAP_MIN_REAR: float = 20.0      # [m] Minimum rear gap for LC
    
    # === AEB Hysteresis ===
    AEB_ACTIVATION_FACTOR: float = 0.95   # RSS × factor to trigger AEB (stable 0.85)
    AEB_DEACTIVATION_FACTOR: float = 2.0 # RSS × factor to release AEB
    
    # === Lateral Safety ===
    LATERAL_OVERLAP_THRESHOLD: float = 1.0  # [m] Lateral distance for overlap
    
    # === V2V Cooperative LC Parameters (v22.0) ===
    TTC_CAV_COOPERATE: float = 5.0      # [s] TTC to start V2V cooperation (early!)
    V2V_INTENT_HORIZON: float = 3.0     # [s] How far ahead to broadcast LC intent
    V2V_GAP_RESERVATION: float = 30.0   # [m] Gap to reserve for LC vehicle
    COOP_YIELD_DECEL: float = -2.0      # [m/s²] Soft decel for cooperative yielding
    V2V_RANGE: float = 50.0             # [m] V2V communication range


# Global instance for easy access
SAFETY = SafetyConstants()


# ============================================================================
# Enumerations
# ============================================================================

class AebSeverity(Enum):
    """AEB activation severity levels."""
    NONE = 0        # No AEB needed
    MODERATE = 1    # Comfort braking (-3.5 m/s²)
    URGENT = 2      # Strong braking (-5.0 m/s²)
    CRITICAL = 3    # Maximum braking (-6.0 m/s²)


class SafetyDecision(Enum):
    """High-level safety decision for speed planning."""
    CRUISE = 0      # Normal cruise - no safety concern
    FOLLOW = 1      # Follow mode - maintain headway
    YIELD = 2       # Yield mode - reduce speed proactively
    STOP = 3        # Stop decision - emergency deceleration
    AEB = 4         # AEB active - override all other control


# ============================================================================
# Data Transfer Objects (DTOs)
# ============================================================================

@dataclass
class AebCommand:
    """
    AEB evaluation result with commanded acceleration and metadata.
    
    Attributes:
        activate: Whether AEB should be activated
        ax: Commanded longitudinal acceleration [m/s²]
        severity: Severity level of the AEB activation
        reason: Human-readable explanation for logging
        ttc: Computed time-to-collision [s] (None if infinite/invalid)
        gap: Current gap to obstacle [m]
    """
    activate: bool
    ax: float
    severity: AebSeverity
    reason: str
    ttc: Optional[float] = None
    gap: Optional[float] = None


@dataclass
class SafetyEvaluation:
    """
    Comprehensive safety evaluation result.
    
    Attributes:
        decision: High-level safety decision
        aeb_cmd: AEB command if applicable
        v_ref_adjusted: Recommended reference velocity [m/s]
        rss_distance: Computed RSS safe distance [m]
        is_follow_too_close: Apollo IsFollowTooClose flag
    """
    decision: SafetyDecision
    aeb_cmd: Optional[AebCommand]
    v_ref_adjusted: float
    rss_distance: float
    is_follow_too_close: bool = False


# ============================================================================
# Core Safety Functions
# ============================================================================

def compute_ttc(
    v_ego: float,
    v_front: float,
    gap: float,
    min_rel_v: float = 0.1
) -> Optional[float]:
    """
    Compute Time-to-Collision (TTC) between ego and front vehicle.
    
    TTC is the time until collision assuming constant velocities.
    Returns None if vehicles are not approaching (infinite TTC).
    
    Formula:
        TTC = gap / (v_ego - v_front)  if v_ego > v_front
        TTC = ∞ (None)                  otherwise
    
    Args:
        v_ego: Ego vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        gap: Bumper-to-bumper gap [m] (can be negative for overlap)
        min_rel_v: Minimum relative velocity threshold [m/s]
    
    Returns:
        TTC in seconds, or None if not approaching
    
    Example:
        >>> compute_ttc(20.0, 15.0, 25.0)  # Approaching at 5 m/s
        5.0
        >>> compute_ttc(15.0, 20.0, 25.0)  # Not approaching
        None
    """
    rel_v = v_ego - v_front
    
    if rel_v <= min_rel_v:
        # Not approaching or approaching too slowly
        return None
    
    if gap <= 0:
        # Already overlapping - TTC is 0 (immediate collision)
        return 0.0
    
    return gap / rel_v


def compute_required_decel(
    rel_v: float,
    gap: float,
    min_gap: float = 0.1
) -> float:
    """
    Compute required deceleration to avoid collision.
    
    Based on kinematic equation: v² = v₀² + 2·a·d
    Solving for a: a = -v²/(2·d)
    
    Args:
        rel_v: Relative velocity (v_ego - v_front) [m/s]
        gap: Current gap [m]
        min_gap: Minimum gap to avoid division by zero [m]
    
    Returns:
        Required deceleration magnitude (positive value) [m/s²]
    
    Example:
        >>> compute_required_decel(10.0, 50.0)  # 10 m/s closing, 50m gap
        1.0  # Need 1 m/s² deceleration
    """
    safe_gap = max(gap, min_gap)
    return (rel_v ** 2) / (2 * safe_gap)


def compute_rss_safe_distance(
    v_ego: float,
    v_front: float,
    t_reaction: float = SAFETY.RSS_REACTION_TIME,
    a_brake: float = abs(SAFETY.A_MAX_BRAKE),
    min_gap: float = SAFETY.RSS_MIN_GAP
) -> float:
    """
    Compute RSS (Responsibility-Sensitive Safety) safe distance.
    
    RSS safe distance ensures that even if the front vehicle brakes maximally,
    the ego vehicle can react and brake to avoid collision.
    
    Formula (simplified, same-lane following):
        d_safe = v_ego × t_reaction + v_ego²/(2·a_brake) - v_front²/(2·a_brake) + d_min
    
    Full RSS formula considers:
        - Ego reaction distance: v_ego × t_reaction
        - Ego braking distance: v_ego²/(2·a_ego_brake)
        - Front braking distance: -v_front²/(2·a_front_brake) (they stop earlier)
        - Minimum standstill gap: d_min
    
    Args:
        v_ego: Ego vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        t_reaction: System reaction time [s]
        a_brake: Maximum braking capability [m/s²]
        min_gap: Minimum standstill gap [m]
    
    Returns:
        RSS safe following distance [m]
    
    Reference:
        Shalev-Shwartz et al., "On a Formal Model of Safe and Scalable
        Self-driving Cars", arXiv:1708.06374
    
    Example:
        >>> compute_rss_safe_distance(20.0, 15.0)
        ~17.5m (depends on parameters)
    """
    # Reaction distance
    reaction_dist = v_ego * t_reaction
    
    # Braking distance difference
    ego_brake_dist = (v_ego ** 2) / (2 * a_brake)
    front_brake_dist = (v_front ** 2) / (2 * a_brake)
    
    # RSS distance
    rss_dist = reaction_dist + ego_brake_dist - front_brake_dist
    
    # Ensure minimum gap
    return max(rss_dist, min_gap)


def compute_rss_safe_distance_cav(
    v_ego: float,
    v_front: float,
    v_rel: Optional[float] = None
) -> float:
    """
    Compute RSS safe distance for CAV-to-CAV interaction.
    
    CAVs have:
    - Shorter reaction time (V2V communication)
    - Dynamic minimum gap based on relative velocity
    
    Args:
        v_ego: Ego CAV velocity [m/s]
        v_front: Front CAV velocity [m/s]
        v_rel: Relative velocity (computed if not provided) [m/s]
    
    Returns:
        CAV-specific RSS safe distance [m]
    """
    if v_rel is None:
        v_rel = max(0.0, v_ego - v_front)
    
    t_react = SAFETY.RSS_REACTION_TIME_CAV
    a_brake = abs(SAFETY.A_MAX_BRAKE)
    
    # Physics-based minimum: stopping distance + reaction distance
    physics_min = (v_rel ** 2) / (2 * a_brake) + v_rel * t_react
    
    # Add safety margin
    return max(physics_min + SAFETY.RSS_SAFETY_MARGIN, SAFETY.RSS_MIN_GAP_CAV)


def is_follow_too_close(
    v_ego: float,
    v_front: float,
    gap: float,
    max_decel: float = 2.5
) -> bool:
    """
    Apollo IsFollowTooClose implementation.
    
    Determines if ego vehicle is dangerously close to front vehicle,
    requiring STOP decision instead of normal FOLLOW.
    
    Apollo Logic (speed_decider.cc lines 190-218):
        1. Only applies if v_front < v_ego (closing in)
        2. min_distance = (v_ego - v_front)² × 0.5 / max_decel
        3. Return True if gap < min_distance
    
    Args:
        v_ego: Ego vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        gap: Current gap [m]
        max_decel: Maximum comfortable deceleration [m/s²]
    
    Returns:
        True if follow distance is dangerously close
    
    Reference:
        Apollo speed_decider.cc: IsFollowTooClose()
    """
    if v_front >= v_ego:
        # Not approaching - not too close
        return False
    
    # Apollo formula
    v_diff = v_ego - v_front
    min_follow_distance = (v_diff ** 2) * 0.5 / max_decel
    
    return gap < min_follow_distance


def evaluate_aeb(
    v_ego: float,
    v_front: float,
    gap: float,
    is_cav_front: bool = True,
    has_valid_trajectory: bool = False,
    current_rss: Optional[float] = None
) -> AebCommand:
    """
    Evaluate AEB (Automatic Emergency Braking) activation.
    
    Multi-stage adaptive braking based on TTC and gap severity:
    
    Stage 1 (CRITICAL): TTC < 1.0s or gap < -2.0m
        → Maximum braking (-6.0 m/s²)
        → UN R157 emergency limit
    
    Stage 2 (URGENT): TTC < 1.5s (CAV) or TTC < 2.0s (HDV)
        → Strong braking (-5.0 m/s²)
        → Required decel capped at strong limit
    
    Stage 3 (MODERATE): TTC < 3.0s
        → Comfort braking (-3.5 to -4.0 m/s²)
        → Smooth deceleration profile
    
    Stage 4 (NONE): TTC >= 3.0s and gap > RSS
        → No AEB activation
        → Normal QP control
    
    Args:
        v_ego: Ego vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        gap: Bumper-to-bumper gap [m]
        is_cav_front: Whether front vehicle is a CAV
        has_valid_trajectory: Whether front CAV has shared trajectory
        current_rss: Pre-computed RSS distance (computed if None)
    
    Returns:
        AebCommand with activation decision and parameters
    """
    # Compute TTC
    ttc = compute_ttc(v_ego, v_front, gap)
    rel_v = max(0.0, v_ego - v_front)
    
    # Compute RSS if not provided
    if current_rss is None:
        if is_cav_front and has_valid_trajectory:
            current_rss = compute_rss_safe_distance_cav(v_ego, v_front, rel_v)
        else:
            current_rss = compute_rss_safe_distance(v_ego, v_front)
    
    # === Stage 1: CRITICAL Emergency ===
    if gap < -2.0 or (ttc is not None and ttc < SAFETY.TTC_CRITICAL):
        return AebCommand(
            activate=True,
            ax=SAFETY.A_MAX_BRAKE,
            severity=AebSeverity.CRITICAL,
            reason=f"CRITICAL: gap={gap:.1f}m, TTC={ttc:.2f}s" if ttc else f"CRITICAL: overlap gap={gap:.1f}m",
            ttc=ttc,
            gap=gap
        )
    
    # === Check if AEB needed at all ===
    # For CAV-CAV with valid trajectory, use dynamic threshold
    if is_cav_front and has_valid_trajectory:
        aeb_threshold = compute_rss_safe_distance_cav(v_ego, v_front, rel_v)
    else:
        aeb_threshold = current_rss * SAFETY.AEB_ACTIVATION_FACTOR
    
    if gap >= aeb_threshold:
        return AebCommand(
            activate=False,
            ax=0.0,
            severity=AebSeverity.NONE,
            reason=f"Safe: gap={gap:.1f}m >= threshold={aeb_threshold:.1f}m",
            ttc=ttc,
            gap=gap
        )
    
    # === Stage 2: URGENT ===
    ttc_urgent = SAFETY.TTC_URGENT if is_cav_front else SAFETY.TTC_WARNING
    if ttc is not None and ttc < ttc_urgent:
        if rel_v > 0.1 and gap > 0:
            required = compute_required_decel(rel_v, gap)
            ax = max(SAFETY.A_STRONG_BRAKE, -required)
        else:
            ax = SAFETY.A_STRONG_BRAKE
        
        return AebCommand(
            activate=True,
            ax=ax,
            severity=AebSeverity.URGENT,
            reason=f"URGENT: TTC={ttc:.2f}s < {ttc_urgent:.1f}s",
            ttc=ttc,
            gap=gap
        )
    
    # === Stage 3: MODERATE ===
    if ttc is not None and ttc < SAFETY.TTC_CAUTION:
        if rel_v > 0.1 and gap > 0:
            required = compute_required_decel(rel_v, gap)
            ax = max(SAFETY.A_COMFORT_BRAKE, -required)
        else:
            ax = SAFETY.A_COMFORT_BRAKE
        
        return AebCommand(
            activate=True,
            ax=ax,
            severity=AebSeverity.MODERATE,
            reason=f"MODERATE: TTC={ttc:.2f}s, gap={gap:.1f}m < RSS={current_rss:.1f}m",
            ttc=ttc,
            gap=gap
        )
    
    # === Default: Gap-based moderate AEB ===
    if rel_v > 0.1 and gap > 0:
        required = compute_required_decel(rel_v, gap)
        ax = max(SAFETY.A_COMFORT_BRAKE, -required)
    else:
        ax = SAFETY.A_STRONG_BRAKE
    
    return AebCommand(
        activate=True,
        ax=ax,
        severity=AebSeverity.MODERATE,
        reason=f"GAP-AEB: gap={gap:.1f}m < threshold={aeb_threshold:.1f}m",
        ttc=ttc,
        gap=gap
    )


def compute_gap_bumper_to_bumper(
    x_front: float,
    x_ego: float,
    l_vehicle: float = SAFETY.L_VEHICLE
) -> float:
    """
    Compute bumper-to-bumper gap between two vehicles.
    
    Assumes both vehicles have the same length and positions are
    at vehicle center.
    
    Formula:
        gap = (x_front - L/2) - (x_ego + L/2)
            = x_front - x_ego - L
    
    Args:
        x_front: Front vehicle center position [m]
        x_ego: Ego vehicle center position [m]
        l_vehicle: Vehicle length [m]
    
    Returns:
        Bumper-to-bumper gap [m] (negative means overlap)
    """
    return x_front - x_ego - l_vehicle


def has_lateral_overlap(
    d_ego: float,
    d_other: float,
    threshold: float = SAFETY.LATERAL_OVERLAP_THRESHOLD
) -> bool:
    """
    Check if two vehicles have lateral overlap (potential collision path).
    
    Args:
        d_ego: Ego vehicle lateral position (Frenet d) [m]
        d_other: Other vehicle lateral position [m]
        threshold: Lateral distance threshold for overlap [m]
    
    Returns:
        True if lateral positions overlap
    """
    return abs(d_ego - d_other) < threshold


def evaluate_comprehensive_safety(
    v_ego: float,
    v_front: float,
    gap: float,
    d_ego: float,
    d_front: float,
    v_ref: float,
    is_same_lane: bool,
    is_cav_front: bool = True,
    has_valid_trajectory: bool = False
) -> SafetyEvaluation:
    """
    Comprehensive safety evaluation combining all safety checks.
    
    Integrates:
    - Lateral overlap check
    - RSS safe distance
    - IsFollowTooClose (Apollo STOP decision)
    - AEB evaluation
    - Speed reference adjustment
    
    Args:
        v_ego: Ego vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        gap: Bumper-to-bumper gap [m]
        d_ego: Ego lateral position [m]
        d_front: Front lateral position [m]
        v_ref: Current reference velocity [m/s]
        is_same_lane: Whether vehicles are in the same lane
        is_cav_front: Whether front vehicle is a CAV
        has_valid_trajectory: Whether front CAV has shared trajectory
    
    Returns:
        SafetyEvaluation with comprehensive safety decision
    """
    # Check lateral overlap
    lat_overlap = has_lateral_overlap(d_ego, d_front)
    should_check_safety = is_same_lane or lat_overlap
    
    if not should_check_safety:
        return SafetyEvaluation(
            decision=SafetyDecision.CRUISE,
            aeb_cmd=None,
            v_ref_adjusted=v_ref,
            rss_distance=0.0,
            is_follow_too_close=False
        )
    
    # Compute RSS distance
    if is_cav_front and has_valid_trajectory:
        rss_dist = compute_rss_safe_distance_cav(v_ego, v_front)
    else:
        rss_dist = compute_rss_safe_distance(v_ego, v_front)
    
    # Check IsFollowTooClose
    follow_too_close = is_follow_too_close(v_ego, v_front, gap)
    
    # Evaluate AEB
    aeb_cmd = evaluate_aeb(
        v_ego, v_front, gap,
        is_cav_front, has_valid_trajectory, rss_dist
    )
    
    # Determine decision and v_ref adjustment
    v_ref_adjusted = v_ref
    
    if aeb_cmd.activate:
        decision = SafetyDecision.AEB
        # AEB takes over, no v_ref needed
    elif follow_too_close:
        decision = SafetyDecision.STOP
        v_ref_adjusted = min(v_ref, v_front * 0.9)
    elif gap < rss_dist * 1.2:
        decision = SafetyDecision.YIELD
        v_ref_adjusted = min(v_ref, v_front * 0.8)
    elif gap < rss_dist * 1.5:
        decision = SafetyDecision.FOLLOW
        v_ref_adjusted = min(v_ref, v_front)
    else:
        decision = SafetyDecision.CRUISE
    
    return SafetyEvaluation(
        decision=decision,
        aeb_cmd=aeb_cmd if aeb_cmd.activate else None,
        v_ref_adjusted=v_ref_adjusted,
        rss_distance=rss_dist,
        is_follow_too_close=follow_too_close
    )


# ============================================================================
# Logging Utilities
# ============================================================================

def format_aeb_log(
    vehicle_id: int,
    aeb_cmd: AebCommand,
    front_vehicle_id: Optional[int] = None
) -> str:
    """
    Format AEB event for logging with standardized prefix.
    
    Args:
        vehicle_id: Ego vehicle ID
        aeb_cmd: AEB command result
        front_vehicle_id: Front vehicle ID (if known)
    
    Returns:
        Formatted log string
    """
    severity_prefix = {
        AebSeverity.NONE: "[AEB-NONE]",
        AebSeverity.MODERATE: "[AEB-MODERATE]",
        AebSeverity.URGENT: "[AEB-URGENT]",
        AebSeverity.CRITICAL: "[AEB-CRITICAL]"
    }
    
    prefix = severity_prefix.get(aeb_cmd.severity, "[AEB]")
    front_str = f" → V{front_vehicle_id}" if front_vehicle_id is not None else ""
    
    # v18.13 FIX: Handle None values for ttc and gap
    ttc_str = f"{aeb_cmd.ttc:.2f}" if aeb_cmd.ttc is not None else "inf"
    gap_str = f"{aeb_cmd.gap:.1f}" if aeb_cmd.gap is not None else "N/A"
    
    return (
        f"{prefix} V{vehicle_id}{front_str}: "
        f"gap={gap_str}m, TTC={ttc_str}s, "
        f"ax={aeb_cmd.ax:.2f}m/s² | {aeb_cmd.reason}"
    )


def format_rss_log(
    vehicle_id: int,
    gap: float,
    rss_dist: float,
    decision: SafetyDecision
) -> str:
    """
    Format RSS evaluation for logging.
    
    Args:
        vehicle_id: Ego vehicle ID
        gap: Current gap [m]
        rss_dist: RSS safe distance [m]
        decision: Safety decision made
    
    Returns:
        Formatted log string
    """
    ratio = gap / rss_dist if rss_dist > 0 else float('inf')
    return (
        f"[RSS] V{vehicle_id}: gap={gap:.1f}m, RSS={rss_dist:.1f}m "
        f"(ratio={ratio:.2f}) → {decision.name}"
    )
