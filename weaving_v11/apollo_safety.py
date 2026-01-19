# -*- coding: utf-8 -*-
"""
weaving_v11/apollo_safety.py (v27.0 / 2026-01-05)

Apollo-CAV統合安全マネージャー
==================================

全ての安全計算の単一の真実。
V2V軌道共有とQP境界制約を統合。

v27.0 変更点:
- 5段階安全状態追加 (SAFE/CAUTION/WARNING/CRITICAL/AEB)
- check_lc_start_safety(): LC開始前の統一安全チェック
- check_mid_lc_safety(): LC実行中の連続監視
- compute_early_warning_speed(): 早期減速速度計算
- evaluate_safety_state(): 統合安全状態評価
- 厳格化された閾値: RSS×2.0で減速開始、Guardian閾値15m

設計哲学:
- 安全性はQPの制約であり、反応層ではない
- V2V軌道により予測的(反応的ではない)安全を実現
- 単一クラスで全ての安全ロジックを処理 = スパゲッティなし

参照:
- Apollo: modules/planning/tasks/deciders/speed_bounds_decider/
- Apollo: modules/planning/common/obstacle_blocking_analyzer.cc
- RSS: 責任感応型安全モデル

作者: CAV Weaving Control Team
================================================================================
"""

import numpy as np
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING, TypedDict
from typing import Union
from typing import Literal
from dataclasses import dataclass
from enum import Enum

if TYPE_CHECKING:
    from .vehicle import Vehicle
    from .parameters import IntegratedZoneParameters

# Safety constants
L_VEHICLE = 5.0            # [m] Vehicle length
RSS_REACTION_TIME = 0.5    # [s] Reaction time for HDV
RSS_REACTION_TIME_CAV = 0.15   # [s] Reaction time for CAV (V2V)
RSS_MIN_GAP = 2.0          # [m] Minimum standstill gap
A_MAX_BRAKE = 6.0          # [m/s²] Maximum braking
A_COMFORT_BRAKE = 3.5      # [m/s²] Comfortable braking
A_STRONG_BRAKE = 5.0       # [m/s²] Strong braking


class SafetyState(Enum):
    """5-stage safety state for graduated response."""
    SAFE = 0       # 通常走行 - 減速不要
    CAUTION = 1    # 注意 - RSS×2.0以内、速度マッチング
    WARNING = 2    # 警告 - RSS×1.5以内、積極的減速
    CRITICAL = 3   # 危険 - RSS×1.2以内、緊急減速
    AEB = 4        # AEB発動 - 最大制動


@dataclass
class SafetyEvaluation:
    """Result of safety state evaluation."""
    state: SafetyState
    recommended_ax: float      # Recommended acceleration [m/s²]
    recommended_v_ref: float   # Recommended reference velocity [m/s]
    gap: float                 # Current gap [m]
    ttc: Optional[float]       # Time-to-collision [s]
    rss_distance: float        # RSS safe distance [m]
    reason: str                # Human-readable explanation


@dataclass
class SafetyBoundary:
    """ST-Graph safety boundary for a single obstacle."""
    vehicle_id: int
    s_lower: np.ndarray  # Min safe position per timestep
    s_upper: np.ndarray  # Max safe position per timestep
    is_front: bool
    is_cav: bool


@dataclass
class LCSafetyResult:
    """Result of LC safety check (v28.0 extended)."""
    is_safe: bool
    front_gap: float
    rear_gap: float
    front_ttc: Optional[float]
    rear_ttc: Optional[float]
    blocking_vehicle_id: Optional[int]
    reason: str
    # v28.0: Apollo-style urgency support
    urgency_level: str = 'normal'           # 'normal', 'urgent', 'emergency'
    distance_to_exit: float = float('inf')  # Distance to weaving zone exit [m]
    v2v_request: Optional[str] = None       # V2V cooperation request type


class DynamicThresholds(TypedDict):
    """動的安全しきい値のセット (v28.0)。"""
    aggressiveness: float
    urgency_level: Literal['normal', 'urgent', 'emergency']
    distance_to_exit: float
    front_gap: float
    rear_gap: float
    front_ttc: float
    rear_ttc: float
    mid_lc_long: float
    mid_lc_lat: float


class ApolloSafetyManager:
    """
    Unified safety management for Apollo-style QP planning.
    
    v27.0: Single Source of Truth for ALL safety logic.
    
    Responsibilities:
    1. Collect V2V trajectories from CAV vehicles
    2. Predict HDV trajectories (constant velocity model)
    3. Compute ST-boundaries per vehicle
    4. Merge into unified safe corridor for QP
    5. LC start safety verification
    6. Mid-LC continuous safety monitoring
    7. Early warning speed calculation
    8. Unified safety state evaluation
    9. Fallback AEB for extreme edge cases
    """
    
    def __init__(self, horizon: int = 80, dt: float = 0.1, params: Optional['IntegratedZoneParameters'] = None):
        """
        Args:
            horizon: Planning horizon [steps]
            dt: Time step [s]
            params: Parameters object (optional)
        """
        self.horizon = horizon
        self.dt = dt
        self.horizon_time = horizon * dt  # 8.0s
        self.params = params  # v27.29: Store params for dynamic threshold computation
        
        # v28.0: Safety margins for ST-boundary (QP constraints)
        # Increased to reduce QP infeasibility and relaxation rate
        self.front_margin_base = 7.0   # [m] Base margin for front vehicles (was 5.0, +40%)
        self.front_margin_cav = 5.0    # [m] Reduced margin for CAV-CAV with V2V (was 3.0, +67%)
        self.rear_margin = 10.0        # [m] Margin for rear vehicles (unchanged)
        
        # Base Thresholds (Default or from params)
        if params:
            self.lc_min_front_gap = params.lc_aggressive_base_front
            self.lc_min_rear_gap = params.lc_aggressive_base_rear
            self.lc_min_front_ttc = params.lc_aggressive_base_front_ttc
            self.lc_min_rear_ttc = 3.0

        # v28.0: Mid-LC safety thresholds (Increased for robustness)
        # Apollo EM Planner uses 15-20m for highway scenarios
        self.mid_lc_long_threshold = 15.0  # [m] v28.0: Restored to 15.0 (more conservative)
        self.mid_lc_lat_threshold = 2.0    # [m] v28.0: Increased from 1.8 (wider safety margin)
        self.mid_lc_prediction_horizon = 3.0  # [s] Look ahead time

        # v28.1: Safety state thresholds (Further relaxed to reduce AEB triggers)
        # Philosophy: QP handles safety via ST-boundaries, AEB is last resort only
        if params:
            self.early_warning_rss_factor = getattr(params, 'early_warning_rss_factor', 2.2)  # v28.0: 2.0→2.2
            self.warning_rss_factor = getattr(params, 'warning_rss_factor', 1.7)              # v28.0: 1.5→1.7
            self.critical_rss_factor = getattr(params, 'critical_rss_factor', 1.3)            # v28.0: 1.2→1.3
            self.aeb_rss_factor = getattr(params, 'aeb_rss_factor', 0.98)                     # v28.1: 0.95→0.98 (closer to Apollo 1.0)
            
            # LC Gap Override from params (v27.11)
            if hasattr(params, 'lc_min_front_gap'):
                self.lc_min_front_gap = params.lc_min_front_gap
            if hasattr(params, 'lc_min_rear_gap'):
                self.lc_min_rear_gap = params.lc_min_rear_gap
        else:
            self.early_warning_rss_factor = 2.2   # v28.0: Caution at RSS × 2.2 (was 2.0)
            self.warning_rss_factor = 1.7         # v28.0: Warning at RSS × 1.7 (was 1.5)
            self.critical_rss_factor = 1.3        # v28.0: Critical at RSS × 1.3 (was 1.2)
            self.aeb_rss_factor = 0.95            # v28.0: AEB at RSS × 0.95 (was 0.8, Apollo default)

    def update_parameters(self, params: 'IntegratedZoneParameters'):
        """Update safety parameters dynamically."""
        self.params = params  # v27.29: Store params for dynamic threshold access
        if hasattr(params, 'lc_min_front_gap'):
            self.lc_min_front_gap = params.lc_min_front_gap
        if hasattr(params, 'lc_min_rear_gap'):
            self.lc_min_rear_gap = params.lc_min_rear_gap
        
        self.early_warning_rss_factor = getattr(params, 'early_warning_rss_factor', self.early_warning_rss_factor)
        self.warning_rss_factor = getattr(params, 'warning_rss_factor', self.warning_rss_factor)
        self.critical_rss_factor = getattr(params, 'critical_rss_factor', self.critical_rss_factor)
        self.aeb_rss_factor = getattr(params, 'aeb_rss_factor', self.aeb_rss_factor)
    
    # =========================================================================
    # v27.0 NEW: LC Start Safety Check
    # =========================================================================
    # =========================================================================
    # v27.0 NEW: LC Start Safety Check
    # =========================================================================
    def check_lc_start_safety(
        self,
        ego: 'Vehicle',
        target_lane: str,
        vehicles: List['Vehicle'],
        cav_trajectories: Dict,
        prep_zone_len: float = 300.0,  # Default for testing
        total_len: float = 600.0       # Default for testing
    ) -> LCSafetyResult:
        """
        Check if lane change is safe to START.
        
        Unified replacement for controllers.py _check_gap_acceptance().
        Uses CAV trajectories for predictive checks.
        v27.2: Dynamic thresholds based on location (Aggressiveness).
        
        Args:
            ego: Ego vehicle requesting LC
            target_lane: Target lane for LC
            vehicles: All vehicles in simulation
            cav_trajectories: V2V shared trajectories
            prep_zone_len: Length of preparation zone [m]
            total_len: Total track length [m]
            
        Returns:
            LCSafetyResult with safety decision and details
        """
        # Calculate dynamic thresholds
        thresholds: DynamicThresholds = self._compute_dynamic_thresholds(ego.x, prep_zone_len, total_len)
        min_front_gap = thresholds['front_gap']
        min_rear_gap = thresholds['rear_gap']
        min_front_ttc = thresholds['front_ttc']
        min_rear_ttc = thresholds['rear_ttc']
        
        front_gap = float('inf')
        rear_gap = float('inf')
        front_ttc = None
        rear_ttc = None
        front_vehicle = None
        rear_vehicle = None
        
        # Find front and rear vehicles in target lane
        for v in vehicles:
            if v.id == ego.id or getattr(v, 'exited', False):
                continue
            if v.lane != target_lane:
                continue
            
            if v.x > ego.x:
                gap = v.x - ego.x - L_VEHICLE
                if gap < front_gap:
                    front_gap = gap
                    front_vehicle = v
            else:
                gap = ego.x - v.x - L_VEHICLE
                if gap < rear_gap:
                    rear_gap = gap
                    rear_vehicle = v
        
        # Compute TTCs
        if front_vehicle is not None:
            rel_v = ego.v - front_vehicle.v
            if rel_v > 0.1:
                front_ttc = front_gap / rel_v
        
        if rear_vehicle is not None:
            rel_v = rear_vehicle.v - ego.v
            if rel_v > 0.1:
                rear_ttc = rear_gap / rel_v
        
        # v28.0: Determine V2V cooperation request based on urgency
        urgency_level = thresholds['urgency_level']
        distance_to_exit = thresholds['distance_to_exit']
        v2v_request = None

        if urgency_level == 'emergency':
            v2v_request = 'urgent_yield'  # Apollo MUST_CHANGE mode
        elif urgency_level == 'urgent':
            v2v_request = 'cooperative_yield'  # Request cooperation

        # Check front gap
        if front_gap < min_front_gap:
            return LCSafetyResult(
                is_safe=False,
                front_gap=front_gap,
                rear_gap=rear_gap,
                front_ttc=front_ttc,
                rear_ttc=rear_ttc,
                blocking_vehicle_id=front_vehicle.id if front_vehicle else None,
                reason=f"Front gap {front_gap:.1f}m < {min_front_gap:.1f}m (Dyn)",
                urgency_level=urgency_level,
                distance_to_exit=distance_to_exit,
                v2v_request=v2v_request
            )

        # Check front TTC
        if front_ttc is not None and front_ttc < min_front_ttc:
            return LCSafetyResult(
                is_safe=False,
                front_gap=front_gap,
                rear_gap=rear_gap,
                front_ttc=front_ttc,
                rear_ttc=rear_ttc,
                blocking_vehicle_id=front_vehicle.id if front_vehicle else None,
                reason=f"Front TTC {front_ttc:.1f}s < {min_front_ttc:.1f}s (Dyn)",
                urgency_level=urgency_level,
                distance_to_exit=distance_to_exit,
                v2v_request=v2v_request
            )
        
        # Check rear gap with RSS-based dynamic threshold (clamped by dynamic min)
        if rear_vehicle is not None:
            is_rear_cav = not getattr(rear_vehicle, 'is_hdv', False)
            rel_v = rear_vehicle.v - ego.v
            
            if rel_v > 0:
                # Closing: use RSS-based threshold
                t_react = RSS_REACTION_TIME_CAV if is_rear_cav else RSS_REACTION_TIME
                a_brake = 4.0 if is_rear_cav else 3.0
                rss_gap = rel_v * t_react + (rel_v ** 2) / (2 * a_brake) + 5.0
                curr_min_rear = max(min_rear_gap, rss_gap)
            else:
                curr_min_rear = min_rear_gap
                # Absolute minimum fallback (force 8m if aggressive)
                abs_min = 8.0 if thresholds['aggressiveness'] > 0.8 else 10.0
                curr_min_rear = max(abs_min, curr_min_rear)
            
            if rear_gap < curr_min_rear:
                return LCSafetyResult(
                    is_safe=False,
                    front_gap=front_gap,
                    rear_gap=rear_gap,
                    front_ttc=front_ttc,
                    rear_ttc=rear_ttc,
                    blocking_vehicle_id=rear_vehicle.id,
                    reason=f"Rear gap {rear_gap:.1f}m < {curr_min_rear:.1f}m (RSS/Dyn)",
                    urgency_level=urgency_level,
                    distance_to_exit=distance_to_exit,
                    v2v_request=v2v_request
                )

        # Check rear TTC
        if rear_ttc is not None and rear_ttc < min_rear_ttc:
            return LCSafetyResult(
                is_safe=False,
                front_gap=front_gap,
                rear_gap=rear_gap,
                front_ttc=front_ttc,
                rear_ttc=rear_ttc,
                blocking_vehicle_id=rear_vehicle.id if rear_vehicle else None,
                reason=f"Rear TTC {rear_ttc:.1f}s < {min_rear_ttc:.1f}s (Dyn)",
                urgency_level=urgency_level,
                distance_to_exit=distance_to_exit,
                v2v_request=v2v_request
            )

        # All checks passed
        return LCSafetyResult(
            is_safe=True,
            front_gap=front_gap,
            rear_gap=rear_gap,
            front_ttc=front_ttc,
            rear_ttc=rear_ttc,
            blocking_vehicle_id=None,
            reason="All checks passed (Dyn)",
            urgency_level=urgency_level,
            distance_to_exit=distance_to_exit,
            v2v_request=None  # No request needed when safe
        )

    def _compute_dynamic_thresholds(
        self,
        x: float,
        prep_len: float,
        total_len: float
    ) -> DynamicThresholds:
        """
        Compute dynamic safety thresholds based on location.

        Policy (v28.0 - Apollo-style Urgent LC):
        - Prep Zone: Normal thresholds (Scale = 1.0)
        - Weaving Zone: Threshold decreases quadratically but maintains minimum 30%
          Scale = max(0.3, ((Total - x) / WeaveLen) ^ 2)
        - Near Exit (<50m): Emergency LC mode with cooperative V2V requests

        Apollo Reference: Lane Change Decider + Scenario Manager
        """
        # Calculate distance to exit
        distance_to_exit = max(0.0, total_len - x)

        if x < prep_len:
            # Prep Zone: Standard safety
            scale = 1.0
            urgency_level = 'normal'
            mid_lc_long = 15.0  # v28.0: Increased from 10.0
            mid_lc_lat = 2.0    # v28.0: Increased from 1.8
        else:
            # Weaving Zone: Decay with floor
            weave_len = total_len - prep_len
            remain = max(0.0, total_len - x)
            ratio = remain / weave_len

            # v27.29: Read exit parameters from params (Bayesian optimizable)
            dist_urgent = getattr(self.params, 'exit_urgent_dist', 100.0) if self.params else 100.0
            dist_emergency = getattr(self.params, 'exit_emergency_dist', 50.0) if self.params else 50.0
            scale_floor = getattr(self.params, 'exit_scale_floor', 0.3) if self.params else 0.3
            
            # Ensure emergency < urgent (physical consistency)
            if dist_emergency >= dist_urgent:
                dist_emergency = dist_urgent * 0.8
            
            # v28.0: Quadratic decay with configurable minimum (Apollo-style)
            scale = max(scale_floor, ratio ** 2)

            # Urgency-based LC mode (Apollo Scenario Manager)
            if distance_to_exit < dist_emergency:
                urgency_level = 'emergency'  # MUST_CHANGE mode
                # v27.29: Read relaxation factors from params
                long_relax = getattr(self.params, 'exit_long_relax_emerg', 0.5) if self.params else 0.5
                lat_relax = getattr(self.params, 'exit_lat_relax_emerg', 0.6) if self.params else 0.6
                mid_lc_long = max(5.0, 15.0 * scale * long_relax)
                mid_lc_lat = max(1.2, 2.0 * scale * lat_relax)
            elif distance_to_exit < dist_urgent:
                urgency_level = 'urgent'
                # v27.29: Read relaxation factors from params
                long_relax = getattr(self.params, 'exit_long_relax_urgent', 0.7) if self.params else 0.7
                lat_relax = getattr(self.params, 'exit_lat_relax_urgent', 0.75) if self.params else 0.75
                mid_lc_long = max(8.0, 15.0 * scale * long_relax)
                mid_lc_lat = max(1.5, 2.0 * scale * lat_relax)
            else:
                urgency_level = 'normal'
                mid_lc_long = max(10.0, 15.0 * scale)
                mid_lc_lat = max(1.8, 2.0 * scale)

        # Base values with speed-dependent adjustment (v28.0)
        base_front = 15.0      # v28.0: Increased from 10.0 (1.5s @ 10m/s)
        base_rear = 18.0       # v28.0: Increased from 12.0 (1.8s @ 10m/s)
        base_front_ttc = 2.5   # v28.0: Increased from 2.0
        base_rear_ttc = 3.5    # v28.0: Increased from 3.0

        return {
            'aggressiveness': 1.0 - scale,
            'urgency_level': urgency_level,
            'distance_to_exit': distance_to_exit,
            'front_gap': max(10.0, base_front * scale),    # Minimum 10m
            'rear_gap': max(12.0, base_rear * scale),      # Minimum 12m
            'front_ttc': max(1.5, base_front_ttc * scale), # Minimum 1.5s
            'rear_ttc': max(2.0, base_rear_ttc * scale),   # Minimum 2.0s
            'mid_lc_long': mid_lc_long,
            'mid_lc_lat': mid_lc_lat
        }
    
    # =========================================================================
    # v27.0 NEW: Mid-LC Safety Check
    # =========================================================================
    def check_mid_lc_safety(
        self,
        ego: 'Vehicle',
        vehicles: List['Vehicle'],
        cav_trajectories: Dict,
        t: float,
        prep_zone_len: float = 300.0,
        total_len: float = 600.0
    ) -> Tuple[bool, Optional['Vehicle'], Optional[float]]:
        """
        Check if LC-in-progress is still safe.

        v28.0 Improvements:
        - Relative velocity consideration (small rel_v → relax threshold)
        - Apollo-style 3-second trajectory prediction
        - Dynamic thresholds based on exit proximity

        Args:
            ego: Ego vehicle during LC
            vehicles: All vehicles in simulation
            cav_trajectories: V2V shared trajectories
            t: Current simulation time
            prep_zone_len: Length of preparation zone [m]
            total_len: Total track length [m]

        Returns:
            (is_safe, collision_vehicle, collision_time)
        """
        # Calculate dynamic thresholds
        thresholds: DynamicThresholds = self._compute_dynamic_thresholds(ego.x, prep_zone_len, total_len)
        long_threshold_base = thresholds['mid_lc_long']
        lat_threshold = thresholds['mid_lc_lat']

        if not getattr(ego, 'changing_lane', False):
            return True, None, None

        ego_lane_from = getattr(ego, 'lane_from', ego.lane)
        ego_lane_to = getattr(ego, 'lane_to', None)

        n_steps = int(self.mid_lc_prediction_horizon / self.dt)

        for step in range(n_steps):
            t_future = step * self.dt

            # Predict ego position
            ego_x_pred = ego.x + ego.v * t_future
            ego_d_pred = self._predict_lateral_position(ego, t_future, t)

            for other in vehicles:
                if other.id == ego.id or getattr(other, 'exited', False):
                    continue

                # Get predicted position
                other_traj = cav_trajectories.get(other.id)
                if other_traj and 'x' in other_traj and step < len(other_traj.get('x', [])):
                    other_x_pred = other_traj['x'][step]
                    other_d_pred = other_traj['d'][step] if 'd' in other_traj else other.d
                else:
                    other_x_pred = other.x + other.v * t_future
                    other_d_pred = other.d

                # Check collision
                long_gap = abs(ego_x_pred - other_x_pred)
                lat_gap = abs(ego_d_pred - other_d_pred)

                # v28.0: Adjust threshold based on relative velocity
                rel_v = abs(ego.v - other.v)
                if rel_v < 2.0:  # Small relative velocity (<2 m/s)
                    # Relax threshold by 30% (vehicles moving at similar speeds)
                    long_threshold = long_threshold_base * 0.7
                elif rel_v < 5.0:  # Moderate relative velocity
                    # Relax threshold by 15%
                    long_threshold = long_threshold_base * 0.85
                else:
                    # High relative velocity - use full threshold
                    long_threshold = long_threshold_base

                if long_gap < long_threshold and lat_gap < lat_threshold:
                    return False, other, t_future

                # v28.0: Counter-flow LC check with velocity-adjusted threshold
                other_changing = getattr(other, 'changing_lane', False)
                other_target = getattr(other, 'target_lane', None)
                if other_changing and other_target == ego_lane_from:
                    # Counter-flow: use stricter threshold (20m base, but adjust for rel_v)
                    counter_threshold = max(15.0, 20.0 - rel_v * 1.0)  # Reduce by 1m per 1m/s rel_v
                    if long_gap < counter_threshold:
                        return False, other, t_future

        return True, None, None
    
    def _predict_lateral_position(
        self,
        ego: 'Vehicle',
        t_future: float,
        t_current: float
    ) -> float:
        """Predict lateral position during LC."""
        if not getattr(ego, 'changing_lane', False):
            return ego.d
        
        from .coordinate_transform import LANE_OFFSETS
        
        lc_start_time = getattr(ego, 'lc_start_time', t_current)
        lc_duration = 3.0  # Default LC duration
        
        elapsed = (t_current - lc_start_time) + t_future
        progress = np.clip(elapsed / lc_duration, 0.0, 1.0)
        
        d_from = LANE_OFFSETS.get(getattr(ego, 'lane_from', ego.lane), 0.0)
        d_to = LANE_OFFSETS.get(getattr(ego, 'lane_to', ego.lane), 0.0)
        
        ease_factor = 0.5 * (1.0 - np.cos(np.pi * progress))
        return d_from + (d_to - d_from) * ease_factor
    
    # =========================================================================
    # v27.0 NEW: Safety State Evaluation
    # =========================================================================
    def evaluate_safety_state(
        self,
        ego: 'Vehicle',
        front_vehicle: Optional['Vehicle'],
        cav_trajectories: Dict,
        v_desired: float = 20.0
    ) -> SafetyEvaluation:
        """
        Evaluate safety state and return recommended action.
        
        5-stage graduated response:
        - SAFE: Normal operation
        - CAUTION: RSS×2.0 - Match front vehicle speed
        - WARNING: RSS×1.5 - Active slow down to 90% of front
        - CRITICAL: RSS×1.2 - Emergency decel at -4.0 m/s²
        - AEB: RSS×0.8 - Maximum braking at -6.0 m/s²
        
        ★ CRITICAL ADDITION (v27.19): Apollo準拠の横方向距離チェック
        - 横方向距離 > 2.0m の場合、AEB対象外として扱う
        - 理由: 異なるレーンの車両に対してAEBを発動すべきでない
        - Reference: Apollo speed_decider.cc - IsOnReferenceLine()
        
        Args:
            ego: Ego vehicle
            front_vehicle: Front vehicle (if any)
            cav_trajectories: V2V shared trajectories
            v_desired: Desired velocity [m/s]
            
        Returns:
            SafetyEvaluation with state and recommended action
        """
        if front_vehicle is None:
            return SafetyEvaluation(
                state=SafetyState.SAFE,
                recommended_ax=0.0,
                recommended_v_ref=v_desired,
                gap=float('inf'),
                ttc=None,
                rss_distance=0.0,
                reason="No front vehicle"
            )
        
        # ================================================================
        # v27.19 NEW: Lateral Distance Check (Apollo Alignment)
        # ================================================================
        # Check if front vehicle is laterally close enough to be a threat.
        # If lateral distance > 2.0m, treat as not on same path.
        # This prevents AEB triggering for vehicles in adjacent lanes.
        # ================================================================
        ego_d = getattr(ego, 'd', 0.0)
        front_d = getattr(front_vehicle, 'd', 0.0)
        lateral_distance = abs(front_d - ego_d)
        
        AEB_LATERAL_THRESHOLD = 2.0  # [m] Apollo standard: ~0.5 lane width
        
        if lateral_distance > AEB_LATERAL_THRESHOLD:
            # Front vehicle is laterally too far - not a direct threat
            return SafetyEvaluation(
                state=SafetyState.SAFE,
                recommended_ax=0.0,
                recommended_v_ref=v_desired,
                gap=float('inf'),
                ttc=None,
                rss_distance=0.0,
                reason=f"Lateral separation: {lateral_distance:.1f}m > {AEB_LATERAL_THRESHOLD}m"
            )
        # ================================================================
        
        # Compute gap and TTC
        gap = front_vehicle.x - ego.x - L_VEHICLE
        rel_v = ego.v - front_vehicle.v
        ttc = gap / rel_v if rel_v > 0.1 else None
        
        # Compute RSS distance
        is_front_cav = not getattr(front_vehicle, 'is_hdv', False)
        has_traj = front_vehicle.id in cav_trajectories
        
        if is_front_cav and has_traj:
            t_react = RSS_REACTION_TIME_CAV
        else:
            t_react = RSS_REACTION_TIME
        
        v_ego = ego.v
        v_front = front_vehicle.v
        rss_dist = (v_ego * t_react + 
                    (v_ego ** 2) / (2 * A_MAX_BRAKE) - 
                    (v_front ** 2) / (2 * A_MAX_BRAKE) + 
                    RSS_MIN_GAP)
        rss_dist = max(rss_dist, RSS_MIN_GAP)
        
        # Determine safety state
        if gap < 0:
            # Physical overlap - immediate AEB
            return SafetyEvaluation(
                state=SafetyState.AEB,
                recommended_ax=-A_MAX_BRAKE,
                recommended_v_ref=0.0,
                gap=gap,
                ttc=ttc,
                rss_distance=rss_dist,
                reason=f"OVERLAP: gap={gap:.1f}m"
            )
        
        if gap < rss_dist * self.aeb_rss_factor or (ttc is not None and ttc < 0.8):  # v27.6: TTC < 0.8s for AEB (was 1.5)
            return SafetyEvaluation(
                state=SafetyState.AEB,
                recommended_ax=-A_MAX_BRAKE,
                recommended_v_ref=0.0,
                gap=gap,
                ttc=ttc,
                rss_distance=rss_dist,
                reason=f"AEB: gap={gap:.1f}m < RSS×{self.aeb_rss_factor:.2f} or TTC<1.5s"
            )
        
        if gap < rss_dist * self.critical_rss_factor or (ttc is not None and ttc < 1.5):  # v27.6: TTC < 1.5s for CRITICAL
            # v27.5 FIX: CRITICAL uses -4.0 m/s² (not -6.0 which is AEB-only)
            return SafetyEvaluation(
                state=SafetyState.CRITICAL,
                recommended_ax=-4.0,  # Strong but not emergency (was -A_MAX_BRAKE)
                recommended_v_ref=max(v_front * 0.8, 3.0),  # Stronger speed reduction
                gap=gap,
                ttc=ttc,
                rss_distance=rss_dist,
                reason=f"CRITICAL: gap={gap:.1f}m < RSS×{self.critical_rss_factor:.2f} or TTC<1.5s"
            )
        
        if gap < rss_dist * self.warning_rss_factor or (ttc is not None and ttc < 2.5):  # v27.6: TTC < 2.5s for WARNING (was 3.0)
            return SafetyEvaluation(
                state=SafetyState.WARNING,
                recommended_ax=-A_COMFORT_BRAKE,
                recommended_v_ref=max(v_front * 0.9, 5.0),
                gap=gap,
                ttc=ttc,
                rss_distance=rss_dist,
                reason=f"WARNING: gap={gap:.1f}m < RSS×{self.warning_rss_factor:.2f} or TTC<3.0s"
            )
        
        if gap < rss_dist * self.early_warning_rss_factor or (ttc is not None and ttc < 4.0):  # v27.6: TTC < 4.0s for CAUTION (was 5.0)
            return SafetyEvaluation(
                state=SafetyState.CAUTION,
                recommended_ax=0.0,
                recommended_v_ref=v_front,  # Match front speed
                gap=gap,
                ttc=ttc,
                rss_distance=rss_dist,
                reason=f"CAUTION: gap={gap:.1f}m < RSS×{self.early_warning_rss_factor:.2f} or TTC<5.0s"
            )
        
        return SafetyEvaluation(
            state=SafetyState.SAFE,
            recommended_ax=0.0,
            recommended_v_ref=v_desired,
            gap=gap,
            ttc=ttc,
            rss_distance=rss_dist,
            reason="Safe"
        )
    
    # =========================================================================
    # Existing methods (unchanged)
    # =========================================================================
    def compute_safe_boundaries(
        self,
        ego: 'Vehicle',
        vehicles: List['Vehicle'],
        cav_trajectories: Dict[int, List[Tuple[float, float, float, float]]],
        t: float,
        horizon: Optional[int] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute unified safe corridor for QP.
        """
        N = horizon if horizon is not None else self.horizon
        dt = self.dt
        
        s_lower = np.full(N, -np.inf)
        s_upper = np.full(N, np.inf)
        
        ego_s = ego.x
        ego_v = ego.v
        ego_lane = ego.lane
        ego_target = getattr(ego, 'target_lane', None)
        
        relevant_lanes = {ego_lane}
        if ego_target:
            relevant_lanes.add(ego_target)
        
        for other in vehicles:
            if other.id == ego.id or getattr(other, 'exited', False):
                continue
                
            other_lane = other.lane
            other_target = getattr(other, 'target_lane', None)
            other_lanes = {other_lane}
            if other_target:
                other_lanes.add(other_target)
            
            if not relevant_lanes.intersection(other_lanes):
                continue
            
            is_cav = not getattr(other, 'is_hdv', False)
            has_traj = other.id in cav_trajectories and len(cav_trajectories[other.id]) > 0
            
            other_s_traj = self._predict_trajectory(other, cav_trajectories, N, dt, t)
            
            is_front = other.x > ego_s
            
            if is_cav and has_traj:
                margin = self.front_margin_cav
            else:
                margin = self.front_margin_base
            
            if is_front:
                for k in range(N):
                    safe_s = other_s_traj[k] - margin - L_VEHICLE
                    s_upper[k] = min(s_upper[k], safe_s)
            else:
                for k in range(N):
                    safe_s = other_s_traj[k] + self.rear_margin + L_VEHICLE
                    s_lower[k] = max(s_lower[k], safe_s)
        
        s_lower, s_upper = self._enforce_kinematic_limits(
            s_lower, s_upper, ego_s, ego_v, N, dt
        )
        
        return s_lower, s_upper
    
    def _predict_trajectory(
        self,
        vehicle: 'Vehicle',
        cav_trajectories: Dict,
        N: int,
        dt: float,
        t: float
    ) -> np.ndarray:
        """Predict vehicle trajectory over horizon."""
        traj = np.zeros(N)
        
        is_cav = not getattr(vehicle, 'is_hdv', False)
        shared_traj = cav_trajectories.get(vehicle.id, [])
        
        if is_cav and len(shared_traj) > 0:
            for k in range(N):
                t_target = k * dt
                traj[k] = self._interpolate_trajectory(shared_traj, t_target)
        else:
            s0 = vehicle.x
            v0 = vehicle.v
            for k in range(N):
                t_k = k * dt
                traj[k] = s0 + v0 * t_k
        
        return traj
    
    def _interpolate_trajectory(
        self,
        traj: List[Tuple[float, float, float, float]],
        t_target: float
    ) -> float:
        """Interpolate position from trajectory at target time."""
        if len(traj) == 0:
            return 0.0
        
        if t_target <= traj[0][0]:
            return traj[0][1]
        if t_target >= traj[-1][0]:
            return traj[-1][1]
        
        for i in range(len(traj) - 1):
            t1, s1 = traj[i][0], traj[i][1]
            t2, s2 = traj[i + 1][0], traj[i + 1][1]
            if t1 <= t_target <= t2:
                alpha = (t_target - t1) / (t2 - t1) if t2 > t1 else 0.0
                return s1 + alpha * (s2 - s1)
        
        return traj[-1][1]
    
    def _enforce_kinematic_limits(
        self,
        s_lower: np.ndarray,
        s_upper: np.ndarray,
        s0: float,
        v0: float,
        N: int,
        dt: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Ensure bounds are kinematically feasible."""
        v_max = 25.0
        a_max = 3.0
        a_min = -6.0
        
        s_max_reachable = np.zeros(N)
        v = v0
        s = s0
        for k in range(N):
            v = min(v + a_max * dt, v_max)
            s = s + v * dt
            s_max_reachable[k] = s
        
        s_min_reachable = np.zeros(N)
        v = v0
        s = s0
        for k in range(N):
            v = max(v + a_min * dt, 0.0)
            s = s + v * dt
            s_min_reachable[k] = s
        
        s_upper = np.minimum(s_upper, s_max_reachable)
        s_lower = np.maximum(s_lower, s_min_reachable)
        
        for k in range(N):
            if s_lower[k] > s_upper[k]:
                mid = (s_lower[k] + s_upper[k]) / 2
                s_lower[k] = mid - 1.0
                s_upper[k] = mid + 1.0
        
        return s_lower, s_upper
    
    def check_emergency_aeb(
        self,
        ego: 'Vehicle',
        front_vehicle: Optional['Vehicle'],
        gap: float
    ) -> Tuple[bool, float]:
        """
        Fallback AEB check - should RARELY trigger if QP works correctly.
        """
        if front_vehicle is None or gap > 10.0:
            return False, 0.0
        
        if gap < 1.0:
            return True, -A_MAX_BRAKE
        
        rel_v = ego.v - front_vehicle.v
        if rel_v > 0.1 and gap > 0:
            ttc = gap / rel_v
            if ttc < 0.8:
                return True, -A_MAX_BRAKE
        
        return False, 0.0


# Singleton for easy access
_safety_manager: Optional[ApolloSafetyManager] = None

def get_safety_manager(params: Optional['IntegratedZoneParameters'] = None) -> ApolloSafetyManager:
    """Get or create global ApolloSafetyManager instance."""
    global _safety_manager
    if _safety_manager is None:
        _safety_manager = ApolloSafetyManager(params=params)
    elif params is not None:
        # Update existing manager with new params (useful for batch testing)
        _safety_manager.lc_min_front_gap = params.lc_aggressive_base_front
        _safety_manager.lc_min_rear_gap = params.lc_aggressive_base_rear
        _safety_manager.lc_min_front_ttc = params.lc_aggressive_base_front_ttc
        _safety_manager.lc_min_rear_ttc = params.lc_aggressive_base_rear_ttc
        
    return _safety_manager
