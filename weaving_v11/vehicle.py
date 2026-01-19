# -*- coding: utf-8 -*-
"""
weaving_v11/vehicle.py (ver.11.0 / 2025-12-17)

車両データクラスと関連モデル:
- Vehicle: 緊急度と速度プロファイル属性を持つメイン車両データクラス
- SafetyMetrics: 安全評価メトリクス
- ComputationMetrics: 計算性能メトリクス
- IDMModel: インテリジェントドライバーモデル実装
- SafetyAnalyzer: 安全分析ユーティリティ
- ObservedDistributionModel: 観測LC分布モデル
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, TYPE_CHECKING
import numpy as np
from scipy.interpolate import interp1d
import sys
from .safety import SAFETY

if TYPE_CHECKING:
    pass

# Global flag for debug output (will be set by importing module)
ENABLE_DEBUG_OUTPUT = True


@dataclass
class SafetyMetrics:
    """Safety evaluation metrics for simulation"""

    collision_count: int = 0
    near_miss_count: int = 0
    critical_ttc_count: int = 0
    collision_details: List[Dict] = field(default_factory=list)
    min_headway: float = float('inf')
    mean_headway: float = 0.0
    std_headway: float = 0.0
    headway_samples: List[float] = field(default_factory=list)
    min_ttc: float = float('inf')
    mean_ttc: float = 0.0
    std_ttc: float = 0.0
    ttc_samples: List[float] = field(default_factory=list)
    acceleration_violations: int = 0
    velocity_violations: int = 0
    extreme_risk_count: int = 0
    high_risk_count: int = 0
    moderate_risk_count: int = 0
    # Per-vehicle breakdown for diagnostics
    accel_violations_by_vehicle: Dict[int, int] = field(default_factory=dict)


@dataclass
class ComputationMetrics:
    """
    Computation performance metrics for controllers (v11.0 Updated)

    New metrics for Urgency-based Rolling Horizon Control:
    - level1_update_count: Number of urgency updates
    - qp_success_count: Number of successful QP solves
    - qp_failure_count: Number of failed QP solves
    - fallback_idm_count: Number of IDM fallback executions
    """

    # Level 1 metrics (Urgency Planner)
    level1_update_count: int = 0
    level1_prep_time: float = 0.0
    level1_prep_mipgap: float = 0.0
    level1_prep_feasible_pairs: int = 0
    level1_prep_success: bool = False

    level1_weave_time: float = 0.0
    level1_weave_mipgap: float = 0.0
    level1_weave_feasible_pairs: int = 0
    level1_weave_success: bool = False

    # Level 2 metrics (QP Controller)
    level2_update_count: int = 0
    qp_success_count: int = 0
    qp_failure_count: int = 0
    fallback_idm_count: int = 0

    level2a_time_left: float = 0.0
    level2a_iter_left: int = 0
    level2_unified_success: bool = False

    total_time: float = 0.0
    success: bool = False


class SafetyAnalyzer:
    """
    Safety analysis utilities for vehicle interactions

    Evaluates:
    - Headway (inter-vehicle spacing)
    - Time-to-Collision (TTC)
    - Collision detection
    - Near-miss events
    - Constraint violations
    """

    def __init__(self,
                 collision_threshold: float = 0.1,
                 near_miss_threshold: float = 2.0,
                 critical_ttc_threshold: float = 1.5):
        """
        Initialize safety analyzer

        Args:
            collision_threshold: Minimum headway for collision [m]
            near_miss_threshold: Minimum headway for near-miss [m]
            critical_ttc_threshold: Critical TTC threshold [s]
        """
        self.collision_threshold = collision_threshold
        self.near_miss_threshold = near_miss_threshold
        self.critical_ttc_threshold = critical_ttc_threshold
        self.metrics = SafetyMetrics()
        self.current_time = 0.0

    def set_current_time(self, t: float):
        """Set current simulation time"""
        self.current_time = t

    @staticmethod
    def compute_ttc(x_rear: float, x_front: float,
                   v_rear: float, v_front: float,
                   L_vehicle: float = 5.0) -> Optional[float]:
        """
        Compute Time-to-Collision (TTC)

        TTC represents the time remaining until the rear vehicle's front bumper
        collides with the front vehicle's rear bumper, assuming constant velocities.

        【視点による理解】
        - 後続車（rear）から見ると、前方車（front）の「後端」が見える
        - TTCは、後続車の「前端」が前方車の「後端」に到達するまでの時間

        【計算】
        headway = 前方車の後端 - 後続車の前端
                = (x_front - L_vehicle/2) - (x_rear + L_vehicle/2)
                = x_front - x_rear - L_vehicle

        TTC = headway / relative_velocity
            where relative_velocity = v_rear - v_front

        Args:
            x_rear: Rear vehicle center position [m]
            x_front: Front vehicle center position [m]
            v_rear: Rear vehicle velocity [m/s]
            v_front: Front vehicle velocity [m/s]
            L_vehicle: Vehicle length [m] (default: 5.0m)

        Returns:
            TTC in seconds, or None if no collision risk (rear vehicle slower or equal)
        """
        # Calculate physical headway (front bumper to rear bumper distance)
        headway = x_front - x_rear - L_vehicle
        relative_velocity = v_rear - v_front

        # No collision risk if rear is slower or equal
        if relative_velocity <= 0:
            return None

        # TTC = time until front bumper hits rear bumper
        ttc = headway / relative_velocity
        return ttc if ttc > 0 else None

    def analyze_vehicle_pair(self, v_rear: 'Vehicle', v_front: 'Vehicle',
                            L_vehicle: float = 5.0) -> bool:
        """
        Analyze safety metrics for a vehicle pair

        Args:
            v_rear: Rear vehicle
            v_front: Front vehicle
            L_vehicle: Vehicle length [m]
            
        Returns:
            bool: True if collision detected, False otherwise (v13.4 modification)
        """
        # --- modified (Final Fix v11.7 / 2025-12-18): Physics-based strict collision detection ---
        # Previous: Lane-based logic caused false positives during overtaking
        # Current: Requires BOTH longitudinal AND lateral physical overlap
        # ---------------------------------------------------------------------------------

        # 1. Longitudinal Overlap Check (headway-based)
        # ---------------------------------------------------------------------------------
        # headway = gap between rear vehicle's front bumper and front vehicle's rear bumper
        #
        # 【視点による理解】
        # - 後続車（v_rear）から見ると、前方車（v_front）の「後端（ケツ）」が見える
        # - headway = 前方車の後端 - 後続車の前端
        #
        # 【計算】
        #   前方車の後端 = v_front.x - L_vehicle/2  (中心位置 - 半車長)
        #   後続車の前端 = v_rear.x + L_vehicle/2   (中心位置 + 半車長)
        #
        #   headway = (v_front.x - L/2) - (v_rear.x + L/2)
        #           = v_front.x - v_rear.x - L_vehicle
        # ---------------------------------------------------------------------------------
        headway = v_front.x - v_rear.x - L_vehicle
        is_longitudinal_collision = (headway < 0.1)  # 0.1m tolerance for numerical stability

        # 2. Lateral Overlap Check (d-coordinate based)
        # Vehicle width ~1.8-2.0m. Center-to-center distance < 1.8m means contact
        rear_d = getattr(v_rear, 'd', 0.0)
        front_d = getattr(v_front, 'd', 0.0)
        lateral_dist = abs(rear_d - front_d)
        # Use centralized vehicle width from SafetyConstants for consistency
        is_lateral_collision = (lateral_dist < SAFETY.VEHICLE_WIDTH)

        # Physical collision requires BOTH conditions
        # This eliminates false positives during lane changes and overtaking
        is_collision = is_longitudinal_collision and is_lateral_collision
        # ---------------------------------------------------------------------------------

        # Record headway
        self.metrics.headway_samples.append(headway)
        if headway < self.metrics.min_headway:
            self.metrics.min_headway = headway

        # Collision detection (strict 2D check)
        if is_collision:
            self.metrics.collision_count += 1

            # Record collision details
            collision_detail = {
                'time': self.current_time,
                'rear_vehicle_id': v_rear.id,
                'front_vehicle_id': v_front.id,
                'rear_lane': v_rear.lane,
                'front_lane': v_front.lane,
                'rear_x': v_rear.x,
                'front_x': v_front.x,
                'rear_v': v_rear.v,
                'front_v': v_front.v,
                'headway': headway,
                'relative_velocity': v_rear.v - v_front.v,
                'rear_changing_lane': getattr(v_rear, 'changing_lane', False),
                'front_changing_lane': getattr(v_front, 'changing_lane', False),
            }
            self.metrics.collision_details.append(collision_detail)

            # ====================================================================
            # Collision Detection Logging
            # ====================================================================
            print(f"\n{'='*80}")
            print(f"[COLLISION] Detected @ t={self.current_time:.2f}s")
            print(f"  Rear V#{v_rear.id} → Front V#{v_front.id}")
            print(f"  Position: rear_x={v_rear.x:.2f}m, front_x={v_front.x:.2f}m")
            print(f"  Velocity: rear_v={v_rear.v:.2f}m/s, front_v={v_front.v:.2f}m/s")
            print(f"  Headway: {headway:.3f}m (< 0.1m threshold)")
            print(f"  Lateral distance: {lateral_dist:.2f}m (< 1.8m threshold)")
            print(f"  Relative velocity: {v_rear.v - v_front.v:+.2f}m/s")
            print(f"  Lane: rear={v_rear.lane}, front={v_front.lane}")
            print(f"  LC status: rear_changing={getattr(v_rear, 'changing_lane', False)}, "
                  f"front_changing={getattr(v_front, 'changing_lane', False)}")
            print(f"  AEB status: rear_aeb={getattr(v_rear, 'aeb_active', False)}, "
                  f"front_aeb={getattr(v_front, 'aeb_active', False)}")
            print(f"{'='*80}\n")
            # ====================================================================

            # --- modified (Final Fix v11.7 / 2025-12-18): Enhanced collision debug logging ---
            # Log collision with complete physical analysis
            print(f"\n{'='*80}")
            print(f"[COLLISION DETECTED] t={self.current_time:.2f}s")
            print(f"{'='*80}")
            print(f"  🔴 Rear Vehicle:  ID={v_rear.id:3d}, Lane={v_rear.lane:7s}")
            print(f"     Position: x={v_rear.x:7.2f}m, d={rear_d:+6.2f}m (Frenet)")
            print(f"     Velocity: v={v_rear.v:5.2f}m/s, LC={getattr(v_rear, 'changing_lane', False)}")
            if getattr(v_rear, 'changing_lane', False):
                print(f"     LC Info:  {getattr(v_rear, 'lane_from', '?')} → {getattr(v_rear, 'lane_to', '?')}")

            print(f"  🔴 Front Vehicle: ID={v_front.id:3d}, Lane={v_front.lane:7s}")
            print(f"     Position: x={v_front.x:7.2f}m, d={front_d:+6.2f}m (Frenet)")
            print(f"     Velocity: v={v_front.v:5.2f}m/s, LC={getattr(v_front, 'changing_lane', False)}")
            if getattr(v_front, 'changing_lane', False):
                print(f"     LC Info:  {getattr(v_front, 'lane_from', '?')} → {getattr(v_front, 'lane_to', '?')}")

            print("  📏 Physical Overlap Analysis:")
            print(f"     Longitudinal: headway={headway:.3f}m < 0.1m (collision threshold) ✓")
            print(f"     Lateral:      distance={lateral_dist:.2f}m < 1.8m (vehicle width) ✓")
            print(f"     Relative velocity: {v_rear.v - v_front.v:+.2f}m/s")
            print("  ⚠️  PHYSICAL CONTACT CONFIRMED - Both overlap conditions satisfied!")
            print(f"{'='*80}\n")
            # -----------------------------------------------------------------------
            sys.stdout.flush()

        # Risk level classification
        if headway < 1.0:
            self.metrics.extreme_risk_count += 1
        elif headway < 2.0:
            self.metrics.high_risk_count += 1
            if headway < self.near_miss_threshold:
                self.metrics.near_miss_count += 1
        elif headway < 5.0:
            self.metrics.moderate_risk_count += 1

        # TTC computation
        ttc = self.compute_ttc(v_rear.x, v_front.x, v_rear.v, v_front.v, L_vehicle)
        if ttc is not None:
            self.metrics.ttc_samples.append(ttc)
            if ttc < self.metrics.min_ttc:
                self.metrics.min_ttc = ttc
            if ttc < self.critical_ttc_threshold:
                self.metrics.critical_ttc_count += 1
        
        # v13.4: Return collision status for collision recovery handling
        return is_collision

    def check_constraint_violations(self, vehicle: 'Vehicle',
                                   v_min: float = 5.0,
                                   v_max: float = 30.0,
                                   a_min: float = -3.0,
                                   a_max: float = 2.0,
                                   dt: float = 0.1) -> None:
        """
        Check for velocity and acceleration constraint violations

        Args:
            vehicle: Vehicle to check
            v_min: Minimum allowed velocity [m/s]
            v_max: Maximum allowed velocity [m/s]
            a_min: Minimum allowed acceleration [m/s²]
            a_max: Maximum allowed acceleration [m/s²]
            dt: Time step [s]
        """
        # Skip once if flagged (e.g., collision recovery forced velocity reset)
        if getattr(vehicle, 'skip_next_accel_check', False):
            try:
                setattr(vehicle, 'skip_next_accel_check', False)
            except Exception:
                pass
            # Align previous velocity to current to avoid artificial spike
            vehicle.v_prev = vehicle.v
            return

        # Velocity constraint violation
        if vehicle.v < v_min or vehicle.v > v_max:
            self.metrics.velocity_violations += 1

        # Acceleration constraint violation (computed from velocity difference)
        if hasattr(vehicle, 'v_prev'):
            a = (vehicle.v - vehicle.v_prev) / dt
            if a < a_min or a > a_max:
                self.metrics.acceleration_violations += 1
                # Per-vehicle count
                vid = getattr(vehicle, 'id', None)
                if vid is not None:
                    self.metrics.accel_violations_by_vehicle[vid] = (
                        self.metrics.accel_violations_by_vehicle.get(vid, 0) + 1
                    )

    def finalize_metrics(self) -> SafetyMetrics:
        """Finalize and compute aggregate statistics"""
        # Headway statistics
        if len(self.metrics.headway_samples) > 0:
            self.metrics.mean_headway = float(np.mean(self.metrics.headway_samples))
            self.metrics.std_headway = float(np.std(self.metrics.headway_samples))

        # TTC statistics
        if len(self.metrics.ttc_samples) > 0:
            self.metrics.mean_ttc = float(np.mean(self.metrics.ttc_samples))
            self.metrics.std_ttc = float(np.std(self.metrics.ttc_samples))

        return self.metrics

    def get_metrics_dict(self) -> Dict:
        """Get metrics as dictionary"""
        return {
            'collision_count': self.metrics.collision_count,
            'near_miss_count': self.metrics.near_miss_count,
            'critical_ttc_count': self.metrics.critical_ttc_count,
            'min_headway': self.metrics.min_headway,
            'mean_headway': self.metrics.mean_headway,
            'std_headway': self.metrics.std_headway,
            'min_ttc': self.metrics.min_ttc,
            'mean_ttc': self.metrics.mean_ttc,
            'std_ttc': self.metrics.std_ttc,
            'acceleration_violations': self.metrics.acceleration_violations,
            'velocity_violations': self.metrics.velocity_violations,
            'extreme_risk_count': self.metrics.extreme_risk_count,
            'high_risk_count': self.metrics.high_risk_count,
            'moderate_risk_count': self.metrics.moderate_risk_count,
            'accel_violations_by_vehicle': dict(self.metrics.accel_violations_by_vehicle),
        }

    def analyze_collision_causes(self) -> Dict:
        """Analyze causes of collisions"""
        analysis = {
            'total_collisions': len(self.metrics.collision_details),
            'lc_related': 0,  # LC-involved collisions
            'same_lane': 0,   # Same-lane collisions
            'high_speed_diff': 0,  # High speed difference (>5m/s)
            'rear_faster': 0,  # Rear vehicle faster
            'both_lc': 0,     # Both vehicles changing lanes
            'collisions_by_lane': {},
            'collisions_by_time': [],
        }

        for detail in self.metrics.collision_details:
            # LC-related
            if detail['rear_changing_lane'] or detail['front_changing_lane']:
                analysis['lc_related'] += 1
            if detail['rear_changing_lane'] and detail['front_changing_lane']:
                analysis['both_lc'] += 1

            # Same lane
            if detail['rear_lane'] == detail['front_lane']:
                analysis['same_lane'] += 1

            # High speed difference
            if abs(detail['relative_velocity']) > 5.0:
                analysis['high_speed_diff'] += 1

            # Rear faster
            if detail['relative_velocity'] > 0.01:
                analysis['rear_faster'] += 1

            # By lane
            rear_lane = detail['rear_lane']
            if rear_lane not in analysis['collisions_by_lane']:
                analysis['collisions_by_lane'][rear_lane] = 0
            analysis['collisions_by_lane'][rear_lane] += 1

            # Time series
            analysis['collisions_by_time'].append({
                'time': detail['time'],
                'vehicles': f"V{detail['rear_vehicle_id']} -> V{detail['front_vehicle_id']}",
                'headway': detail['headway'],
                'rel_vel': detail['relative_velocity']
            })

        return analysis


class ObservedDistributionModel:
    """Observed lane change distribution model (S-curve CDF)"""

    def __init__(self):
        # S-curve cumulative distribution (0m: 0%, 350m: 100%)
        self.x_data = np.array([0, 50, 100, 150, 200, 350])
        self.F_data = np.array([0, 60, 80, 90, 95, 100]) / 100.0

        # Linear interpolation
        self.cdf_interp = interp1d(
            self.x_data, self.F_data,
            kind='linear',
            bounds_error=False
        )

    def sample_lc_position(self, x_start: float = 0.0, x_end: float = 400.0) -> float:
        """Sample LC position from observed distribution"""
        u = np.random.uniform(0, 1)
        u = np.clip(u, 0.0, 1.0)

        # Inverse CDF (binary search)
        x_candidates = np.linspace(x_start, x_end, 1000)
        F_candidates = self.cdf_interp(x_candidates)

        # Find closest match
        idx = np.argmin(np.abs(F_candidates - u))
        x_lc = x_candidates[idx]

        return x_lc

    def get_time_distribution_std(self) -> float:
        """Get time distribution standard deviation"""
        return 2.1  # σ_t = 30m / 14m/s ≈ 2.1s


class IDMModel:
    """Intelligent Driver Model (IDM) for car-following"""

    def __init__(self,
                 v0: float = 20.0,
                 T: float = 1.5,
                 s0: float = 2.0,
                 a: float = 1.0,
                 b: float = 1.5,
                 delta: int = 4):
        """
        Initialize IDM model

        Args:
            v0: Desired velocity [m/s]
            T: Safe time headway [s]
            s0: Minimum spacing [m]
            a: Maximum acceleration [m/s²]
            b: Comfortable deceleration [m/s²]
            delta: Acceleration exponent
        """
        self.v0 = v0
        self.T = T
        self.s0 = s0
        self.a = a
        self.b = b
        self.delta = delta

    def desired_gap(self, v: float, delta_v: float) -> float:
        """Compute desired gap s*"""
        braking_term = (v * delta_v) / (2 * np.sqrt(self.a * self.b))
        s_star = self.s0 + v * self.T + braking_term
        return max(s_star, self.s0)

    def acceleration(self, v: float, s: float, v_leader: float) -> float:
        """
        Compute IDM acceleration

        Args:
            v: Current velocity [m/s]
            s: Gap to leader [m]
            v_leader: Leader velocity [m/s]

        Returns:
            Acceleration [m/s²]
        """
        # Free flow term
        free_term = 1 - (v / self.v0) ** self.delta

        # Interaction term
        delta_v = v - v_leader
        s_star = self.desired_gap(v, delta_v)

        if s < 0.1:
            s = 0.1

        interaction_term = (s_star / s) ** 2

        acc = self.a * (free_term - interaction_term)
        acc = np.clip(acc, -self.b * 2, self.a)

        return acc


@dataclass
class Vehicle:
    """
    Main vehicle dataclass (v11.0 Updated)

    New attributes for Urgency-based Rolling Horizon Control:
    - urgency: Urgency score [0, 1] computed by UrgencyPlanner
    - urgency_state: Full UrgencyState object from planner
    - velocity_profile: Velocity trajectory from QP controller
    - velocity_profile_time: Timestamp of velocity profile generation
    """

    # Required fields (no defaults)
    id: int
    lane: str

    # Position and kinematics (with defaults)
    x: float = 0.0
    v: float = 14.0
    spawn_time: float = 0.0

    # Previous velocity (for acceleration constraint checking)
    v_prev: float = 14.0

    # ========================================================================
    # Frenet Coordinates (v11.3): Physical lateral position tracking
    # ========================================================================
    # These attributes enable true Frenet-based obstacle detection and control
    # - d: Lateral offset in Frenet frame [m] (positive = left of centerline)
    # - y: Global Y coordinate [m] (for straight road: y = d)
    # Updated continuously during lane changes for smooth physics simulation
    # ========================================================================
    d: float = 0.0  # Frenet lateral offset [m]
    y: float = 0.0  # Global Y coordinate [m]

    # IDM parameters
    v_desired: float = 25.0
    T_idm: float = 1.5

    side: str = "left"
    destination_area: str = "LEFT"
    in_destination_area: bool = False

    target_lane_prep: Optional[str] = None
    target_lane_weave: Optional[str] = None

    needs_initial_lc: bool = False
    can_density_adjust: bool = False

    target_lane: Optional[str] = None
    lc_scheduled: bool = False
    scheduled_cell: int = -1
    scheduled_kappa: int = -1
    scheduled_time: float = -1.0
    target_position: float = -1.0

    # Consistency tracking
    initial_schedule: Optional[Tuple[int, float]] = None

    lc_started: bool = False
    lc_start_time: float = -1.0
    lc_start_position: float = 0.0
    lc_completed: bool = False
    lc_end_time: float = -1.0
    lc_end_position: float = 0.0

    # Lane change state tracking
    changing_lane: bool = False
    lane_from: Optional[str] = None
    lane_to: Optional[str] = None

    lc_count_prep: int = 0
    lc_count_weave: int = 0
    lc_count_total: int = 0

    entered_weave_zone: bool = False
    weave_zone_entry_time: float = -1.0
    lc_completed_prep: bool = False
    lc_completed_weave: bool = False
    lc_counted_in_stats: bool = False

    exited: bool = False
    x_exit: float = -1.0

    history_time: List[float] = field(default_factory=list)
    history_x: List[float] = field(default_factory=list)
    history_v: List[float] = field(default_factory=list)
    history_lane: List[str] = field(default_factory=list)

    # ========================================================================
    # v11.0 NEW ATTRIBUTES: Controller Interface
    # ========================================================================
    ax: float = 0.0  # Acceleration command from controller [m/s²]
    steering: float = 0.0  # Steering command from controller [rad]

    # Urgency-based planning
    urgency: float = 0.0  # Urgency score [0, 1]
    urgency_state: Optional[object] = None  # Full UrgencyState object

    # Velocity profile from QP controller
    velocity_profile: List[float] = field(default_factory=list)
    velocity_profile_time: float = -1.0

    # --- v11.8 NEW: AEB Latch Flag ---
    # Latches AEB state to prevent premature release during emergency braking
    # Set by controller when AEB triggers, cleared when safe distance restored
    aeb_active: bool = False

    # Previous state tracking for trajectory reset (set by controllers)
    _prev_changing_lane: bool = False
    _prev_aeb_active: bool = False

    # ========================================================================
    # v12.0 NEW: CAV Cooperative Control (Trajectory Sharing)
    # ========================================================================
    # Paradigm shift: "Prediction-Based" → "Agreement-Based"
    # - CAVs share "committed future trajectories" computed by QP solver
    # - Other CAVs use these trajectories (not predictions) for planning
    # - Eliminates uncertainty and enables minimal safety buffers
    # - Uses "1-step-old" trajectory to avoid circular dependencies
    # ========================================================================

    # Predicted trajectory: List of (t_abs, s, v, a)
    # This is the "committed schedule" that will be broadcast to other CAVs
    # Updated after each successful QP solve
    predicted_trajectory: List[Tuple[float, float, float, float]] = field(default_factory=list)

    # Communication capability flag
    # - False: CAV (Connected Automated Vehicle) - shares trajectory
    # - True: HDV (Human-Driven Vehicle) - trajectory unknown, use prediction
    is_hdv: bool = False

    # Lane change intent signal (for cooperative yielding)
    # Dict structure: {'target_lane': 'left', 'start_time': 10.5, 'status': 'preparing'/'executing'}
    # Used by adjacent vehicles to proactively create space
    lc_intent: Optional[Dict] = None

    # Yield tracking for v12.6 (timeout handling)
    yield_start_time: Optional[float] = None

    # --- v11.12: LC Check Interval Timer ---
    # Enforces 5s interval between LC probability checks
    # Prevents excessive LC attempts and ensures statistical validity
    last_lc_check_time: float = -999.0  # Initial: far in past (immediate first check)
    # ---
    # ========================================================================
    # v27.0+: 動的協調・一時停止・衝突回復フラグ (型安全用)
    # ========================================================================
    cooperative_decel_request: float = 0.0
    cooperative_yield_request: Optional[Dict] = None
    v2v_lc_requests: List[Dict] = field(default_factory=list)
    v2v_action: Optional[str] = None
    v2v_requester: Optional[int] = None
    v2v_urgency: float = 0.0

    lc_paused: bool = False
    lc_pause_until: float = 0.0
    lc_pause_start_time: float = 0.0
    lc_pause_progress: float = 0.0
    lc_pause_count: int = 0
    lc_progress: float = 0.0
    mid_lc_gap_pause_logged: bool = False

    trajectory_conflict_v_reduction: float = 0.0

    aeb_reason: str = ""
    front_vehicle: Optional['Vehicle'] = None
    v_ref: float = 0.0

    collision_recovery_steps: int = 0
    collision_recovery_dx: float = 0.0
    # ========================================================================

    def get_reference_trajectory(
        self,
        current_time: float,
        dt: float,
        horizon: int
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Generate reference trajectory (s_ref, v_ref) from previous plan for stitching.

        v12.0: Trajectory Stitching
        - Uses previous QP solution as baseline for current planning cycle
        - Provides smooth continuity and reduces plan oscillation
        - Penalizes deviations from committed trajectory

        Args:
            current_time: Current absolute time [s]
            dt: Time step [s]
            horizon: Planning horizon length

        Returns:
            (s_ref, v_ref): Reference position and velocity arrays, or (None, None)
        """
        if not self.predicted_trajectory or len(self.predicted_trajectory) < 5:
            return None, None

        # Extract trajectory data
        traj_t = np.array([p[0] for p in self.predicted_trajectory])
        traj_s = np.array([p[1] for p in self.predicted_trajectory])
        traj_v = np.array([p[2] for p in self.predicted_trajectory])

        # Current planning time axis
        plan_t = current_time + np.arange(horizon) * dt

        # Validate time range (discard stale or incomplete data)
        # Allow small extrapolation (0.1s before, 2.0s after) for robustness
        if plan_t[0] < traj_t[0] - 0.1 or plan_t[-1] > traj_t[-1] + 2.0:
            return None, None

        # Linear interpolation from previous trajectory
        s_ref = np.interp(plan_t, traj_t, traj_s)
        v_ref = np.interp(plan_t, traj_t, traj_v)

        return s_ref, v_ref

    @property
    def needs_lane_change(self) -> bool:
        """Check if vehicle needs lane change"""
        return self.needs_initial_lc or self.can_density_adjust

    def update_history(self, t: float):
        """Update vehicle history"""
        self.history_time.append(t)
        self.history_x.append(self.x)
        self.history_v.append(self.v)
        self.history_lane.append(self.lane)

    def check_lc_count_constraint(self, max_lc_total: int) -> bool:
        """Check if LC count is within limit"""
        return self.lc_count_total <= max_lc_total

    def remaining_lc_count(self, max_lc_total: int) -> int:
        """Get remaining LC count"""
        return max(0, max_lc_total - self.lc_count_total)

    def __init__(self, vehicle_id: int, x: float, y: float, v: float, lane: str, spawn_time: float,
                 destination_area: str, in_destination_area: bool,
                 needs_initial_lc: bool, can_density_adjust: bool, v_desired: float, T_idm: float,
                 is_hdv: bool, params):
        """Initialize vehicle"""
        self.id = vehicle_id
        self.x = x
        self.y = y
        self.v = v
        # Initialize previous velocity to current to avoid false accel spikes
        self.v_prev = v
        self.lane = lane
        self.spawn_time = spawn_time
        self.destination_area = destination_area
        self.in_destination_area = in_destination_area
        self.needs_initial_lc = needs_initial_lc
        self.can_density_adjust = can_density_adjust
        self.v_desired = v_desired
        self.T_idm = T_idm
        self.is_hdv = is_hdv
        self.params = params

        # v13.1: Initialize history fields (required for dataclass fields with default_factory)
        self.history_time = []
        self.history_x = []
        self.history_v = []
        self.history_lane = []
        self.velocity_profile = []
        self.predicted_trajectory = []

        # Ensure lateral coordinate is initialized (straight road: d=y)
        self.d = getattr(self, 'd', y)

        # v13.9: Cooperative acceleration request (from rear vehicle)
        # Used in coordination to open gaps cooperatively
        self.cooperative_accel_request = 0.0  # [m/s²]

        # v27.0+: Cooperative decel/yield requests and V2V LC requests
        self.cooperative_decel_request = 0.0
        self.cooperative_yield_request = None
        self.v2v_lc_requests = []
        self.v2v_action = None
        self.v2v_requester = None
        self.v2v_urgency = 0.0

        # Lane-change pause/resume bookkeeping
        self.lc_paused = False
        self.lc_pause_until = 0.0
        self.lc_pause_start_time = 0.0
        self.lc_pause_progress = 0.0
        self.lc_pause_count = 0
        self.lc_progress = 0.0
        self.mid_lc_gap_pause_logged = False

        # Conflict handling + safety annotations
        self.trajectory_conflict_v_reduction = 0.0
        self.aeb_reason = ""
        self.front_vehicle = None
        self.v_ref = v

        # Collision recovery helpers
        self.collision_recovery_steps = 0
        self.collision_recovery_dx = 0.0

        # Previous state tracking for trajectory stitching reset logic
        self._prev_changing_lane = False
        self._prev_aeb_active = False
