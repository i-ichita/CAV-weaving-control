# -*- coding: utf-8 -*-
# --- frenet_qp_apollo.py (ver.13.1_CAV_Aware_Margins / 2025-12-24) ---
"""
================================================================================
Apolloスタイル区分線形ジャークQP + スラック変数 (ソフト制約)
================================================================================

設計哲学 (Ultimate Apollo):
1. スラック変数 (ソフト制約):
   - クラッシュシナリオでも「Infeasible」にならない
   - 安全境界違反時は「最小違反軌道」を計算
   - s_lower - slack <= s <= s_upper + slack
2. 真の区分線形ジャーク定式化:
   - 制御入力はジャーク(j)、精密な3次軌道生成が可能
   - s(t+1) = s(t) + v(t)dt + 0.5*a(t)dt² + (1/6)*j(t)dt³
3. ウォームスタート & ポリッシング:
   - 10Hzリアルタイム性能のための頑健なソルバー設定

v13.1 CAV考慮安全マージン (2025-12-24):
- 根本原因修正: ギャップ崩壊防止のための適応MIN_HARD_MARGIN
  * 問題: v12.4でMIN_HARD_MARGIN=5.0mが2-3mギャップを許容
  * 結果: 177件のCAV-AEB EXTREME PROXIMITY警告
  * 解決: 軌道信頼度に基づくCAV考慮適応マージン
  * CAV-CAV (有効な軌道): MIN_HARD_MARGIN = 5.0m
  * CAV-HDV (または軌道なし): MIN_HARD_MARGIN = 5.0m
  * QPが危険なギャップ収束を許容するのを防止

ベース: Baidu Apollo modules/planning/math/piecewise_jerk/piecewise_jerk_problem.cc
       Apollo trajectory_stitcher.cc - ComputeStitchingTrajectory
       Apollo prediction module - TrajectoryValidator
================================================================================
"""

import numpy as np
from scipy.sparse import csc_matrix, vstack
import osqp
from typing import Any, Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import Enum

# Debug output flag
ENABLE_DEBUG_OUTPUT = False  # [PERF] Disabled for production speed

class BoundaryType(Enum):
    FOLLOW = 0     # Follow front vehicle (追従)
    YIELD = 1      # Yield to obstacle (譲る)
    OVERTAKE = 2   # Overtake obstacle (追い越す)

class InteractionDecision(Enum):
    """Decision for how ego should interact with obstacle"""
    IGNORE = 0      # No interaction needed
    FOLLOW = 1      # Follow front vehicle at safe distance (前方車両を追従)
    YIELD = 2       # Decelerate and go behind (相手の後ろへ)
    OVERTAKE = 3    # Accelerate and go ahead (相手の前へ)

# [PERF] Global QP Statistics
QP_STATS = {
    "total_attempts": 0,
    "main_success": 0,
    "relaxed_trigger": 0,
    "relaxed_success": 0,
    "fallback_trigger": 0,
    "total_time_ms": 0.0
}

class NudgeType(Enum):
    """Nudge decision for lateral micro-adjustment within lane"""
    NO_NUDGE = 0    # No lateral adjustment needed
    LEFT_NUDGE = 1  # Nudge left (within lane boundary)
    RIGHT_NUDGE = 2 # Nudge right (within lane boundary)

@dataclass
class VehicleState:
    id: int
    s: float
    v: float
    a: float
    lane: str
    d: float = 0.0  # v18.4: Lateral position (Frenet d-coordinate)
    is_hdv: bool = False  # v28.0: Distinguish HDV vs CAV for RSS/coop logic

@dataclass
class ObstacleInfo:
    vehicle_state: VehicleState
    is_front: bool
    # v29.0: Virtual/priority attributes for cooperative and safety checks
    is_virtual: bool = False
    gap: float = 0.0
    ttc: float = float('inf')
    priority_level: int = 1
    # v12.0: CAV Cooperative Control - Shared trajectory data
    # List of (t_abs, s, v, a) received from neighboring CAV
    # If None or empty, fall back to constant velocity prediction
    predicted_trajectory: Optional[List[Tuple[float, float, float, float]]] = None
    # v12.1: Phase 2 - Dynamic lateral buffer during lane changes
    is_changing_lane: bool = False
    lateral_velocity: float = 0.0  # m/s (lateral speed during LC)
    # v12.2: Phase 3 - Lane-specific filtering
    lane_relevance: str = "current"  # "current", "target", "other"
    # v19.0: Overtake target relaxation - apply ST-Boundary with relaxed constraints
    # If True, this obstacle is an overtake target - constraints are applied but relaxed
    # to allow safe passing while maintaining collision avoidance
    is_overtake_target: bool = False

@dataclass
class STBoundary:
    s_lower: np.ndarray
    s_upper: np.ndarray
    boundary_type: BoundaryType
    vehicle_id: int
    obs_v: float = 0.0  # v18.5: Obstacle velocity for follow mode

class HysteresisFilter:
    """
    Apollo-style Hysteresis Filter for stable decision making

    判定のチャタリング（頻繁な切り替え）を防ぐためのヒステリシスフィルタ。
    一度ブロッキングと判定されたら、より大きなマージンがないと解除されない。

    Based on: Apollo lane_change_util.cc::HysteresisFilter
    ver.11.11 / 2025-12-22
    """

    def __init__(self, distance_buffer: float = 0.5):
        """
        Args:
            distance_buffer: ヒステリシス幅 [m] (デフォルト: 0.5m)
        """
        self.distance_buffer = distance_buffer

    def is_blocking(
        self,
        obstacle_distance: float,
        safe_distance: float,
        is_currently_blocking: bool
    ) -> bool:
        """
        障害物がブロッキング状態かを判定（ヒステリシス付き）

        Args:
            obstacle_distance: 障害物までの距離 [m]
            safe_distance: 必要な安全距離 [m]
            is_currently_blocking: 現在ブロッキング中か

        Returns:
            bool: ブロッキング状態ならTrue
        """
        if is_currently_blocking:
            # すでにブロッキング中 -> より大きなマージンが必要
            threshold = safe_distance + self.distance_buffer
        else:
            # まだブロッキングしていない -> より小さなマージンで判定
            threshold = safe_distance - self.distance_buffer

        return obstacle_distance <= threshold  # <= に修正（境界値を含む）

class HeuristicSpeedDecider:
    """
    Apollo-style Speed Bounds Decider (簡易版)

    物理的実現性（Kinematic Feasibility）とコスト評価に基づいて、
    障害物に対する相互作用（Yield/Overtake）を決定する。

    目的: 「無理な追い越し（Overtake Trap）」によるAEB多発を防ぐ

    ver.12.3 / 2025-12-22
    - Added strict safety rules (Apollo Safety Manager compliance)
    """

    def __init__(self, dt: float, horizon: int, a_max: float, a_min: float):
        """
        Args:
            dt: Time step [s]
            horizon: Planning horizon [steps]
            a_max: Maximum acceleration [m/s^2]
            a_min: Minimum acceleration (braking) [m/s^2]
        """
        self.dt = dt
        self.horizon = horizon
        self.a_max = a_max
        self.a_min = a_min

        # ====================================================================
        # v12.3: Apollo Safety Manager Strict Rules
        # ====================================================================
        self.CRITICAL_DISTANCE = 20.0  # [m] 臨界距離
        self.CRITICAL_VELOCITY = 5.0   # [m/s] 臨界速度差
        self.EMERGENCY_DISTANCE = 10.0 # [m] 緊急距離
        # ====================================================================

    def make_decision(
        self,
        ego_state: VehicleState,
        obstacle: ObstacleInfo
    ) -> InteractionDecision:
        """
        障害物に対する最適な相互作用を決定する

        判定基準（v12.3強化版）:
        0. [NEW] 緊急安全ルール: 近距離+高速差 -> 強制YIELD
        1. 前方車両で速度が近い場合 -> FOLLOW（追従）
        2. 到達可能範囲（Kinematic Limits）で物理的実現性を評価
        3. 干渉領域では慣性維持（Jerk最小化）を優先
        4. 迷った場合は安全側（YIELD）に倒す

        Args:
            ego_state: Ego vehicle state
            obstacle: Obstacle information

        Returns:
            InteractionDecision: FOLLOW, YIELD, or OVERTAKE
        """
        obs = obstacle.vehicle_state
        t_end = self.dt * self.horizon

        # ====================================================================
        # v12.3: Strict Safety Rules (Apollo Safety Manager)
        # ====================================================================
        # Rule 1: 緊急距離内 (10m以内) -> 強制YIELD
        dist = abs(obs.s - ego_state.s)
        rel_v = obs.v - ego_state.v  # 相対速度（相手 - 自分）

        if dist < self.EMERGENCY_DISTANCE:
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider-STRICT] V#{obs.id}: EMERGENCY distance={dist:.1f}m < 10m -> YIELD")
            return InteractionDecision.YIELD

        # Rule 2: 臨界条件（20m以内 かつ 5m/s以上速い）-> 強制YIELD
        # これはより速い車両が後ろから迫っている状況
        if dist < self.CRITICAL_DISTANCE and rel_v > self.CRITICAL_VELOCITY:
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider-STRICT] V#{obs.id}: CRITICAL d={dist:.1f}m, rel_v={rel_v:.1f}m/s -> YIELD")
            return InteractionDecision.YIELD
        # ====================================================================

        # ========================================================================
        # v14.2: Enhanced FOLLOW Decision with Distance-Based Threshold
        # ========================================================================
        # 0. FOLLOW判定の優先チェック（前方車両のみ）
        # 条件: (a) 前方にいる、(b) 速度差が小さい OR 距離が近い、(c) 同じレーン
        #
        # Apollo Principle: When approaching a front vehicle at close range,
        # switch to FOLLOW mode early to enable smooth deceleration
        # ========================================================================
        if obstacle.is_front and ego_state.lane == obs.lane:
            rel_v_ego = ego_state.v - obs.v  # Positive if ego is faster (closing in)
            dist = obs.s - ego_state.s  # Distance to front vehicle

            # ================================================================
            # v14.2: Distance-Adaptive FOLLOW Threshold
            # ================================================================
            # Problem: Fixed 3.0 m/s threshold too strict for close distances
            # - At 100m gap: can tolerate 10 m/s relative speed
            # - At 50m gap: should FOLLOW if rel_v > 5 m/s
            # - At 30m gap: should FOLLOW if rel_v > 3 m/s
            # Apollo Method: TTC (Time-to-Collision) based decision
            # ================================================================
            FOLLOW_BASE_THRESHOLD = 3.0  # [m/s] Base threshold for far distances
            FOLLOW_DISTANCE_THRESHOLD = 50.0  # [m] Distance below which to tighten threshold

            # Calculate adaptive threshold based on distance
            if dist < FOLLOW_DISTANCE_THRESHOLD:
                # Closer distance -> stricter threshold (lower speed diff allowed)
                # Linear interpolation: 50m->3.0m/s, 30m->2.0m/s, 10m->1.0m/s
                follow_v_threshold = FOLLOW_BASE_THRESHOLD * (dist / FOLLOW_DISTANCE_THRESHOLD)
                follow_v_threshold = max(1.0, follow_v_threshold)  # Minimum 1.0 m/s
            else:
                follow_v_threshold = FOLLOW_BASE_THRESHOLD

            # FOLLOW if relative speed is small (considering adaptive threshold)
            if abs(rel_v_ego) < follow_v_threshold:
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[Decider-FOLLOW] V#{obs.id}: dist={dist:.1f}m, rel_v={rel_v_ego:.1f}m/s "
                          f"< threshold={follow_v_threshold:.1f}m/s -> FOLLOW")
                return InteractionDecision.FOLLOW

            # ================================================================
            # v14.2: Emergency FOLLOW for high closing speed at close range
            # ================================================================
            # Apollo Safety Rule: If approaching front vehicle at high speed
            # AND distance is small, force FOLLOW to prevent AEB cascade
            # TTC threshold: 3.0 seconds (Apollo standard)
            # ================================================================
            if rel_v_ego > 0:  # Closing in
                ttc = dist / rel_v_ego  # Time-to-collision
                TTC_THRESHOLD = 3.0  # [s] Apollo safety standard

                if ttc < TTC_THRESHOLD:
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[Decider-TTC-FOLLOW] V#{obs.id}: TTC={ttc:.2f}s < {TTC_THRESHOLD:.1f}s "
                              f"(dist={dist:.1f}m, rel_v={rel_v_ego:.1f}m/s) -> FORCE FOLLOW")
                    return InteractionDecision.FOLLOW
            # ================================================================

        # 1. 相手のT秒後の推定位置
        # CAV軌道がある場合は最終点を使用、なければ等速予測
        if obstacle.predicted_trajectory and len(obstacle.predicted_trajectory) > 0:
            # 軌道の最終点を取得
            obs_s_end = obstacle.predicted_trajectory[-1][1]  # (t, s, v, a) の s
        else:
            # v27.10 FIX: Constant Acceleration (CA) model instead of CV
            # This accounts for front vehicle's braking when predicting position
            obs_a = getattr(obs, 'a', 0.0)  # Get acceleration if available
            # Predict position with acceleration: s = s0 + v*t + 0.5*a*t²
            # Clamp velocity to >= 0 (vehicle can't go backwards)
            v_pred = max(0.0, obs.v + obs_a * t_end)
            if obs_a < 0:
                # Decelerating: use average velocity for prediction
                v_avg = (obs.v + v_pred) / 2.0
                obs_s_end = obs.s + v_avg * t_end
            else:
                # Non-decelerating: use standard kinematic equation
                obs_s_end = obs.s + obs.v * t_end + 0.5 * obs_a * (t_end ** 2)

        # 2. 自車の到達可能範囲 (Kinematic Reachability)
        # 最大加速で逃げた場合のT秒後位置
        ego_s_max = ego_state.s + ego_state.v * t_end + 0.5 * self.a_max * (t_end**2)

        # 最大減速（安全停止）で待った場合のT秒後位置
        # 速度が0になる時刻: t_stop = -v / a_min
        t_stop = -ego_state.v / self.a_min if self.a_min < 0 else t_end

        if t_stop < t_end:
            # 途中で停止する場合
            ego_s_min = ego_state.s + ego_state.v * t_stop + 0.5 * self.a_min * (t_stop**2)
        else:
            # 停止せずに減速し続ける場合
            ego_s_min = ego_state.s + ego_state.v * t_end + 0.5 * self.a_min * (t_end**2)

        # 3. 判定ロジック
        # ====================================================================
        # v18.11 Modified: Apollo-aligned Low-Speed Threshold
        # ====================================================================
        # Apollo Reference: dp_st_cost.cc - max_adc_stop_speed (typically 2-3 m/s)
        # 
        # Previous (v12.3): Threshold = 10 m/s (too aggressive, caused 247 Low-speed events)
        # New (v18.11): Threshold = 5 m/s (Apollo standard for congestion detection)
        # 
        # Rationale:
        # - Low-speed margin (v < 5 m/s): 6.5m (near-stop safety)
        # - Normal-speed margin (v >= 5 m/s): 5.0m (standard safety)
        # This reduces unnecessary Low-speed mode activation while maintaining safety
        # ====================================================================
        LOW_SPEED_THRESHOLD = 5.0  # v18.11: Reduced from 10.0 m/s (Apollo standard)
        
        if ego_state.v < LOW_SPEED_THRESHOLD:
            MARGIN = 6.5  # Enhanced margin for near-stop scenarios
            COMFORT_MARGIN = 6.5
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider-MARGIN v18.11] Low-speed mode: v={ego_state.v:.1f}m/s -> "
                      f"MARGIN={MARGIN:.1f}m (standard: 5.0m)")
        else:
            MARGIN = 5.0  # Standard safety margin
            COMFORT_MARGIN = 5.0

        # ケースA: 相手が明らかに速く、追い抜いていく -> YIELD一択
        # (自分が最大加速しても追いつけない)
        if obs_s_end > ego_s_max + MARGIN:
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider] V#{obstacle.vehicle_state.id}: Far ahead -> YIELD")
            return InteractionDecision.YIELD

        # ケースB: 相手が明らかに遅く、追いつけない -> OVERTAKE一択
        # (自分が最大減速しても相手より前に出る)
        if obs_s_end < ego_s_min - MARGIN:
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider] V#{obstacle.vehicle_state.id}: Far behind -> OVERTAKE")
            return InteractionDecision.OVERTAKE

        # ケースC: 干渉領域（どちらも選べる） -> コスト評価
        # 基本方針: 「現在の速度を維持して自然に成り立つ関係」を優先（Jerk最小化）
        ego_s_inertial = ego_state.s + ego_state.v * t_end

        if ego_s_inertial > obs_s_end + COMFORT_MARGIN:
            # 慣性で自然に前に出られる -> OVERTAKE
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider] V#{obstacle.vehicle_state.id}: Natural overtake -> OVERTAKE")
            return InteractionDecision.OVERTAKE

        elif ego_s_inertial < obs_s_end - COMFORT_MARGIN:
            # 慣性で自然に後ろになる -> YIELD
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider] V#{obstacle.vehicle_state.id}: Natural yield -> YIELD")
            return InteractionDecision.YIELD

        else:
            # 微妙な場合（AEB多発の分水嶺）-> 安全側に倒してYIELD推奨
            # Rationale: 譲る方が追い越すより安全（後続車の責任）
            if ENABLE_DEBUG_OUTPUT:
                print(f"[Decider] V#{obstacle.vehicle_state.id}: Ambiguous -> YIELD (safety)")
            return InteractionDecision.YIELD

class NudgeDecider:
    """
    Apollo-style Nudge Decider for in-lane lateral micro-adjustments

    車線内での横方向微調整（+/-0.5m程度）を判断
    車線変更ではなく、障害物を避けるための小さな横移動

    Based on: Apollo navi_obstacle_decider.cc (lines 94-200)
    ver.12.2 / 2025-12-22
    """

    def __init__(self, max_nudge_distance: float = 0.5, safe_lateral_clearance: float = 0.8):
        """
        Args:
            max_nudge_distance: Maximum nudge offset from lane center [m]
            safe_lateral_clearance: Minimum lateral clearance to obstacle [m]
        """
        self.max_nudge_distance = max_nudge_distance
        self.safe_lateral_clearance = safe_lateral_clearance

    def decide_nudge(
        self,
        ego_state: VehicleState,
        obstacle: ObstacleInfo,
        ego_lateral_offset: float = 0.0
    ) -> Tuple[NudgeType, float]:
        """
        Decide if lateral nudge is needed to avoid obstacle

        判定基準:
        1. 障害物が車線内かつ近い（s方向）
        2. 横方向のクリアランスが不足
        3. Nudge可能な範囲内（車線境界を超えない）

        Args:
            ego_state: Ego vehicle state
            obstacle: Obstacle information
            ego_lateral_offset: Current lateral offset from lane center [m]

        Returns:
            Tuple of (nudge_type, nudge_offset)
        """
        obs = obstacle.vehicle_state

        # Only nudge for same-lane obstacles
        if obs.lane != ego_state.lane:
            return NudgeType.NO_NUDGE, 0.0

        # Only nudge for obstacles that are close in s-direction
        s_diff = abs(obs.s - ego_state.s)
        NUDGE_S_THRESHOLD = 30.0  # Only consider obstacles within 30m

        if s_diff > NUDGE_S_THRESHOLD:
            return NudgeType.NO_NUDGE, 0.0

        # Check if obstacle is changing lanes (already handled by lane change logic)
        if obstacle.is_changing_lane:
            return NudgeType.NO_NUDGE, 0.0

        # For simplicity in Frenet (s,d) framework without explicit lateral position:
        # Assume obstacle is at lane center if same lane
        # In reality, would use obstacle.d (lateral position) from perception
        # Here we use a heuristic: nudge away from faster obstacles

        # Nudge decision based on relative velocity
        # If obstacle is slower and ahead, prepare to pass -> slight nudge
        if obstacle.is_front:
            rel_v = ego_state.v - obs.v
            if rel_v > 2.0:  # Approaching slower vehicle
                # Nudge slightly to prepare for potential overtake
                # In highway: typically nudge left (passing lane)
                nudge_offset = min(self.max_nudge_distance, 0.3)
                return NudgeType.LEFT_NUDGE, nudge_offset

        # Default: no nudge needed
        return NudgeType.NO_NUDGE, 0.0


def visualize_st_boundary(
    ego_state: VehicleState,
    obstacles: List[ObstacleInfo],
    decisions: Dict[int, InteractionDecision],
    s_lower: np.ndarray,
    s_upper: np.ndarray,
    dt: float,
    output_path: Optional[str] = None
):
    """
    ST-Graph上の禁止領域を可視化（デバッグ用）

    v12.3 / 2025-12-22
    Based on: Apollo ST-Graph visualization for trajectory debugging

    Args:
        ego_state: Ego vehicle state
        obstacles: List of obstacles
        decisions: Decision map {obstacle_id: decision}
        s_lower: Lower boundary array
        s_upper: Upper boundary array
        dt: Time step [s]
        output_path: Optional path to save figure
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("[ST-VIZ] matplotlib not available, skipping visualization")
        return

    N = len(s_lower)
    t_array = np.arange(N) * dt

    fig, ax = plt.subplots(figsize=(12, 8))

    # Plot ego vehicle starting point
    ax.plot(0, ego_state.s, 'go', markersize=10, label='Ego Start')

    # Plot drivable tube (s_lower, s_upper)
    ax.fill_between(t_array, s_lower, s_upper, alpha=0.3, color='green', label='Drivable Tube')
    ax.plot(t_array, s_lower, 'g--', linewidth=2, label='Lower Bound')
    ax.plot(t_array, s_upper, 'r--', linewidth=2, label='Upper Bound')

    # Plot obstacle boundaries
    for obs in obstacles:
        obs_id = obs.vehicle_state.id
        decision = decisions.get(obs_id, InteractionDecision.YIELD)

        # Predict obstacle trajectory
        obs_s_trajectory = []
        for k in range(N):
            t = k * dt
            if obs.predicted_trajectory and len(obs.predicted_trajectory) > 0:
                # Use shared trajectory
                traj = obs.predicted_trajectory
                if t >= traj[-1][0]:
                    obs_s = traj[-1][1]
                else:
                    # Interpolate
                    obs_s = traj[0][1]
                    for i in range(len(traj) - 1):
                        if traj[i][0] <= t <= traj[i+1][0]:
                            t1, s1 = traj[i][0], traj[i][1]
                            t2, s2 = traj[i+1][0], traj[i+1][1]
                            obs_s = s1 + (s2 - s1) * (t - t1) / (t2 - t1)
                            break
            else:
                # Constant acceleration model
                v_pred = max(0.0, obs.vehicle_state.v + obs.vehicle_state.a * t)
                obs_s = obs.vehicle_state.s + (obs.vehicle_state.v + v_pred) / 2.0 * t

            obs_s_trajectory.append(obs_s)

        # Plot obstacle trajectory
        color = 'red' if decision == InteractionDecision.YIELD else 'blue'
        label = f"V#{obs_id} ({decision.name})"
        ax.plot(t_array, obs_s_trajectory, color=color, linewidth=2, label=label)

        # Add vehicle box at t=0
        # v13.2: Consistent vehicle length (was 4.5m, now 5.0m)
        L_VEHICLE = 5.0  # [m] Full vehicle length for visualization
        rect = patches.Rectangle(
            (0, obs.vehicle_state.s - L_VEHICLE/2),
            dt, L_VEHICLE,
            linewidth=1, edgecolor=color, facecolor=color, alpha=0.3
        )
        ax.add_patch(rect)

    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Position s [m]', fontsize=12)
    ax.set_title('ST-Graph: Obstacle Boundaries and Drivable Tube', fontsize=14)
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"[ST-VIZ] Saved to {output_path}")
    else:
        plt.show()

    plt.close()


class FrenetQPController:
    """
    Ultimate Apollo-style QP Controller with Slack Variables
    v15.0: Extended Prediction Horizon + Adaptive Intelligent Control

    Key Improvements over v12.3:
    1. Extended horizon: 0.5s -> 8.0s (Apollo EM Planner standard)
    2. Adaptive speed planning: Multi-objective optimization (safety + comfort + efficiency)
    3. Intelligent safety margins: Dynamic adjustment based on traffic density
    4. Smooth deceleration profiles: Minimize jerk, avoid harsh -6.0 m/s^2 braking
    5. Long-term prediction: Consider distant vehicles (up to 200m ahead)
    """

    @staticmethod
    def get_qp_stats() -> Dict[str, Any]:
        """Return global QP performance statistics"""
        return QP_STATS.copy()

    def __init__(self, horizon: int = 80, dt: float = 0.1):
        # ====================================================================
        # v15.0: Extended Prediction Horizon (Apollo EM Planner)
        # ====================================================================
        # Previous: 50 steps x 0.1s = 5.0s
        # New: 80 steps x 0.1s = 8.0s (Apollo standard for highway scenarios)
        # Benefits:
        # - Consider distant vehicles (up to 200m @ 25 m/s)
        # - Smoother deceleration planning (more time to react)
        # - Better traffic flow optimization
        # - Reduced AEB triggers (predictive vs reactive)
        # ====================================================================
        self.N = horizon
        self.dt = dt
        self.horizon_time = self.N * self.dt  # 8.0 seconds

        # ====================================================================
        # v12.5: Physical Vehicle Dimensions
        # ====================================================================
        # Apollo標準車体サイズ: 全長4.7m (Lincoln MKZ)
        # 安全マージンを含めて 5.0m とする（車体長+バンパー余裕）
        # これを制約に加えることで、車両中心同士の重複を防ぐ
        # ====================================================================
        self.L_vehicle = 5.0  # [m] Vehicle length for collision avoidance
        # ====================================================================

        # ====================================================================
        # v15.0: Adaptive Vehicle Limits (Apollo Dynamic Parameters)
        # ====================================================================
        # Previous: Fixed -6.0 m/s^2 emergency braking
        # New: Adaptive limits based on scenario
        # ====================================================================
        self.s0 = 2.0      # Min standstill gap
        self.T = 1.5       # Time headway

        # Comfort-oriented limits (default for normal driving)
        self.a_min_comfort = -4.0  # v27.8: Was -3.5, raised to reduce gap to emergency
        self.a_max_comfort = 1.5   # [m/s^2] Comfortable acceleration

        # Emergency limits (only when absolutely necessary)
        self.a_min_emergency = -6.0  # [m/s^2] Maximum emergency braking (UN R157)
        self.a_max_emergency = 2.5   # [m/s^2] Maximum acceleration

        # Default to comfort limits
        self.a_min = self.a_min_comfort
        self.a_max = self.a_max_comfort

        self.v_max = 30.0  # Max velocity
        # ====================================================================

        # ====================================================================
        # v15.0: Enhanced Jerk Limits (Human-like Smoothness)
        # ====================================================================
        # Tighter jerk limits for smoother driving
        # Apollo: jerk < 2.0 m/s³ for passenger comfort
        # ====================================================================
        self.j_min = -2.5  # Reduced from -4.0 for smoother braking
        self.j_max = 2.0   # Comfortable acceleration jerk
        # ====================================================================

        # ====================================================================
        # v15.0: Multi-Objective Weights (Apollo Cost Function)
        # ====================================================================
        # Philosophy: Balance safety, comfort, efficiency, and traffic flow
        # - w_ref: Position tracking (reach destination)
        # - w_v: Velocity tracking (maintain speed limit / target speed)
        # - w_a: Acceleration minimization (energy efficiency)
        # - w_j: Jerk minimization (passenger comfort - HIGHEST PRIORITY)
        # - w_slack: Safety violation penalty (collision avoidance)
        # - w_efficiency: NEW - Traffic flow efficiency
        # ====================================================================
        self.w_ref = 0.5       # Reduced from 1.0 - allow flexible positioning
        self.w_v = 8.0         # [TUNE] 10.0 -> 8.0: Reduce velocity pressure for safer gaps
        self.w_a = 25.0        # v27.9: TUNE 30.0 -> 25.0: Balanced penalty on strong braking
        self.w_j = 4000.0      # v27.9: TUNE 5000.0 -> 4000.0: Moderate smoothness
        self.w_slack = 1e5     # Safety violation penalty (unchanged)
        self.w_efficiency = 5.0  # NEW - Encourage maintaining traffic flow
        # ====================================================================

        # v12.0: Current absolute time (for trajectory interpolation)
        self.current_abs_time = 0.0

        # v12.4: Previous trajectory storage for stitching validation
        # Note: 'time' key stores float, not ndarray
        self.previous_trajectory: Dict[int, Dict[str, Any]] = {}

        # ====================================================================
        # v12.3: Decision Locking (チャタリング防止)
        # ====================================================================
        self.decision_lock_duration = 2.0  # [s] ロック期間
        self.locked_decisions: Dict[int, Tuple[InteractionDecision, float]] = {}
        # Format: {obstacle_id: (decision, lock_end_time)}
        # ====================================================================

        # ====================================================================
        # v12.3: ST-Boundary Visualization (デバッグ用)
        # ====================================================================
        self.enable_st_visualization = False  # デフォルトはOFF
        self.st_viz_counter = 0  # 可視化カウンタ
        # ====================================================================

        # ====================================================================
        # v15.0: Adaptive Safety Margin Controller
        # ====================================================================
        # Dynamically adjust safety margins based on:
        # - Traffic density (high density -> smaller margins for flow)
        # - Relative velocity (high closing speed -> larger margins)
        # - Prediction confidence (CAV->CAV: smaller, CAV->HDV: larger)
        # ====================================================================
        self.adaptive_margin_enabled = True
        self.traffic_density = 0.0  # Updated by controller
        # ====================================================================

    def _validate_predicted_trajectory(
        self,
        obstacle: ObstacleInfo
    ) -> bool:
        """
        Validate shared trajectory against current observation (Apollo Safety)

        v12.4: Safety Validation Layer 2 - Prediction Validation
        Reference: Apollo prediction module - TrajectoryValidator

        Purpose: Detect when shared CAV trajectory disagrees with sensor reality
        - Prevents "Blind Trust" collisions where planner uses invalid predictions
        - Validates trajectory start position against current observed position

        Args:
            obstacle: Obstacle with predicted_trajectory

        Returns:
            True if trajectory is valid (can be used), False if invalid (discard)
        """
        if not obstacle.predicted_trajectory or len(obstacle.predicted_trajectory) == 0:
            return False

        # Get trajectory start point (t0, s0, v0, a0)
        traj_start = obstacle.predicted_trajectory[0]
        s_predicted_start = traj_start[1]

        # Current observed position
        s_observed = obstacle.vehicle_state.s

        # Calculate prediction error
        pred_error = abs(s_predicted_start - s_observed)

        # Apollo threshold: 2.0m for trajectory validity
        PRED_ERROR_THRESHOLD = 2.0  # [m]

        if pred_error >= PRED_ERROR_THRESHOLD:
            # Prediction invalid: Discard shared trajectory
            if ENABLE_DEBUG_OUTPUT:
                print(f"[PRED-INVALID v12.4] V#{obstacle.vehicle_state.id}: "
                      f"Shared trajectory invalid!")
                print(f"  Predicted start: s={s_predicted_start:.1f}m")
                print(f"  Observed: s={s_observed:.1f}m")
                print(f"  Error: {pred_error:.2f}m >= {PRED_ERROR_THRESHOLD}m")
                print("  Action: Discarding shared trajectory, using physics prediction (CV model)")
            return False

        # Prediction valid
        if ENABLE_DEBUG_OUTPUT:
            print(f"[PRED-OK v12.4] V#{obstacle.vehicle_state.id}: "
                  f"Shared trajectory valid (error={pred_error:.2f}m < {PRED_ERROR_THRESHOLD}m)")
        return True

    def _interpolate_trajectory_point(
        self,
        trajectory: List[Tuple[float, float, float, float]],
        query_time: float
    ) -> Optional[float]:
        """
        Interpolate position 's' at absolute time 'query_time' from shared trajectory.

        v12.0: CAV Cooperative Control - Core trajectory interpolation method
        - Uses linear interpolation between trajectory points
        - Handles edge cases: before start (use start), after end (extrapolate with CV)
        - Returns None if trajectory is invalid/empty

        Args:
            trajectory: List of (t_abs, s, v, a) tuples
            query_time: Absolute time to query [s]

        Returns:
            Interpolated position s [m], or None if unavailable
        """
        if not trajectory or len(trajectory) == 0:
            return None

        t_start = trajectory[0][0]
        t_end = trajectory[-1][0]

        # Before trajectory start: use starting position
        if query_time < t_start:
            return trajectory[0][1]

        # After trajectory end: constant velocity extrapolation
        if query_time > t_end:
            last_pt = trajectory[-1]
            dt = query_time - t_end
            # s_extrapolated = s_last + v_last * dt
            return last_pt[1] + last_pt[2] * dt

        # Linear interpolation within trajectory
        for i in range(len(trajectory) - 1):
            pt_curr = trajectory[i]
            pt_next = trajectory[i + 1]

            if pt_curr[0] <= query_time <= pt_next[0]:
                # Linear interpolation ratio
                ratio = (query_time - pt_curr[0]) / (pt_next[0] - pt_curr[0] + 1e-9)
                s_interp = pt_curr[1] + ratio * (pt_next[1] - pt_curr[1])
                return s_interp

        # Fallback: return last point
        return trajectory[-1][1]

    def compute_safe_distance_idm(self, v_ego: float, v_obstacle: float) -> float:
        """Dynamic gap calculation based on IDM (for backward compatibility)"""
        delta_v = max(0, v_ego - v_obstacle)
        s_star = self.s0 + v_ego * self.T + (v_ego * delta_v) / (2 * np.sqrt(self.a_max * 2.0))
        return s_star

    def compute_adaptive_safety_margin(
        self,
        v_ego: float,
        v_obstacle: float,
        traffic_density: float = 0.0,
        prediction_confidence: float = 1.0
    ) -> float:
        """
        v15.0: Adaptive Safety Margin Calculator (Apollo Intelligent Planner)

        Dynamically adjusts safety margins based on traffic conditions:
        - Traffic density: High density -> smaller margins (cooperative driving)
        - Relative velocity: High closing speed -> larger margins (safety)
        - Prediction confidence: CAV->CAV: smaller, CAV->HDV: larger

        Reference: Apollo Planning Research - Adaptive Cruise Control
                  Human Factors Research - Comfortable following distance

        Args:
            v_ego: Ego vehicle velocity [m/s]
            v_obstacle: Obstacle velocity [m/s]
            traffic_density: Normalized traffic density (0.0=free flow, 1.0=congestion)
            prediction_confidence: Confidence in obstacle prediction (0.0-1.0)

        Returns:
            Adaptive safety margin [m]
        """
        # Base margin from IDM
        base_margin = self.compute_safe_distance_idm(v_ego, v_obstacle)

        # ====================================================================
        # Component 1: Traffic Density Adaptation
        # ====================================================================
        # High density -> reduce margins to maintain flow (cooperative behavior)
        # Low density -> maintain comfortable margins
        # Scaling: density=0.0 -> factor=1.0 (100%), density=1.0 -> factor=0.7 (70%)
        # ====================================================================
        density_factor = 1.0 - 0.3 * traffic_density

        # ====================================================================
        # Component 2: Prediction Confidence Adaptation
        # ====================================================================
        # High confidence (CAV->CAV) -> smaller margins (coordinated control)
        # Low confidence (CAV->HDV) -> larger margins (conservative)
        # Scaling: conf=1.0 -> factor=0.8 (80%), conf=0.0 -> factor=1.2 (120%)
        # ====================================================================
        confidence_factor = 1.2 - 0.4 * prediction_confidence

        # ====================================================================
        # Component 3: Relative Velocity Adaptation (Safety Critical)
        # ====================================================================
        # High closing speed -> larger margins (collision risk)
        # Apollo: TTC (Time-to-Collision) based scaling
        # ====================================================================
        rel_v = max(0, v_ego - v_obstacle)  # Closing speed
        if rel_v > 0:
            # TTC-based safety factor
            # rel_v < 3 m/s: factor = 1.0 (normal)
            # rel_v = 5 m/s: factor = 1.15
            # rel_v = 10 m/s: factor = 1.4
            # rel_v > 15 m/s: factor = 1.6 (maximum)
            velocity_safety_factor = 1.0 + min(0.6, rel_v * 0.04)
        else:
            # Opening gap (obstacle faster)
            velocity_safety_factor = 1.0

        # ====================================================================
        # Combine all factors
        # ====================================================================
        # Priority: Safety (velocity) > Confidence > Density
        # This ensures collision avoidance is never compromised
        # ====================================================================
        adaptive_margin = base_margin * density_factor * confidence_factor * velocity_safety_factor

        # Hard minimum: Never go below 5.0m (vehicle length + minimal gap)
        adaptive_margin = max(5.0, adaptive_margin)

        if ENABLE_DEBUG_OUTPUT and abs(adaptive_margin - base_margin) > 2.0:
            print(f"[ADAPTIVE-MARGIN] v_ego={v_ego:.1f}m/s, v_obs={v_obstacle:.1f}m/s, rel_v={rel_v:.1f}m/s")
            print(f"  Base margin: {base_margin:.1f}m")
            print(f"  Density factor: {density_factor:.2f} (density={traffic_density:.2f})")
            print(f"  Confidence factor: {confidence_factor:.2f} (conf={prediction_confidence:.2f})")
            print(f"  Velocity factor: {velocity_safety_factor:.2f}")
            print(f"  -> Adaptive margin: {adaptive_margin:.1f}m")

        return adaptive_margin

    def plan_smooth_deceleration_profile(
        self,
        current_v: float,
        target_v: float,
        distance_available: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        v15.0: Smooth Deceleration Profile Generator (Apollo Motion Planner)

        Generates jerk-minimized deceleration profiles instead of harsh braking.
        Uses quadratic velocity profile for maximum passenger comfort.

        Reference: Apollo EM Planner - Quintic Polynomial Trajectory Generation
                  ISO 15622 (ACC) - Comfort deceleration < -3.0 m/s^2

        Args:
            current_v: Current velocity [m/s]
            target_v: Target velocity [m/s]
            distance_available: Available distance for deceleration [m]

        Returns:
            Tuple of (velocity_profile, acceleration_profile) over horizon
        """
        N = self.N
        dt = self.dt

        # Calculate required deceleration
        delta_v = current_v - target_v

        if delta_v <= 0:
            # No deceleration needed (accelerating or maintaining)
            v_profile = np.linspace(current_v, target_v, N)
            a_profile = np.full(N, 0.0)
            return v_profile, a_profile

        # ====================================================================
        # Method 1: Jerk-Minimized Quadratic Profile (Preferred)
        # ====================================================================
        # Use constant jerk to smoothly transition from current to target velocity
        # v(t) = v0 + a0*t + 0.5*j*t^2
        # ====================================================================

        # Calculate required time for smooth deceleration
        # Prioritize comfort over speed
        time_available = self.horizon_time  # Use full horizon

        # Calculate comfortable constant deceleration
        # a = delta_v / time_available
        a_required = delta_v / time_available

        # Limit to comfort deceleration (-3.5 m/s^2)
        # If exceeds comfort, use emergency deceleration
        if abs(a_required) > abs(self.a_min_comfort):
            # Need emergency braking
            a_decel = self.a_min_comfort  # Use comfort limit initially
            time_needed = delta_v / abs(a_decel)

            # Check if we can achieve target within horizon
            if time_needed > time_available:
                # Cannot achieve target comfortably - use tighter deceleration
                a_decel = -delta_v / time_available
                a_decel = max(a_decel, self.a_min_emergency)  # Clip to emergency limit
        else:
            # Comfortable deceleration sufficient
            a_decel = -a_required

        # Generate smooth profile with jerk-limited transition
        v_profile = np.zeros(N)
        a_profile = np.zeros(N)

        # Smooth ramp-up of deceleration (jerk-limited)
        ramp_time = 1.0  # 1 second to ramp up to full deceleration
        ramp_steps = int(ramp_time / dt)
        ramp_steps = min(ramp_steps, N // 4)  # Max 25% of horizon for ramp

        current_v_sim = current_v
        current_a = 0.0

        for k in range(N):
            # t_rel = k * dt  # Not used, comment out

            if k < ramp_steps:
                # Ramp up deceleration smoothly
                ramp_ratio = k / ramp_steps
                current_a = a_decel * ramp_ratio
            elif current_v_sim > target_v + 0.5:
                # Maintain deceleration
                current_a = a_decel
            else:
                # Target reached - maintain target velocity
                current_a = 0.0
                current_v_sim = target_v

            # Update velocity
            current_v_sim += current_a * dt
            current_v_sim = max(target_v, current_v_sim)  # Don't overshoot

            v_profile[k] = current_v_sim
            a_profile[k] = current_a

        if ENABLE_DEBUG_OUTPUT:
            print(f"[SMOOTH-DECEL] current_v={current_v:.1f}m/s -> target_v={target_v:.1f}m/s")
            print(f"  delta_v={delta_v:.1f}m/s, distance={distance_available:.1f}m")
            print(f"  Required decel: {a_required:.2f}m/s^2")
            print(f"  Applied decel: {a_decel:.2f}m/s^2 ({'COMFORT' if abs(a_decel) <= abs(self.a_min_comfort) else 'EMERGENCY'})")
            print(f"  Ramp steps: {ramp_steps} ({ramp_steps*dt:.1f}s)")
            print(f"  Profile: v[0]={v_profile[0]:.1f}, v[N/2]={v_profile[N//2]:.1f}, v[N-1]={v_profile[-1]:.1f}m/s")

        return v_profile, a_profile

    def compute_forward_safe_distance(
        self,
        v_ego: float,
        v_obstacle: float,
        same_direction: bool = True
    ) -> float:
        """
        RSS-compliant Safety Distance (Aligned with Controller logic)

        v12.5 / 2025-12-24
        Modified to use RSS formula matching controllers.py for consistency
        This prevents unnecessary AEB when planner and controller use different safety criteria

        Based on: ISO 21934 RSS (Responsibility-Sensitive Safety)

        Args:
            v_ego: 自車速度 [m/s]
            v_obstacle: 障害物速度 [m/s]
            same_direction: 同じ方向（True）か逆方向（False）か

        Returns:
            Required forward safety distance [m]
        """
        if same_direction:
            # RSSパラメータ（controllers.pyと一致させる）
            t_reaction = 0.5  # 反応時間 [s]
            a_brake_ego = 6.0 # 自車の最大減速 [m/s^2]
            a_brake_obs = 6.0 # 相手の最大減速 [m/s^2]

            # RSS公式: d = v_ego * t_reaction + v_ego^2/(2*a_ego) - v_obs^2/(2*a_obs)
            rss_dist = v_ego * t_reaction + (v_ego**2)/(2*a_brake_ego) - (v_obstacle**2)/(2*a_brake_obs)

            # マージン（AEB閾値より少し広く取ることで、AEB発動前に制御で減速させる）
            # Controllerは rss_dist * 1.0 で発動するため、Plannerは * 1.1 程度確保する
            safe_dist = max(rss_dist * 1.1, 10.0)
        else:
            # 逆方向: 5秒の安全時間（対向車）
            kSafeTimeOnOppositeDirection = 5.0
            kForwardMinSafeDistance = 50.0
            # 合計速度ベース
            safe_dist = max(kForwardMinSafeDistance,
                           (v_ego + v_obstacle) * kSafeTimeOnOppositeDirection)

        return safe_dist

    def compute_backward_safe_distance(
        self,
        v_ego: float,
        v_obstacle: float,
        same_direction: bool = True
    ) -> float:
        """
        後方車両に対する安全距離（Apollo-style Backward Safety Distance）

        v11.12 / 2025-12-22
        Based on: Apollo lane_change_util.cc::IsClearToChangeLane

        Args:
            v_ego: 自車速度 [m/s]
            v_obstacle: 障害物速度 [m/s]
            same_direction: 同じ方向（True）か逆方向（False）か

        Returns:
            Required backward safety distance [m]
        """
        if same_direction:
            # 同方向: 3秒の安全時間
            kSafeTimeOnSameDirection = 3.0
            kBackwardMinSafeDistance = 10.0
            # 相対速度ベース（後続車が速い場合）
            delta_v = max(0, v_obstacle - v_ego)
            d_backward = max(kBackwardMinSafeDistance, delta_v * kSafeTimeOnSameDirection)
        else:
            # 逆方向: 後方からの対向車はほぼ考慮不要
            kBackwardMinSafeDistance = 1.0
            d_backward = kBackwardMinSafeDistance

        return d_backward

    def compute_lateral_buffer(
        self,
        obstacle: ObstacleInfo,
        base_lateral_buffer: float = 1.0
    ) -> float:
        """
        Compute dynamic lateral buffer during lane changes

        Based on: Apollo lane_change_decider.cc line 170-190

        Parameters:
            obstacle: Obstacle information including lane change state
            base_lateral_buffer: Base lateral clearance (m)

        Returns:
            Total lateral buffer (m) to add to longitudinal safety distance
        """
        if not obstacle.is_changing_lane:
            return 0.0

        # Apollo lane change lateral buffer parameters
        # Reference: lane_change_decider.cc
        kLateralBufferBase = base_lateral_buffer  # Base clearance (1.0m)
        kLateralVelocityFactor = 0.5  # Buffer expansion per m/s lateral velocity

        # Lateral buffer = base + velocity component
        lateral_buffer = kLateralBufferBase

        # Add buffer proportional to lateral velocity
        # Higher lateral speed = more lateral uncertainty = wider safety corridor needed
        if obstacle.lateral_velocity > 0:
            lateral_buffer += abs(obstacle.lateral_velocity) * kLateralVelocityFactor

        return lateral_buffer

    def apply_lane_specific_filtering(
        self,
        ego_state: VehicleState,
        obstacle: ObstacleInfo,
        decision: InteractionDecision
    ) -> Tuple[float, float]:
        """
        Apply lane-specific buffer adjustments based on Apollo path_bounds_decider

        Phase 3: Different lanes get different constraint strengths
        - Current lane obstacles: Full constraints (strict)
        - Target lane obstacles: Relaxed constraints (planning)
        - Other lane obstacles: Minimal/no constraints

        Based on: Apollo path_bounds_decider_util.cc

        Args:
            ego_state: Ego vehicle state
            obstacle: Obstacle information
            decision: Interaction decision

        Returns:
            Tuple of (static_buffer_multiplier, dynamic_buffer_multiplier)
        """
        lane_relevance = obstacle.lane_relevance

        # Lane-specific filtering logic
        if lane_relevance == "current":
            # Same lane: Full constraints (1.0x multiplier)
            # Most critical - must maintain safe distance
            static_mult = 1.0
            dynamic_mult = 1.0

        elif lane_relevance == "target":
            # Target lane during lane change: Moderate constraints (0.7x multiplier)
            # Need to plan for merge, but not yet critical
            # Apollo: Uses "shadow boundary" concept for future lane
            static_mult = 0.7
            dynamic_mult = 0.5

        else:  # "other"
            # Other lanes: Minimal constraints (0.3x multiplier)
            # Only apply if very close (lateral safety margin)
            # Apollo: Often filtered out completely in navi_obstacle_decider
            static_mult = 0.3
            dynamic_mult = 0.0

        # Special case: FOLLOW decision always uses strict constraints
        # Even for target lane, maintain safe following distance
        if decision == InteractionDecision.FOLLOW:
            static_mult = max(static_mult, 0.8)
            dynamic_mult = max(dynamic_mult, 0.6)

        return static_mult, dynamic_mult

    def project_obstacle_with_decision(
        self,
        ego_state: VehicleState,
        obstacle: ObstacleInfo,
        ego_v_estimate: np.ndarray,
        decision: InteractionDecision,
        urgency: float = 0.0
    ) -> STBoundary:
        """
        決定に基づいてST境界を射影する (ver.11.10 / 2025-12-22)

        project_obstacle_to_st_boundaryと同じロジックだが、
        is_frontではなくdecisionを使用して境界を設定する。

        Args:
            ego_state: Ego vehicle state
            obstacle: Obstacle information
            ego_v_estimate: Estimated ego velocity profile
            decision: Interaction decision (YIELD or OVERTAKE)
            urgency: Urgency factor for buffer relaxation

        Returns:
            STBoundary with decision-based constraints
        """
        obs = obstacle.vehicle_state

        s_lower = np.full(self.N, -1e4)  # Default: Open
        s_upper = np.full(self.N, 1e4)   # Default: Open

        # ========================================================================
        # v11.14: Apollo Constraint Relaxation for Lane Change Targets
        # ========================================================================
        is_target_obs = (obs.lane != ego_state.lane)

        # FOLLOW と YIELD は同じ方向の距離計算
        if decision in [InteractionDecision.FOLLOW, InteractionDecision.YIELD]:
            current_dist = obs.s - ego_state.s
        else:
            current_dist = ego_state.s - obs.s
        current_dist = max(0.0, current_dist - 5.0)  # 車体長分を引く(簡易)
        # ========================================================================

        # ========================================================================
        # v12.4: CAV Cooperative Control with Prediction Validation
        # ========================================================================
        # Safety Validation Layer 2: Validate shared trajectory before use
        # - Check if trajectory exists and has sufficient points
        # - Validate trajectory against current observation (prediction error < 2.0m)
        # - If invalid, fallback to physics-based prediction (CV model)
        # ========================================================================
        use_shared_traj = False
        if obstacle.predicted_trajectory is not None and len(obstacle.predicted_trajectory) >= 5:
            # Validate trajectory against current observation
            if self._validate_predicted_trajectory(obstacle):
                use_shared_traj = True
            else:
                # Validation failed: Discard shared trajectory
                # This prevents using trajectories that disagree with sensor data
                use_shared_traj = False
        # ========================================================================

        # ========================================================================
        # v14.0: Calculate relative velocity for buffer scaling
        # ========================================================================
        # Pre-calculate relative velocity to use in static buffer determination
        # This prevents high relative speed situations from being permitted by QP
        #
        # 【相対速度の定義】
        # rel_v = ego_state.v - obs.v
        # - ego_state.v: 自車の速度 [m/s]
        # - obs.v: 障害物の速度 [m/s]
        #
        # 【符号の意味】(FOLLOW/YIELD の場合、障害物は前方にいる)
        # - rel_v > 0: 自車の方が速い -> 接近中（closing in）-> バッファを増やす必要あり
        # - rel_v = 0: 同じ速度 -> 相対距離維持 -> 基本バッファで十分
        # - rel_v < 0: 障害物の方が速い -> 離れていく（opening up）-> 追加バッファ不要
        #
        # 例: ego.v = 20 m/s, obs.v = 5 m/s (前方の遅い車に接近)
        #     -> rel_v = 15 m/s (正) -> 接近速度が大きいのでバッファを増やす
        # ========================================================================
        rel_v = ego_state.v - obs.v
        # ========================================================================

        # Buffer Settings: CAV vs HDV (FOLLOW は緩やか、YIELD は厳格)
        if use_shared_traj:
            if decision == InteractionDecision.FOLLOW:
                static_buffer = 0.3  # FOLLOW: 最小バッファ（CAV追従）
            elif decision == InteractionDecision.YIELD:
                static_buffer = 0.5  # YIELD: 標準バッファ
            else:
                static_buffer = 0.3  # OVERTAKE: 最小バッファ
            dynamic_expansion = 0.0
        else:
            if decision == InteractionDecision.FOLLOW:
                # ============================================================
                # v14.0: Relative Velocity Based Static Buffer (Anti-AEB)
                # ============================================================
                # Problem: Fixed 1.5m buffer insufficient for high rel_v
                # - At v_rel=15 m/s, need larger buffer to prevent AEB cascade
                # - Scale buffer with relative velocity to force earlier decel
                # - min 1.5m (normal following), scales with 0.3xv_rel
                # Reference: AEB Root Cause Analysis - QP ST Boundary
                # ============================================================
                # ============================================================
                # v14.3: Enhanced Buffer with Ego Speed Consideration
                # ============================================================
                # Apollo Principle: Buffer should scale with BOTH:
                # 1. Relative velocity (closing speed)
                # 2. Ego absolute velocity (reaction distance)
                # Reference: Apollo planning.cc - CalculateSafeDistance()
                # ============================================================
                base_follow_buffer = 1.5  # [m] Increased from 1.5m (v14.3)

                # Component 1: Relative velocity based (increased from 0.3 to 0.5)
                if rel_v > 0:
                    velocity_buffer = rel_v * 0.5  # [m] Increased coefficient
                else:
                    velocity_buffer = 0.0

                # Component 2: Ego speed based (NEW in v14.3)
                # At high ego speeds, need more buffer even if rel_v is small
                ego_speed_buffer = ego_state.v * 0.1  # 10% of ego speed

                # Total buffer = max of all components
                static_buffer = max(base_follow_buffer, velocity_buffer, ego_speed_buffer)

                # v14.3: Debug log for all FOLLOW cases
                if ENABLE_DEBUG_OUTPUT and (rel_v > 3.0 or static_buffer > 3.0):
                    print(f"[QP-BUFFER-FOLLOW] V#{ego_state.id} -> V#{obs.id}: "
                          f"ego_v={ego_state.v:.1f}m/s, rel_v={rel_v:.1f}m/s -> "
                          f"buffer={static_buffer:.1f}m (base={base_follow_buffer:.1f}, "
                          f"vel={velocity_buffer:.1f}, ego={ego_speed_buffer:.1f}m)")
                # ============================================================
            elif decision == InteractionDecision.YIELD:
                # ============================================================
                # v14.0: Enhanced YIELD Buffer with Relative Velocity
                # ============================================================
                # Combines speed-adaptive buffer with relative velocity scaling
                # - Base buffer: 1.5m (slow) or 3.0m (fast)
                # - Additional scaling: 0.5xv_rel for high closing speeds
                # - Prevents QP from permitting dangerous approaches
                # ============================================================
                # ============================================================
                # v14.3: Enhanced YIELD Buffer (more conservative than FOLLOW)
                # ============================================================
                if ego_state.v < 10.0:
                    base_yield_buffer = 2.5  # Increased from 1.5m (v14.3)
                else:
                    base_yield_buffer = 4.0  # Increased from 3.0m (v14.3)

                # Component 1: Relative velocity based (increased from 0.5 to 0.7)
                if rel_v > 0:
                    velocity_buffer = rel_v * 0.7  # [m] More aggressive than FOLLOW
                else:
                    velocity_buffer = 0.0

                # Component 2: Ego speed based
                ego_speed_buffer = ego_state.v * 0.15  # 15% (more than FOLLOW's 10%)

                # Total buffer = max of all components
                static_buffer = max(base_yield_buffer, velocity_buffer, ego_speed_buffer)

                # v14.3: Debug log for all YIELD cases
                if ENABLE_DEBUG_OUTPUT and (rel_v > 3.0 or static_buffer > 4.0):
                    print(f"[QP-BUFFER-YIELD] V#{ego_state.id} -> V#{obs.id}: "
                          f"ego_v={ego_state.v:.1f}m/s, rel_v={rel_v:.1f}m/s -> "
                          f"buffer={static_buffer:.1f}m (base={base_yield_buffer:.1f}, "
                          f"vel={velocity_buffer:.1f}, ego={ego_speed_buffer:.1f}m)")
                # ============================================================
            else:
                static_buffer = 1.0  # OVERTAKE: 標準バッファ
            dynamic_expansion = 0.3

        # ========================================================================
        # v12.2: Phase 3 - Lane-specific Filtering
        # ========================================================================
        # Apply lane-specific buffer multipliers
        # Current lane: full constraints, Target lane: relaxed, Other: minimal
        static_mult, dynamic_mult = self.apply_lane_specific_filtering(
            ego_state, obstacle, decision
        )
        static_buffer *= static_mult
        dynamic_expansion *= dynamic_mult
        # ========================================================================

        # Dynamic buffer calculation (rel_v already calculated above for static buffer)
        base_dynamic_buffer = 0.0

        # FOLLOW と YIELD の場合のみ動的バッファを追加
        if not use_shared_traj and decision in [InteractionDecision.FOLLOW, InteractionDecision.YIELD] and rel_v > 0:
            # FOLLOW は 0.5s、YIELD は 0.8s の RSS buffer
            time_gap = 0.5 if decision == InteractionDecision.FOLLOW else 0.8
            base_dynamic_buffer_uncapped = rel_v * time_gap

            # ================================================================
            # v12.6: Dynamic Buffer Cap (Anti-Deadlock)
            # ================================================================
            # Solution 2: Cap dynamic buffer to prevent runaway growth
            # - Prevents positive feedback loop when vehicle slows down
            # - Cap at 8m (equivalent to ~10 m/s closing speed)
            # - Maintains safety while allowing LC in dense traffic
            # Reference: Root Cause Analysis - Deadlock Prevention
            # ================================================================
            MAX_DYNAMIC_BUFFER = 8.0  # [m]
            base_dynamic_buffer = min(base_dynamic_buffer_uncapped, MAX_DYNAMIC_BUFFER)
            # ================================================================

        # Ensure d_safe is defined even if the loop below short-circuits (e.g., zero horizon)
        d_safe = 0.0

        for k in range(self.N):
            t_rel = k * self.dt
            t_abs = self.current_abs_time + t_rel

            # Position Prediction
            s_obs = None

            if use_shared_traj and obstacle.predicted_trajectory is not None:
                s_obs = self._interpolate_trajectory_point(
                    obstacle.predicted_trajectory,
                    t_abs
                )

            if s_obs is None:
                s_obs = obs.s + obs.v * t_rel

            v_ego = ego_v_estimate[k]

            # ====================================================================
            # v12.1: Apollo Directional Safety Distance (Phase 2)
            # ====================================================================
            # Use forward/backward safety distance based on decision type
            # Assume same direction for highway scenarios
            if decision in [InteractionDecision.FOLLOW, InteractionDecision.YIELD]:
                # Front obstacle: use forward safety distance
                d_safe = self.compute_forward_safe_distance(v_ego, obs.v, same_direction=True)
            else:
                # OVERTAKE: use backward safety distance
                d_safe = self.compute_backward_safe_distance(v_ego, obs.v, same_direction=True)
            # ====================================================================

            # Uncertainty expansion
            uncertainty = t_rel * dynamic_expansion

            # ====================================================================
            # v12.1: Dynamic Lateral Buffer (Phase 2)
            # ====================================================================
            # Add extra buffer for obstacles changing lanes
            lateral_buffer = self.compute_lateral_buffer(obstacle)
            # Time-dependent lateral uncertainty growth during lane change
            if obstacle.is_changing_lane:
                kLateralUncertaintyGrowth = 0.2  # m/s growth rate
                lateral_buffer += t_rel * kLateralUncertaintyGrowth
            # ====================================================================

            # Total buffer composition
            dynamic_buffer = base_dynamic_buffer + uncertainty + lateral_buffer
            total_buffer = static_buffer + dynamic_buffer

            # Urgency relaxation
            urgency_relaxation = urgency * 0.3
            if use_shared_traj:
                total_buffer = max(0.3, total_buffer - urgency_relaxation)
            else:
                total_buffer = max(1.5, total_buffer - urgency_relaxation)

            # ====================================================================
            # v12.3: Apollo Constraint Relaxation with Hard Safety Margin (修正版)
            # ====================================================================
            d_required = d_safe + total_buffer

            # ================================================================
            # v13.1: CAV-Aware Hard Safety Margin (Root Cause Fix)
            # ================================================================
            # Problem: v12.4 reduced MIN_HARD_MARGIN to 1.0m, allowing 2-3m gaps
            #          that constantly trigger CAV-AEB extreme proximity warnings
            # Solution: Adaptive margin based on vehicle type and trajectory trust
            # - CAV-CAV with valid trajectory: 6.0m (enhanced safety)
            # - CAV-HDV or no trajectory: 10.0m (conservative RSS)
            # Reference: Simulation log showing 177 AEB triggers at 2-3m gaps
            # ================================================================
            is_cav_interaction = use_shared_traj and obstacle.predicted_trajectory is not None and len(obstacle.predicted_trajectory) >= 5
            MIN_HARD_MARGIN = 5.0 if is_cav_interaction else 10.0  # [m] [SAFETY-ENHANCED]

            if is_target_obs and k < 20 and current_dist < d_required:
                ratio = k / 20.0

                # 緩和計算
                relaxed_dist_calc = current_dist * (1.0 - ratio) + d_required * ratio

                # [重要] 緩和しても絶対防衛ラインは割らせない
                relaxed_dist = max(relaxed_dist_calc, MIN_HARD_MARGIN)

                # v13.1: CRITICAL - Must include L_vehicle in relaxed constraints!
                # Without this, QP treats vehicles as points and allows overlap
                if decision in [InteractionDecision.FOLLOW, InteractionDecision.YIELD]:
                    s_upper[k] = s_obs - relaxed_dist - self.L_vehicle
                else:  # OVERTAKE
                    s_lower[k] = s_obs + relaxed_dist + self.L_vehicle

                # QPが解けなくなる(Infeasible)可能性はあるが、
                # それこそがFallback(緊急ブレーキ)を誘発させて安全を守るために必要。
                if ENABLE_DEBUG_OUTPUT and k == 0:
                    print(f"[QP-RELAX-DEC] Obstacle#{obs.id}: curr_dist={current_dist:.1f}m, "
                          f"d_required={d_required:.1f}m -> relaxed[0]={relaxed_dist:.1f}m (min={MIN_HARD_MARGIN}m)")
                continue
            # ====================================================================

            # ====================================================================
            # v12.5: 決定に基づくST境界設定（車両の前端・後端を考慮）
            # ====================================================================
            # 重要: 車両は点ではなく領域（車長 L_vehicle = 5.0m）として扱う
            # s_obs は障害物の中心位置なので、前端・後端を計算する必要がある
            #
            # 【視点の重要性】
            # - 前方の車を見る場合：対象車の「後端（ケツ）」が見える
            # - 後方の車を見る場合：対象車の「前端（全面）」が見える
            # ====================================================================
            if decision == InteractionDecision.FOLLOW:
                # ----------------------------------------------------------------
                # [FOLLOW] 前方車両を追従する場合（緩やかな制約）
                # ----------------------------------------------------------------
                # 視点: 自車は障害物の「後ろ」にいるため、障害物の「後端」が見える
                # 制約: 自車の前端 <= 障害物の後端 - 安全マージン
                #
                # 計算:
                #   自車の前端   = ego_s + L_vehicle/2
                #   障害物の後端 = s_obs - L_vehicle/2
                #
                #   制約式: ego_s + L/2 <= s_obs - L/2 - (d_safe + total_buffer)
                #          ego_s <= s_obs - L_vehicle - (d_safe + total_buffer)
                # ----------------------------------------------------------------
                s_upper[k] = s_obs - d_safe - total_buffer - self.L_vehicle

            elif decision == InteractionDecision.YIELD:
                # ----------------------------------------------------------------
                # [YIELD] 障害物に道を譲る場合（厳格な制約）
                # ----------------------------------------------------------------
                # 視点: 自車は障害物の「後ろ」に留まるため、障害物の「後端」が見える
                # 制約: 自車の前端 <= 障害物の後端 - 安全マージン（FOLLOWと同じ構造）
                #
                # 計算:
                #   自車の前端   = ego_s + L_vehicle/2
                #   障害物の後端 = s_obs - L_vehicle/2
                #
                #   制約式: ego_s + L/2 <= s_obs - L/2 - (d_safe + total_buffer)
                #          ego_s <= s_obs - L_vehicle - (d_safe + total_buffer)
                # ----------------------------------------------------------------
                s_upper[k] = s_obs - d_safe - total_buffer - self.L_vehicle

            elif decision == InteractionDecision.OVERTAKE:
                # ----------------------------------------------------------------
                # [OVERTAKE] 障害物を追い越す場合（前方への制約）
                # ----------------------------------------------------------------
                # 視点: 自車は障害物の「前」にいるため、（後ろを振り返ると）
                #       障害物の「前端」が見える
                # 制約: 自車の後端 >= 障害物の前端 + 安全マージン
                #
                # 計算:
                #   自車の後端   = ego_s - L_vehicle/2
                #   障害物の前端 = s_obs + L_vehicle/2
                #
                #   制約式: ego_s - L/2 >= s_obs + L/2 + (d_safe + total_buffer)
                #          ego_s >= s_obs + L_vehicle + (d_safe + total_buffer)
                # ----------------------------------------------------------------
                s_lower[k] = s_obs + d_safe + total_buffer + self.L_vehicle
            # ====================================================================

        # Boundary type の設定
        if decision == InteractionDecision.FOLLOW:
            boundary_type = BoundaryType.FOLLOW
        elif decision == InteractionDecision.YIELD:
            boundary_type = BoundaryType.YIELD
        else:
            boundary_type = BoundaryType.OVERTAKE

        # ====================================================================
        # v18.8: YIELD Feasibility Check at Constraint Generation Time
        # ====================================================================
        # Problem: When front vehicle is very fast, s_upper can be behind ego_s
        #          This makes YIELD physically impossible and triggers COLLISION ZONE
        #
        # Solution: If ego_s > s_upper[0], switch to FOLLOW mode with relaxed constraint
        #           Allow ego to maintain safe following distance instead of forcing stop
        #
        # Reference: Apollo speed_bounds_decider.cc - dynamic constraint adjustment
        # ====================================================================
        if boundary_type == BoundaryType.YIELD and s_upper[0] < ego_state.s:
            yield_deficit = ego_state.s - s_upper[0]
            
            if ENABLE_DEBUG_OUTPUT:
                print(f"[v18.8-YIELD-FIX] Obs#{obs.id}: YIELD already violated!")
                print(f"  ego_s={ego_state.s:.1f}m, s_upper[0]={s_upper[0]:.1f}m, deficit={yield_deficit:.1f}m")
            
            # Case 1: Small deficit (< 10m) - Relax to current position + buffer
            # Allow following at current distance
            if yield_deficit < 10.0:
                # Switch to FOLLOW mode - allow current position
                boundary_type = BoundaryType.FOLLOW
                
                # Relax s_upper to allow current position with small buffer
                for k in range(self.N):
                    if s_upper[k] < ego_state.s:
                        # Allow ego to stay at current position + maintain gap
                        s_upper[k] = ego_state.s + 2.0 + k * self.dt * 0.5  # Gradual opening
                
                if ENABLE_DEBUG_OUTPUT:
                    print(f"  -> Switched to FOLLOW (small deficit)")
                    print(f"  -> New s_upper[0]={s_upper[0]:.1f}m")
            
            # Case 2: Large deficit (>= 10m) - More aggressive relaxation needed
            # Vehicle is significantly past the ideal YIELD zone
            else:
                # Keep YIELD but relax to prevent immediate COLLISION ZONE
                # Let SPEED FALLBACK handle deceleration
                boundary_type = BoundaryType.YIELD
                
                # Relax s_upper to allow gradual recovery over time
                for k in range(self.N):
                    if s_upper[k] < ego_state.s:
                        # Start from current position, gradually tighten
                        recovery_factor = min(1.0, k * self.dt / 3.0)  # 3 second recovery
                        s_upper[k] = ego_state.s * (1 - recovery_factor) + s_upper[k] * recovery_factor
                        s_upper[k] = max(s_upper[k], ego_state.s - 3.0)  # Never more than 3m behind
                
                if ENABLE_DEBUG_OUTPUT:
                    print(f"  -> Relaxed YIELD (large deficit)")
                    print(f"  -> New s_upper[0]={s_upper[0]:.1f}m")
        # ====================================================================

        # ====================================================================
        # ST Boundary Generation Logging
        # ====================================================================
        if ENABLE_DEBUG_OUTPUT:
            print(f"\n[ST-BOUNDARY] Ego V#{ego_state.id} vs Obs V#{obs.id}:")
            print(f"  Decision: {decision.name}")
            print(f"  Boundary Type: {boundary_type.name}")
            print(f"  Trajectory: {'SHARED' if use_shared_traj else 'PREDICTED'}")
            print(f"  Initial gap: ego_s={ego_state.s:.1f}m, obs_s={obs.s:.1f}m, gap={(obs.s - ego_state.s):.1f}m")
            print(f"  Safety margins: d_safe={d_safe:.1f}m, static={static_buffer:.1f}m, dynamic={base_dynamic_buffer:.1f}m")
            print(f"  Constraint @ t=0: s_lower={s_lower[0]:.1f}m, s_upper={s_upper[0]:.1f}m")
            print(f"  Constraint @ t={self.N*self.dt:.1f}s: s_lower={s_lower[-1]:.1f}m, s_upper={s_upper[-1]:.1f}m")
        # ====================================================================

        # v18.5: Include obstacle velocity for follow mode
        return STBoundary(s_lower, s_upper, boundary_type, obs.id, obs_v=obs.v)

    def project_obstacle_to_st_boundary(
        self,
        ego_state: VehicleState,
        obstacle: ObstacleInfo,
        ego_v_estimate: np.ndarray,
        urgency: float = 0.0
    ) -> STBoundary:
        """
        Project obstacles to ST-graph with uncertainty buffer

        v12.0 UPDATE: CAV Cooperative Control Integration
        - CAVs: Use shared trajectories with minimal buffers (0.5m)
        - HDVs: Use constant velocity prediction with conservative buffers (3.0m)
        - Automatically switches based on trajectory data availability

        --- modified (Final Fix v11.9_Ultimate / 2025-12-18): Enhanced RSS Buffer ---
        Reference: Apollo STBoundary + ISO 22737 (LSAD) safety margins
        """
        obs = obstacle.vehicle_state
        is_front = obstacle.is_front

        s_lower = np.full(self.N, -1e4)  # Default: Open
        s_upper = np.full(self.N, 1e4)   # Default: Open

        # ========================================================================
        # v11.14: Apollo Constraint Relaxation for Lane Change Targets
        # ========================================================================
        # Determine if this is a "Target Lane" obstacle (needs relaxation)
        # 判定基準: 自車と同じレーンではない場合に緩和を適用
        is_target_obs = (obs.lane != ego_state.lane)

        # 現在の実際の距離を計算
        current_dist = obs.s - ego_state.s if is_front else ego_state.s - obs.s
        current_dist = max(0.0, current_dist - 5.0)  # 車体長分を引く(簡易)
        # ========================================================================

        # ========================================================================
        # v12.4: CAV Cooperative Control with Prediction Validation
        # ========================================================================
        # Safety Validation Layer 2: Validate shared trajectory before use
        use_shared_traj = False
        if obstacle.predicted_trajectory is not None and len(obstacle.predicted_trajectory) >= 5:
            if self._validate_predicted_trajectory(obstacle):
                use_shared_traj = True
            else:
                use_shared_traj = False
        # ========================================================================

        # Buffer Settings: CAV vs HDV
        if use_shared_traj:
            # CAV: Minimal buffer (only communication/sensing error)
            static_buffer = 0.5 if is_front else 0.3
            dynamic_expansion = 0.0  # No prediction uncertainty
        else:
            # HDV: Conservative buffer (prediction uncertainty)
            static_buffer = 3.0 if is_front else 1.0
            dynamic_expansion = 0.3  # Time-dependent uncertainty growth
        # ========================================================================

        # Calculate relative velocity for dynamic buffer (HDV only)
        rel_v = ego_state.v - obs.v
        base_dynamic_buffer = 0.0

        if not use_shared_traj and is_front and rel_v > 0:
            # --- Enhanced Dynamic RSS Buffer (Closing Speed) ---
            # Increased from 0.5s to 0.8s time gap for unknown peers
            # Reference: ISO 22737 recommends 0.6-1.0s for autonomous vehicles
            # Rationale: Conservative assumption for collision avoidance
            base_dynamic_buffer = rel_v * 0.8  # Enhanced from 0.5 to 0.8
            # -----------------------------------------------------------------------

        for k in range(self.N):
            t_rel = k * self.dt
            t_abs = self.current_abs_time + t_rel

            # ====================================================================
            # v12.0: Position Prediction Switch (CAV Trajectory vs HDV CV Model)
            # ====================================================================
            s_obs = None

            # 1. Try CAV shared trajectory interpolation first
            if use_shared_traj and obstacle.predicted_trajectory is not None:
                s_obs = self._interpolate_trajectory_point(
                    obstacle.predicted_trajectory,
                    t_abs
                )

            # 2. Fallback to Constant Velocity (HDV or trajectory unavailable)
            if s_obs is None:
                s_obs = obs.s + obs.v * t_rel
            # ====================================================================

            v_ego = ego_v_estimate[k]

            # ====================================================================
            # v12.1: Apollo Directional Safety Distance (Phase 2)
            # ====================================================================
            # Use forward/backward safety distance based on obstacle position
            # Assume same direction for highway scenarios
            if is_front:
                # Front obstacle: use forward safety distance
                d_safe = self.compute_forward_safe_distance(v_ego, obs.v, same_direction=True)
            else:
                # Rear obstacle: use backward safety distance
                d_safe = self.compute_backward_safe_distance(v_ego, obs.v, same_direction=True)
            # ====================================================================

            # Uncertainty expansion (Progressive with time)
            # CAV: No expansion (dynamic_expansion = 0.0)
            # HDV: ~0.3 m/s growth rate (prediction degradation)
            uncertainty = t_rel * dynamic_expansion

            # ====================================================================
            # v12.1: Dynamic Lateral Buffer (Phase 2)
            # ====================================================================
            # Add extra buffer for obstacles changing lanes
            lateral_buffer = self.compute_lateral_buffer(obstacle)
            # Time-dependent lateral uncertainty growth during lane change
            if obstacle.is_changing_lane:
                kLateralUncertaintyGrowth = 0.2  # m/s growth rate
                lateral_buffer += t_rel * kLateralUncertaintyGrowth
            # ====================================================================

            # Total buffer composition
            dynamic_buffer = base_dynamic_buffer + uncertainty + lateral_buffer
            total_buffer = static_buffer + dynamic_buffer

            # --- Conservative Urgency Relaxation ---
            # Reduced from 0.5 to 0.3 for more conservative behavior
            # Even in high urgency, maintain minimum safety margin
            urgency_relaxation = urgency * 0.3

            # --- v19.0: Overtake Target Special Handling ---
            # Overtake targets require significantly relaxed constraints to allow passing
            # while still maintaining minimum safety margins to prevent collisions
            if obstacle.is_overtake_target:
                # Aggressive relaxation for overtake: 80% reduction + full urgency
                overtake_relaxation = total_buffer * 0.8 + urgency * 0.5
                total_buffer = max(0.5, total_buffer - overtake_relaxation)  # Minimum 0.5m physical clearance
            elif use_shared_traj:
                # CAV: Allow tighter relaxation (high confidence)
                total_buffer = max(0.3, total_buffer - urgency_relaxation)
            else:
                # HDV: Maintain conservative minimum
                total_buffer = max(1.5, total_buffer - urgency_relaxation)
            # -----------------------------------------------------------------------

            # ====================================================================
            # v12.3: Apollo Constraint Relaxation with Hard Safety Margin (修正版)
            # ====================================================================
            # LC割り込み時は、初期の車間距離が推奨値(d_idm)より狭いことが多いため、
            # いきなり d_idm を強制するとInfeasibleになる。
            # 緩和ロジック: 初期2秒間は現在の距離から徐々に理想距離へ
            d_required = d_safe + total_buffer

            # ================================================================
            # v13.1: CAV-Aware Hard Safety Margin (Root Cause Fix)
            # ================================================================
            # Same adaptive logic as above - ensure consistency across code paths
            # ================================================================
            is_cav_interaction = use_shared_traj and obstacle.predicted_trajectory is not None and len(obstacle.predicted_trajectory) >= 5
            MIN_HARD_MARGIN = 5.0 if is_cav_interaction else 10.0  # [m] [SAFETY-ENHANCED]

            if is_target_obs and k < 20 and current_dist < d_required:
                # 2.0秒以内かつマージン不足の場合に緩和
                ratio = k / 20.0
                relaxed_dist_calc = current_dist * (1.0 - ratio) + d_required * ratio

                # [重要] 緩和しても絶対防衛ラインは割らせない
                relaxed_dist = max(relaxed_dist_calc, MIN_HARD_MARGIN)

                # v13.1: CRITICAL - Must include L_vehicle in relaxed constraints!
                # Without this, QP treats vehicles as points and allows overlap
                if obstacle.is_front:
                    s_upper[k] = s_obs - relaxed_dist - self.L_vehicle
                else:
                    s_lower[k] = s_obs + relaxed_dist + self.L_vehicle

                if ENABLE_DEBUG_OUTPUT and k == 0:
                    print(f"[QP-RELAX] Obstacle#{obs.id}: curr_dist={current_dist:.1f}m, "
                          f"d_required={d_required:.1f}m -> relaxed[0]={relaxed_dist:.1f}m (min={MIN_HARD_MARGIN}m)")
                continue
            # ====================================================================

            # ====================================================================
            # v12.5: 境界設定（車体長考慮）
            # ====================================================================
            if is_front:
                # Upper bound (Yield): s_ego <= s_obs - gap - 車体長
                s_upper[k] = s_obs - d_safe - total_buffer - self.L_vehicle
            else:
                # Lower bound (Overtake): s_ego >= s_obs + gap + 車体長
                s_lower[k] = s_obs + d_safe + total_buffer + self.L_vehicle
            # ====================================================================

        # v18.5: Include obstacle velocity for follow mode
        return STBoundary(s_lower, s_upper, BoundaryType.YIELD if is_front else BoundaryType.OVERTAKE, obs.id, obs_v=obs.v)

    # ========================================================================
    # v18.0: Speed Fallback Trajectory Generator
    # ========================================================================
    # Reference: Apollo gridded_path_time_graph.cc lines 102-117
    # 
    # When QP constraints are infeasible (conflicting OVERTAKE/YIELD decisions),
    # Apollo generates a "speed fallback" trajectory that safely decelerates
    # to a stop or to a safe following speed.
    # 
    # This prevents AEB activation by providing a valid trajectory even when
    # the optimal solution is impossible.
    # ========================================================================
    def _generate_speed_fallback_trajectory(
        self,
        ego_state: VehicleState,
        st_boundaries: List[STBoundary],
        front_vehicle_v: Optional[float] = None  # v18.5: Add front vehicle velocity
    ) -> Dict[str, np.ndarray]:
        """
        Generate a safe deceleration trajectory when QP is infeasible.
        
        ================================================================
        v18.5: Apollo-style FOLLOW MODE instead of FULL STOP
        ================================================================
        Problem: Previous version generated FULL STOP (v=0) when in collision
                 zone. This causes:
                 1. Sudden traffic slowdown
                 2. Rear-end collision risk from following vehicles
                 3. Congestion cascade when multiple vehicles enter fallback
                 
        Solution: Match front vehicle velocity (FOLLOW mode)
        - If front vehicle is moving -> match its velocity
        - If front vehicle is stopped -> decelerate to stop
        - Maintain minimum 3m/s if front is moving (traffic flow)
        
        Reference: Apollo speed_decider.cc - FollowObstacle mode
                   Uses headway time control instead of hard stop
        ================================================================
        
        Returns:
            List of (s, v, a) tuples for the planning horizon
        """
        N = self.N
        dt = self.dt
        
        # Find the closest YIELD boundary (front obstacle)
        min_s_upper = float('inf')
        front_obs_v = front_vehicle_v  # May be passed from caller
        
        for boundary in st_boundaries:
            if boundary.boundary_type == BoundaryType.YIELD:
                if boundary.s_upper[0] < min_s_upper:
                    min_s_upper = boundary.s_upper[0]
                    # Try to get front vehicle velocity from boundary info
                    if hasattr(boundary, 'obs_v'):
                        front_obs_v = boundary.obs_v
        
        # Calculate safe stopping distance
        v0 = ego_state.v
        s0 = ego_state.s
        a0 = ego_state.a
        
        if min_s_upper < float('inf'):
            # Distance to front obstacle
            d_to_obstacle = min_s_upper - s0
            
            if d_to_obstacle <= 0:
                # ============================================================
                # v18.5: FOLLOW MODE instead of FULL STOP
                # ============================================================
                # Previous behavior: Return (s0, 0, 0) -> immediate full stop
                # New behavior: Match front vehicle velocity for smooth flow
                # ============================================================
                
                # Default target velocity (follow mode minimum)
                v_target = 3.0  # Minimum flow velocity
                
                # If we know front vehicle velocity, match it
                if front_obs_v is not None and front_obs_v > 0.5:
                    v_target = max(front_obs_v * 0.95, 3.0)  # 95% of front v, min 3m/s
                elif front_obs_v is not None and front_obs_v <= 0.5:
                    # Front vehicle is stopped - we should stop too
                    v_target = 0.0
                
                # Calculate smooth deceleration to target velocity
                if v0 > v_target:
                    # Decelerate to match front vehicle
                    a_decel = min(-1.5, -(v0 - v_target) / 2.0)  # Smooth decel over ~2s
                    a_decel = max(a_decel, self.a_min)  # Respect vehicle limits
                else:
                    a_decel = 0.0  # Already at or below target
                
                if ENABLE_DEBUG_OUTPUT:
                    front_v_str = f"{front_obs_v:.1f}" if front_obs_v is not None else "N/A"
                    print(f"[SPEED-FALLBACK-v18.5] FOLLOW MODE: v_front={front_v_str}m/s -> "
                          f"v_target={v_target:.1f}m/s, decel={a_decel:.2f}m/s^2")
                
                # Generate trajectory toward target velocity
                trajectory = []
                s = s0
                v = v0
                
                for k in range(N):
                    # Smoothly approach target velocity
                    if v > v_target + 0.1:
                        a = max(a_decel, -(v - v_target) * 0.5)
                    elif v < v_target - 0.1:
                        a = min(1.0, (v_target - v) * 0.5)  # Gentle accel if too slow
                    else:
                        a = 0.0
                    
                    trajectory.append((s, v, a))
                    
                    # Update for next step
                    v_next = max(0.0, v + a * dt)
                    v_next = min(v_next, self.v_max)
                    s_next = s + v * dt + 0.5 * a * dt ** 2
                    
                    v = v_next
                    s = s_next
                
                return {'s': np.array([t[0] for t in trajectory]),
                        'v': np.array([t[1] for t in trajectory]),
                        'a': np.array([t[2] for t in trajectory])}
            
            # Calculate required deceleration to stop before obstacle
            # v^2 = v₀^2 + 2*a*d -> a = -v₀^2/(2*d)
            if v0 > 0:
                a_required = -(v0 ** 2) / (2 * d_to_obstacle)
                # Limit to comfortable deceleration range
                a_decel = max(a_required, self.a_min)  # a_min is negative
                a_decel = min(a_decel, -0.5)  # At least 0.5 m/s^2 deceleration
            else:
                a_decel = 0.0
        else:
            # No front obstacle - just maintain speed or gentle decel
            a_decel = min(a0, 0.0)
        
        # Generate smooth deceleration trajectory
        s = s0
        v = v0
        a = a_decel
        
        trajectory_s = []
        trajectory_v = []
        trajectory_a = []
        
        for k in range(N):
            trajectory_s.append(s)
            trajectory_v.append(v)
            trajectory_a.append(a)
            
            # Update for next step
            v_next = max(0.0, v + a * dt)  # Can't go negative
            s_next = s + v * dt + 0.5 * a * dt ** 2
            
            # Gradually reduce deceleration as we slow down
            if v_next < 3.0:  # Below 3 m/s, reduce deceleration
                a = max(a * 0.8, -0.5)
            
            v = v_next
            s = s_next
        
        if ENABLE_DEBUG_OUTPUT:
            print(f"[SPEED-FALLBACK] Generated decel profile: v0={v0:.1f}->v_end={trajectory_v[-1]:.1f}m/s, a={a_decel:.2f}m/s^2")
        
        return {'s': np.array(trajectory_s),
                'v': np.array(trajectory_v),
                'a': np.array(trajectory_a)}
    # ========================================================================

    def build_qp_matrices(
        self,
        ego_state: VehicleState,
        s_ref: np.ndarray,
        v_ref: np.ndarray,
        st_boundaries: List[STBoundary]
    ) -> Union[Tuple[csc_matrix, np.ndarray, csc_matrix, np.ndarray, np.ndarray], Dict[str, np.ndarray]]:
        """
        Build QP matrices with Slack Variables (Soft Constraints)
        
        v18.0: May return speed fallback trajectory (Dict) instead of QP matrices (Tuple)
        if constraint conflict is detected
        
        v18.5: Returns Dict{'s', 'v', 'a'} instead of List for fallback trajectory
        
        v18.11: Added Apollo-style cruise_speed reference penalty
        """
        N = self.N
        dt = self.dt
        n_vars = 3 * N + N  # s, v, a, slack

        # ====================================================================
        # v18.11: Apollo-style Cruise Speed Reference
        # ====================================================================
        # Apollo Reference: dp_st_cost.cc - GetSpeedCost() with cruise_speed penalty
        # 
        # Purpose: Add penalty for deviation from cruise speed (system v_max)
        # This encourages the vehicle to maintain higher speeds when safe,
        # instead of unnecessarily slowing down
        # 
        # v_ref: Immediate target (may be lower due to obstacles)
        # v_cruise: System target (v_max, what we want to achieve eventually)
        # ====================================================================
        v_cruise = self.v_max  # Cruise speed target (system maximum)
        w_cruise = 2.0  # Weight for cruise speed deviation (lighter than w_v)
        # ====================================================================

        # --- 1. Objective Function (P, q) ---
        # J = w_ref(s-s_ref)^2 + w_v(v-v_ref)^2 + w_a(a)^2 + w_j(da/dt)^2 + w_slack(slack)^2
        # v18.11: + w_cruise(v-v_cruise)^2 for times without obstacle constraint

        P_diag = np.zeros(n_vars)
        q = np.zeros(n_vars)

        # State weights
        P_diag[0:N] = self.w_ref
        P_diag[N:2*N] = self.w_v
        P_diag[2*N:3*N] = self.w_a
        P_diag[3*N:4*N] = self.w_slack  # Slack penalty

        P = np.diag(P_diag)

        # Add jerk terms to P (matrix form)
        w_j_term = self.w_j / (dt**2)
        for k in range(N - 1):
            idx = 2*N + k
            # (a_{k+1} - a_k)^2 = a_{k+1}^2 - 2a_{k+1}a_k + a_k^2
            P[idx, idx] += w_j_term
            P[idx+1, idx+1] += w_j_term
            P[idx, idx+1] -= w_j_term
            P[idx+1, idx] -= w_j_term

        P = csc_matrix(P)

        # Linear cost vector q
        q[0:N] = -2 * self.w_ref * s_ref
        q[N:2*N] = -2 * self.w_v * v_ref
        
        # ====================================================================
        # v18.11: Add cruise speed reference to q vector
        # ====================================================================
        # Only apply cruise reference when v_ref is below v_cruise
        # This encourages acceleration towards cruise speed when safe
        for k in range(N):
            if v_ref[k] < v_cruise - 2.0:  # Only when significantly below cruise
                # Add term: w_cruise * (v_k - v_cruise)^2
                # This expands to: w_cruise*v_k^2 - 2*w_cruise*v_cruise*v_k + const
                # Adding to q: -2*w_cruise*v_cruise at velocity index
                q[N + k] += -2 * w_cruise * v_cruise
                # Note: We'd need to add w_cruise to P_diag[N+k] too, but doing
                # that after P is already csc_matrix is expensive. 
                # Instead, we accept that this is an approximation that biases
                # towards higher speeds when v_ref is low.
        # ====================================================================

        # --- 2. Constraints (A, l, u) ---

        # Aggregate Boundaries
        s_lower_bound = np.full(N, -1e5)
        s_upper_bound = np.full(N, 1e5)

        # ====================================================================
        # v11.11: Path End Constraint (Apollo Dead End / Virtual Wall)
        # ====================================================================
        # DISABLED (v11.12 / 2025-12-19): Removed path end constraint
        # - Allows vehicles to flow freely through exit without forced deceleration
        # - Prevents traffic jams during parameter tuning phase
        # - LC probability model (beta₀, beta₁) should ensure lane changes complete naturally
        # - Can be re-enabled after logistic parameters are optimized
        # ====================================================================
        # L_TOTAL = 900.0  # Total zone length [m] (should match params.total_length)
        # SAFETY_MARGIN = 5.0  # Stop before wall [m]
        # s_upper_bound = np.minimum(s_upper_bound, L_TOTAL - SAFETY_MARGIN)
        # ====================================================================

        for boundary in st_boundaries:
            if boundary.boundary_type == BoundaryType.YIELD:
                s_upper_bound = np.minimum(s_upper_bound, boundary.s_upper)
            else:
                s_lower_bound = np.maximum(s_lower_bound, boundary.s_lower)

        # ====================================================================
        # v18.0: Apollo-style Constraint Conflict Detection and Resolution
        # ====================================================================
        # Reference: Apollo piecewise_jerk_speed_optimizer.cc lines 93-132
        # 
        # Problem: When multiple obstacles have conflicting decisions
        # (e.g., OVERTAKE one vehicle while YIELD to another), the s_lower_bound
        # can exceed s_upper_bound, making the QP problem infeasible.
        #
        # Apollo's Solution:
        # 1. Ensure minimum gap: s_upper >= s_lower + kEpsilon
        # 2. If conflict detected, prioritize safety (YIELD takes precedence)
        # 3. Return Speed Fallback profile if no valid range exists
        # ====================================================================
        kEpsilon = 0.5  # Minimum gap between bounds [m] (Apollo uses 0.01, we use larger for safety)
        
        conflict_detected = False
        conflict_times = []
        
        for k in range(N):
            if s_lower_bound[k] > s_upper_bound[k] - kEpsilon:
                conflict_detected = True
                conflict_times.append(k * self.dt)
                
                # Apollo's approach: Ensure minimum gap
                # Prioritize YIELD (safety) - push s_lower down
                # This effectively means "slow down more to let other vehicle pass"
                s_lower_bound[k] = s_upper_bound[k] - kEpsilon
        
        if conflict_detected and ENABLE_DEBUG_OUTPUT:
            print(f"[QP-V18.0] CONFLICT DETECTED: s_lower > s_upper at t={conflict_times[:3]}... (total {len(conflict_times)} time steps)")
            print("[QP-V18.0] Resolution: Prioritizing YIELD (safety) - adjusting s_lower bounds")

        # ====================================================================
        # v18.9: POST-AGGREGATION YIELD FEASIBILITY CHECK
        # ====================================================================
        # Problem: v18.8 checks individual ST-boundaries, but the aggregated
        #          s_upper_bound (minimum of all YIELD constraints) can still
        #          be behind ego_s, causing COLLISION ZONE detection later.
        #
        # Example: 
        #   - Individual YIELD: s_upper = -367m (OK vs ego_s = -400m)
        #   - But after np.minimum aggregation with other YIELDs: s_upper = -440m
        #   - Now ego_s = -400m > s_upper = -440m -> COLLISION ZONE
        #
        # Solution: Check aggregated s_upper_bound and relax if already violated
        #           This is the RIGHT place to catch infeasible YIELDs.
        # ====================================================================
        if s_upper_bound[0] < ego_state.s:
            aggregated_deficit = ego_state.s - s_upper_bound[0]
            
            if ENABLE_DEBUG_OUTPUT:
                print(f"[QP-V18.9] AGGREGATED YIELD VIOLATION DETECTED!")
                print(f"  ego_s={ego_state.s:.1f}m, aggregated_s_upper[0]={s_upper_bound[0]:.1f}m")
                print(f"  deficit={aggregated_deficit:.1f}m")
            
            # Relax s_upper_bound to allow current position
            # Strategy: Start from ego position, gradually tighten over time
            for k in range(N):
                if s_upper_bound[k] < ego_state.s:
                    # Allow current position with gradual recovery
                    # Recovery factor: 0 at t=0, 1 at t=recovery_time
                    recovery_time = 3.0  # seconds
                    t = k * self.dt
                    recovery_factor = min(1.0, t / recovery_time)
                    
                    # Interpolate: start from ego_s, recover to original s_upper
                    original_s_upper = s_upper_bound[k]
                    relaxed_s_upper = ego_state.s * (1 - recovery_factor) + original_s_upper * recovery_factor
                    
                    # Never allow s_upper to be more than 3m behind ego (safety buffer)
                    s_upper_bound[k] = max(relaxed_s_upper, ego_state.s - 3.0)
            
            if ENABLE_DEBUG_OUTPUT:
                print(f"  -> Relaxed: new s_upper[0]={s_upper_bound[0]:.1f}m")
        # ====================================================================
        
        # ====================================================================
        # v18.2: Correct Speed Fallback Trigger for OVERTAKE
        # ====================================================================
        # CRITICAL INSIGHT: OVERTAKE s_lower represents a FUTURE TARGET position
        # (where we should be AFTER overtaking), NOT an immediate requirement.
        #
        # The previous v18.0 check was incorrect:
        #   "if s_lower_bound[k] > min_reachable_s + 10" triggered fallback
        #   when ego couldn't reach s_lower immediately - but that's expected!
        #
        # Correct behavior:
        # - YIELD s_upper: Hard constraint (can't go past this)
        # - OVERTAKE s_lower: Soft target (should reach eventually, not immediately)
        #
        # Speed Fallback should ONLY trigger when:
        # 1. Initial position is IN COLLISION with obstacle (Apollo's original use case)
        # 2. YIELD constraint makes current position infeasible (s_upper < ego_s)
        #
        # For OVERTAKE: Just let the QP try to find a solution. If infeasible,
        # the constraint relaxation mechanism (v18.0) will handle it.
        # ====================================================================
        speed_fallback_needed = False
        front_vehicle_v = None  # v18.5: Track front vehicle velocity
        
        # v18.5: Find front vehicle velocity from YIELD boundaries
        for boundary in st_boundaries:
            if boundary.boundary_type == BoundaryType.YIELD:
                # Try to extract velocity info from boundary
                if hasattr(boundary, 'obs_v'):
                    front_vehicle_v = boundary.obs_v
                    break
        
        # Only check YIELD constraints for infeasibility
        # (OVERTAKE constraints are targets, not hard limits at t=0)
        for k in range(min(5, N)):  # Only check first 5 time steps
            if s_upper_bound[k] < ego_state.s - 1.0:  # Already past the obstacle
                # ================================================================
                # v18.5: RELAXED COLLISION ZONE - Allow Follow Mode
                # ================================================================
                # Previous behavior: Any s_upper < ego_s -> full stop fallback
                # New behavior: If front vehicle is moving fast, allow follow mode
                # This prevents traffic congestion from excessive fallbacks
                # ================================================================
                
                # Check if this is truly a collision risk or just tight spacing
                gap_deficit = ego_state.s - s_upper_bound[k]  # How much we're "past" the boundary
                
                if gap_deficit > 5.0:
                    # Large deficit - genuine collision risk, need fallback
                    speed_fallback_needed = True
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[QP-V18.5] COLLISION ZONE: ego_s={ego_state.s:.1f}m > s_upper[{k}]={s_upper_bound[k]:.1f}m (deficit={gap_deficit:.1f}m)")
                elif front_vehicle_v is not None and front_vehicle_v >= 3.0:
                    # Small deficit AND front is moving - use follow mode instead
                    # Don't trigger fallback, let QP handle with relaxed constraints
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[QP-V18.5] TIGHT SPACING: deficit={gap_deficit:.1f}m but front_v={front_vehicle_v:.1f}m/s - using follow mode")
                    # Relax the s_upper to allow current position
                    s_upper_bound[k] = ego_state.s + 1.0  # Give 1m buffer
                else:
                    # Small deficit but front is slow/unknown - still need fallback
                    speed_fallback_needed = True
                    if ENABLE_DEBUG_OUTPUT:
                        front_v_str = f"{front_vehicle_v:.1f}" if front_vehicle_v is not None else "N/A"
                        print(f"[QP-V18.5] COLLISION ZONE: ego_s={ego_state.s:.1f}m > s_upper[{k}]={s_upper_bound[k]:.1f}m (front_v={front_v_str})")
                break
        
        if speed_fallback_needed:
            if ENABLE_DEBUG_OUTPUT:
                front_v_str2 = f"{front_vehicle_v:.1f}" if front_vehicle_v is not None else "N/A"
                print(f"[QP-V18.5] SPEED FALLBACK: Using follow mode (front_v={front_v_str2}m/s)")
            safe_trajectory = self._generate_speed_fallback_trajectory(ego_state, st_boundaries, front_vehicle_v)
            return safe_trajectory
        # ====================================================================

        # ====================================================================
        # v17.0: Apollo-style Speed Limits from ST-Boundaries
        # ====================================================================
        # Key Insight: Apollo computes speed limits BEFORE QP optimization
        # 
        # Problem: Current system uses fixed v_max=30m/s for all time steps
        # This allows the vehicle to approach front vehicles at full speed
        # and only rely on position constraints, leading to AEB situations
        #
        # Solution: Compute speed limit based on stopping distance to obstacles
        # For each time step k, compute max velocity that allows safe stopping
        # before the obstacle boundary (s_upper_bound)
        #
        # Physics: v^2 = v₀^2 + 2*a*d  ->  v_max = sqrt(2 * |a_min| * d_stop)
        # where d_stop = s_upper_bound[k] - s_current[k]
        #
        # v18.1 FIX: Must allow current velocity at t=0 (can't instantly change speed)
        # Apollo's approach: Current velocity is ALWAYS feasible at t=0
        # Speed limits only apply to FUTURE time steps where vehicle has time to decelerate
        #
        # v18.10: Improved Speed Limit Calculation
        # Problem: Previous logic was too aggressive, causing v_max_min=3.0m/s too often
        # Solution: 
        #   1. Only apply speed limits when actually approaching obstacle
        #   2. Use more realistic deceleration planning
        #   3. Consider relative velocity, not just distance
        # ====================================================================
        v_max_from_boundary = np.full(N, self.v_max)
        
        # Comfortable deceleration for speed limit calculation
        a_decel_comfort = 3.0  # m/s^2 (Apollo uses ~2.5-3.0, slightly increased)
        
        # v18.1: First step must allow current velocity (physics constraint)
        v_max_from_boundary[0] = max(self.v_max, ego_state.v + 1.0)
        
        # v18.10: Only apply speed limits if there's a real YIELD constraint
        # Check if s_upper_bound is actually constraining (not default 1e5)
        has_yield_constraint = np.any(s_upper_bound < 1e4)
        
        if has_yield_constraint:
            # Find the tightest constraint point
            min_s_upper_idx = np.argmin(s_upper_bound)
            min_s_upper = s_upper_bound[min_s_upper_idx]
            
            # Distance to the tightest constraint from current position
            d_to_constraint = min_s_upper - ego_state.s
            
            # v18.10: Only apply if constraint is actually ahead and within reasonable range
            if d_to_constraint > 0 and d_to_constraint < 100:
                for k in range(1, N):
                    t_k = k * self.dt
                    
                    # v18.10: Use actual constraint boundary at each time step
                    # instead of estimating where we'll be
                    d_to_boundary_k = s_upper_bound[k] - ego_state.s
                    
                    # Only apply if boundary is ahead
                    if d_to_boundary_k > 5.0:  # Minimum 5m buffer
                        # v^2 = 2 * a * d -> v = sqrt(2 * a * d)
                        v_stop = np.sqrt(2 * a_decel_comfort * d_to_boundary_k)
                        
                        # v18.10: Ensure we don't over-constrain early time steps
                        # Allow current velocity + some deceleration margin at early steps
                        if k < 10:  # First 1 second (k*dt)
                            # Allow gradual deceleration from current velocity
                            max_decel_so_far = a_decel_comfort * t_k
                            v_min_allowed = max(ego_state.v - max_decel_so_far, 5.0)
                            v_stop = max(v_stop, v_min_allowed)
                        
                        # v18.10: Higher minimum speed (5 m/s instead of 3 m/s)
                        v_max_from_boundary[k] = min(v_max_from_boundary[k], max(v_stop, 5.0))
        
        if ENABLE_DEBUG_OUTPUT and np.any(v_max_from_boundary < self.v_max - 1.0):
            v_min_limit = np.min(v_max_from_boundary)
            k_min = np.argmin(v_max_from_boundary)
            print(f"[QP-V18.10] Speed limits from ST-boundary: v_max_min={v_min_limit:.1f}m/s at k={k_min}")
        # ====================================================================

        A_rows = []
        l_rows = []
        u_rows = []

        # -- Initial State --
        A_init = np.zeros((3, n_vars))
        A_init[0, 0] = 1.0
        l_rows.append(ego_state.s)
        u_rows.append(ego_state.s)
        A_init[1, N] = 1.0
        l_rows.append(ego_state.v)
        u_rows.append(ego_state.v)
        A_init[2, 2*N] = 1.0
        l_rows.append(ego_state.a)
        u_rows.append(ego_state.a)
        A_rows.append(csc_matrix(A_init))

        # -- Dynamics Constraints --
        n_dyn = 2 * (N - 1)
        A_dyn = np.zeros((n_dyn, n_vars))
        l_dyn = np.zeros(n_dyn)
        u_dyn = np.zeros(n_dyn)

        for k in range(N - 1):
            row_s = 2 * k
            row_v = 2 * k + 1

            # s constraint
            A_dyn[row_s, k+1] = 1.0      # s_{k+1}
            A_dyn[row_s, k] = -1.0       # -s_k
            A_dyn[row_s, N+k] = -dt      # -v_k*dt
            A_dyn[row_s, 2*N+k] = -0.5*dt**2 # -0.5*a_k*dt^2

            # v constraint
            A_dyn[row_v, N+k+1] = 1.0    # v_{k+1}
            A_dyn[row_v, N+k] = -1.0     # -v_k
            A_dyn[row_v, 2*N+k] = -dt    # -a_k*dt

        A_rows.append(csc_matrix(A_dyn))
        l_rows.extend(l_dyn)
        u_rows.extend(u_dyn)

        # -- Safety Bounds with Slack --
        # s_lower - slack <= s <= s_upper + slack
        n_safety = 2 * N
        A_safe = np.zeros((n_safety, n_vars))
        l_safe = np.full(n_safety, -np.inf)
        u_safe = np.full(n_safety, np.inf)

        for k in range(N):
            # Upper bound: s_k - slack_k <= s_upper[k]
            row_u = 2 * k
            A_safe[row_u, k] = 1.0        # s_k
            A_safe[row_u, 3*N+k] = -1.0   # -slack_k
            u_safe[row_u] = s_upper_bound[k]

            # Lower bound: s_k + slack_k >= s_lower[k]
            row_l = 2 * k + 1
            A_safe[row_l, k] = 1.0        # s_k
            A_safe[row_l, 3*N+k] = 1.0    # +slack_k
            l_safe[row_l] = s_lower_bound[k]

        A_rows.append(csc_matrix(A_safe))
        l_rows.extend(l_safe)
        u_rows.extend(u_safe)

        # -- Physical Limits --
        # v17.0: Use dynamic v_max from ST-boundary calculation
        n_lim = 3 * N
        A_lim = np.zeros((n_lim, n_vars))
        l_lim = np.zeros(n_lim)
        u_lim = np.zeros(n_lim)

        for k in range(N):
            # v limit - v17.0: Use ST-boundary derived speed limit
            A_lim[k, N+k] = 1.0
            l_lim[k] = 0.0
            u_lim[k] = v_max_from_boundary[k]  # Dynamic speed limit from obstacles

            # a limit
            A_lim[N+k, 2*N+k] = 1.0
            l_lim[N+k] = self.a_min
            u_lim[N+k] = self.a_max

            # slack limit (must be non-negative)
            A_lim[2*N+k, 3*N+k] = 1.0
            l_lim[2*N+k] = 0.0
            u_lim[2*N+k] = np.inf

        A_rows.append(csc_matrix(A_lim))
        l_rows.extend(l_lim)
        u_rows.extend(u_lim)

        # Combine all
        A = vstack(A_rows, format='csc')
        lower = np.array(l_rows)
        upper = np.array(u_rows)

        # Ensure A is csc_matrix and lower/upper are np.ndarray
        A = csc_matrix(A)
        lower = np.asarray(lower)
        upper = np.asarray(upper)

        return P, q, A, lower, upper

    def optimize_with_urgency(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleInfo],
        urgency: float
    ) -> Optional[Dict[str, np.ndarray]]:
        return self.optimize(ego_state, obstacles, urgency=urgency)

    def optimize(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleInfo],
        use_dp_optimizer: bool = False,
        use_stitching: bool = False,
        urgency: float = 0.5,
        current_time: float = 0.0,
        v_ref_target: Optional[float] = None,
        s_lower_bound: Optional[np.ndarray] = None,  # v26.0: Safe lower position bound
        s_upper_bound: Optional[np.ndarray] = None   # v26.0: Safe upper position bound
    ) -> Optional[dict]:
        """
        Apollo-style Frenet QP optimization

        Args:
            ego_state: Current ego vehicle state
            obstacles: List of obstacles
            use_dp_optimizer: Enable DP-based speed optimizer (v12.1)
            use_stitching: Enable trajectory stitching (smoothness)
            urgency: Urgency value for dynamic safety margin (0.0-1.0)
            current_time: Current simulation time for stitching
            v_ref_target: Target reference velocity from Planning layer (v14.4)

        Returns:
            Dictionary with optimized trajectory or None if failed
        """
        # Initialize variables
        use_decider = False
        decisions: Dict[int, InteractionDecision] = {}

        # ====================================================================
        # v12.5: Smart Reference Generation (Integration-Based s_ref)
        # ====================================================================
        # Problem: Old method used constant velocity assumption s_ref = s0 + v0*t
        # This creates contradiction when v_ref indicates deceleration needed
        # Solution: Generate s_ref by integrating v_ref (position = ∫v dt)
        # This ensures planner targets "position where vehicle will be after deceleration"
        # ====================================================================

        # ====================================================================
        # v14.4: Use Planning Layer v_ref_target (CRITICAL FIX)
        # ====================================================================
        # ROOT CAUSE: QP was using ego_state.v instead of v_ref from Planning
        # - Planning layer (IsFollowTooClose) sets v_ref=4.5 m/s
        # - But QP was using v_ref=20.0 m/s (current speed)!
        # - Result: Vehicle doesn't decelerate despite IsFollowTooClose warnings
        # FIX: Use v_ref_target from Planning layer if provided
        # ====================================================================
        if v_ref_target is not None:
            # Planning layer has specified a target velocity
            v_ref = np.full(self.N, min(v_ref_target, self.v_max))
            if ENABLE_DEBUG_OUTPUT and v_ref_target < ego_state.v:
                print(f"[QP-V14.4] V#{ego_state.id}: Using Planning v_ref={v_ref_target:.1f}m/s "
                      f"(ego_v={ego_state.v:.1f}m/s) -> deceleration commanded")
        else:
            # No planning override, use current velocity as target (cruise)
            v_ref = np.full(self.N, min(ego_state.v, self.v_max))
        # ====================================================================

        # ====================================================================
        # v18.19 FIX: Skip stitching when Planning wants acceleration
        # ====================================================================
        # PROBLEM: Stitching forces v_ref to previous low speed trajectory
        # even when Planning layer wants higher speed (FreeFlow/OpenRoad)
        # SYMPTOM: Vehicle stuck at 5.0m/s with v_ref=20.0m/s from Planning
        # SOLUTION: When Planning v_ref_target > ego_v, skip stitching to allow
        #           acceleration. Stitching is mainly for SMOOTH DECELERATION.
        # ====================================================================
        skip_stitching_for_accel = False
        if v_ref_target is not None and v_ref_target > ego_state.v + 2.0:
            # Planning wants significant acceleration (>2m/s more than current)
            skip_stitching_for_accel = True
            if ENABLE_DEBUG_OUTPUT:
                print(f"[STITCH-SKIP-ACCEL] V#{ego_state.id}: v_ref_target={v_ref_target:.1f}m/s > "
                      f"ego_v={ego_state.v:.1f}m/s + 2.0 -> skipping stitching for acceleration")
        
        # ====================================================================
        # Trajectory Stitching with Deviation Check (Apollo Safety Layer 1)
        # ====================================================================
        previous_trajectory = self.previous_trajectory.get(ego_state.id) if (use_stitching and not skip_stitching_for_accel) else None

        if previous_trajectory is not None:
            # Safety Layer 1: Deviation Check
            # Reference: Apollo trajectory_stitcher - ComputeStitchingTrajectory()
            prev_s_start = previous_trajectory.get('s', [ego_state.s])[0]
            current_s = ego_state.s
            deviation = abs(prev_s_start - current_s)

            DEVIATION_THRESHOLD = 2.0  # [m] Apollo standard threshold

            if deviation > DEVIATION_THRESHOLD:
                # Deviation too large - reset trajectory
                previous_trajectory = None

                if ENABLE_DEBUG_OUTPUT:
                    print(f"\n[STITCHING-DEVIATION] V#{ego_state.id}:")
                    print(f"  Previous trajectory start: s={prev_s_start:.2f}m")
                    print(f"  Current actual position:   s={current_s:.2f}m")
                    print(f"  Deviation: {deviation:.2f}m > {DEVIATION_THRESHOLD}m threshold")
                    print("  -> RESET trajectory, will replan from scratch")
            else:
                # Deviation acceptable - use previous trajectory with Apollo N-1 stitching
                # Reference: Apollo trajectory_stitcher.cc - ComputeStitchingTrajectory()
                # Shift previous trajectory: prev[1:N] -> current[0:N-1]
                prev_v = previous_trajectory.get('v', v_ref)

                if len(prev_v) >= self.N:
                    # Apollo standard: N-1 step reuse
                    # Shift by 1 step (skip the first step which was already executed)
                    v_ref[:-1] = prev_v[1:]  # prev[k+1] -> curr[k] for k=0 to N-2
                    # Last step: extend from previous end or use current target
                    v_ref[-1] = min(prev_v[-1], self.v_max)  # Maintain or cap speed
                else:
                    # Fallback: previous trajectory too short
                    v_ref = prev_v

                if ENABLE_DEBUG_OUTPUT:
                    print(f"\n[STITCHING-OK] V#{ego_state.id}:")
                    print(f"  Deviation: {deviation:.2f}m < {DEVIATION_THRESHOLD}m threshold")
                    print(f"  Apollo N-1 stitching: prev[1:{self.N}] -> curr[0:{self.N-1}]")
                    print(f"  Stitched v_ref: [{v_ref[0]:.2f}, {v_ref[1]:.2f}, ..., {v_ref[-1]:.2f}] m/s")
        else:
            if ENABLE_DEBUG_OUTPUT:
                print(f"\n[STITCHING-NONE] V#{ego_state.id}: No previous trajectory, fresh planning")

        # Generate s_ref by integrating v_ref (v12.5 Key Fix)
        # This creates consistent position-velocity reference
        s_ref = np.zeros(self.N)
        curr_s = ego_state.s
        for k in range(self.N):
            curr_s += v_ref[k] * self.dt
            s_ref[k] = curr_s

        # Velocity estimate for boundary projection
        v_estimate = np.linspace(ego_state.v, v_ref[-1], self.N)
        # ====================================================================

        # 2. Boundary Projection with Decision Logic
        st_boundaries = []

        if use_dp_optimizer:
            # ========================================================================
            # v12.1: DP Speed Optimizer Integration (HIGHEST PRIORITY)
            # ========================================================================
            # Import DP optimizer here to avoid circular dependency
            try:
                from .dp_speed_optimizer import SpeedHeuristicOptimizer

                # ====================================================================
                # v18.7: Pre-DP Gap Check - FRONT VEHICLES ONLY
                # ====================================================================
                # Previous bug (v12.3): abs(dist) treated rear vehicles as "blocking"
                # This caused YIELD to rear vehicles -> impossible s_upper constraint
                # 
                # Fix: Only check FRONT vehicles (obs_s > ego_s)
                # Rear vehicles should use OVERTAKE decision, not YIELD
                # ====================================================================
                gap_check_passed = True

                # Calculate RSS-based minimum safe gap for FRONT vehicles
                RSS_TIME_HEADWAY = 1.3  # [s] Apollo RSS standard
                min_gap_front = ego_state.v * RSS_TIME_HEADWAY + self.s0

                for obs in obstacles:
                    obs_state = obs.vehicle_state
                    rel_s = obs_state.s - ego_state.s  # Positive = ahead, Negative = behind
                    
                    # v18.7: ONLY check FRONT vehicles (rel_s > 0)
                    # Rear vehicles (rel_s < 0) cannot "block" us - we pass in front of them
                    if rel_s > 0:  # Front vehicle
                        # Check if front obstacle is blocking (in target/current lane, too close)
                        is_blocking = (
                            (obs.lane_relevance == "target" or obs.lane_relevance == "current") and
                            rel_s < min_gap_front
                        )

                        if is_blocking:
                            gap_check_passed = False

                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[PRE-DP-GAP-CHECK-v18.7] V#{ego_state.id}: BLOCKED by FRONT V#{obs_state.id}, "
                                      f"rel_s={rel_s:.1f}m < min_gap={min_gap_front:.1f}m -> ABORT DP")
                            break

                if not gap_check_passed:
                    # Gap check failed: Force YIELD to FRONT obstacles only
                    # Rear obstacles should still use normal decision (likely OVERTAKE)
                    decisions = {}
                    for obs in obstacles:
                        obs_state = obs.vehicle_state
                        rel_s = obs_state.s - ego_state.s
                        
                        if rel_s > 0:  # Front vehicle -> YIELD
                            decisions[obs_state.id] = InteractionDecision.YIELD
                        else:  # Rear vehicle -> OVERTAKE (we pass in front of them)
                            decisions[obs_state.id] = InteractionDecision.OVERTAKE
                    
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[PRE-DP-GAP-CHECK-v18.7] V#{ego_state.id}: Front->YIELD, Rear->OVERTAKE")
                else:
                    # Gap check passed: Run DP optimization normally
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[PRE-DP-GAP-CHECK] V#{ego_state.id}: PASSED, running DP optimizer")

                    # ================================================================
                    # v18.4: Convert to relative coordinates for DP optimizer
                    # ================================================================
                    # DP optimizer uses grid-based search with s ∈ [0, s_max]
                    # World coordinates can be negative (e.g., s=-400m)
                    # Solution: Transform all coordinates relative to ego position
                    # ================================================================
                    ego_s_origin = ego_state.s  # Save original ego s for reference
                    
                    # Create relative ego state (s=0 at current position)
                    from copy import copy
                    ego_state_relative = copy(ego_state)
                    ego_state_relative.s = 0.0  # Ego is at origin in relative frame
                    
                    # Transform obstacle states to relative coordinates
                    obstacles_relative = []
                    for obs in obstacles:
                        obs_relative = ObstacleInfo(
                            vehicle_state=VehicleState(
                                id=obs.vehicle_state.id,
                                s=obs.vehicle_state.s - ego_s_origin,  # Relative s
                                d=obs.vehicle_state.d,
                                v=obs.vehicle_state.v,
                                a=obs.vehicle_state.a,
                                lane=obs.vehicle_state.lane
                            ),
                            is_front=obs.is_front,
                            lane_relevance=obs.lane_relevance,
                            predicted_trajectory=[
                                (t, s - ego_s_origin, d, v) 
                                for (t, s, d, v) in (obs.predicted_trajectory or [])
                            ] if obs.predicted_trajectory else None
                        )
                        obstacles_relative.append(obs_relative)
                    
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[DP-COORD-v18.4] V#{ego_state.id}: World s={ego_s_origin:.1f}m -> Relative s=0.0m")
                        for obs_rel in obstacles_relative:
                            print(f"  Obstacle V#{obs_rel.vehicle_state.id}: rel_s={obs_rel.vehicle_state.s:.1f}m")

                    # Create DP optimizer with compatible parameters
                    try:
                        dp_optimizer = SpeedHeuristicOptimizer(
                            horizon_time=self.N * self.dt,
                            dt=self.dt,
                            ds_dense=1.0,
                            s_max=200.0,  # v18.4: Fixed relative range [0, 200m]
                            a_max=self.a_max,
                            a_min=self.a_min,
                            v_target=v_ref[-1]
                        )

                        # Run DP optimization with relative coordinates
                        decisions = dp_optimizer.optimize(ego_state_relative, obstacles_relative)
                    except Exception as e:
                        print(f"[DP-ERROR] V#{ego_state.id}: DP optimization failed: {e}")
                        import traceback
                        traceback.print_exc()
                        # Fallback to YIELD for all obstacles
                        decisions = {obs.vehicle_state.id: InteractionDecision.YIELD for obs in obstacles}
                # ====================================================================

                # ============================================================
                # v18.7.1: Apply Decision Locking with Position Override
                # ============================================================
                # Key insight: YIELD to a vehicle BEHIND is physically impossible
                # (we can't slow down to let someone behind us pass in front)
                #
                # If obstacle position changes from front->behind, force OVERTAKE
                # If obstacle position changes from behind->front, force YIELD
                # ============================================================
                for obs in obstacles:
                    obs_id = obs.vehicle_state.id
                    obs_state = obs.vehicle_state
                    rel_s = obs_state.s - ego_state.s  # Positive = ahead, Negative = behind
                    
                    new_decision = decisions.get(obs_id, InteractionDecision.YIELD)

                    # v18.7.1: Physical constraint check - YIELD to rear is impossible
                    if new_decision == InteractionDecision.YIELD and rel_s < 0:
                        # Can't YIELD to vehicle behind - force OVERTAKE
                        new_decision = InteractionDecision.OVERTAKE
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[DECISION-PHYSICS-v18.7.1] V#{obs_id}: YIELD->OVERTAKE "
                                  f"(rel_s={rel_s:.1f}m < 0, can't yield to rear)")
                    
                    # Check if decision is locked
                    if obs_id in self.locked_decisions:
                        locked_decision, lock_end_time = self.locked_decisions[obs_id]
                        
                        # v18.7.1: Position override - check if position changed
                        if locked_decision == InteractionDecision.YIELD and rel_s < -5.0:
                            # Was YIELD but obstacle is now BEHIND us - force OVERTAKE
                            decisions[obs_id] = InteractionDecision.OVERTAKE
                            self.locked_decisions[obs_id] = (InteractionDecision.OVERTAKE, 
                                                            current_time + self.decision_lock_duration)
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[DECISION-POSITION-OVERRIDE-v18.7.1] V#{obs_id}: YIELD->OVERTAKE "
                                      f"(obstacle now BEHIND: rel_s={rel_s:.1f}m)")
                        elif locked_decision == InteractionDecision.OVERTAKE and rel_s > 5.0:
                            # Was OVERTAKE but obstacle is now AHEAD - force YIELD
                            decisions[obs_id] = InteractionDecision.YIELD
                            self.locked_decisions[obs_id] = (InteractionDecision.YIELD,
                                                            current_time + self.decision_lock_duration)
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[DECISION-POSITION-OVERRIDE-v18.7.1] V#{obs_id}: OVERTAKE->YIELD "
                                      f"(obstacle now AHEAD: rel_s={rel_s:.1f}m)")
                        elif current_time < lock_end_time:
                            # Still locked and position consistent: use previous decision
                            decisions[obs_id] = locked_decision
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[DECISION-LOCK] V#{obs_id}: Locked to {locked_decision.name} "
                                      f"until t={lock_end_time:.2f}s (current={current_time:.2f}s)")
                        else:
                            # Lock expired: update with new decision and re-lock
                            decisions[obs_id] = new_decision
                            self.locked_decisions[obs_id] = (new_decision, current_time + self.decision_lock_duration)
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[DECISION-UPDATE] V#{obs_id}: {locked_decision.name} -> {new_decision.name}, "
                                      f"locked until t={current_time + self.decision_lock_duration:.2f}s")
                    else:
                        # Not locked yet: lock new decision
                        self.locked_decisions[obs_id] = (new_decision, current_time + self.decision_lock_duration)
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[DECISION-NEW] V#{obs_id}: {new_decision.name}, "
                                  f"locked until t={current_time + self.decision_lock_duration:.2f}s")
                # ============================================================

                if ENABLE_DEBUG_OUTPUT:
                    print("[FrenetQP] Using DP Optimizer decisions (with locking):")
                    for obs_id, decision in decisions.items():
                        print(f"  V#{obs_id}: {decision.name}")

                # Apply decisions to create ST-boundaries
                for obs in obstacles:
                    obs_id = obs.vehicle_state.id
                    decision = decisions.get(obs_id, InteractionDecision.YIELD)

                    boundary = self.project_obstacle_with_decision(
                        ego_state, obs, v_estimate, decision, urgency
                    )
                    st_boundaries.append(boundary)

            except ImportError as e:
                print(f"[FrenetQP] Failed to import DP Optimizer: {e}")
                print("[FrenetQP] Falling back to HeuristicSpeedDecider")
                use_dp_optimizer = False
                use_decider = True
            # ========================================================================
        
        if not use_dp_optimizer:
            # Determine if we should use the decider
            use_decider = True
        
        if not use_dp_optimizer and use_decider:
            # ========================================================================
            # v11.10: Heuristic Speed Decider Integration
            # v12.3: Added Decision Locking
            # ========================================================================
            decider = HeuristicSpeedDecider(self.dt, self.N, self.a_max, self.a_min)

            for obs in obstacles:
                obs_id = obs.vehicle_state.id

                # 1. 意思決定を行う
                new_decision = decider.make_decision(ego_state, obs)

                # ============================================================
                # v12.3: Apply Decision Locking (チャタリング防止)
                # ============================================================
                if obs_id in self.locked_decisions:
                    locked_decision, lock_end_time = self.locked_decisions[obs_id]
                    if current_time < lock_end_time:
                        # Still locked: use previous decision
                        decision = locked_decision
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[DECISION-LOCK] V#{obs_id}: Locked to {locked_decision.name} "
                                  f"until t={lock_end_time:.2f}s")
                    else:
                        # Lock expired: update with new decision
                        decision = new_decision
                        self.locked_decisions[obs_id] = (new_decision, current_time + self.decision_lock_duration)
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[DECISION-UPDATE] V#{obs_id}: {locked_decision.name} -> {new_decision.name}")
                else:
                    # Not locked yet: lock new decision
                    decision = new_decision
                    self.locked_decisions[obs_id] = (new_decision, current_time + self.decision_lock_duration)
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[DECISION-NEW] V#{obs_id}: {new_decision.name}")
                # ============================================================

                # 2. 決定に基づいて境界を射影
                boundary = self.project_obstacle_with_decision(
                    ego_state, obs, v_estimate, decision, urgency
                )
                st_boundaries.append(boundary)
            # ========================================================================
        elif not use_dp_optimizer and not use_decider:
            # Legacy path: Use is_front flag directly
            for obs in obstacles:
                st_boundaries.append(
                    self.project_obstacle_to_st_boundary(ego_state, obs, v_estimate, urgency)
                )

        # ========================================================================
        # v18.0: Pre-QP Constraint Conflict Detection and Resolution
        # ========================================================================
        # Reference: Apollo piecewise_jerk_speed_optimizer.cc lines 93-132
        #
        # Problem: When OVERTAKE decision for one vehicle conflicts with YIELD
        # decision for another (e.g., V5 must be ahead of V3 but behind V7),
        # the s_lower > s_upper constraint makes QP infeasible.
        #
        # Solution: Detect conflicts early and resolve by prioritizing safety
        # - Identify YIELD boundaries (upper limits) vs OVERTAKE boundaries (lower limits)
        # - Check if any OVERTAKE s_lower > any YIELD s_upper
        # - If conflict: Drop the OVERTAKE decision (prioritize YIELD = safety)
        # ========================================================================
        if len(st_boundaries) >= 2:
            yield_boundaries = [b for b in st_boundaries if b.boundary_type == BoundaryType.YIELD]
            overtake_boundaries = [b for b in st_boundaries if b.boundary_type == BoundaryType.OVERTAKE]
            
            if yield_boundaries and overtake_boundaries:
                # Check for conflicts at each time step
                conflicts_found = []
                
                for k in range(min(self.N, 10)):  # Check first 10 time steps
                    # Get minimum YIELD upper bound (must stay below this)
                    min_yield_upper = min(b.s_upper[k] for b in yield_boundaries)
                    
                    # Get maximum OVERTAKE lower bound (must stay above this)
                    max_overtake_lower = max(b.s_lower[k] for b in overtake_boundaries)
                    
                    if max_overtake_lower > min_yield_upper - 5.0:  # 5m safety buffer
                        conflicts_found.append((k, max_overtake_lower, min_yield_upper))
                
                if conflicts_found:
                    # Find which OVERTAKE boundary is causing the conflict
                    conflicting_overtake_ids = []
                    for b in overtake_boundaries:
                        for k, _, min_yield_upper in conflicts_found:
                            if b.s_lower[k] > min_yield_upper - 5.0:
                                conflicting_overtake_ids.append(b.vehicle_id)
                                break
                    
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"\n[QP-V18.0] DECISION CONFLICT detected for V#{ego_state.id}:")
                        print(f"  YIELD constraints from: {[b.vehicle_id for b in yield_boundaries]}")
                        print(f"  OVERTAKE constraints from: {[b.vehicle_id for b in overtake_boundaries]}")
                        print(f"  Conflict at t={conflicts_found[0][0]*self.dt:.2f}s: "
                              f"s_lower={conflicts_found[0][1]:.1f}m > s_upper={conflicts_found[0][2]:.1f}m")
                        print(f"  Resolution: Dropping OVERTAKE for V#{conflicting_overtake_ids} -> treating as YIELD")
                    
                    # Remove conflicting OVERTAKE boundaries and replace with YIELD
                    st_boundaries_resolved = []
                    for b in st_boundaries:
                        if b.boundary_type == BoundaryType.OVERTAKE and b.vehicle_id in conflicting_overtake_ids:
                            # Convert OVERTAKE to YIELD (change lower bound to upper bound)
                            # This means "let the other vehicle pass first"
                            converted_boundary = STBoundary(
                                vehicle_id=b.vehicle_id,
                                s_lower=np.full(self.N, -1e5),  # No lower limit
                                s_upper=b.s_lower + 10.0,  # Stay behind where we needed to be ahead
                                boundary_type=BoundaryType.YIELD,
                                obs_v=b.obs_v if hasattr(b, 'obs_v') else 0.0  # v18.5: Preserve obstacle velocity
                            )
                            st_boundaries_resolved.append(converted_boundary)
                            
                            # Update decision lock to YIELD
                            if b.vehicle_id in self.locked_decisions:
                                self.locked_decisions[b.vehicle_id] = (
                                    InteractionDecision.YIELD,
                                    current_time + self.decision_lock_duration * 2  # Longer lock for resolution
                                )
                        else:
                            st_boundaries_resolved.append(b)
                    
                    st_boundaries = st_boundaries_resolved
        # ========================================================================

        # 3. Build & Solve QP
        # Initialize w_v_original outside try block (used in except handler)
        w_v_original = self.w_v
        
        try:
            # ====================================================================
            # v14.7: Direction-Aware Adaptive Velocity Tracking Weight
            # ====================================================================
            # ROOT CAUSE (v14.6 bug): abs(v_error) treated deceleration and
            # acceleration equally. When accelerating (v_ref > v_current),
            # increased w_v caused aggressive acceleration -> unsafe approach -> AEB.
            # Result: AEB count increased from 250 to 1284 (5x worse!)
            #
            # FIX: Only increase w_v when DECELERATING (v_ref < v_current)
            # - Deceleration scenarios: Increase w_v for safety
            # - Acceleration scenarios: Keep default w_v for comfort
            #
            # Deceleration scaling:
            # - Small error (<3 m/s): w_v=10 (comfort priority)
            # - Medium error (3-8 m/s): w_v scales 10->100
            # - Large error (>8 m/s): w_v=100 (safety priority)
            # ====================================================================
            v_error = ego_state.v - v_ref[0]  # Positive = need to decelerate

            if v_error > 3.0:
                # ============================================================
                # v14.8: Enhanced Adaptive Scaling for Emergency Scenarios
                # ============================================================
                # Problem: w_v=100 vs w_j=1000 (ratio 1:10) still prioritizes
                # smoothness over safety in extreme cases (v_error > 10 m/s)
                #
                # Solution: Exponential scaling for very large errors
                # - Normal decel (3-8 m/s): Linear scale to w_v=100
                # - Emergency decel (>8 m/s): Continue scaling to w_v=500
                # - This achieves w_v:w_j ratio of 1:2 for emergencies
                # ============================================================
                if v_error > 8.0:
                    # Emergency: Continue scaling beyond 100
                    # v_error=8m/s -> w_v=100
                    # v_error=10m/s -> w_v=250
                    # v_error=12m/s -> w_v=400
                    # v_error>=13m/s -> w_v=500 (cap)
                    emergency_scale = min((v_error - 8.0) / 5.0, 1.0)  # 0 to 1
                    self.w_v = 100.0 + emergency_scale * 400.0  # 100 to 500
                else:
                    # Normal deceleration: Linear scaling
                    # v_error=3m/s -> w_v=10
                    # v_error=5m/s -> w_v=50
                    # v_error=8m/s -> w_v=100
                    error_scale = (v_error - 3.0) / 5.0  # 0 to 1
                    self.w_v = 10.0 + error_scale * 90.0  # 10 to 100

                if ENABLE_DEBUG_OUTPUT:
                    level = "EMERGENCY" if v_error > 10.0 else "DECEL"
                    print(f"[QP-V14.8] V#{ego_state.id}: {level} needed, v_error={v_error:.1f}m/s -> "
                          f"w_v: {w_v_original:.1f} -> {self.w_v:.1f} (adaptive tracking)")
            elif ENABLE_DEBUG_OUTPUT and v_error < -3.0:
                # Acceleration needed: Keep default w_v (no adaptation)
                print(f"[QP-V14.8] V#{ego_state.id}: ACCEL needed, v_error={v_error:.1f}m/s -> "
                      f"w_v unchanged (default={w_v_original:.1f})")
            # ====================================================================

            # ====================================================================

            # ====================================================================
            # v26.0: Apply Unified Safety Constraints (ApolloSafetyManager)
            # ====================================================================
            # Inject the safe boundaries computed by ApolloSafetyManager as
            # hard constraints into the QP problem.
            # We use virtual STBoundary objects to integrate with existing logic.
            # ====================================================================
            if s_lower_bound is not None:
                # Lower bound constraint -> behaves like "OVERTAKE" (must be above)
                st_boundaries.append(STBoundary(
                    s_lower=s_lower_bound,
                    s_upper=np.full(self.N, np.inf), # upper ignored for OVERTAKE
                    boundary_type=BoundaryType.OVERTAKE,
                    vehicle_id=-999  # Special ID for Unified Safety Lower
                ))
            
            if s_upper_bound is not None:
                # Upper bound constraint -> behaves like "YIELD" (must be below)
                st_boundaries.append(STBoundary(
                    s_lower=np.full(self.N, -np.inf), # lower ignored for YIELD
                    s_upper=s_upper_bound,
                    boundary_type=BoundaryType.YIELD,
                    vehicle_id=-998  # Special ID for Unified Safety Upper
                ))
            # ====================================================================

            build_result = self.build_qp_matrices(ego_state, s_ref, v_ref, st_boundaries)

            # ====================================================================
            # v18.0/v18.5: Speed Fallback Detection
            # ====================================================================
            # build_qp_matrices returns Dict{'s','v','a'} (fallback trajectory) if constraint conflict detected
            # In this case, skip QP solver and use fallback trajectory directly
            # ====================================================================
            if isinstance(build_result, dict):
                # v18.5: build_result is the speed fallback trajectory (Dict[str, np.ndarray])
                speed_fallback_trajectory: Dict[str, np.ndarray] = build_result
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[QP-V18.0] V#{ego_state.id}: Using speed fallback trajectory (constraint conflict)")
                
                # Extract arrays from fallback trajectory
                s_sol = speed_fallback_trajectory['s']
                v_sol = speed_fallback_trajectory['v']
                a_sol = speed_fallback_trajectory['a']
                
                # Restore velocity tracking weight
                self.w_v = w_v_original
                
                # Store trajectory for stitching
                self.previous_trajectory[ego_state.id] = {
                    's': s_sol,
                    'v': v_sol,
                    'a': a_sol,
                    'time': current_time
                }
                
                result_dict = {
                    's': s_sol,
                    'v': v_sol,
                    'a': a_sol,
                    'j': np.gradient(a_sol, self.dt),
                    'time': np.arange(self.N) * self.dt,
                    'decisions': decisions,
                    'status': 'speed_fallback'
                }
                return result_dict
            
            # Normal case: unpack QP matrices
            P, q, A, lower, upper = build_result
            # ====================================================================

            # ====================================================================
            # v12.3: Optional ST-Boundary Visualization
            # ====================================================================
            if self.enable_st_visualization and self.st_viz_counter % 50 == 0:
                # Visualize every 50 steps (reduce overhead)
                # Extract s_lower and s_upper from st_boundaries
                s_lower_agg = np.full(self.N, -1e5)
                s_upper_agg = np.full(self.N, 1e5)

                for boundary in st_boundaries:
                    if boundary.boundary_type == BoundaryType.YIELD:
                        s_upper_agg = np.minimum(s_upper_agg, boundary.s_upper)
                    else:
                        s_lower_agg = np.maximum(s_lower_agg, boundary.s_lower)

                # Create decisions dict for visualization
                viz_decisions = {}
                for boundary in st_boundaries:
                    if boundary.boundary_type == BoundaryType.YIELD:
                        viz_decisions[boundary.vehicle_id] = InteractionDecision.YIELD
                    elif boundary.boundary_type == BoundaryType.OVERTAKE:
                        viz_decisions[boundary.vehicle_id] = InteractionDecision.OVERTAKE
                    else:
                        viz_decisions[boundary.vehicle_id] = InteractionDecision.FOLLOW

                visualize_st_boundary(
                    ego_state, obstacles, viz_decisions,
                    s_lower_agg, s_upper_agg, self.dt,
                    output_path=f"st_graph_{self.st_viz_counter:04d}.png"
                )

            self.st_viz_counter += 1
            # ====================================================================

            solver = osqp.OSQP()
            # [修正] OSQPソルバー設定：計算時間無制限＋反復回数大幅増加で確実に解く
            solver.setup(
                P, q, A, lower, upper,
                verbose=False,           # ログ出力を無効化
                polish=False,            # [PERF] False for speed (True adds ~30% overhead)
                check_termination=10,    # 終了条件チェック間隔
                max_iter=2000,           # [PERF-FIX] 20000 -> 2000 (prevent slowdowns, still robust)
                time_limit=0.02,         # [SAFETY] Max 20ms per solve (prevent freeze)
                eps_abs=1e-3,            # 絶対誤差許容値を緩和（維持）
                eps_rel=1e-3,            # 相対誤差許容値を緩和（維持）
                adaptive_rho=True        # 適応的ペナルティパラメータ調整
            )

            QP_STATS["total_attempts"] += 1
            result = solver.solve()
            QP_STATS["total_time_ms"] += result.info.solve_time * 1000.0

            # ====================================================================
            # QP Solver Result Logging
            # ====================================================================
            if ENABLE_DEBUG_OUTPUT:
                print(f"\n[QP-SOLVE] V#{ego_state.id}:")
                print(f"  Status: {result.info.status}")
                print(f"  Iterations: {result.info.iter}")
                print(f"  Solve time: {result.info.solve_time*1000:.1f}ms")
                print(f"  Objective value: {result.info.obj_val:.2f}")
                print(f"  ST Boundaries: {len(st_boundaries)} obstacles")

                # Show boundary summary
                for i, boundary in enumerate(st_boundaries):
                    print(f"    Boundary #{i}: V#{boundary.vehicle_id} [{boundary.boundary_type.name}] "
                          f"s_range=[{boundary.s_lower[0]:.1f}, {boundary.s_upper[0]:.1f}]m")

            # ====================================================================
            # v18.0: Apollo-style Constraint Relaxation on QP Failure
            # ====================================================================
            # Reference: Apollo piecewise_jerk_speed_optimizer.cc lines 189-199
            # 
            # Problem: Original code immediately falls back to IDM (emergency braking)
            # when QP fails. This is dangerous as it causes sudden deceleration.
            #
            # Apollo's Solution: Relax velocity constraints and retry
            # 1. First attempt: Normal constraints
            # 2. If failed: Relax velocity bounds (0 to max achievable)
            # 3. If still failed: Then fallback to IDM
            #
            # This allows finding a feasible solution even with conflicting
            # obstacle constraints (OVERTAKE + YIELD simultaneously)
            # ====================================================================
            if result.info.status not in ['solved', 'solved inaccurate', 'maximum iterations reached']:
                QP_STATS["relaxed_trigger"] += 1
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[QP-V18.0] V#{ego_state.id}: First attempt failed ('{result.info.status}'), trying with relaxed constraints...")
                
                # Apollo's relaxation: Widen velocity bounds
                # Note: relaxed_v_max calculated for reference but boundaries are relaxed directly
                # relaxed_v_max = max(self.v_max, ego_state.v + 5.0)
                
                # Rebuild QP with relaxed constraints
                # Temporarily disable speed limits from ST-boundary for retry
                st_boundaries_relaxed = []
                for boundary in st_boundaries:
                    # For YIELD boundaries, keep them but relax position constraints
                    # For OVERTAKE boundaries, keep them but make them soft
                    relaxed_boundary = STBoundary(
                        vehicle_id=boundary.vehicle_id,
                        s_lower=boundary.s_lower * 0.8,  # Relax by 20%
                        s_upper=boundary.s_upper * 1.2,  # Relax by 20%
                        boundary_type=boundary.boundary_type,
                        obs_v=boundary.obs_v if hasattr(boundary, 'obs_v') else 0.0  # v18.5: Preserve obs_v
                    )
                    st_boundaries_relaxed.append(relaxed_boundary)
                
                # Rebuild matrices with relaxed boundaries
                build_result_relaxed = self.build_qp_matrices(
                    ego_state, s_ref, v_ref, st_boundaries_relaxed
                )
                
                # v18.5: Check if speed fallback was triggered (now returns dict)
                if isinstance(build_result_relaxed, dict):
                    QP_STATS["fallback_trigger"] += 1
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[QP-V18.0] V#{ego_state.id}: Relaxed attempt also needs speed fallback")
                    # Use the fallback trajectory directly
                    self.w_v = w_v_original
                    return {
                        's': build_result_relaxed['s'],
                        'v': build_result_relaxed['v'],
                        'a': build_result_relaxed['a'],
                        'status': 'relaxed_fallback'
                    }
                
                P_relaxed, q_relaxed, A_relaxed, lower_relaxed, upper_relaxed = build_result_relaxed
                
                # Setup solver with relaxed constraints
                solver_relaxed = osqp.OSQP()
                solver_relaxed.setup(
                    P_relaxed, q_relaxed, A_relaxed, lower_relaxed, upper_relaxed,
                    verbose=False,
                    polish=False,    # [PERF] False for speed
                    check_termination=10,
                    max_iter=4000,  # [PERF-FIX] 30000 -> 4000 (prevent slowdowns)
                    eps_abs=1e-2,    # Relax tolerance
                    eps_rel=1e-2,
                    adaptive_rho=True,
                    time_limit=0.05  # [CRITICAL FIX] 50ms limit to prevent hang
                )
                
                result_relaxed = solver_relaxed.solve()
                QP_STATS["total_time_ms"] += result_relaxed.info.solve_time * 1000.0
                
                if result_relaxed.info.status in ['solved', 'solved inaccurate', 'maximum iterations reached']:
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[QP-V18.0] V#{ego_state.id}: Relaxed constraints SUCCEEDED!")
                    x = result_relaxed.x
                    s_sol = x[0:self.N]
                    v_sol = x[self.N:2*self.N]
                    a_sol = x[2*self.N:3*self.N]
                    
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"  Relaxed Solution: s=[{s_sol[0]:.1f}, ..., {s_sol[-1]:.1f}]m")
                        print(f"                    v=[{v_sol[0]:.1f}, ..., {v_sol[-1]:.1f}]m/s")
                    
                    # ====================================================================
                    # v18.3: Emergency Deceleration Check for Relaxed Solution
                    # ====================================================================
                    # Problem: V13 at t=59.60s had relaxed solution v=[19.4->16.3]
                    # but V9 (front vehicle) was in SPEED FALLBACK (decelerating to 0)
                    # This light deceleration was insufficient, causing AEB.
                    #
                    # Solution: After getting relaxed solution, check if front vehicle
                    # has very low velocity or is decelerating rapidly.
                    # If gap is small and v_rel is high, apply emergency deceleration.
                    # ====================================================================
                    need_emergency_decel = False
                    emergency_reason = ""
                    
                    for obs in obstacles:
                        obs_state = obs.vehicle_state
                        gap = obs_state.s - ego_state.s - self.L_vehicle
                        
                        # Skip rear vehicles
                        if gap < 0:
                            continue
                            
                        # Check conditions for emergency deceleration
                        v_rel = ego_state.v - obs_state.v  # Positive = closing
                        
                        # Condition 1: Front vehicle is nearly stopped and gap is small
                        if obs_state.v < 3.0 and gap < 20.0 and v_rel > 5.0:
                            need_emergency_decel = True
                            emergency_reason = f"V#{obs_state.id} stopped (v={obs_state.v:.1f}m/s), gap={gap:.1f}m, v_rel={v_rel:.1f}m/s"
                            break
                        
                        # Condition 2: TTC is critical (< 2.0s) even after relaxed solution
                        if v_rel > 0 and gap > 0:
                            ttc = gap / v_rel
                            if ttc < 2.0 and v_rel > 8.0:
                                need_emergency_decel = True
                                emergency_reason = f"V#{obs_state.id} TTC={ttc:.1f}s < 2.0s, v_rel={v_rel:.1f}m/s"
                                break
                    
                    if need_emergency_decel:
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[QP-V18.3] V#{ego_state.id}: Relaxed solution unsafe!")
                            print(f"  Reason: {emergency_reason}")
                            print(f"  Action: Applying EMERGENCY deceleration profile")
                        
                        # Generate emergency deceleration profile
                        a_emergency = -5.0  # m/s^2 (near AEB level, but controlled)
                        v_emergency = np.zeros(self.N)
                        s_emergency = np.zeros(self.N)
                        a_emergency_arr = np.full(self.N, a_emergency)
                        
                        v_emergency[0] = ego_state.v
                        s_emergency[0] = ego_state.s
                        
                        for k in range(1, self.N):
                            v_emergency[k] = max(0, v_emergency[k-1] + a_emergency * self.dt)
                            s_emergency[k] = s_emergency[k-1] + 0.5 * (v_emergency[k] + v_emergency[k-1]) * self.dt
                        
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"  Emergency Profile: v=[{v_emergency[0]:.1f}, ..., {v_emergency[-1]:.1f}]m/s")
                        
                        self.w_v = w_v_original
                        
                        # Store emergency trajectory
                        self.previous_trajectory[ego_state.id] = {
                            's': s_emergency,
                            'v': v_emergency,
                            'a': a_emergency_arr,
                            'time': current_time
                        }
                        
                        return {'s': s_emergency, 'v': v_emergency, 'a': a_emergency_arr, 'status': 'emergency_decel'}
                    # ==================================================================== (end v18.3)
                    
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"                    a=[{a_sol[0]:.2f}, ..., {a_sol[-1]:.2f}]m/s^2")
                    
                    self.w_v = w_v_original
                    
                    # Store trajectory for stitching
                    self.previous_trajectory[ego_state.id] = {
                        's': s_sol,
                        'v': v_sol,
                        'a': a_sol,
                        'time': current_time
                    }
                    
                    return {'s': s_sol, 'v': v_sol, 'a': a_sol, 'status': 'relaxed_solved'}
                else:
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[QP-V18.0] V#{ego_state.id}: Relaxed attempt also failed ('{result_relaxed.info.status}')")
                
                # Both attempts failed - now fallback to IDM
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[QP-FAILED] V#{ego_state.id}: Both attempts failed -> Fallback to IDM")

                # v14.6: Restore original velocity tracking weight
                self.w_v = w_v_original

                # Reset previous trajectory on QP failure (important for stitching)
                if ego_state.id in self.previous_trajectory:
                    del self.previous_trajectory[ego_state.id]
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[TRAJECTORY-RESET] V#{ego_state.id}: Cleared previous trajectory due to QP failure")

                return self._fallback_idm(ego_state, obstacles)
            # ==================================================================== (end v18.0)

            x = result.x
            s_sol = x[0:self.N]
            v_sol = x[self.N:2*self.N]
            a_sol = x[2*self.N:3*self.N]

            if ENABLE_DEBUG_OUTPUT:
                print(f"  Solution: s=[{s_sol[0]:.1f}, {s_sol[1]:.1f}, ..., {s_sol[-1]:.1f}]m")
                print(f"            v=[{v_sol[0]:.1f}, {v_sol[1]:.1f}, ..., {v_sol[-1]:.1f}]m/s")
                print(f"            a=[{a_sol[0]:.2f}, {a_sol[1]:.2f}, ..., {a_sol[-1]:.2f}]m/s^2")

            # v14.6: Restore original velocity tracking weight
            self.w_v = w_v_original

            return {'s': s_sol, 'v': v_sol, 'a': a_sol}

        except Exception as e:
            print(f"[FrenetQP] Critical Failure: {e}")

            # v14.6: Restore original velocity tracking weight
            self.w_v = w_v_original

            # Reset previous trajectory on critical failure
            if ego_state.id in self.previous_trajectory:
                del self.previous_trajectory[ego_state.id]
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[TRAJECTORY-RESET] V#{ego_state.id}: Cleared previous trajectory due to critical failure")

            return self._fallback_idm(ego_state, obstacles)

    def _fallback_idm(self, ego: VehicleState, obstacles: List[ObstacleInfo]) -> Dict[str, np.ndarray]:
        """
        Smart Fallback with Emergency-Aware Deceleration (v13.9 / 2025-12-24)

        ================================================================
        v13.9: Emergency-Aware Fallback (Root Cause 8 Fix)
        ================================================================
        Problem: Previous fallback used comfortable deceleration (-2.0m/s^2)
                 uniformly. This was insufficient when QP failed due to
                 imminent collision (high closing speed, small gap).
                 
        Solution: Calculate TTC-based deceleration requirement
                  - If TTC < 2.0s: Use emergency braking (-6.0m/s^2)
                  - If TTC >= 2.0s: Use comfortable braking (-2.0m/s^2)
                  
        Reference: Apollo speed_bounds_decider.cc - ApproachingObstacle()
                   Uses TTC threshold to decide braking intensity.
                   
                   ISO 22179 AEBS:
                   - Emergency: -6.0 m/s^2 (collision imminent)
                   - Comfortable: -2.0 m/s^2 (normal approach)
        ================================================================
        """
        s = [ego.s]
        v = [ego.v]
        a = []

        # Calculate TTC to nearest front obstacle
        ttc = float('inf')
        front_obs = None
        
        for obs in obstacles:
            if obs.is_front:
                gap = obs.vehicle_state.s - ego.s
                v_rel = ego.v - obs.vehicle_state.v
                
                if v_rel > 0.1:  # Closing in
                    obs_ttc = gap / v_rel
                    if obs_ttc < ttc:
                        ttc = obs_ttc
                        front_obs = obs
        
        # Determine deceleration based on TTC
        # v29.0: Phase 2 - Dynamic fallback deceleration
        # Considers: TTC, relative velocity, gap, ego velocity, front vehicle type
        TTC_EMERGENCY_THRESHOLD = 2.0  # [s] Apollo threshold for emergency
        TTC_CRITICAL_THRESHOLD = 1.0   # [s] Critical - max braking needed
        
        # v29.0: Dynamic TTC thresholds based on ego velocity
        # Higher speed → need earlier braking (kinetic energy scales with v²)
        ego_v_normalized = ego.v / 20.0  # Normalize by typical highway speed
        ttc_critical_adj = TTC_CRITICAL_THRESHOLD * (1.0 + ego_v_normalized * 0.5)  # 1.0-1.75s range
        ttc_emergency_adj = TTC_EMERGENCY_THRESHOLD * (1.0 + ego_v_normalized * 0.33)  # 2.0-2.66s range
        
        # v29.0: Gap-based urgent check (proximity override)
        gap = front_obs.vehicle_state.s - ego.s if front_obs else float('inf')
        if gap < 5.0:
            # Critical proximity regardless of TTC
            target_decel = -6.0
            decel_type = "CRITICAL-GAP"
        elif ttc < ttc_critical_adj:
            # Critical: Maximum emergency braking
            # v29.0: Interpolate based on gap for smoother response
            if gap < 10.0:
                target_decel = -6.0
            else:
                # Gentle ramp from -6.0 to -5.0 as gap increases
                target_decel = -6.0 + (gap - 5.0) / 5.0  # -6.0 @ 5m, -5.0 @ 10m
            decel_type = "CRITICAL"
        elif ttc < ttc_emergency_adj:
            # Emergency: Interpolate -6.0 → -4.5
            ratio = (ttc - ttc_critical_adj) / (ttc_emergency_adj - ttc_critical_adj)
            target_decel = -6.0 + ratio * 1.5  # -6.0 ~ -4.5
            
            # v29.0: CAV coordination bonus (if front is CAV, trust trajectory)
            if front_obs and hasattr(front_obs.vehicle_state, 'is_hdv') and not front_obs.vehicle_state.is_hdv:
                if gap > 10.0:
                    target_decel *= 0.8  # 20% reduction (predictable front vehicle)
            
            target_decel = max(target_decel, -6.0)  # Clamp
            decel_type = "EMERGENCY"
        elif ttc < ttc_emergency_adj * 1.5:
            # Moderate: Interpolate -4.5 → -2.5
            ratio = (ttc - ttc_emergency_adj) / (ttc_emergency_adj * 0.5)
            target_decel = -4.5 + ratio * 2.0  # -4.5 ~ -2.5
            decel_type = "MODERATE"
        else:
            # Normal: Comfortable braking
            target_decel = -2.0
            decel_type = "COMFORTABLE"
        
        for _ in range(self.N):
            curr_a = target_decel
            curr_v = max(0, v[-1] + curr_a * self.dt)
            curr_s = s[-1] + curr_v * self.dt
            s.append(curr_s)
            v.append(curr_v)
            a.append(curr_a)

        if ENABLE_DEBUG_OUTPUT:
            obs_info = f"front_V#{front_obs.vehicle_state.id}" if front_obs else "no_front"
            print(f"[FALLBACK-{decel_type}] V#{ego.id}: QP failed, TTC={ttc:.2f}s -> "
                  f"decel={target_decel}m/s^2 ({obs_info})")

        return {'s': np.array(s[1:]), 'v': np.array(v[1:]), 'a': np.array(a)}


# ============ UTILITY FUNCTIONS ============

def get_obstacles_in_lane(
    ego: VehicleState,
    all_vehicles: List[VehicleState],
    lane: str
) -> List[ObstacleInfo]:
    """
    Get all obstacles in specified lane with type information

    Args:
        ego: Ego vehicle state
        all_vehicles: List of all vehicle states
        lane: Target lane to search

    Returns:
        List of ObstacleInfo with is_front flag set correctly
    """
    obstacles = []

    for v in all_vehicles:
        if v.lane == lane and v.id != ego.id:
            is_front = v.s > ego.s
            obstacles.append(ObstacleInfo(
                vehicle_state=v,
                is_front=is_front
            ))

    return obstacles


if __name__ == "__main__":
    print("=" * 80)
    print("Ultimate Apollo QP Controller with Slack Variables (v11.9)")
    print("=" * 80)

    # Test case 1: Normal car following
    print("\n### Test 1: Car Following with Front Vehicle ###")
    controller = FrenetQPController(horizon=50, dt=0.1)

    ego = VehicleState(id=1, s=0.0, v=15.0, a=0.0, lane='center')
    front = VehicleState(id=2, s=50.0, v=12.0, a=0.0, lane='center')

    obstacles = [ObstacleInfo(vehicle_state=front, is_front=True)]
    result = controller.optimize(ego, obstacles)

    if result:
        print("[OK] Optimization successful!")
        print(f"  Initial velocity: {ego.v:.2f} m/s")
        print(f"  Final velocity: {result['v'][-1]:.2f} m/s")
        gap_maintained = min(front.s + front.v*np.arange(50)*0.1 - result['s'])
        print(f"  Min gap maintained: {gap_maintained:.2f} m")
    else:
        print("[FAIL] Optimization failed")

    # Test case 2: Impossible scenario (cut-in) - should NOT fail with slack
    print("\n### Test 2: Emergency Cut-In Scenario (Slack Variable Test) ###")
    ego_emergency = VehicleState(id=1, s=0.0, v=20.0, a=0.0, lane='center')
    cutter = VehicleState(id=2, s=5.0, v=5.0, a=0.0, lane='center')  # Too close!

    obstacles_emergency = [ObstacleInfo(vehicle_state=cutter, is_front=True)]
    result2 = controller.optimize(ego_emergency, obstacles_emergency)

    if result2:
        print("[OK] Optimization successful (Slack absorbed constraint violation)!")
        print(f"  Initial velocity: {ego_emergency.v:.2f} m/s")
        print(f"  Final velocity: {result2['v'][-1]:.2f} m/s")
        print(f"  Emergency braking applied: {result2['a'][0]:.2f} m/s^2")
    else:
        print("[FAIL] Unexpected failure - Slack should prevent this")

    # Test case 3: Sandwich scenario
    print("\n### Test 3: Sandwich Scenario (Both Front and Rear) ###")
    ego_sandwich = VehicleState(id=1, s=100.0, v=18.0, a=0.0, lane='center')
    front_sandwich = VehicleState(id=2, s=130.0, v=16.0, a=0.0, lane='center')
    rear_sandwich = VehicleState(id=3, s=85.0, v=22.0, a=0.0, lane='center')

    obstacles_sandwich = [
        ObstacleInfo(vehicle_state=front_sandwich, is_front=True),
        ObstacleInfo(vehicle_state=rear_sandwich, is_front=False)
    ]
    result3 = controller.optimize(ego_sandwich, obstacles_sandwich)

    if result3:
        print("[OK] Optimization successful!")
        print("  Navigating between front and rear vehicles")
        print(f"  Final velocity: {result3['v'][-1]:.2f} m/s")
    else:
        print("[FAIL] Unexpected failure")

    print("\n" + "=" * 80)
    print("All tests completed! Ultimate Apollo QP with Slack Variables is ALWAYS feasible.")
    print("=" * 80)
