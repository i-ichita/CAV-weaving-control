# -*- coding: utf-8 -*-
"""
weaving_v11/controllers.py (ver.13.1 / 2025-12-24)

統合ゾーンコントローラ - 緊急度ベースローリングホライズン制御の実装
================================================================================

参照アーキテクチャ:
- Baidu Apollo: modules/planning/planning_component.cc (フレームベースプランニング)
- Apollo PathDecider: Frenet座標ベース障害物検出
- Apollo SpeedBoundsDecider: 複数レーン障害物処理
- Apollo DecisionMaker: 安全な車線変更のためのギャップアクセプタンス
- Autoware: motion_velocity_optimizer (プランナー/コントローラ分離)

設計哲学:
- Level 1 (戦略層): UrgencyPlannerが0.5秒ごとに緊急度スコアを計算
- Level 2 (戦術層): FrenetQPが0.1秒ごとに衝突回避軌道を解く (10Hz、ホライズン≈8s/80ステップ)
- Gurobi MIQPなし: ApolloスタイルQPのみ (OSQPソルバー)

主な革新 (v11.9 - Apollo完全アーキテクチャ):
1. **10Hz制御ループ (Apollo標準)**
   - 制御間隔: 1.0s -> 0.1s (10倍高速応答)
   - リアルタイム衝突回避を実現
2. **強化緊急ブレーキ (UN R157)**
   - 制動能力: -3.0 -> -6.0 m/s²
   - RSS閾値: 100% (v11.8の90%より厳格)
3. **Frenet座標ベース障害物検出 (Apollo PathDecider)**
   - レーン文字列一致ではなく物理的横位置(d)を使用
   - レーン遷移、割り込み、エッジケースに対して頑健
4. **CIPV選択 (最近接車両)**
   - 直接の前方/後方車両のみを考慮 (Apollo標準)
   - 誤検知と計算負荷を削減
5. **RSS安全保証 (ISO 21934 / Apollo Safety Manager)**
   - QPのハード安全制約 (前方制限の緩和なし)
   - RSSベース車線変更ゲート (危険な割り込みを防止)
   - AEBラッチ機構 (RSS×2のヒステリシス)
6. **スロットベースプランニング (Apollo Gap Filling戦略)**
   - LC中にターゲットレーンのギャップを識別
   - スムーズな合流のために速度を調整 (人間らしい挙動)
   - 3ケース: 挙み撃ち、先行車のみ、後続車のみ
7. **不確実性考慮段階的バッファ**
   - 静的 + 動的 + 時間依存の不確実性拡張
   - ホライズン上の予測劣化を考慮
8. **シャドウモード + インテリジェント速度調和**
   - LC中のデュアルレーン監視
   - 攻撃的な挙動を防ぐために前方車速度に追従
9. **後方脅威に対する譲歩ロジック**
   - TTCベースの速度低減 (高速車両接近時)
10. **物理的オーバーラップガード + 予測TTC事前チェック**

バージョン: 11.9 Final (Apollo RSS + スロットベースプランニング)
日付: 2025-12-18
作者: Apollo EM Planner + PathDecider + SpeedBoundsDecider + DecisionMaker に基づく
================================================================================
"""

from typing import List, Dict, TYPE_CHECKING
import numpy as np
import sys

if TYPE_CHECKING:
    from .simulator import IntegratedZoneSimulator

from .vehicle import Vehicle, SafetyAnalyzer
from .parameters import IntegratedZoneParameters
from .mpc_controller import (
    UrgencyPlanner,
    compute_lane_density
)
from .frenet_qp_apollo import FrenetQPController, VehicleState, ObstacleInfo

# v16.1: Import centralized safety module
from .safety import (
    SAFETY,
    compute_ttc,
    compute_required_decel,
    compute_rss_safe_distance,
    compute_rss_safe_distance_cav,
    compute_gap_bumper_to_bumper,
    is_follow_too_close,
    evaluate_aeb,
    has_lateral_overlap,
    format_aeb_log,
)
from .coordinate_transform import LANE_OFFSETS, LANE_WIDTH

# v26.0: Unified Apollo Safety Manager
# v27.0: Added SafetyState and SafetyEvaluation for graduated response
from .apollo_safety import (
    ApolloSafetyManager, 
    get_safety_manager,
    SafetyState,
    SafetyEvaluation,
)

# Debug output control (compatible with existing codebase)
ENABLE_DEBUG_OUTPUT = True  # v28.0: Enabled for V2V debugging and analysis


# ============================================================================
# v12.0: Wide-Area Cooperative Control Utilities
# ============================================================================
def get_adjacent_lanes(lane: str) -> List[str]:
    """
    Get adjacent lanes for wide-area scanning.

    v12.0: Used for detecting merge conflicts from "neighboring neighbor" lanes
    - Example: If target_lane="lcenter", also scan "left" and "rcenter"
    - Prevents collision when multiple vehicles from different lanes target same slot

    Args:
        lane: Lane name

    Returns:
        List of adjacent lane names
    """
    adjacent_map = {
        'left': ['lcenter'],
        'lcenter': ['left', 'rcenter'],
        'rcenter': ['lcenter', 'right'],
        'right': ['rcenter']
    }
    return adjacent_map.get(lane, [])
# ============================================================================


# ============================================================================
# Backwards Compatibility: Dummy Classes
# ============================================================================
class PreparationZoneController:
    """Dummy class for backwards compatibility"""
    def __init__(self, parent):
        self.parent = parent


class WeavingZoneController:
    """Dummy class for backwards compatibility"""
    def __init__(self, parent):
        self.parent = parent
# ============================================================================


class IntegratedZoneController:
    """
    Unified controller managing both Level 1 (Strategic) and Level 2 (Tactical)

    Reference:
    - Apollo: PlanningComponent -> Frame -> Planner -> Task
    - Apollo SpeedBoundsDecider: Multi-lane obstacle integration
    - Apollo DecisionMaker: Gap Acceptance for lane change safety
    - Autoware: BehaviorPlanner -> MotionPlanner -> Controller

    Architecture:
    - Level 1 (Strategic, 0.5s cycle): Urgency-based spatial distribution
    - Level 2 (Tactical, 0.1s cycle): Frenet QP collision-free trajectory

    Key Design Decisions:
    1. No MIQP lane change scheduling (too slow, not real-time capable)
    2. Urgency-based relaxation (dynamically adjust safety margins)
    3. Multi-lane obstacle detection (current + target lanes during LC)
    4. YIELD (front) + OVERTAKE (rear) ST-Boundaries
    5. Gap Acceptance: Apollo-style safe lane change triggering
    6. Always have a fallback (IDM when QP fails)
    """

    def __init__(self, params: IntegratedZoneParameters, simulator: 'IntegratedZoneSimulator'):
        """
        Initialize integrated controller

        Args:
            params: System parameters
            simulator: Reference to simulator (for accessing vehicle lists)
        """
        self.params = params
        self.simulator = simulator

        # Level 1: Strategic Urgency Planner
        # Recommended: gamma=3.0 for natural spatial distribution
        self.urgency_planner = UrgencyPlanner(
            gamma=getattr(params, 'urgency_gamma', 3.0),
            alpha=getattr(params, 'urgency_alpha', 0.2),
            replan_interval=getattr(params, 'replan_interval', 0.5),
            urgency_min=0.0,
            urgency_max=1.0
        )

        # Level 2: Tactical Frenet QP Controller
        # --- modified (v15.0 / 2025-12-26): Extended Horizon Apollo Smart Control ---
        # Previous: horizon=30 (3s @ 0.1s) - Short-sighted reactive control
        # New: horizon=80 (8s @ 0.1s) - Long-term predictive control (Apollo standard)
        # Benefits:
        # - Predictive vs reactive: Plan smooth trajectories 8 seconds ahead
        # - Reduced emergency braking: More time to decelerate comfortably
        # - Traffic flow optimization: Consider distant vehicles
        # - Jerk minimization: Smoother acceleration/deceleration profiles
        self.frenet_controller = FrenetQPController(
            horizon=getattr(params, 'horizon', 80),  # 8.0s prediction horizon
            dt=params.dt_control  # Use control timestep (0.1s) not physics (0.01s)
        )

        # Safety analyzer for Gap Acceptance
        self.safety_analyzer = SafetyAnalyzer(
            collision_threshold=0.1,
            near_miss_threshold=2.0,
            critical_ttc_threshold=1.5
        )

        # Gap Acceptance parameters (Apollo DecisionMaker)
        # v18.27: CAV Cooperative Gap Opening with RSS Dynamic Threshold
        # - 固定閾値から動的RSS閾値に変更
        # - 後方CAVへの協調減速要求でギャップを作る
        # - CAV同士なら反応時間短縮 (0.5s vs 1.5s)
        self.gap_acceptance_params = {
            'min_front_gap': 10.0,  # Base front gap (RSS adjusts dynamically)
            'min_rear_gap': 12.0,   # v18.27: 8->12m base (RSS adjusts)
            'min_front_ttc': 2.0,   # CAV response time
            'min_rear_ttc': 3.0,    # v18.27: 2.5->3.0s (safety margin)
            'cav_reaction_time': 0.5,   # CAV-CAV reaction time [s]
            'hdv_reaction_time': 1.5,   # CAV-HDV reaction time [s]
        }

        # ====================================================================
        # v12.3 Modified: Trajectory Stitching Support
        # ====================================================================
        # Store previous trajectory for each vehicle to enable:
        # 1. Smooth acceleration continuation (no jerk spikes)
        # 2. Consistent planning across control cycles
        # 3. Apollo-style trajectory_stitcher behavior
        # Key: Retrieve previous trajectory and pass to optimize(previous_trajectory=...)
        # ====================================================================
        self.prev_traj: Dict[int, Dict[str, np.ndarray]] = {}  # vehicle_id -> trajectory dict
        # ====================================================================

        # Backwards compatibility: create dummy sub-controllers
        # (simulator may expect these attributes)
        self.prep_controller = PreparationZoneController(self)
        self.weave_controller = WeavingZoneController(self)

        # Statistics
        self.level1_update_count = 0
        self.level2_update_count = 0
        self.qp_success_count = 0
        self.qp_failure_count = 0
        self.fallback_idm_count = 0
        self.gap_acceptance_reject_count = 0

        if ENABLE_DEBUG_OUTPUT:
            print("[IntegratedZoneController] Initialized (v15.0 - Apollo Intelligent Control)")
            print(f"  Level 1: UrgencyPlanner (gamma={self.urgency_planner.gamma}, "
                  f"replan={self.urgency_planner.replan_interval}s)")
            print(f"  Level 2: FrenetQP Extended Horizon (horizon={self.frenet_controller.N} steps = "
                  f"{self.frenet_controller.horizon_time:.1f}s prediction)")
            print("  Control Frequency: 10Hz (0.1s interval)")
            print("  Adaptive Braking: -2.5 m/s^2 (comfort) to -6.0 m/s^2 (emergency)")  # v28.0
            print("  Intelligent Features:")
            print("    - Extended prediction horizon: 8.0s (vs 3.0s previous)")
            print("    - Adaptive safety margins: Traffic density + TTC aware")
            print("    - Smooth deceleration: Jerk-minimized profiles")
            print("    - Long-range planning: Considers distant vehicles (200m)")
            print("  Obstacle Detection: Frenet d-coordinate with CIPV selection")
            print(f"  Gap Acceptance: min_front_gap={self.gap_acceptance_params['min_front_gap']}m, "
                  f"min_rear_gap={self.gap_acceptance_params['min_rear_gap']}m")
            print("  Trajectory Stitching: ENABLED (Apollo N-1 continuity)")

    def control_update(self, t: float, vehicles: List[Vehicle]):
        """
        Main control loop called by simulator

        Reference:
        - Apollo: PlanningComponent::Proc() calls Frame::Plan()
        - Autoware: MotionVelocityOptimizer::onTimer()

        Workflow:
        1. Level 1 Update (if replan_interval elapsed): Update urgency for all vehicles
        2. Level 2 Execute (always): Solve QP for each vehicle, apply acceleration/steering

        Args:
            t: Current simulation time [s]
            vehicles: List of vehicles to control
        """
        if not vehicles:
            return

        # Update safety analyzer time
        self.safety_analyzer.set_current_time(t)

        # Step 1: Strategic Planning (Rolling Horizon)
        if self.urgency_planner.should_replan(t):
            self._update_level1_strategy(t, vehicles)
            self.level1_update_count += 1

        # Step 2: Tactical Control (Every step)
        self._execute_level2_control(t, vehicles)
        self.level2_update_count += 1

    def _check_gap_acceptance(self, vehicle: Vehicle, target_lane: str, all_vehicles: List[Vehicle]) -> bool:
        """
        Check if lane change is safe using Gap Acceptance with Predictive Check

        Reference:
        - Apollo DecisionMaker: modules/planning/tasks/deciders/path_lane_borrow_decider/
        - NGSIM lane change safety criteria
        --- modified (Final Fix v11.9_Ultimate / 2025-12-18): Predictive Gap Check ---
        - Added future position prediction based on LC duration
        - Prevents accepting gaps that will close during lane change execution
        
        --- modified (v18.29 / 2026-01-01): Urgency-based Gap Relaxation ---
        - High urgency vehicles get relaxed gap thresholds
        - Enables LC success rate improvement while maintaining safety
        - gap_factor = 1.0 - urgency_gap_relax_coeff * urgency
        - Example: U=0.8, coeff=0.3 → thresholds reduced by 24%

        Args:
            vehicle: Vehicle requesting lane change
            target_lane: Target lane to change to
            all_vehicles: All vehicles in simulation

        Returns:
            True if gap is acceptable (safe to change lanes), False otherwise
        """
        # v18.29: Urgency-based gap threshold relaxation
        urgency = getattr(vehicle, 'urgency', 0.0)
        gap_relax_coeff = getattr(self.params, 'urgency_gap_relax_coeff', 0.0)
        gap_factor = 1.0 - gap_relax_coeff * urgency  # High urgency → lower factor → relaxed thresholds
        gap_factor = max(0.7, gap_factor)  # Never relax below 70% of base threshold
        
        # Find vehicles in target lane
        target_lane_vehicles = [v for v in all_vehicles if v.lane == target_lane and not v.exited]

        if not target_lane_vehicles:
            # Empty lane: always safe
            return True

        # Find front and rear vehicles in target lane
        front_vehicle = None
        rear_vehicle = None

        for v in target_lane_vehicles:
            if v.x > vehicle.x:
                if front_vehicle is None or v.x < front_vehicle.x:
                    front_vehicle = v
            elif v.x < vehicle.x:
                if rear_vehicle is None or v.x > rear_vehicle.x:
                    rear_vehicle = v

        # === Apollo Predictive Gap Check ===
        # LC takes time - check if gap will still be safe at completion
        LC_DURATION = getattr(self.params, 'lc_duration', 3.0)  # Use configured LC duration

        # v18.29: Apply urgency-based relaxation to gap thresholds
        min_front_gap = self.gap_acceptance_params['min_front_gap'] * gap_factor
        min_front_ttc = self.gap_acceptance_params['min_front_ttc'] * gap_factor

        # Check front gap
        if front_vehicle is not None:
            front_gap = front_vehicle.x - vehicle.x - self.params.L_vehicle
            if front_gap < min_front_gap:
                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_gap_debug', False):
                    print(f"[Gap Acceptance] V{vehicle.id} REJECT: front_gap={front_gap:.1f}m < "
                          f"{min_front_gap:.1f}m (U={urgency:.2f}, factor={gap_factor:.2f})")
                return False

            # Check front TTC
            front_ttc = SafetyAnalyzer.compute_ttc(vehicle.x, front_vehicle.x, vehicle.v, front_vehicle.v, self.params.L_vehicle)
            if front_ttc is not None and front_ttc < min_front_ttc:
                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_gap_debug', False):
                    print(f"[Gap Acceptance] V{vehicle.id} REJECT: front_ttc={front_ttc:.2f}s < "
                          f"{min_front_ttc:.2f}s (U={urgency:.2f})")
                return False

        # Check rear gap (CRITICAL: rear vehicle must not be forced to brake hard)
        if rear_vehicle is not None:
            rear_gap = vehicle.x - rear_vehicle.x - self.params.L_vehicle
            rel_v = rear_vehicle.v - vehicle.v  # Positive = closing
            is_rear_cav = not getattr(rear_vehicle, 'is_hdv', False)

            # === v18.27: RSS Dynamic Threshold (Apollo-style) ===
            # RSS safe distance = v_rel * t_reaction + v_rel^2 / (2 * a_brake)
            # CAV-CAV: t_reaction=0.5s, a_brake=4.0m/s²
            # CAV-HDV: t_reaction=1.5s, a_brake=3.0m/s²
            if is_rear_cav:
                t_react = self.gap_acceptance_params['cav_reaction_time']
                a_brake = 4.0  # CAV can brake harder
            else:
                t_react = self.gap_acceptance_params['hdv_reaction_time']
                a_brake = 3.0  # HDV more conservative

            if rel_v > 0:
                # Closing: RSS-based dynamic threshold
                rss_gap = rel_v * t_react + (rel_v ** 2) / (2 * a_brake) + 5.0
                min_rear_gap = max(self.gap_acceptance_params['min_rear_gap'], rss_gap)
            else:
                # Opening or stable: can use smaller gap for CAV
                min_rear_gap = self.gap_acceptance_params['min_rear_gap'] - (3.0 if is_rear_cav else 0.0)
                min_rear_gap = max(8.0, min_rear_gap)  # Never below 8m

            # === v18.27: CAV Cooperative Gap Opening ===
            # If gap is tight but rear is CAV, request cooperation
            if is_rear_cav and rear_gap < min_rear_gap and rear_gap >= 8.0:
                # Calculate required deceleration for rear CAV
                gap_deficit = min_rear_gap - rear_gap
                required_decel = min(2.0, gap_deficit / LC_DURATION)  # Cap at 2.0m/s²
                
                # Send cooperative deceleration request
                rear_vehicle.cooperative_decel_request = required_decel
                
                # Predict gap with cooperation
                # rear slows down by required_decel, gap opens
                predicted_gap_with_coop = rear_gap + 0.5 * required_decel * (LC_DURATION ** 2)
                
                if predicted_gap_with_coop >= min_rear_gap * 0.9:  # 90% threshold OK
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[Gap Acceptance] V{vehicle.id} ACCEPT with COOPERATION: "
                              f"rear_gap={rear_gap:.1f}m, predicted={predicted_gap_with_coop:.1f}m, "
                              f"rear V{rear_vehicle.id} decel_req={required_decel:.1f}m/s²")
                    return True

            # --- Predictive Check: Will gap still be safe after LC duration? ---
            if rel_v > 0:
                predicted_gap = rear_gap - (rel_v * LC_DURATION)
                if predicted_gap < min_rear_gap * 0.8:  # 80% threshold for rejection
                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_gap_debug', False):
                        print(f"[Gap Acceptance] V{vehicle.id} REJECT: predicted_gap={predicted_gap:.1f}m < "
                              f"{min_rear_gap * 0.8:.1f}m (current={rear_gap:.1f}m, closing={rel_v:.1f}m/s)")
                    return False

            # Current gap check with dynamic threshold
            if rear_gap < min_rear_gap:
                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_gap_debug', False):
                    print(f"[Gap Acceptance] V{vehicle.id} REJECT: rear_gap={rear_gap:.1f}m < "
                          f"dynamic_min={min_rear_gap:.1f}m (RSS-based)")
                return False

            # Check rear TTC (rear vehicle approaching ego)
            rear_ttc = SafetyAnalyzer.compute_ttc(rear_vehicle.x, vehicle.x, rear_vehicle.v, vehicle.v, self.params.L_vehicle)
            if rear_ttc is not None and rear_ttc < self.gap_acceptance_params['min_rear_ttc']:
                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_gap_debug', False):
                    print(f"[Gap Acceptance] V{vehicle.id} REJECT: rear_ttc={rear_ttc:.2f}s < "
                          f"{self.gap_acceptance_params['min_rear_ttc']}s")
                return False

        # All checks passed
        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_gap_debug', False):
            print(f"[Gap Acceptance] V{vehicle.id} ACCEPT: safe to change to {target_lane}")
        return True

    def _update_level1_strategy(self, t: float, vehicles: List[Vehicle]):
        """
        Level 1: Strategic Urgency Update

        Reference:
        - Liao et al. (2022): Deep RL learns urgency implicitly
        - Khan et al. (2014): Front-loading causes +40% delay

        Implementation:
        1. Compute lane densities (traffic state)
        2. Compute urgency scores for all vehicles
        3. Attach urgency to vehicle.urgency attribute
        4. Urgency-based lane change decision making (probabilistic triggering)
        5. **Gap Acceptance: Safety check before scheduling lane change**

        Args:
            t: Current simulation time [s]
            vehicles: List of vehicles to update
        """
        if not vehicles:
            return

        # Compute lane densities
        lanes = ['left', 'lcenter', 'rcenter', 'right']
        rho_per_lane = {}
        lane_length = max(1.0, self.params.prep_zone_length + self.params.weave_zone_length)

        for lane in lanes:
            vehicles_in_lane = [v for v in self.simulator.vehicles
                               if v.lane == lane and not v.exited]
            rho_per_lane[lane] = compute_lane_density(
                [{'id': v.id, 'x': v.x} for v in vehicles_in_lane],
                lane_length
            )

        # Define weaving zone boundaries for urgency normalization
        s_entry = 0.0
        s_exit = max(1.0, self.params.total_length)

        # Prepare vehicle data for batch urgency computation
        veh_dicts = []
        for v in vehicles:
            target_lane = (v.target_lane or
                          v.target_lane_prep or
                          v.target_lane_weave or
                          v.lane)
            veh_dicts.append({
                'id': v.id,
                'x': v.x,
                'target_lane': target_lane
            })

        # Batch compute urgencies
        urgency_states = self.urgency_planner.compute_urgencies_batch(
            veh_dicts,
            s_entry,
            s_exit,
            rho_per_lane
        )

        # Attach urgency back to vehicles
        id_to_vehicle = {v.id: v for v in vehicles}
        for us in urgency_states:
            v = id_to_vehicle.get(us.vehicle_id)
            if v is not None:
                # Store urgency and target window in vehicle
                v.urgency = us.urgency
                v.urgency_state = us  # Store full state for analysis

                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_urgency_debug', False):
                    print(f"[Level1] t={t:.2f}s V{v.id} urgency={v.urgency:.3f} "
                          f"s={us.s_current:.1f}m lane={v.lane}->{us.target_lane}")

        # ========================================================================
        # Urgency-based Lane Change Decision Making with Gap Acceptance
        # ========================================================================
        # Reference:
        # - Probabilistic triggering: P_trigger = BaseProb + Coeff x Urgency
        # - Apollo DecisionMaker: Gap Acceptance before committing to lane change
        # - When Urgency=0.0 -> P_trigger=0% (no rush)
        # - When Urgency=1.0 -> P_trigger=100% (immediate execution)
        #
        # Implementation:
        # - BaseProb = 0.0, Coeff = 1.0 (direct urgency threshold)
        # - Stochastic check: if U > random(0,1), check gap acceptance
        # - If gap is safe, schedule lane change; otherwise, wait and keep urgency high
        # ========================================================================

        lc_decision_count = 0
        gap_reject_count = 0

        if ENABLE_DEBUG_OUTPUT:
            print(f"\n[Level1-Start] t={t:.2f}s: Checking {len(vehicles)} vehicles for LC scheduling")

        for v in vehicles:
            # --- modified (v19.4 / 2025-12-29): Allow LC in prep zone ---
            # --- v18.21: LCはx>=0のみ許可 ---
            # Spawn区間: [-400, 0] (warmup_length) - LCなし、走行安定化のみ
            # 準備LC区間: [0, 500] (prep_zone_length) - LC開始可能
            # 織り込み区間: [500, 1000] (weave_zone_length) - LC完了目標
            # 旧ロジック: x < -300m でスキップ（prep zone[-400,0]の一部でLCを許可）
            # 新ロジック: x < 0 でスキップ（spawn区間では一切LCしない）
            if v.x < 0.0:
                # Spawn区間[-400, 0]ではLCしない - 準備LC区間[0, 500]以降で開始
                if ENABLE_DEBUG_OUTPUT and getattr(v, 'needs_initial_lc', False):
                    print(f"[LC-Skip] V{v.id}: x={v.x:.1f}m < 0m (in spawn zone, LC starts at x>=0)")
                continue
            # ------------------------------------------------------------

            # ====================================================================
            # v12.6: Enhanced Deadlock Prevention with Yield Timeout
            # ====================================================================
            # Solution 3: Track yielding duration and abort if stuck too long
            # - Speed-based abort (v12.5): Low speed vs. high lane flow
            # - Time-based abort (v12.6): Yielding > 5s at low speed
            # Reference: Root Cause Analysis - Deadlock Logic
            # ====================================================================
            if getattr(v, 'lc_scheduled', False) and not getattr(v, 'lc_started', False):
                # LC scheduled but not yet started (preparing phase)

                # Initialize yield tracking timer
                if not hasattr(v, 'yield_start_time') or v.yield_start_time is None:
                    v.yield_start_time = t

                yield_duration = t - v.yield_start_time

                # Abort Condition 1: Time-based (v13.3 strengthened)
                # If yielding too long (>3s) AND slow (<5.5 m/s), give up immediately
                # Reduced from 5s/10m/s to break deadlocks faster
                if yield_duration > 3.0 and v.v < 5.5:
                    v.lc_scheduled = False
                    v.target_lane = None
                    v.urgency = 0.0
                    v.lc_intent = None
                    v.yield_start_time = None

                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[YIELD-TIMEOUT] V{v.id} aborted LC after {yield_duration:.1f}s "
                              f"of yielding at v={v.v:.1f}m/s. Traffic flow restored.")
                    continue

                # Abort Condition 2: Speed-based (v13.3 strengthened)
                # If crawling (<5.5 m/s) while trying to LC, abort immediately
                # This is the PRIMARY deadlock breaker
                if v.v < 5.5:
                    v.lc_scheduled = False
                    v.target_lane = None
                    v.urgency = 0.0
                    v.lc_intent = None
                    v.yield_start_time = None

                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[DEADLOCK-ABORT] V{v.id} crawling at {v.v:.1f}m/s during LC prep. "
                              f"Aborting LC to restore flow.")
                    continue
            else:
                # LC not scheduled or already started - clear timer
                if hasattr(v, 'yield_start_time') and v.yield_start_time is not None:
                    v.yield_start_time = None
            # ====================================================================

            # Condition 1: Not already completed
            if getattr(v, 'lc_completed', False):
                continue

            # Condition 2: Not already scheduled
            if getattr(v, 'lc_scheduled', False):
                continue

            # Condition 3: Needs lane change
            if not getattr(v, 'needs_initial_lc', False):
                if ENABLE_DEBUG_OUTPUT and v.x > -300.0:  # v18.20: Extended logging range
                    print(f"[LC-Skip-NoNeed] V{v.id}: x={v.x:.1f}m, needs_initial_lc=False (no LC needed)")
                continue

            if ENABLE_DEBUG_OUTPUT and v.x > -300.0:
                print(f"[LC-Candidate] V{v.id}: x={v.x:.1f}m, needs_initial_lc=True, checking target_lane...")

            # Condition 4: Target lane must be set
            target_lane = (v.target_lane or
                          v.target_lane_prep or
                          v.target_lane_weave)
            if not target_lane or target_lane == v.lane:
                continue

            # ====================================================================
            # v11.12/v18.29: Position-Based LC Probability with Spatial Distribution
            # ====================================================================
            # P(LC|x) = sigmoid(β₀ + β₁·x_norm)
            # 
            # Design Intent: LC position distribution via stochastic triggering
            # - x_norm=0 (entry): Low probability → few early LCs
            # - x_norm=1 (exit): High probability → ensure completion
            # - Natural spatial distribution emerges from cumulative 0.5s checks
            # 
            # Gap Acceptance緩和 (v18.29): Urgency高→閾値緩和でLC成功率向上
            # Reference: Continuous-time stochastic decision process
            # ====================================================================

            # Normalized position (0.0 at entrance, 1.0 at exit)
            x_norm = np.clip(v.x / self.params.total_length, 0.0, 1.0)

            # Logistic function: logit = β₀ + β₁·x_norm
            logit = self.params.lc_beta_0 + self.params.lc_beta_1 * x_norm

            # Optional exponential urgency term near exit
            if self.params.lc_beta_2 > 0:
                exp_term = np.exp(self.params.lc_beta_k * (x_norm - 1.0))
                logit += self.params.lc_beta_2 * exp_term

            # Sigmoid activation: P ∈ [0, 1]
            trigger_prob = 1.0 / (1.0 + np.exp(-logit))

            # v30.0: Removed hard-threshold emergency override
            # Instead, urgency smoothly increases near exit via UrgencyPlanner.emergency_distance
            # Benefit: No discontinuous probability jump, smoother LC distribution
            # This is now handled in compute_urgency() with quadratic boost

            # Stochastic decision (Bernoulli trial)
            random_val = np.random.random()
            if ENABLE_DEBUG_OUTPUT:
                print(f"[LC-Check] V{v.id} x={v.x:.1f}m: "
                      f"prob={trigger_prob:.3f} vs rand={random_val:.3f}")

            if trigger_prob > random_val:
                # --- v18.21: LC開始位置チェック ---
                # LCは準備LC区間[0,500]以降で開始
                # LC実行後の位置が織り込み区間[500,1000]内に入ることを確認
                future_x = v.x + v.v * self.params.lc_prep_duration
                # 現在x>=0が保証されている（上のチェックで）
                # future_xが負の場合は速度が低すぎる（異常ケース）
                if future_x < 0.0:
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[LC-Skip-Future] V{v.id}: future_x={future_x:.1f}m < 0 (abnormal low speed)")
                # ------------------------------------------------------------
                # ====================================================================
                # v27.0: Unified LC Safety Check (ApolloSafetyManager)
                # ====================================================================
                # Replaces scattered _check_gap_acceptance() logic.
                # Uses CAV trajectory prediction for proactive safety verification.
                # Single source of truth for LC start decision.
                # ====================================================================
                
                safety_manager = get_safety_manager()
                lc_safety = safety_manager.check_lc_start_safety(
                    ego=v,
                    target_lane=target_lane,
                    vehicles=self.simulator.vehicles,
                    cav_trajectories=getattr(self.simulator, 'cav_trajectories', {}),
                    prep_zone_len=self.params.prep_zone_length,
                    total_len=self.params.total_length
                )

                if not lc_safety.is_safe:
                    # v28.0: Apollo-style urgent LC request handling
                    # If gap is not safe but vehicle is near exit, send V2V cooperation request
                    if lc_safety.v2v_request is not None and lc_safety.blocking_vehicle_id is not None:
                        # Find blocking vehicle
                        blocking_vehicle = None
                        for other_v in self.simulator.vehicles:
                            if other_v.id == lc_safety.blocking_vehicle_id:
                                blocking_vehicle = other_v
                                break

                        if blocking_vehicle is not None and not getattr(blocking_vehicle, 'is_hdv', False):
                            # Send V2V cooperation request to CAV
                            if not hasattr(blocking_vehicle, 'v2v_lc_requests'):
                                blocking_vehicle.v2v_lc_requests = []

                            blocking_vehicle.v2v_lc_requests.append({
                                'requester_id': v.id,
                                'request_type': lc_safety.v2v_request,  # 'urgent_yield' or 'cooperative_yield'
                                'urgency_level': lc_safety.urgency_level,
                                'distance_to_exit': lc_safety.distance_to_exit,
                                't_request': t,
                                'target_lane': target_lane
                            })

                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[V28-V2V-REQUEST] V{v.id} sends {lc_safety.v2v_request} to V{blocking_vehicle.id} "
                                      f"(urgency={lc_safety.urgency_level}, exit_dist={lc_safety.distance_to_exit:.1f}m)")

                    # Gap not safe: reject lane change for now
                    gap_reject_count += 1
                    self.gap_acceptance_reject_count += 1

                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_lc_decision_debug', False):
                        print(f"[LC Decision] t={t:.2f}s V{v.id} GAP REJECTED "
                              f"prob={trigger_prob:.3f} {v.lane}->{target_lane} "
                              f"x={v.x:.1f}m (urgency={lc_safety.urgency_level}, will retry)")
                    continue

                # ================================================================
                # v12.0: LC Booking with Preparation Phase
                # ================================================================
                # Instead of immediate execution, book a future start time
                # - Preparation period: configurable via lc_prep_duration (default 5.0s)
                # - During preparation: signal intent, adjust speed, others yield
                # - At scheduled time: execute if still safe
                # Reference: Human driver cooperation + V2V early notification
                # ================================================================
                PREPARATION_TIME = self.params.lc_prep_duration  # seconds

                # ================================================================
                # v29.0: Phase 2 - LC Preparation Conflict Check
                # ================================================================
                # Check if other vehicles are already scheduled for LC in same time window
                # If conflict detected, delay this vehicle's LC to avoid simultaneous merges
                # ================================================================
                lc_conflict_detected = False
                for other in self.simulator.vehicles:
                    if other.id == v.id or not getattr(other, 'lc_scheduled', False):
                        continue
                    
                    other_scheduled_time = getattr(other, 'scheduled_time', 0)
                    time_overlap = abs(other_scheduled_time - (t + PREPARATION_TIME)) < PREPARATION_TIME
                    
                    other_target = getattr(other, 'target_lane', None)
                    lane_conflict = (other_target == target_lane or 
                                   other.lane == target_lane or
                                   getattr(other, 'lane_from', None) == target_lane)
                    
                    if time_overlap and lane_conflict:
                        # Conflict: Delay this vehicle's LC
                        PREPARATION_TIME += 2.0  # Add 2s delay
                        lc_conflict_detected = True
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V29.0-LC-PREP-CONFLICT] V{v.id} delayed +2.0s due to V{other.id} " +
                                  f"(both target {target_lane} around t={other_scheduled_time:.1f}s)")
                        break  # One conflict is enough
                # ================================================================

                # Gap is safe: schedule lane change execution
                v.lc_scheduled = True
                v.target_position = v.x  # Start from current position
                v.scheduled_time = t + PREPARATION_TIME  # Book future execution

                # Ensure target_lane is properly set
                if not v.target_lane:
                    v.target_lane = target_lane

                # ================================================================
                # v12.0: Broadcast Lane Change Intent
                # ================================================================
                # Signal to nearby vehicles that we plan to merge
                # Other vehicles can proactively create space
                v.lc_intent = {
                    'target_lane': target_lane,
                    'start_time': v.scheduled_time,
                    'status': 'preparing'
                }
                # ================================================================

                lc_decision_count += 1

                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_lc_decision_debug', False):
                    print(f"[LC Booking] t={t:.2f}s V{v.id} BOOKED for t={v.scheduled_time:.2f}s "
                          f"prob={trigger_prob:.3f} {v.lane}->{v.target_lane} "
                          f"x={v.x:.1f}m (preparing...)")
                # ================================================================

        if ENABLE_DEBUG_OUTPUT and (lc_decision_count > 0 or gap_reject_count > 0):
            print(f"[Level1] t={t:.2f}s: {lc_decision_count} LC scheduled, {gap_reject_count} gap rejected")

    def _execute_level2_control(self, t: float, vehicles: List[Vehicle]):
        """
        Level 2: Tactical QP Control with Frenet-Based Obstacle Selection (Apollo Standard)
        Modified (v11.10 / 2025-12-19): HDV filtering + Apollo 200m scan range

        Reference:
        - Apollo: PiecewiseJerkSpeedOptimizer + PathDecider
        - Apollo SpeedBoundsDecider: ST-Boundary projection with geometric checks
        - Apollo Spec: 200m obstacle detection range (forward + backward)
        - Fan et al. (2018): Baidu Apollo EM Planner

        Implementation:
        1. Filter control targets: Only CAVs (HDVs use IDM in simulator)
        2. Convert all vehicles to Frenet coordinates (s, d)
        3. Detect obstacles using geometric lateral overlap: |d_obj - d_ego| < THRESHOLD
        4. Apply Apollo 200m scan radius (absolute distance filter)
        5. This approach is robust to:
           - Lane ID errors or inconsistencies
           - Vehicles cutting in between lanes
           - Edge cases during lane changes
        6. Solve Frenet QP with urgency-based safety margin relaxation
        7. Fallback to IDM when QP is infeasible (safety guarantee)

        CRITICAL FIX:
        - OLD: String-based lane matching -> fragile to lane transitions
        - NEW: Frenet 'd' coordinate geometric check -> robust to all scenarios
        - v11.10: HDVs excluded from control, 200m scan range applied

        Args:
            t: Current simulation time [s]
            vehicles: List of vehicles to control
        """
        if not vehicles:
            return

        # ====================================================================
        # v11.10: Filter CAVs only (HDVs handled by simulator IDM)
        # ====================================================================
        cav_vehicles = [v for v in vehicles if not getattr(v, 'is_hdv', False)]

        if not cav_vehicles:
            return
        # ====================================================================

        for v in cav_vehicles:
            # Get urgency (default 0.0 if not computed yet)
            urgency = getattr(v, 'urgency', 0.0)

            # ====================================================================
            # v13.2: Enable Car-Following in Negative Zone (Critical Fix)
            # ====================================================================
            # Problem: Rear-end collisions at x < 0 due to skipped control
            # Root Cause: Level 2 control was active but x < 0 vehicles were
            #             not detected as obstacles in some edge cases
            # Fix: Explicitly process ALL vehicles for car-following, regardless
            #      of position. Only Lane Change decisions require x >= 0.
            # Reference: Incident Report 2025-12-24 - Negative zone collisions
            # ====================================================================

            # ====================================================================
            # v11.13: Overtake Obstacle Exclusion
            # ====================================================================
            # 除外すべき障害物のIDリスト (Overtake対象など)
            ignore_obstacle_ids = []

            # ====================================================================
            # FRENET COORDINATE CONVERSION (Apollo PathDecider Standard)
            # ====================================================================
            # v13.9: Process Cooperative Acceleration Request (Front Vehicle Side)
            # ====================================================================
            # If rear vehicle requested acceleration to open gap, honor the request.
            # This enables cooperative deadlock resolution without rear vehicle
            # needing to decelerate below v_min.
            #
            # Conditions for honoring request:
            # 1. Not AEB-active ourselves (safety first)
            # 2. Have road space ahead (can safely accelerate)
            # 3. Request is reasonable (< 2.0 m/s^2)
            #
            # Reference: Apollo cooperative_planner.cc - ProcessGapOpeningRequest
            # ====================================================================
            coop_accel_request = getattr(v, 'cooperative_accel_request', 0.0)
            if coop_accel_request > 0.1 and not getattr(v, 'aeb_active', False):
                # Check if we have space ahead to accelerate
                front_gap = float('inf')
                for other in self.simulator.vehicles:
                    if other.id != v.id and not other.exited:
                        if other.lane == v.lane and other.x > v.x:
                            gap = other.x - v.x - self.params.L_vehicle
                            front_gap = min(front_gap, gap)
                
                # Only accelerate if sufficient gap ahead (> 15m)
                if front_gap > 15.0 and v.v < self.params.v_max - 2.0:
                    # Apply cooperative acceleration
                    v.ax = min(coop_accel_request, 2.0)
                    
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[COOP-ACCEL-FULFILL] V{v.id} accelerating per rear request: "
                              f"ax={v.ax:.1f}m/s^2, front_gap={front_gap:.1f}m")
                
                # Decay the request (processed or not)
                v.cooperative_accel_request = max(0.0, coop_accel_request - 0.5)
            # ====================================================================

            # ====================================================================
            # Convert ego and all other vehicles to Frenet frame (s, d)
            # For straight highway: s = x (longitudinal), d = y (lateral offset)
            # ====================================================================

            # Ego vehicle Frenet coordinates (use actual physical position)
            ego_s = v.x  # Longitudinal position
            ego_d = v.d  # Lateral offset from centerline (updated by simulator)

            ego_state = VehicleState(
                id=v.id,
                s=ego_s,
                v=v.v,
                a=getattr(v, 'ax', 0.0),
                lane=v.lane
            )

            # ====================================================================
            # FRENET-BASED OBSTACLE SELECTION (Apollo PathDecider)
            # ====================================================================
            # Use lateral offset 'd' to determine which vehicles are on the same path.
            # This is more robust than lane string matching.
            # --- modified (Final Fix v11.6): Shadow Mode + Intelligent Speed Planning ---
            # ====================================================================
            
            # ====================================================================
            # v25.1: Apollo Concurrent LC Decider (Pairwise Cost Optimization)
            # ====================================================================
            # Solves the "Double Merge" problem (e.g. Left->LC and Right->LC).
            # Evaluates discrete decisions (Yield vs Accelerate) for competing pairs
            # based on total cost (Progress + Urgency + Smoothness).
            # ====================================================================
            if v.changing_lane and v.target_lane:
                # 1. Identify Competitors: Vehicles targeting the same lane from different origin
                competitors = [
                    u for u in self.simulator.vehicles
                    if u.id != v.id and not u.exited and
                    u.changing_lane and
                    u.target_lane == v.target_lane and
                    u.lane != v.lane  # Must be from different origin (e.g. Left vs Right)
                ]

                for u in competitors:
                    # check if longitudinal overlap is critical (within 40m projected)
                    # Simple project: x_future = x + v * 3.0
                    v_proj = v.x + v.v * 3.0
                    u_proj = u.x + u.v * 3.0
                    
                    # If their projected intervals overlap, we have a conflict
                    dist_now = abs(v.x - u.x)
                    dist_proj = abs(v_proj - u_proj)
                    
                    if dist_now < 40.0 or dist_proj < 20.0:
                        # 2. Calculate Costs for Two Scenarios
                        # Scenario A: Ego (v) Yields, Competitor (u) Proceeds
                        # Scenario B: Competitor (u) Yields, Ego (v) Proceeds
                        
                        # Cost factors
                        # - Progress: Penalize yielding if already deep in LC
                        # - Urgency: Penalize yielding if close to exit
                        # - Smoothness: Penalize yielding if it requires heavy braking (high speed)
                        
                        # Calculate Cost(v_yields)
                        v_progress = np.clip((t - v.lc_start_time) / self.params.lc_duration, 0, 1)
                        v_urgency_score = v.urgency  # 0.0 to 1.0+
                        cost_v_yield = (v_progress * 5.0) + (v_urgency_score * 3.0) + (v.v * 0.1)
                        
                        # Calculate Cost(u_yields)
                        u_progress = np.clip((t - u.lc_start_time) / self.params.lc_duration, 0, 1)
                        u_urgency_score = u.urgency
                        cost_u_yield = (u_progress * 5.0) + (u_urgency_score * 3.0) + (u.v * 0.1)
                        
                        # 3. Make Decision (Minimize Total System Cost)
                        # We only control 'v' here. We simulate the symmetric logic.
                        
                        if cost_v_yield < cost_u_yield:
                            # Ego should Yield
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[V25.1-DECIDER] Conflict V{v.id} vs V{u.id} -> V{v.id} YIELDS "
                                      f"(Cost: {cost_v_yield:.1f} vs {cost_u_yield:.1f})")
                            
                            # Action: Pause LC and Reduce Speed
                            v.lc_paused = True
                            v.lc_pause_until = max(getattr(v, 'lc_pause_until', 0), t + 0.5) # Short pause, re-eval next cycle
                            
                            # Apply gentle braking (pre-QP speed reduction)
                            # We set a flag that will be used in v_ref calculation later
                            v.trajectory_conflict_v_reduction = max(
                                getattr(v, 'trajectory_conflict_v_reduction', 0.0), 
                                0.3  # 30% reduction
                            )
                        else:
                            # Ego should Proceed (Competitor YIELDS)
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[V25.1-DECIDER] Conflict V{v.id} vs V{u.id} -> V{v.id} PROCEEDS "
                                      f"(Cost: {cost_v_yield:.1f} vs {cost_u_yield:.1f})")
                            
                            # Action: Accelerate slightly if needed to clear gap
                            # Only if we are behind or alongside and need to pass
                            if v.x < u.x + 10.0:
                                v.trajectory_conflict_v_reduction = -0.1 # Negative reduction = Boost (handled later?)
                                # Logic for boost needs to be added to v_ref calculation 
                                # (currently it only does reduction. We can hack it or add explicit boost support)
                                # Let's stick to "Not Yielding" = Maintain speed or normal logic
                                pass
            # ====================================================================
            # Check shared CAV trajectories for predicted collisions.
            # If collision predicted in 1-3s, reduce target velocity for QP.
            # This prevents "approaching too fast" situations that lead to AEB.
            # ====================================================================
            v.trajectory_conflict_v_reduction = 0.0  # Reset each cycle
            
            ego_traj = self.simulator.cav_trajectories.get(v.id)
            if ego_traj:
                for other_id, other_traj in self.simulator.cav_trajectories.items():
                    if other_id == v.id:
                        continue
                    
                    # Check for collision in 3s horizon
                    conflict_time = None
                    horizon_steps = min(len(ego_traj['x']), len(other_traj['x']))
                    
                    for step in range(horizon_steps):
                        long_gap = abs(ego_traj['x'][step] - other_traj['x'][step])
                        lat_gap = abs(ego_traj['d'][step] - other_traj['d'][step])
                        
                        # Collision threshold: 15m longitudinal, 2.5m lateral
                        if long_gap < 15.0 and lat_gap < 2.5:
                            conflict_time = step * self.simulator.trajectory_dt
                            break
                    
                    if conflict_time is not None:
                        # Calculate v_reduction based on conflict urgency
                        if conflict_time < 1.0:
                            v_reduction = 0.5  # 50% reduction (imminent)
                        elif conflict_time < 2.0:
                            v_reduction = 0.3  # 30% reduction (soon)
                        else:
                            v_reduction = 0.15  # 15% reduction (approaching)
                        
                        v.trajectory_conflict_v_reduction = max(
                            v.trajectory_conflict_v_reduction, v_reduction
                        )
                        
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V23.1-REDUCE] V{v.id} conflict with V{other_id} at t+{conflict_time:.1f}s "
                                  f"→ v_reduction={v_reduction*100:.0f}%")
                        break  # One conflict is enough
            # ====================================================================

            obstacles = []

            # ====================================================================
            # v29.0: Phase 2 - TTC Prediction with Target Lane Consideration
            # ====================================================================
            # During lane change, also consider front vehicle in target lane
            # Prevents AEB trigger when LC into narrow gap
            # ====================================================================
            if getattr(v, 'changing_lane', False) and getattr(v, 'lane_to', None) is not None:
                # Find front vehicle in target lane
                target_lane_vehicles = [
                    u for u in self.simulator.vehicles
                    if u.lane == v.lane_to and not u.exited and u.x > v.x
                ]
                if target_lane_vehicles:
                    target_front = min(target_lane_vehicles, key=lambda u: u.x)
                    target_gap = target_front.x - v.x - 5.0  # vehicle length
                    target_v_rel = v.v - target_front.v
                    
                    if target_v_rel > 0 and target_gap > 0:
                        target_ttc = target_gap / target_v_rel
                        # If target lane TTC is critical, add as high-priority obstacle
                        if target_ttc < 3.0:
                            obstacles.append(ObstacleInfo(
                                vehicle_state=VehicleState(
                                    id=target_front.id,
                                    s=target_front.x,
                                    d=target_front.d,
                                    v=target_front.v,
                                    a=getattr(target_front, 'ax', getattr(target_front, 'a', 0.0)),
                                    lane=getattr(target_front, 'lane', v.lane),
                                    is_hdv=getattr(target_front, 'is_hdv', False)
                                ),
                                is_front=True,
                                is_virtual=False,
                                gap=target_gap,
                                ttc=target_ttc,
                                priority_level=2  # Higher than normal (1)
                            ))
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[V29.0-TARGET-LANE-TTC] V{v.id} LC: target_lane front V{target_front.id} " +
                                      f"gap={target_gap:.1f}m, TTC={target_ttc:.2f}s")
            # ====================================================================

            # ====================================================================
            # v11.11: Virtual Obstacle Yielding (Apollo Cooperative Strategy)
            # ====================================================================
            # Project merging vehicles from adjacent lanes as virtual obstacles
            # - Enables QP solver to naturally generate yielding trajectories
            # - Avoids manual speed adjustment (more robust optimization)
            # Reference: Apollo "Yield to Obstacle" + Virtual boundary projection
            # ====================================================================
            adjacent_lanes = get_adjacent_lanes(v.lane)

            for adj_lane in adjacent_lanes:
                # Find vehicles in adjacent lane that intend to merge into ego lane
                mergers = [
                    u for u in self.simulator.vehicles
                    if u.lane == adj_lane and
                    not u.exited and
                    (getattr(u, 'lc_intent', None) or {}).get('target_lane') == v.lane
                ]

                for merger in mergers:
                    # Only yield if ego vehicle is lag (behind or alongside merger)
                    # Apollo: Don't yield if ego is significantly ahead
                    if merger.x > v.x - 10.0:  # Merger ahead or alongside
                        # Create virtual obstacle behind merger to reserve space
                        # This forces QP to slow down and create gap
                        virtual_s = merger.x - 15.0  # 15m gap reservation
                        virtual_v = merger.v  # Match merger's speed

                        # Add as front obstacle (even if behind ego, to create braking)
                        # QP solver will naturally decelerate to maintain safe distance
                        obstacles.append(ObstacleInfo(
                            vehicle_state=VehicleState(
                                id=-merger.id,  # Negative ID indicates virtual obstacle
                                s=virtual_s,
                                v=virtual_v,
                                a=0.0,
                                lane=v.lane  # Project into ego lane
                            ),
                            is_front=(virtual_s > ego_s),  # Geometric front/rear
                            predicted_trajectory=None,  # Virtual obstacle has no trajectory
                            is_changing_lane=False,  # Virtual obstacles don't change lanes
                            lateral_velocity=0.0
                        ))

                        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_virtual_obstacle_debug', False):
                            print(f"[VIRTUAL-OBS] V{v.id} yielding to V{merger.id} via virtual obstacle "
                                  f"at s={virtual_s:.1f}m (ego={ego_s:.1f}m)")
            # ====================================================================

            # Define scan paths (d-coordinates to check)
            scan_d_values = [ego_d]  # Current path
            target_flow_speed = self.params.v_max  # Default: speed limit

            # Add target path if changing lanes (Shadow Mode)
            is_lane_changing = getattr(v, 'lc_scheduled', False) or getattr(v, 'changing_lane', False)
            target_lane = getattr(v, 'target_lane', None)

            if is_lane_changing and target_lane:
                # Use target lane center from LANE_OFFSETS (theoretical destination)
                target_d = LANE_OFFSETS.get(target_lane, ego_d)
                if abs(target_d - ego_d) > 0.1:  # Only add if different
                    scan_d_values.append(target_d)

                # ====================================================================
                # v11.12: ST-Graph Based Overtake/Yield (Apollo Speed Bounds Decider)
                # ====================================================================
                # Modified: Inject Yield constraint directly into ST-Graph (s_upper)
                # - Overtake: v_ref adjustment only (encourage acceleration)
                # - Yield: Add virtual obstacle to enforce s_upper constraint
                # Reference: Apollo SpeedBoundsDecider - ST boundary formation
                # ====================================================================
                target_lane_vehicles = [u for u in self.simulator.vehicles
                                       if u.lane == target_lane and not u.exited and u.id != v.id]

                # Identify side vehicle (parallel vehicle within +/-15m)
                side_vehicle = None
                for u in target_lane_vehicles:
                    if abs(u.x - v.x) < 15.0:  # Parallel threshold
                        side_vehicle = u
                        break

                if side_vehicle:
                    # --- Smart Decision: Overtake or Yield? ---
                    # 修正: 完全に前でなくても、-5.0m (鼻先が少し後ろ) 程度なら加速勝負できるとする
                    is_ahead = v.x > side_vehicle.x - 5.0
                    can_accel = v.v < self.params.v_max - 2.0

                    # 相対速度も考慮: 相手より速い、または同等ならOvertakeしやすい
                    is_faster = v.v >= side_vehicle.v - 1.0

                    if is_ahead and can_accel and is_faster:
                        # 【OVERTAKE MODE】v_ref boost (encourage acceleration)
                        # No ST-Graph constraint (let QP optimize freely)
                        target_flow_speed = min(self.params.v_max, side_vehicle.v + 5.0)

                        # ★重要★: 追い越すと決めた相手は、QPの「前方障害物(Upper Bound)」として認識させてはいけない。
                        # 認識させると「車間距離を空けろ」という制約でブレーキがかかってしまう。
                        ignore_obstacle_ids.append(side_vehicle.id)
                        
                        # ================================================================
                        # v16.3: OVERTAKE縦方向安全チェック（AEB予防）
                        # ================================================================
                        # 追い越しモードでも、相手が前方にいて接近している場合は
                        # 速度を制限してAEB発動を予防する
                        # ================================================================
                        side_gap = side_vehicle.x - v.x - 5.0  # バンパー間距離
                        if side_gap > 0 and side_gap < 30.0:  # 相手が前方で30m以内
                            side_v_rel = v.v - side_vehicle.v
                            if side_v_rel > 0:  # 接近中
                                # TTCチェック
                                side_ttc = side_gap / side_v_rel
                                if side_ttc < 5.0:  # 5秒以内に追いつく
                                    # 速度を相手より少し高い程度に制限
                                    overtake_safe_v = side_vehicle.v + 2.0
                                    if target_flow_speed > overtake_safe_v:
                                        target_flow_speed = overtake_safe_v
                                        if ENABLE_DEBUG_OUTPUT:
                                            print(f"[OVERTAKE-SAFETY] V{v.id} limiting speed for safe overtake: "
                                                  f"gap={side_gap:.1f}m, TTC={side_ttc:.1f}s -> v_ref={target_flow_speed:.1f}m/s")
                        # ================================================================

                        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_overtake_debug', False):
                            print(f"[OVERTAKE] V{v.id} ignoring V{side_vehicle.id} to accelerate: "
                                  f"v_target={target_flow_speed:.1f}m/s (side_v={side_vehicle.v:.1f})")
                    else:
                        # 【YIELD MODE】Speed control + Conditional exclusion (Apollo Style)
                        target_flow_speed = min(target_flow_speed, side_vehicle.v - 3.0)

                        # ★重要★: まだ並走中(Overlap)なのに「後ろにいろ」と制約するとInfeasibleになる。
                        # 物理的に十分後ろ(例: 10m)に下がるまでは、「制約からは除外」し、「速度」だけで下がる。
                        if v.x > side_vehicle.x - 10.0:
                            ignore_obstacle_ids.append(side_vehicle.id)

                            if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_overtake_debug', False):
                                print(f"[YIELD-SPEED] V{v.id} yielding to V{side_vehicle.id} via speed control: "
                                      f"v_target={target_flow_speed:.1f}m/s (ignoring constraint during overlap)")
                        else:
                            # 十分後ろ(gap > 10m)になったら、普通に障害物として認識され
                            # QPが安全距離を維持してくれるので、特別なVirtual Obstacleは不要。
                            if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_overtake_debug', False):
                                print(f"[YIELD-CLEAR] V{v.id} now behind V{side_vehicle.id}: "
                                      f"gap={side_vehicle.x - v.x:.1f}m, v_target={target_flow_speed:.1f}m/s")

                    # Wait until parallel conflict resolves before starting lateral movement
                    # Reschedule LC check in 0.2s (fine-grained monitoring)
                    if v.lc_scheduled and not v.lc_started:
                        v.scheduled_time = t + 0.2

                else:
                    # --- No side vehicle: Clear Lead/Lag case ---
                    # Use slot-based planning (existing logic)
                    lead = min([u for u in target_lane_vehicles if u.x > v.x],
                              key=lambda u: u.x, default=None)
                    lag = max([u for u in target_lane_vehicles if u.x < v.x],
                             key=lambda u: u.x, default=None)

                    if lead and lag:
                        # Sandwiched: Match average speed
                        target_flow_speed = min(target_flow_speed, (lead.v + lag.v) / 2.0)
                    elif lead:
                        # Only lead: Match lead speed
                        target_flow_speed = min(target_flow_speed, lead.v)
                    elif lag:
                        # Only lag: Slightly faster to create gap
                        target_flow_speed = min(self.params.v_max, lag.v + 3.0)
                # ====================================================================

            # Lateral overlap threshold (Apollo: half lane width + safety margin)
            # LANE_WIDTH = 3.5m -> threshold = 1.75m + 0.25m margin = 2.0m
            LATERAL_THRESHOLD = LANE_WIDTH / 2.0 + 0.25

            # Collect all candidate obstacles with lateral overlap
            # --- modified (Final Fix v11.6): Enhanced obstacle tracking for speed harmonization ---
            # Use 'is_active' logic: exclude exited vehicles but include boundary cases
            candidate_obstacles = []
            rear_threat = None  # Track rear threat for Yield logic
            closest_front_dist = float('inf')  # Track closest front vehicle
            closest_front_vehicle = None  # For speed harmonization

            # ====================================================================
            # v11.10: Apollo 200m Scan Range (Obstacle Detection Spec)
            # ====================================================================
            # Apollo production: 200m forward + 200m backward obstacle detection
            # - Reduces computational load
            # - Matches sensor/perception range
            # - Focuses on relevant obstacles only
            SCAN_RADIUS = 200.0  # meters
            # ====================================================================

            # Filter active vehicles: not exited OR just beyond boundary (ghost buffer)
            # Apply 200m scan radius (Apollo spec)
            # v19.0: Include ALL vehicles (overtake targets will be relaxed in QP)
            active_vehicles = [
                u for u in self.simulator.vehicles
                if (not u.exited or u.x > self.params.total_length) and
                u.id != v.id and
                # v19.0: REMOVED ignore_obstacle_ids filter - include all vehicles
                # Overtake targets will have is_overtake_target=True in ObstacleInfo
                abs(u.x - v.x) < SCAN_RADIUS  # Apollo 200m range
            ]
            
            # ================================================================
            # v16.2: AEB用障害物リスト（ignore_obstacle_idsを無視）
            # ================================================================
            # 重要: AEB/安全チェックは全車両に対して行う必要がある
            # ignore_obstacle_idsはQP最適化の制約から除外するだけで、
            # 衝突回避の安全チェックからは除外してはならない
            # ================================================================
            all_active_vehicles_for_aeb = [
                u for u in self.simulator.vehicles
                if (not u.exited or u.x > self.params.total_length) and
                u.id != v.id and
                abs(u.x - v.x) < SCAN_RADIUS
            ]

            for other_v in active_vehicles:
                # Convert other vehicle to Frenet coordinates (use actual physical position)
                other_s = other_v.x
                other_d = other_v.d  # Use actual lateral position from simulator

                # Check lateral overlap with any scan path
                for scan_d in scan_d_values:
                    lateral_distance = abs(other_d - scan_d)

                    if lateral_distance < LATERAL_THRESHOLD:
                        # This vehicle overlaps with our path
                        is_front = other_s > ego_s
                        longitudinal_distance = abs(other_s - ego_s)

                        candidate_obstacles.append({
                            'vehicle': other_v,
                            'is_front': is_front,
                            'long_dist': longitudinal_distance,
                            'lat_dist': lateral_distance,
                            's': other_s,
                            'd': other_d
                        })

                        # --- Front Vehicle Tracking (for Speed Harmonization) ---
                        # Track the closest front vehicle across all scan paths
                        if is_front and longitudinal_distance < closest_front_dist:
                            closest_front_dist = longitudinal_distance
                            closest_front_vehicle = other_v
                        # ---

                        # --- Rear Threat Detection (for Yield logic) ---
                        # Identify fast-approaching rear vehicle in target lane
                        if not is_front and is_lane_changing and target_lane and other_v.lane == target_lane:
                            if other_v.v > v.v:  # Rear vehicle is faster
                                if rear_threat is None or other_v.x > rear_threat.x:
                                    rear_threat = other_v
                        # ---

                        break  # Only count once per vehicle

            # ================================================================
            # v16.2: AEB用前方障害物リスト（全車両スキャン）
            # ================================================================
            # ignore_obstacle_idsを無視して安全性チェック用のリストを作成
            # これにより追い越し/譲り対象でもAEBが発動可能になる
            # 
            # ★ CRITICAL FIX (v27.19): Apollo準拠の横方向距離判定
            # - 従来: scan_d基準で判定 → 5.45m離れていてもAEB発動
            # - 修正: ego_d基準で判定 → 実際の横方向距離で評価
            # - 基準: 2.0m以内（約0.5車線幅）のみAEB対象とする
            # Reference: Apollo modules/planning/tasks/deciders/speed_decider.cc
            #           IsOnReferenceLine() - lateral distance check
            # ================================================================
            aeb_candidate_obstacles = []
            AEB_LATERAL_THRESHOLD = 2.0  # [m] Apollo準拠: 実際の横方向距離閾値
            
            for other_v in all_active_vehicles_for_aeb:
                other_s = other_v.x
                other_d = other_v.d
                
                # 自車との実際の横方向距離で判定（スキャン基準点ではない）
                actual_lateral_distance = abs(other_d - ego_d)
                
                if actual_lateral_distance < AEB_LATERAL_THRESHOLD:
                    is_front = other_s > ego_s
                    longitudinal_distance = abs(other_s - ego_s)
                    aeb_candidate_obstacles.append({
                        'vehicle': other_v,
                        'is_front': is_front,
                        'long_dist': longitudinal_distance,
                        'lat_dist': actual_lateral_distance,  # 実際の距離を記録
                        's': other_s,
                        'd': other_d
                    })
            
            aeb_front_obstacles = [obs for obs in aeb_candidate_obstacles if obs['is_front']]
            aeb_front_obstacles.sort(key=lambda x: x['long_dist'])
            # ================================================================

            # Initialize v_ref_adjusted with target flow speed
            v_ref_adjusted = target_flow_speed
            
            # ====================================================================
            # v25.3: Early Congestion/Stopped Vehicle Detection
            # ====================================================================
            # Problem: AEB triggers via TTC, but if front vehicle is STOPPED (v=0),
            # TTC = gap / v_rel = infinity until we're very close.
            # Solution: Directly check front vehicle speed and gap.
            # ====================================================================
            if closest_front_vehicle is not None and closest_front_dist < 80.0:
                front_v = closest_front_vehicle.v
                gap = closest_front_dist - self.params.L_vehicle
                
                # Case 1: Front vehicle is stopped or nearly stopped
                if front_v < 1.0:  # Stopped
                    # Must stop before hitting front vehicle
                    # Required stopping distance: v^2 / (2 * a_brake)
                    a_brake = 5.0  # Conservative braking
                    stopping_dist = (v.v ** 2) / (2 * a_brake) + 5.0  # +5m buffer
                    
                    if gap < stopping_dist:
                        # We cannot stop in time at current speed - reduce v_ref
                        safe_v = max(0.0, np.sqrt(2 * a_brake * max(0, gap - 5.0)))
                        if safe_v < v_ref_adjusted:
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[V25.3-CONGESTION] V{v.id} front V{closest_front_vehicle.id} stopped, "
                                      f"gap={gap:.1f}m, safe_v={safe_v:.1f}m/s")
                            v_ref_adjusted = max(safe_v, 2.0)  # Min 2 m/s (creep)
                
                # Case 2: Front vehicle is slow (congestion forming)
                elif front_v < 5.0 and gap < 30.0:
                    # Match front vehicle speed plus margin
                    v_ref_adjusted = min(v_ref_adjusted, front_v + 1.0)
            # ====================================================================
            
            # ====================================================================
            # v23.1: Apply trajectory conflict velocity reduction
            # ====================================================================
            # If v23.1 conflict check detected collision, reduce target velocity
            # This enables QP to plan a slower trajectory BEFORE AEB is needed.
            # ====================================================================
            v_reduction = getattr(v, 'trajectory_conflict_v_reduction', 0.0)
            if v_reduction > 0.0:
                # REDUCE speed (Yield/Conflict)
                v_ref_adjusted_before = v_ref_adjusted
                v_ref_adjusted = v_ref_adjusted * (1.0 - v_reduction)
                # Ensure minimum speed of 5 m/s to prevent stopping
                v_ref_adjusted = max(v_ref_adjusted, 5.0)
                
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[V23.1-VREF] V{v.id} v_ref: {v_ref_adjusted_before:.1f} → {v_ref_adjusted:.1f}m/s "
                          f"(reduction={v_reduction*100:.0f}%)")
            elif v_reduction < 0.0:
                # INCREASE speed (Proceed/Overtake)
                # v_reduction is negative, so (1.0 - v_reduction) > 1.0
                v_ref_adjusted_before = v_ref_adjusted
                v_ref_adjusted = min(v_ref_adjusted * (1.0 - v_reduction), self.params.v_max)
                
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[V25.1-VREF] V{v.id} v_ref: {v_ref_adjusted_before:.1f} → {v_ref_adjusted:.1f}m/s "
                          f"(boost={-v_reduction*100:.0f}%)")
            # ====================================================================
            
            # ====================================================================
            # v24.0: Apply GUARDIAN abort speed reduction
            # ====================================================================
            # When LC is aborted due to collision risk, reduce speed to create gap
            # ====================================================================
            guardian_reduction_until = getattr(v, 'guardian_speed_reduction_until', 0.0)
            if guardian_reduction_until > t:
                v_ref_before_guardian = v_ref_adjusted
                v_ref_adjusted = v_ref_adjusted * 0.7  # 30% reduction
                v_ref_adjusted = max(v_ref_adjusted, 5.0)  # Floor at 5 m/s
                
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[V24.0-GUARDIAN-SLOW] V{v.id} v_ref: {v_ref_before_guardian:.1f} → {v_ref_adjusted:.1f}m/s "
                          f"(GUARDIAN abort mode, {guardian_reduction_until - t:.1f}s remaining)")
            # ====================================================================
            
            # ====================================================================
            # v28.0: V2V Urgent LC Request Handler (Apollo MUST_CHANGE mode)
            # ====================================================================
            # Process urgent/emergency LC requests from nearby CAVs near exit.
            # Apollo Scenario Manager: cooperative behaviors in merge scenarios.
            # ====================================================================
            if hasattr(v, 'v2v_lc_requests') and len(v.v2v_lc_requests) > 0:
                # Process most urgent request (emergency > urgent > cooperative)
                v.v2v_lc_requests.sort(key=lambda r: (
                    0 if r['urgency_level'] == 'emergency' else 1 if r['urgency_level'] == 'urgent' else 2,
                    r['distance_to_exit']  # Closer to exit = higher priority
                ))

                request = v.v2v_lc_requests[0]  # Most urgent request
                requester_id = request['requester_id']
                request_type = request['request_type']
                urgency_level = request['urgency_level']

                # Find requester vehicle
                requester = None
                for other_v in self.simulator.vehicles:
                    if other_v.id == requester_id:
                        requester = other_v
                        break

                if requester is not None:
                    # Decide action based on relative position
                    relative_pos = requester.x - v.x

                    if relative_pos > 0:
                        # Requester is ahead: YIELD (slow down to create rear gap)
                        if request_type == 'urgent_yield':
                            v_ref_adjusted = v_ref_adjusted * 0.7  # 30% reduction (emergency)
                        else:
                            v_ref_adjusted = v_ref_adjusted * 0.85  # 15% reduction (cooperative)

                        v_ref_adjusted = max(v_ref_adjusted, 5.0)  # Floor at 5 m/s

                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V28-YIELD] V{v.id} yields to V{requester_id} (type={request_type}, "
                                  f"urgency={urgency_level}, v_ref={v_ref_adjusted:.1f}m/s)")

                    else:
                        # Requester is behind: ACCELERATE (speed up to create front gap)
                        if request_type == 'urgent_yield':
                            v_ref_adjusted = min(v_ref_adjusted * 1.15, self.params.v_max)  # 15% increase
                        else:
                            v_ref_adjusted = min(v_ref_adjusted * 1.08, self.params.v_max)  # 8% increase

                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V28-ACCELERATE] V{v.id} accelerates for V{requester_id} (type={request_type}, "
                                  f"urgency={urgency_level}, v_ref={v_ref_adjusted:.1f}m/s)")

                # Clear old requests (older than 3 seconds)
                v.v2v_lc_requests = [r for r in v.v2v_lc_requests if t - r['t_request'] < 3.0]
            # ====================================================================

            # ====================================================================
            # v25.0: Smart Cooperative Handler (Accelerate / Yield / Reject)
            # ====================================================================
            # When another CAV requests cooperation during their mid-LC conflict,
            # decide the best action based on relative position and situation:
            # - ACCELERATE: If we're ahead, speed up to create front gap
            # - YIELD: If we're behind, slow down to create rear gap
            # - REJECT: If unsafe (AEB active, no space, etc.)
            # ====================================================================
            # ====================================================================
            # v25.2: Unified Apollo V2V Conflict Decider
            # ====================================================================
            # Replaces v25.0 (Coop), v25.1 (Concurrent), and v20.0 (Conflict)
            # with a single cost-based arbitration logic for ALL conflict types.
            # Evaluates pairwise interactions (Straight vs LC, LC vs LC).
            # ====================================================================
            
            # Helper to calculate yield cost for a vehicle
            def calculate_yield_cost(u, is_lc_active):
                # Cost components:
                # 1. Safety/Urgency (Blocking exit, Blocking flow)
                u_urgency = getattr(u, 'urgency', 0.0)
                
                # 2. Progress (State continuity)
                # If LC is active and deep (>50%), high cost to abort/yield
                u_progress_cost = 0.0
                if is_lc_active:
                    lc_start = getattr(u, 'lc_start_time', 0.0)
                    progress = np.clip((t - lc_start) / self.params.lc_duration, 0, 1)
                    u_progress_cost = progress * 5.0  # Up to 5.0 cost
                else:
                    # Straight vehicles have "Right of Way" baseline cost (harder to force yield)
                    u_progress_cost = 2.0
                
                # 3. Comfort (Velocity change)
                # Higher speed = higher cost to brake significantly
                u_comfort_cost = u.v * 0.1
                
                return (u_urgency * 4.0) + u_progress_cost + u_comfort_cost

            # Scan for conflicts (Wide Area Awareness)
            wide_scan_lanes = [v.lane]
            if v.target_lane:
                wide_scan_lanes.append(v.target_lane)
            
            potential_conflicts = [
                u for u in self.simulator.vehicles
                if u.id != v.id and not u.exited and
                u.lane in wide_scan_lanes and
                abs(u.x - v.x) < 60.0  # 60m Relevant Range
            ]

            for u in potential_conflicts:
                # Detect Conflict: Trajectory Overlap
                # Simple linear projection check
                v_future = v.x + v.v * 2.5
                u_future = u.x + u.v * 2.5
                
                dist_now = abs(v.x - u.x)
                dist_future = abs(v_future - u_future)
                
                # Check for lane overlap intent
                v_target = v.target_lane if v.changing_lane else v.lane
                u_target = u.target_lane if u.changing_lane else u.lane
                
                same_target = (v_target == u_target)
                crossing = (v.target_lane == u.lane) or (v.lane == u.target_lane)
                
                is_conflict = (same_target or crossing) and (dist_now < 40.0 or dist_future < 10.0)
                
                if is_conflict:
                    # Calculate System Costs for 2 Scenarios
                    # Scenario A: V Yields, U Proceeds
                    cost_v_yield = calculate_yield_cost(v, v.changing_lane)
                    
                    # Scenario B: U Yields, V Proceeds
                    cost_u_yield = calculate_yield_cost(u, u.changing_lane)
                    
                    # Decision
                    if cost_v_yield < cost_u_yield:
                        # V should Yield (We are V)
                        # Action: Slow down or Pause LC
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V25.2-DECIDER] Conflict V{v.id} vs V{u.id} -> V{v.id} YIELDS "
                                  f"(Cost: {cost_v_yield:.1f} vs {cost_u_yield:.1f})")
                        
                        target_reduction = 0.4 # 40% speed reduction
                        if v.changing_lane:
                             v.lc_paused = True
                             v.lc_pause_until = max(getattr(v, 'lc_pause_until', 0), t + 0.5)
                        
                        v.trajectory_conflict_v_reduction = max(
                            getattr(v, 'trajectory_conflict_v_reduction', 0.0), 
                            target_reduction
                        )
                    else:
                        # V should Proceed (U Yields)
                        # Action: Maintain or slight boost
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V25.2-DECIDER] Conflict V{v.id} vs V{u.id} -> V{v.id} PROCEEDS "
                                  f"(Cost: {cost_v_yield:.1f} vs {cost_u_yield:.1f})")
                        
                        # Proceed doesn't mean reckless accel, but maintaining flow
                        if v.x < u.x + 10.0 and v.v < self.params.v_max:
                             v.trajectory_conflict_v_reduction = -0.1 # 10% boost
            # ====================================================================                # End of v25.2 Unified Decider
            # ====================================================================


            # ====================================================================
            # CIPV SELECTION (Closest In-Path Vehicle)
            # ====================================================================
            # Apollo PathDecider: Select only the most relevant obstacles
            # - Front: Closest vehicle ahead (CIPV-Front)
            # - Rear: Closest vehicle behind (CIPV-Rear)
            # This reduces computational burden and focuses on immediate threats
            # ====================================================================

            # Separate front and rear candidates
            front_obstacles = [obs for obs in candidate_obstacles if obs['is_front']]
            rear_obstacles = [obs for obs in candidate_obstacles if not obs['is_front']]

            # Sort by longitudinal distance (closest first)
            front_obstacles.sort(key=lambda x: x['long_dist'])
            rear_obstacles.sort(key=lambda x: x['long_dist'])

            # Select CIPV-Front (closest front vehicle only)
            # Apollo: Only the immediate leading vehicle matters for speed planning
            for obs in front_obstacles[:1]:  # Only take the closest one
                other_v = obs['vehicle']
                obs_state = VehicleState(
                    id=other_v.id,
                    s=obs['s'],
                    v=other_v.v,
                    a=getattr(other_v, 'ax', 0.0),
                    lane=other_v.lane
                )

                # ================================================================
                # v12.0: CAV Trajectory Sharing
                # ================================================================
                # Get predicted trajectory if available (CAV) or None (HDV)
                predicted_traj = None
                if not getattr(other_v, 'is_hdv', False):  # Only share if CAV
                    predicted_traj = getattr(other_v, 'predicted_trajectory', None)
                    # Validate trajectory data
                    if predicted_traj and len(predicted_traj) < 5:
                        predicted_traj = None  # Too short, ignore
                # ================================================================

                # v12.1: Phase 2 - Add lane change information
                is_changing_lane = getattr(other_v, 'lc_started', False)
                lateral_velocity = abs(getattr(other_v, 'vy', 0.0)) if is_changing_lane else 0.0

                # v12.2: Phase 3 - Lane relevance classification
                # Determine if obstacle is in current lane, target lane, or other lane
                target_lane = getattr(v, 'target_lane', None)
                if other_v.lane == v.lane:
                    lane_relevance = "current"
                elif target_lane and other_v.lane == target_lane:
                    lane_relevance = "target"
                else:
                    lane_relevance = "other"

                obstacles.append(ObstacleInfo(
                    obs_state,
                    is_front=True,
                    predicted_trajectory=predicted_traj,
                    is_changing_lane=is_changing_lane,
                    lateral_velocity=lateral_velocity,
                    lane_relevance=lane_relevance,
                    is_overtake_target=(other_v.id in ignore_obstacle_ids)  # v19.0: Mark overtake targets
                ))

                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_cipv_debug', False):
                    traj_status = "CAV-TRAJ" if predicted_traj else "HDV-CV"
                    print(f"[CIPV-Front] V{v.id} -> V{other_v.id}: "
                          f"Deltas={obs['long_dist']:.1f}m, Deltad={obs['lat_dist']:.2f}m ({traj_status})")

            # Select CIPV-Rear (closest rear vehicle only)
            # Apollo: Monitor immediate follower for safety
            for obs in rear_obstacles[:1]:  # Only take the closest one
                other_v = obs['vehicle']
                obs_state = VehicleState(
                    id=other_v.id,
                    s=obs['s'],
                    v=other_v.v,
                    a=getattr(other_v, 'ax', 0.0),
                    lane=other_v.lane
                )

                # ================================================================
                # v12.0: CAV Trajectory Sharing
                # ================================================================
                # Get predicted trajectory if available (CAV) or None (HDV)
                predicted_traj = None
                if not getattr(other_v, 'is_hdv', False):  # Only share if CAV
                    predicted_traj = getattr(other_v, 'predicted_trajectory', None)
                    # Validate trajectory data
                    if predicted_traj and len(predicted_traj) < 5:
                        predicted_traj = None  # Too short, ignore
                # ================================================================

                # v12.1: Phase 2 - Add lane change information
                is_changing_lane = getattr(other_v, 'lc_started', False)
                lateral_velocity = abs(getattr(other_v, 'vy', 0.0)) if is_changing_lane else 0.0

                # v12.2: Phase 3 - Lane relevance classification
                target_lane = getattr(v, 'target_lane', None)
                if other_v.lane == v.lane:
                    lane_relevance = "current"
                elif target_lane and other_v.lane == target_lane:
                    lane_relevance = "target"
                else:
                    lane_relevance = "other"

                obstacles.append(ObstacleInfo(
                    obs_state,
                    is_front=False,
                    predicted_trajectory=predicted_traj,
                    is_changing_lane=is_changing_lane,
                    lateral_velocity=lateral_velocity,
                    lane_relevance=lane_relevance,
                    is_overtake_target=(other_v.id in ignore_obstacle_ids)  # v19.0: Mark overtake targets
                ))

                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_cipv_debug', False):
                    traj_status = "CAV-TRAJ" if predicted_traj else "HDV-CV"
                    print(f"[CIPV-Rear] V{v.id} ← V{other_v.id}: "
                          f"Deltas={obs['long_dist']:.1f}m, Deltad={obs['lat_dist']:.2f}m ({traj_status})")

            # ====================================================================
            # v11.12: Inject Yield Virtual Obstacle (ST-Graph Constraint)
            # ====================================================================
            # If Yield mode was activated earlier, add the virtual obstacle to
            # enforce ST-Graph s_upper constraint
            # Reference: Apollo SpeedBoundsDecider - ST boundary enforcement
            # ====================================================================
            yield_virtual_obs = getattr(v, '_yield_virtual_obstacle', None)
            if yield_virtual_obs is not None:
                obstacles.append(yield_virtual_obs)
                # Clean up temporary attribute
                setattr(v, '_yield_virtual_obstacle', None)

                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_overtake_debug', False):
                    print(f"[ST-INJECT] V{v.id} added Yield virtual obstacle to ST-Graph")
            # ====================================================================

            # ====================================================================
            # INTELLIGENT SPEED HARMONIZATION (Continuous, v12.5)
            # ====================================================================
            # v12.5 Enhancement: 階段状のロジックを廃止し、連続的な減速カーブを採用
            # 人間のように「徐々に速度を合わせる」挙動を実現
            # Reference: Apollo SpeedDecider + Human-like predictive control
            # ====================================================================
            # v_ref_adjusted already initialized above

            if closest_front_vehicle is not None and closest_front_dist < 60.0:
                # Continuous Speed Harmonization (No more step functions)
                front_speed = closest_front_vehicle.v

                # Desired gap based on 2-second following rule + 5m safety margin
                desired_gap = front_speed * 2.0 + 5.0
                current_gap = closest_front_dist

                # Gap error (positive = too far, negative = too close)
                gap_error = current_gap - desired_gap

                # Proportional speed adjustment based on gap error
                # k = 0.5: Gentle approach (human-like smoothness)
                speed_adjustment = np.clip(gap_error * 0.5, -5.0, 5.0)

                # Target speed = front speed + adjustment
                # When too close (gap_error < 0): slow down below front speed
                # When too far (gap_error > 0): speed up above front speed
                target_v = front_speed + speed_adjustment

                # Apply limit (never exceed flow speed)
                # v16.4 FIX: Minimum speed 5.0m/s to prevent AEB situations
                # If target_v < 5.0, QP should handle the rest, not v_ref
                v_ref_min = max(5.0, front_speed * 0.8)  # At least 80% of front speed or 5m/s
                v_ref_adjusted = np.clip(target_v, v_ref_min, v_ref_adjusted)

                if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_speed_harmonization_debug', False):
                    print(f"[SPEED-HARM v12.5] V{v.id} -> V{closest_front_vehicle.id}: "
                          f"gap={current_gap:.1f}m (desired={desired_gap:.1f}m), "
                          f"error={gap_error:+.1f}m -> v_ref={v_ref_adjusted:.1f}m/s "
                          f"(front={front_speed:.1f}, adj={speed_adjustment:+.1f})")
            # ====================================================================

            # ====================================================================
            # YIELD LOGIC (Intelligent Speed Adjustment for Rear Threats)
            # ====================================================================
            # If a fast rear vehicle is approaching in target lane, reduce v_ref
            # to let them pass instead of forcing aggressive acceleration
            # Reference: Apollo DecisionMaker Yield behavior
            # ====================================================================
            if rear_threat is not None:
                # Calculate Time-To-Collision with rear vehicle
                # v13.1: Bumper-to-bumper distance (ego rear - threat front)
                dist = v.x - rear_threat.x - self.params.L_vehicle
                rel_v = rear_threat.v - v.v
                ttc = dist / (rel_v + 1e-6) if rel_v > 0 else float('inf')

                # If rear vehicle will catch up within 4 seconds, yield
                if ttc < 4.0:
                    # Yield Mode: Reduce target speed to let faster vehicle pass
                    v_ref_yield = rear_threat.v * 0.8  # Slow down to 80% of rear vehicle speed
                    v_ref_adjusted = min(v_ref_adjusted, v_ref_yield)  # Take minimum of both adjustments

                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_yield_debug', False):
                        print(f"[YIELD] V{v.id} yielding to V{rear_threat.id}: "
                              f"v_ref {target_flow_speed:.1f} -> {v_ref_adjusted:.1f} m/s (TTC={ttc:.1f}s)")
            # ====================================================================

            # ====================================================================
            # PERCEPTION DEBUG LOGGING (Visibility Check)
            # ====================================================================
            # Show what obstacles the controller is detecting to diagnose "見落とし"
            # Enable with: params.enable_perception_debug = True
            # ====================================================================
            if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_perception_debug', False):
                print(f"\n{'─'*70}")
                print(f"[PERCEPTION] t={t:.2f}s | V{v.id} | Lane={v.lane} | d={ego_d:+.2f}m")
                print(f"{'─'*70}")
                print("  Ego State:")
                print(f"    Position: s={ego_s:.1f}m, d={ego_d:+.2f}m")
                print(f"    Velocity: v={v.v:.1f}m/s")
                print(f"    LC Status: scheduled={getattr(v, 'lc_scheduled', False)}, "
                      f"changing={getattr(v, 'changing_lane', False)}")
                if is_lane_changing and target_lane:
                    print(f"    Target: {v.lane} -> {target_lane} (Shadow Mode Active)")

                print(f"\n  Scanning d-coordinates: {[f'{d:+.2f}' for d in scan_d_values]}")
                print(f"  Lateral threshold: +/-{LATERAL_THRESHOLD:.2f}m")
                print(f"  Target flow speed: {target_flow_speed:.1f}m/s")

                print(f"\n  Detected Obstacles ({len(obstacles)} total):")
                if obstacles:
                    for idx, obs in enumerate(obstacles, 1):
                        obs_state = obs.vehicle_state
                        obs_type = "🔴 FRONT (YIELD)" if obs.is_front else "🔵 REAR (OVERTAKE)"
                        # Find the actual obstacle in candidate list to get d-coordinate
                        matching_cand = next((c for c in candidate_obstacles
                                            if c['vehicle'].id == obs_state.id), None)
                        obs_d = matching_cand['d'] if matching_cand else 0.0
                        obs_dist = matching_cand['long_dist'] if matching_cand else 0.0
                        obs_lat_dist = matching_cand['lat_dist'] if matching_cand else 0.0

                        print(f"    [{idx}] V{obs_state.id} ({obs_type})")
                        print(f"        Lane: {obs_state.lane}")
                        print(f"        Position: s={obs_state.s:.1f}m, d={obs_d:+.2f}m")
                        print(f"        Distance: Deltas={obs_dist:.1f}m, Deltad={obs_lat_dist:.2f}m")
                        print(f"        Velocity: v={obs_state.v:.1f}m/s (Deltav={obs_state.v - v.v:+.1f}m/s)")
                else:
                    print("    None - Free road ahead!")

                if closest_front_vehicle:
                    print(f"\n  📌 Closest Front: V{closest_front_vehicle.id} @ {closest_front_dist:.1f}m")
                if rear_threat:
                    print(f"  ⚠️  Rear Threat: V{rear_threat.id} (TTC < 4s)")

                print(f"  🎯 Adjusted v_ref: {v_ref_adjusted:.1f}m/s")
                print(f"{'─'*70}\n")
                sys.stdout.flush()
            # ====================================================================

            # ====================================================================
            # v27.0: Unified Longitudinal Safety Control (ApolloSafetyManager)
            # ====================================================================
            # Replaces v11.8 RSS/AEB pre-check, v16.0 proactive speed limiting,
            # and v16.2 emergency recovery logic.
            # Uses unified SafetyState (SAFE/CAUTION/WARNING/CRITICAL/AEB)
            # to drive longitudinal control recommendations.
            # ====================================================================
            
            safety_manager = get_safety_manager()
            
            # 1. Determine primary front vehicle (QP candidate)
            front_eval_vehicle = None
            if closest_front_vehicle:
                front_eval_vehicle = closest_front_vehicle
            elif front_obstacles:
                front_eval_vehicle = front_obstacles[0]['vehicle']
            
            # 2. Evaluate Safety State (Primary)
            safety_eval = safety_manager.evaluate_safety_state(
                ego=v,
                front_vehicle=front_eval_vehicle,
                cav_trajectories=getattr(self.simulator, 'cav_trajectories', {}),
                v_desired=self.params.v_max
            )
            
            # 3. Secondary Check: Ignored Obstacles (Cut-ins/Yield targets)
            # If primary thinks we are SAFE/CAUTION, check if we are ignoring a critical threat
            if safety_eval.state in [SafetyState.SAFE, SafetyState.CAUTION] and aeb_front_obstacles:
                # Check top 3 ignored obstacles
                for obs in aeb_front_obstacles[:3]:
                    ignored_veh = obs['vehicle']
                    # Skip if already checked as primary
                    if front_eval_vehicle and ignored_veh.id == front_eval_vehicle.id:
                        continue
                        
                    ignored_eval = safety_manager.evaluate_safety_state(
                        ego=v,
                        front_vehicle=ignored_veh,
                        cav_trajectories=getattr(self.simulator, 'cav_trajectories', {}),
                        v_desired=self.params.v_max
                    )
                    
                    # If ignored obstacle is dangerous, override primary evaluation
                    if ignored_eval.state in [SafetyState.AEB, SafetyState.CRITICAL, SafetyState.WARNING]:
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[SAFETY-OVERRIDE] V{v.id} Ignored V{ignored_veh.id} triggers {ignored_eval.state.name}")
                        safety_eval = ignored_eval
                        break

            # 4. Apply Safety Recommendations
            rss_emergency_brake = False # Legacy flag, kept for compatibility if needed downstream
            
            if safety_eval.state == SafetyState.AEB:
                # Immediate AEB Latch
                v.aeb_active = True
                v.aeb_reason = safety_eval.reason
                v.ax = safety_eval.recommended_ax
                v.velocity_profile = [] # Clear QP profile
                v.predicted_trajectory = [] # Clear shared trajectory
                rss_emergency_brake = True
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[AEB-LATCH] V{v.id} {safety_eval.reason}")
                continue # Skip QP, execute AEB immediately

            elif safety_eval.state == SafetyState.CRITICAL:
                # Strong Braking (Non-Latched)
                v_ref_adjusted = min(v_ref_adjusted, safety_eval.recommended_v_ref)
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[SAFETY-CRITICAL] V{v.id} {safety_eval.reason} -> Limit v_ref to {v_ref_adjusted:.1f}m/s")
                    
            elif safety_eval.state == SafetyState.WARNING:
                # Active Proactive Slowdown
                v_ref_adjusted = min(v_ref_adjusted, safety_eval.recommended_v_ref)
                if ENABLE_DEBUG_OUTPUT:
                     print(f"[SAFETY-WARNING] V{v.id} {safety_eval.reason} -> Limit v_ref to {v_ref_adjusted:.1f}m/s")

            elif safety_eval.state == SafetyState.CAUTION:
                # Match Speed / Early Warning
                v_ref_adjusted = min(v_ref_adjusted, safety_eval.recommended_v_ref)
                if ENABLE_DEBUG_OUTPUT:
                     print(f"[SAFETY-CAUTION] V{v.id} {safety_eval.reason} -> Limit v_ref to {v_ref_adjusted:.1f}m/s")

            # 5. AEB Release Logic (Hysteresis)
            if getattr(v, 'aeb_active', False):
                # Release only if state returns to SAFE or CAUTION
                if safety_eval.state in [SafetyState.SAFE, SafetyState.CAUTION]:
                     v.aeb_active = False
                     # Clear the latch log flag (defensive try/except)
                     try:
                         if hasattr(v, '_aeb_latch_logged'):
                             delattr(v, '_aeb_latch_logged')
                     except AttributeError:
                         pass
                     if ENABLE_DEBUG_OUTPUT:
                         print(f"[AEB-RELEASE] V{v.id} Released (State: {safety_eval.state.name})")
                else:
                     # Maintain AEB
                     v.ax = -6.0
                     rss_emergency_brake = True
                     continue
            
            # ====================================================================
                # ====================================================================
                # v16.0: CAV Free-Flowing Cruise Control (No Front Vehicle)
                # ====================================================================
                # Philosophy: When no front vehicle is detected, CAVs should:
                # - Accelerate smoothly to optimal cruise speed
                # - Use predictive horizon to anticipate future obstacles
                # - Maintain energy-efficient speed profile
                # - Leverage V2V to detect vehicles beyond sensor range
                # ====================================================================

                # Free-flowing target: reach optimal cruise speed
                is_cav = not getattr(v, 'is_hdv', False)

                if is_cav:
                    # CAV-specific: Use V2V to look ahead beyond sensor range
                    # Check if any CAVs ahead are broadcasting their position
                    same_lane_vehicles = [u for u in self.simulator.vehicles
                                         if u.lane == v.lane and u.x > v.x and not u.exited]

                    if same_lane_vehicles:
                        # Found vehicles ahead via V2V
                        distant_front = min(same_lane_vehicles, key=lambda u: u.x)
                        distant_gap = distant_front.x - v.x - 5.0  # Full vehicle length

                        # If distant vehicle is within 200m, start gentle speed adaptation
                        if distant_gap < 200.0:
                            # Predict arrival time and adjust speed proactively
                            is_distant_cav = not getattr(distant_front, 'is_hdv', False)

                            if is_distant_cav:
                                # Use V2V trajectory prediction
                                distant_traj = getattr(distant_front, 'predicted_trajectory', None)
                                if distant_traj and len(distant_traj) > 10:
                                    # Predict speed 10s ahead
                                    future_idx = min(100, len(distant_traj) - 1)
                                    v_distant_future = distant_traj[future_idx][2]
                                else:
                                    v_distant_future = distant_front.v
                            else:
                                v_distant_future = distant_front.v

                            # Calculate time to reach distant vehicle at current speed
                            if v.v > v_distant_future:
                                time_to_reach = distant_gap / (v.v - v_distant_future + 0.1)

                                # If will reach in < 15s, start gentle deceleration now
                                if time_to_reach < 15.0:
                                    # Gentle speed reduction: target 90% of distant vehicle's future speed
                                    v_free_flow_target = max(v_distant_future * 0.9, target_flow_speed * 0.8)
                                    v_ref_adjusted = min(v_ref_adjusted, v_free_flow_target)

                                    if ENABLE_DEBUG_OUTPUT:
                                        print(f"[CAV-FreeFlow-Anticipate] V{v.id}: Distant V{distant_front.id} "
                                              f"gap={distant_gap:.1f}m, ETA={time_to_reach:.1f}s -> "
                                              f"v_ref: {target_flow_speed:.1f} -> {v_ref_adjusted:.1f}m/s (gentle pre-decel)")
                                else:
                                    # Far away: cruise at optimal speed
                                    if ENABLE_DEBUG_OUTPUT:
                                        print(f"[CAV-FreeFlow-Cruise] V{v.id}: Clear road ahead "
                                              f"(distant V{distant_front.id} at {distant_gap:.1f}m, ETA={time_to_reach:.1f}s) -> "
                                              f"v_ref={v_ref_adjusted:.1f}m/s (cruise)")
                            else:
                                # Distant vehicle is faster: cruise at optimal speed
                                if ENABLE_DEBUG_OUTPUT:
                                    print(f"[CAV-FreeFlow-Cruise] V{v.id}: Distant V{distant_front.id} "
                                          f"faster ({distant_front.v:.1f} > {v.v:.1f}m/s) -> "
                                          f"v_ref={v_ref_adjusted:.1f}m/s (cruise)")
                        else:
                            # Very far (>200m): full cruise mode
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[CAV-FreeFlow-OpenRoad] V{v.id}: Open road "
                                      f"(nearest at {distant_gap:.1f}m) -> v_ref={v_ref_adjusted:.1f}m/s (optimal cruise)")
                    else:
                        # No vehicles ahead at all: full cruise
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[CAV-FreeFlow-Clear] V{v.id}: No vehicles ahead -> "
                                  f"v_ref={v_ref_adjusted:.1f}m/s (optimal cruise)")
                else:
                    # HDV: Limited sensor range, just cruise at target speed
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[HDV-FreeFlow] V{v.id}: No front obstacle -> "
                              f"v_ref={v_ref_adjusted:.1f}m/s (cruise)")
            # ====================================================================

            # ====================================================================
            # SLOT-BASED PLANNING (Apollo DecisionMaker Gap Filling Strategy)
            # ====================================================================
            # During lane changes, identify the "slot" (gap) in target lane and
            # adjust speed to smoothly merge into it. This creates human-like behavior:
            # - Speed up to fill a gap ahead of slow traffic
            # - Slow down to slot in behind faster traffic
            # --- modified (Final Fix v11.9_Ultimate / 2025-12-18): Enhanced Apollo Logic ---
            # Reference: Apollo SpeedBoundsDecider + DecisionMaker gap filling
            # ====================================================================
            if is_lane_changing and target_lane:
                target_lane_vehicles = [u for u in self.simulator.vehicles
                                       if u.lane == target_lane and not u.exited and u.id != v.id]

                # Find lead and lag vehicles in target lane (Slot Search)
                lead_candidates = [u for u in target_lane_vehicles if u.x > v.x]
                lag_candidates = [u for u in target_lane_vehicles if u.x < v.x]

                lead_vehicle = min(lead_candidates, key=lambda u: u.x) if lead_candidates else None
                lag_vehicle = max(lag_candidates, key=lambda u: u.x) if lag_candidates else None

                # === Apollo Slot-Based Speed Planning ===
                if lead_vehicle and lag_vehicle:
                    # Case A: Sandwiched (Merging into a gap)
                    # 1. Match slot flow speed
                    slot_speed = (lead_vehicle.v + lag_vehicle.v) / 2.0

                    # 2. Longitudinal Nudge (Position Alignment)
                    # Calculate where we should be in the slot
                    slot_center_x = (lead_vehicle.x + lag_vehicle.x) / 2.0 - (self.params.L_vehicle / 2.0)
                    dist_to_slot = slot_center_x - v.x

                    # Proportional controller: P-gain = 0.5 (Apollo standard)
                    # If slot is ahead (+), accelerate; if behind (-), decelerate
                    speed_adjustment = np.clip(dist_to_slot * 0.5, -5.0, 5.0)

                    # Apply both slot speed matching AND position adjustment
                    v_ref_adjusted = min(v_ref_adjusted, slot_speed + speed_adjustment)
                    v_ref_adjusted = max(v_ref_adjusted, 0.0)  # No negative speeds

                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_slot_debug', False):
                        print(f"[SLOT-SANDWICH] V{v.id}: slot_speed={slot_speed:.1f}, dist_to_slot={dist_to_slot:+.1f}m, "
                              f"speed_adj={speed_adjustment:+.1f} -> v_ref={v_ref_adjusted:.1f}m/s "
                              f"(lead V{lead_vehicle.id}:{lead_vehicle.v:.1f}, lag V{lag_vehicle.id}:{lag_vehicle.v:.1f})")

                elif lead_vehicle:
                    # Case B: Following (Front vehicle only)
                    dist_to_lead = lead_vehicle.x - v.x - self.params.L_vehicle

                    if dist_to_lead < 20.0:
                        # Close: Match front vehicle speed exactly (safety)
                        v_ref_adjusted = min(v_ref_adjusted, lead_vehicle.v)
                    else:
                        # Far: Slight overspeed allowed to catch up (+2 m/s)
                        v_ref_adjusted = min(v_ref_adjusted, lead_vehicle.v + 2.0)

                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_slot_debug', False):
                        print(f"[SLOT-FOLLOW] V{v.id}: dist={dist_to_lead:.1f}m -> v_ref={v_ref_adjusted:.1f}m/s (V{lead_vehicle.id})")

                elif lag_vehicle:
                    # Case C: Overtaking/Pulling away (Rear vehicle only)
                    # Accelerate to stay ahead, but respect speed limit
                    v_ref_adjusted = min(v_ref_adjusted, min(self.params.v_max, lag_vehicle.v + 3.0))

                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_slot_debug', False):
                        print(f"[SLOT-OVERTAKE] V{v.id}: v_ref={v_ref_adjusted:.1f}m/s (staying ahead of V{lag_vehicle.id})")
            # ====================================================================

            # ====================================================================
            # v12.4: Smart RSS Check Suppression
            # ====================================================================
            # QPを常に試行し、成功時は極端な接近（1m以下）のみでRSS発動
            # QP失敗時は通常のRSSチェックにフォールバック
            # Rationale: QPは安全を考慮済み、過剰なRSS介入を防ぐ
            # ====================================================================

            # ====================================================================
            # Trajectory Reset Logic: Clear previous trajectory on state changes
            # ====================================================================
            # Check if we need to reset previous trajectory due to:
            # 1. Lane change start (new maneuver invalidates old trajectory)
            # 2. AEB activation (emergency situation requires fresh planning)
            should_reset_trajectory = False
            reset_reason = ""

            # Check LC start (compare previous state)
            prev_lc_state = getattr(v, '_prev_changing_lane', False)
            current_lc_state = getattr(v, 'changing_lane', False) or getattr(v, 'lc_scheduled', False)

            if current_lc_state and not prev_lc_state:
                should_reset_trajectory = True
                reset_reason = "LC_START"
                v._prev_changing_lane = True
            elif not current_lc_state:
                v._prev_changing_lane = False

            # Check AEB activation (compare previous state)
            prev_aeb_state = getattr(v, '_prev_aeb_active', False)
            current_aeb_state = getattr(v, 'aeb_active', False)

            if current_aeb_state and not prev_aeb_state:
                should_reset_trajectory = True
                reset_reason = "AEB_ACTIVATED"
                v._prev_aeb_active = True
            elif not current_aeb_state:
                v._prev_aeb_active = False

            # Perform reset if needed
            if should_reset_trajectory and v.id in self.frenet_controller.previous_trajectory:
                del self.frenet_controller.previous_trajectory[v.id]
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[TRAJECTORY-RESET] V#{v.id}: Cleared previous trajectory ({reset_reason})")
            # ====================================================================

            result = None
            
            # v16.5 FIX: Emergency TTC bypass - skip QP when immediate braking is needed
            emergency_ttc_bypass = False
            if front_obstacles and not rss_emergency_brake:
                front_vehicle = front_obstacles[0]['vehicle']
                gap = compute_gap_bumper_to_bumper(front_vehicle.x, v.x)
                v_rel = v.v - front_vehicle.v
                
                # ================================================================
                # v27.10 NEW: Proactive Braking on Front Vehicle Deceleration
                # ================================================================
                # If front vehicle is braking, start braking early rather than
                # waiting for TTC to become critical. This is smarter V2V control.
                # ================================================================
                front_ax = getattr(front_vehicle, 'ax', 0.0)
                
                # Check shared v_ref (V2V intent)
                shared_traj = self.simulator.cav_trajectories.get(front_vehicle.id, {})
                front_v_ref = shared_traj.get('v_ref', None)
                
                # Condition 1: Front is braking (Physical)
                # v27.14: Use optimized threshold
                threshold = getattr(self.params, 'proactive_brake_threshold', -1.0)
                is_braking = front_ax < threshold
                
                # Condition 2: Front intends to be slow (Intent)
                # If front wants to go 15m/s and we are doing 20m/s, start slowing now
                is_intent_slow = front_v_ref is not None and front_v_ref < (v.v - 3.0)
                
                if (is_braking or is_intent_slow) and gap > 0 and gap < 80.0:
                    # Calculate proactive response
                    ax_proactive = 0.0
                    
                    if is_braking:
                        # Match deceleration
                        ax_proactive = max(-4.0, front_ax * 0.8)
                    elif is_intent_slow:
                        # Approach target speed smoothly
                        # Desired decel to reach front_v_ref in ~3 seconds
                        ax_proactive = max(-3.0, (front_v_ref - v.v) / 3.0)
                        
                    # Only apply if ego is closing in
                    if v.v > front_vehicle.v:
                        v.ax = min(getattr(v, 'ax', 0.0), ax_proactive)
                        if ENABLE_DEBUG_OUTPUT:
                            reason = f"ax={front_ax:.2f}" if is_braking else f"v_ref={front_v_ref:.1f}"
                            print(f"[PROACTIVE-BRAKE] V{v.id}: Front V{front_vehicle.id} ({reason}), "
                                  f"applying ax={ax_proactive:.2f}m/s^2")
                # ================================================================
                
                if v_rel > 0 and gap > 0:
                    ttc = gap / v_rel
                    # TTC < 1.5s: Apply strong preventive braking
                    if ttc < 1.5:
                        # Calculate safe following speed and required deceleration
                        required_decel = v_rel / ttc  # What's needed to stop gap shrinkage
                        # Cap at -4.0 m/s^2 (strong but not AEB level)
                        ax_preventive = -min(required_decel, 4.0)
                        
                        v.ax = ax_preventive
                        v.velocity_profile = []
                        v.velocity_profile_time = t
                        emergency_ttc_bypass = True
                        
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[TTC-URGENT] V{v.id} -> V{front_vehicle.id}: "
                                  f"TTC={ttc:.2f}s < 1.5s, gap={gap:.1f}m -> "
                                  f"ax={ax_preventive:.2f}m/s^2 (QP bypassed)")
            
            if emergency_ttc_bypass:
                # Skip QP, go to next vehicle
                continue
            
            try:
                # ============================================================
                # v14.4: Pass v_ref_adjusted to QP (CRITICAL FIX)
                # ============================================================
                # ROOT CAUSE FIX: Planning layer adjusts v_ref but QP wasn't using it!
                # Now QP receives v_ref_target from Planning layer
                # ============================================================
                
                # v16.4 FIX: Final sanity check on v_ref_adjusted
                # Ensure it's never too low (which would cause AEB situations)
                v_ref_min_final = max(5.0, self.params.v_min)
                if v_ref_adjusted < v_ref_min_final:
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[v_ref-CLAMP] V{v.id}: v_ref_adjusted={v_ref_adjusted:.1f}m/s "
                              f"< min={v_ref_min_final:.1f}m/s -> clamped")
                    v_ref_adjusted = v_ref_min_final

                # v14.5: Diagnostic logging for Planning->QP communication
                if ENABLE_DEBUG_OUTPUT and v_ref_adjusted < v.v:
                    print(f"[Planning->QP] V{v.id}: t={t:.1f}s, "
                          f"v_current={v.v:.1f}m/s, v_ref_target={v_ref_adjusted:.1f}m/s "
                          f"(Deltav={v_ref_adjusted - v.v:.1f}m/s) -> Passing to QP optimizer")

                # ============================================================
                # v26.0: Compute Unified Safe Boundaries (ApolloSafetyManager)
                # ============================================================
                # Replace scattered safety checks with single unified call
                # Safety boundaries are passed to QP as hard constraints
                # ============================================================
                safety_manager = get_safety_manager()
                s_lower, s_upper = safety_manager.compute_safe_boundaries(
                    ego=v,
                    vehicles=vehicles,
                    cav_trajectories=self.simulator.cav_trajectories,
                    t=t,
                    horizon=self.frenet_controller.N  # v26.0: Sync horizon with QP (N=30 vs 80)
                )
                # ============================================================
                # v27.11 NEW: Store v_ref for V2V Sharing
                # ============================================================
                v.v_ref = v_ref_adjusted  # Store for trajectory sharing
                
                # Run QP optimization
                # v12.0: Pass current_time for trajectory interpolation
                # Trajectory stitching is handled internally by optimize() via use_stitching=True
                # v11.10: Enable HeuristicSpeedDecider to prevent AEB from unrealistic overtaking
                # v26.0: Pass safety boundaries to QP as constraints
                result = self.frenet_controller.optimize(
                    ego_state,
                    obstacles,  # Now includes ObstacleInfo with is_front flag and predicted_trajectory
                    use_dp_optimizer=True,  # Use DP optimizer for intelligent YIELD/OVERTAKE decisions
                    use_stitching=True,  # Enable trajectory stitching for smoothness
                    urgency=urgency,
                    current_time=t,  # Required for interpolating shared trajectories
                    v_ref_target=v_ref_adjusted,  # v14.4: Pass Planning layer target velocity
                    s_lower_bound=s_lower,  # v26.0: Safe lower position bound
                    s_upper_bound=s_upper   # v26.0: Safe upper position bound
                )
                # ============================================================

            except Exception as e:
                # Unexpected optimizer error: log and fallback
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[Level2] QP exception for V{v.id}: {e}")
                result = None
                self.qp_failure_count += 1
                self.fallback_idm_count += 1

            # Apply result or fallback
            if result is not None:
                # ============================================================
                # v13.4: AEB Priority Guard (Critical Bug Fix)
                # ============================================================
                # Problem: rss_emergency_brake=True set v.ax=-6.0, but then
                #          QP result was applied and OVERWROTE the AEB command!
                # Solution: When AEB is already active, NEVER apply QP result.
                # Reference: Collision log V9-V4 showing AEB warnings but no braking
                # ============================================================
                if rss_emergency_brake:
                    # AEB was triggered earlier - DO NOT override with QP result
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[AEB-GUARD] V{v.id}: QP result ignored, AEB maintains v.ax={v.ax:.1f}m/s^2")
                    # Skip QP application, go to next vehicle
                    self.qp_success_count += 1  # QP succeeded but ignored for safety
                    continue
                # ============================================================
                
                # ============================================================
                # v12.4: QP成功時の極端接近チェック（RSS緩和）
                # ============================================================
                # QPは安全を考慮済みなので、極端な接近（1m以下）以外はRSSを抑制
                # これにより「予測していれば踏まない急ブレーキ」を防ぐ
                qp_override_rss = False
                if front_obstacles:
                    # QP succeeded but check for extreme proximity
                    front_vehicle = front_obstacles[0]['vehicle']
                    # v16.1: Use centralized gap calculation
                    gap = compute_gap_bumper_to_bumper(front_vehicle.x, v.x)

                    if gap < SAFETY.GAP_CRITICAL:
                        # v16.1: Use centralized AEB evaluation
                        is_front_cav = not getattr(front_vehicle, 'is_hdv', False)
                        has_traj = getattr(front_vehicle, 'predicted_trajectory', None) is not None
                        
                        aeb_cmd = evaluate_aeb(
                            v_ego=v.v,
                            v_front=front_vehicle.v,
                            gap=gap,
                            is_cav_front=is_front_cav,
                            has_valid_trajectory=has_traj
                        )
                        
                        v.ax = aeb_cmd.ax
                        v.velocity_profile = []
                        v.velocity_profile_time = t
                        v.predicted_trajectory = []  # CRITICAL: Clear shared trajectory on AEB
                        v.aeb_active = True
                        v.aeb_reason = aeb_cmd.reason  # v19.0: Log reason
                        qp_override_rss = True

                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[QP-RSS-OVERRIDE] V{v.id}: Extreme proximity gap={gap:.2f}m < {SAFETY.GAP_CRITICAL}m -> AEB")
                            print(format_aeb_log(v.id, aeb_cmd, front_vehicle.id))
                # ============================================================

                if not qp_override_rss:
                    # QP succeeded: apply trajectory
                    self._apply_qp_result(v, result, t)
                    
                    # ============================================================
                    # v12.3: Store trajectory for next control cycle (Stitching)
                    # ============================================================
                    # Save result for the next optimization cycle
                    # This enables smooth acceleration continuation
                    self.prev_traj[v.id] = result
                    # ============================================================

                    # ============================================================
                    # v11.11: PREDICTIVE EARLY ABORT (Intent Cancellation)
                    # ============================================================
                    # Modified: Only abort on PANIC BRAKING (< -5.0 m/s^2)
                    # - Overtake mode may require brief moderate braking (-2~-3 m/s^2)
                    # - Only emergency situations (< -5.0 m/s^2) should cancel LC
                    # Reference: UN R157 emergency braking threshold
                    # ============================================================
                    if v.lc_scheduled and not v.lc_started:
                        # Check predicted acceleration profile
                        min_accel = np.min(result['a']) if 'a' in result else 0.0

                        # Abort threshold: -5.0 m/s^2 (panic braking only)
                        # Allow moderate braking for speed adjustment in Overtake/Yield
                        if min_accel < -5.0:
                            # Cancel booking immediately (emergency situation)
                            v.lc_scheduled = False
                            v.lc_intent = None  # ★ Withdraw intent to release others
                            v.scheduled_time = t + 2.0  # Cooldown before retry
                            v.target_lane = None  # Clear target

                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[EARLY-ABORT] V{v.id} cancelled LC booking: "
                                      f"predicted min_accel={min_accel:.2f} m/s^2 < -5.0 "
                                      f"(panic braking - emergency situation)")

                            # Reset urgency to allow reconsideration
                            # (Urgency will rebuild naturally if still needed)
                    # ============================================================

                    # ============================================================
                    # v12.0: Save Trajectory for Cooperative Sharing
                    # ============================================================
                    # Store computed trajectory with absolute timestamps for other
                    # CAVs to use in the next control cycle (1-step-old paradigm)
                    traj = []
                    N = self.frenet_controller.N
                    dt = self.frenet_controller.dt
                    for k in range(N):
                        t_point = t + k * dt
                        s_point = result['s'][k] if 's' in result and k < len(result['s']) else ego_s
                        v_point = result['v'][k] if 'v' in result and k < len(result['v']) else v.v
                        a_point = result['a'][k] if 'a' in result and k < len(result['a']) else 0.0
                        traj.append((t_point, s_point, v_point, a_point))

                    v.predicted_trajectory = traj

                    if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_trajectory_sharing_debug', False):
                        print(f"[TRAJ-SAVE] V{v.id} saved trajectory: {len(traj)} points, "
                              f"t=[{traj[0][0]:.2f}, {traj[-1][0]:.2f}]s, "
                              f"s=[{traj[0][1]:.1f}, {traj[-1][1]:.1f}]m")
                    # ============================================================

                    self.qp_success_count += 1
            else:
                # QP failed or RSS override: use fallback
                # v12.4以降: Comfortable Deceleration fallbackを使用（次タスク）
                self._apply_idm_fallback(v, obstacles, t)
                v.predicted_trajectory = []  # Clear trajectory on failure
                self.qp_failure_count += 1
                self.fallback_idm_count += 1

            if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_control_debug', False):
                print(f"[Level2] t={t:.2f}s V{v.id} v={v.v:.2f}m/s a={v.ax:.2f}m/s^2 "
                      f"urgency={urgency:.3f} result={'QP' if result else 'IDM'}")

    def _apply_qp_result(self, vehicle: Vehicle, result: Dict[str, np.ndarray], t: float):
        """
        Apply QP solution to vehicle

        Args:
            vehicle: Vehicle to update
            result: QP solution dict with 's', 'v', 'a' trajectories
            t: Current time
        """
        # Store velocity profile (for trajectory visualization)
        vehicle.velocity_profile = list(result['v']) if 'v' in result else []
        vehicle.velocity_profile_time = t

        # Apply first-step acceleration (instantaneous command)
        a_raw = float(result['a'][0]) if ('a' in result and len(result['a']) > 0) else 0.0
        vehicle.ax = a_raw

        # ================================================================
        # v13.4: Clip acceleration to safety bounds (Expanded Range)
        # ================================================================
        # Problem: Previous clipping to -4.0 m/s^2 prevented emergency braking
        #          when QP determines strong braking is needed
        # Solution: Allow up to -6.0 m/s^2 (UN R157 emergency braking)
        #          Comfort braking (-4.0) is handled by QP cost functions
        # Reference: UN R157 AEB capability, ISO 22179 AEBS
        # ================================================================
        a_clipped = np.clip(vehicle.ax, -6.0, 2.5)

        # ================================================================
        # v14.5: Diagnostic Logging for QP Application
        # ================================================================
        if ENABLE_DEBUG_OUTPUT and a_raw != a_clipped:
            print(f"[QP-CLIP] V{vehicle.id}: a_raw={a_raw:.2f} -> a_clipped={a_clipped:.2f}m/s^2")

        vehicle.ax = a_clipped

        # ================================================================
        # v27.6 FIX: Enforce Safety Floor After QP Output
        # ================================================================
        # If front vehicle is close with positive closing velocity, ensure
        # minimum braking is applied even if QP thinks it's okay.
        # This prevents QP from being too lenient in critical situations.
        # ================================================================
        if hasattr(vehicle, 'front_vehicle') and vehicle.front_vehicle is not None:
            front = vehicle.front_vehicle
            gap = front.x - vehicle.x - 5.0  # L_VEHICLE = 5.0
            rel_v = vehicle.v - front.v
            if rel_v > 0.1 and gap > 0:
                ttc = gap / rel_v
                # Apply graduated safety floor (v27.8: Softened for smarter control)
                if ttc < 0.8:  # Would trigger AEB
                    vehicle.ax = min(vehicle.ax, -5.0)  # v27.8: Was -6.0
                elif ttc < 1.5:  # CRITICAL
                    vehicle.ax = min(vehicle.ax, -4.5)  # v27.8: Was -4.0
                elif ttc < 2.5:  # WARNING
                    vehicle.ax = min(vehicle.ax, -2.5)  # v28.0: Was -3.5 (more comfortable)
                elif ttc < 4.0:  # CAUTION
                    vehicle.ax = min(vehicle.ax, max(-2.0, front.v - vehicle.v))  # v28.0: Was -2.5
        # ================================================================

        # Diagnostic output for significant deceleration or acceleration
        if ENABLE_DEBUG_OUTPUT and abs(vehicle.ax) > 2.0:
            v_target = result['v'][0] if 'v' in result and len(result['v']) > 0 else vehicle.v
            print(f"[QP-APPLY] V{vehicle.id}: t={t:.1f}s, "
                  f"v_current={vehicle.v:.1f}m/s -> v_target={v_target:.1f}m/s, "
                  f"a_commanded={vehicle.ax:.2f}m/s^2")
        # ================================================================

        # Lateral control: not handled by Frenet QP (longitudinal only)
        # Placeholder for future lateral planner
        vehicle.steering = 0.0

    def _apply_idm_fallback(self, vehicle: Vehicle, obstacles: List[ObstacleInfo], t: float):
        """
        Apply IDM fallback when QP fails

        Reference:
        - Treiber et al. (2000): Intelligent Driver Model
        - Apollo: PlanningComponent uses backup trajectory on failure

        Implementation:
        - Safe deceleration towards front vehicle
        - Free acceleration if no front vehicle

        Args:
            vehicle: Vehicle to update
            obstacles: List of obstacles (ObstacleInfo)
            t: Current time
        """
        # ================================================================
        # v13.7: Emergency Braking in Fallback Mode (Root Cause 3 Fix)
        # ================================================================
        # Problem (Root Cause 3): When QP fails during high-speed approach,
        #          fallback to IDM with only comfortable deceleration b=2.0m/s^2.
        #          Insufficient braking to prevent collision (~10m stopping distance
        #          from 20m/s vs 33m needed to avoid 0.92m gap).
        #
        # Solution: Use conservative IDM parameters + allow up to -6.0m/s^2 
        #          emergency braking when approaching collision (gap < 10m).
        #          Apollo SafetyManager: Fallback must support emergency capability.
        #
        # Reference: Apollo speed_bounds_decider.cc - ApproachingObstacle()
        #           When gap critically small, command maximum deceleration
        #           UN R157: Emergency braking -6.0m/s^2 (ALKS minimum)
        # ================================================================
        
        # IDM parameters (conservative for safety)
        v0 = 25.0  # desired velocity [m/s]
        T = 1.5    # safe time headway [s]
        s0 = 2.0   # minimum gap [m]
        a_max = 2.0  # max acceleration [m/s^2]
        b_comfortable = 2.0    # comfortable deceleration [m/s^2]
        b_emergency = 6.0      # emergency deceleration [m/s^2] (NEW)

        # Find closest front obstacle (YIELD type)
        front_obstacles = [obs for obs in obstacles if obs.is_front]

        if not front_obstacles:
            # Free road: accelerate to desired velocity
            a_idm = a_max * (1 - (vehicle.v / v0) ** 4)
            a_final = np.clip(a_idm, -2.0, 2.0)  # Conservative for free road
        else:
            # Car following: IDM formula with emergency detection
            front_obs = min(front_obstacles, key=lambda obs: obs.vehicle_state.s - vehicle.x)
            front = front_obs.vehicle_state
            gap = max(0.1, front.s - vehicle.x)
            delta_v = vehicle.v - front.v

            # Detect emergency condition (approaching collision)
            # TTC (Time-to-Collision) = gap / delta_v when delta_v > 0
            # Emergency threshold: TTC < 1.5s (Apollo: 2.0s, but we use 1.5s for safety)
            is_emergency = False
            if delta_v > 0.1:
                ttc = gap / delta_v
                is_emergency = ttc < 1.5  # Critical approach
            
            if is_emergency or gap < 5.0:
                # Emergency braking: ignore IDM, use max deceleration
                a_idm = -b_emergency  # Maximum emergency braking
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[IDM-Fallback-EMERGENCY] V{vehicle.id} Emergency brake: "
                          f"gap={gap:.2f}m, delta_v={delta_v:.2f}m/s, TTC={gap/max(delta_v,0.1):.2f}s -> "
                          f"a={a_idm:.1f}m/s^2 (Apollo SafetyManager)")
            else:
                # Normal car following: standard IDM with comfortable deceleration
                s_star = s0 + vehicle.v * T + vehicle.v * delta_v / (2 * np.sqrt(a_max * b_comfortable))
                a_idm = a_max * (1 - (vehicle.v / v0) ** 4 - (s_star / gap) ** 2)
            
            # Clip to emergency range
            a_final = np.clip(a_idm, -6.0, 2.0)

        vehicle.ax = a_final
        vehicle.steering = 0.0

        # Store empty profile (IDM doesn't produce trajectory)
        vehicle.velocity_profile = []
        vehicle.velocity_profile_time = t

        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_fallback_debug', False):
            front_info = f"front_s={front_obstacles[0].vehicle_state.s:.1f}m" if front_obstacles else "free_road"
            print(f"[IDM Fallback] V{vehicle.id} a={vehicle.ax:.2f}m/s^2 {front_info}")
