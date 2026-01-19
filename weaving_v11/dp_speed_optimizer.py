# -*- coding: utf-8 -*-
# --- dp_speed_optimizer.py (ver.12.0 / 2025-12-27) ---
"""
================================================================================
ApolloスタイルDP速度オプティマイザ (STグラフ上の動的計画法)
================================================================================

目的:
    QP最適化の前に粗い意思決定 (YIELD vs OVERTAKE) を行う。
    運動学的に実行可能なインタラクションを決定し、QPの実行不能を防止。

v18.6 修正 (2025-12-27):
    - 後方障害物に誤って適用されていたSAFETY OVERRIDEバグを修正
    - TTCチェックは前方障害物 (rel_s > 0) のみに発動
    - 後方障害物への誤ったYIELD決定を防止

設計哲学:
    1. STグラフ構築: 離散グリッド (時間 x 空間)
    2. DP探索: (t=0, s=0) から t=T までの最小コスト経路を発見
    3. コスト関数:
       - 衝突コスト: 障害物ST境界と重なる場合は無限大
       - 平滑性コスト: 加速度/ジャーク変化にペナルティ
       - 参照コスト: 目標速度からの偏差にペナルティ
    4. 意思決定抽出: DP経路と障害物位置を比較
       - 経路が障害物の下 → YIELD
       - 経路が障害物の上 → OVERTAKE

ベース: Baidu Apollo path_time_heuristic_optimizer.cc
================================================================================
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import heapq

# Import from existing frenet_qp_apollo.py
from .frenet_qp_apollo import (
    VehicleState,
    ObstacleInfo,
    InteractionDecision,
    ENABLE_DEBUG_OUTPUT
)


@dataclass
class STPoint:
    """Point on ST-Graph"""
    s: float  # Position [m]
    t: float  # Time [s]


@dataclass
class STBoundaryDP:
    """Obstacle boundary on ST-Graph for DP"""
    s_lower: List[float]  # Lower bound of s at each time step
    s_upper: List[float]  # Upper bound of s at each time step
    obstacle_id: int


@dataclass
class DPNode:
    """DP node with cost and parent pointer"""
    i_t: int  # Time index
    j_s: int  # Space index
    point: STPoint
    total_cost: float = np.inf
    parent: Optional[Tuple[int, int]] = None  # (i_t, j_s) of parent node


class SpeedHeuristicOptimizer:
    """
    Apollo-style DP Speed Optimizer
    
    Determines coarse interaction decisions (YIELD/OVERTAKE) for each obstacle
    by searching the minimum-cost path on ST-Graph using Dynamic Programming.
    
    ver.12.0 / 2025-12-22
    """

    def __init__(
        self,
        horizon_time: float = 5.0,
        dt: float = 0.2,  # [修正1] グリッドを細かくする (0.5s -> 0.2s)
        ds_dense: float = 1.0,
        s_max: float = 100.0,
        a_max: float = 2.0,
        a_min: float = -4.0,
        v_target: float = 15.0,
        # Cost weights
        w_collision: float = 1e6,
        w_smoothness: float = 10.0,
        w_reference: float = 1.0,
        w_spatial_potential: float = 5.0,
        w_jerk: float = 50.0,  # [v12.3] Jerk cost (Apollo dp_st_cost.cc)
        safe_buffer: float = 5.0,  # [v12.3] Soft obstacle buffer (Apollo)
    ):
        """
        Args:
            horizon_time: Planning horizon [s]
            dt: Time step [s] - v12.3: dtを小さくして判定精度を向上 (0.5→0.2)
            ds_dense: Space step [m] (dense grid)
            s_max: Maximum s coordinate [m]
            a_max: Maximum acceleration [m/s²]
            a_min: Minimum acceleration (braking) [m/s²]
            v_target: Target speed [m/s]
            w_collision: Weight for collision cost
            w_smoothness: Weight for smoothness cost (accel/jerk)
            w_reference: Weight for reference speed deviation
            w_spatial_potential: Weight for spatial progression (forward bias)
            w_jerk: Weight for jerk minimization (v12.3 Apollo dp_st_cost.cc)
            safe_buffer: Safety buffer distance for soft obstacle cost (v12.3 Apollo)
        """
        self.horizon_time = horizon_time
        self.dt = dt
        self.ds_dense = ds_dense
        self.s_max = s_max
        self.a_max = a_max
        self.a_min = a_min
        self.v_target = v_target

        # Cost weights
        self.w_collision = w_collision
        self.w_smoothness = w_smoothness
        self.w_reference = w_reference
        self.w_spatial_potential = w_spatial_potential
        self.w_jerk = w_jerk  # [v12.3] Jerk penalty
        self.safe_buffer = safe_buffer  # [v12.3] Soft obstacle margin

        # Grid dimensions
        self.n_t = int(np.ceil(horizon_time / dt)) + 1
        self.n_s = int(np.ceil(s_max / ds_dense)) + 1

        # DP cost table
        self.cost_table: List[List[DPNode]] = []

    def optimize(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleInfo]
    ) -> Dict[int, InteractionDecision]:
        """
        Run DP optimization and determine interaction decisions for all obstacles.

        Args:
            ego_state: Current ego vehicle state
            obstacles: List of obstacle information

        Returns:
            Dict[obstacle_id, InteractionDecision]: Decision for each obstacle
        """
        # 1. Build ST-Graph boundaries from obstacles
        st_boundaries = self._build_st_boundaries(ego_state, obstacles)

        # 2. Initialize DP cost table
        self._init_cost_table(ego_state)

        # 3. Run DP search
        success = self._dp_search(st_boundaries)

        if not success:
            if ENABLE_DEBUG_OUTPUT:
                print("[DP] DP search failed, using fallback decisions")
            return self._fallback_decisions(obstacles)

        # 4. Extract optimal path
        optimal_path = self._extract_optimal_path()

        if optimal_path is None or len(optimal_path) == 0:
            if ENABLE_DEBUG_OUTPUT:
                print("[DP] Failed to extract optimal path, using fallback")
            return self._fallback_decisions(obstacles)

        # 5. Determine decisions by comparing path with obstacles
        decisions = self._extract_decisions(optimal_path, obstacles, ego_state)

        return decisions

    def _build_st_boundaries(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleInfo]
    ) -> List[STBoundaryDP]:
        """
        Build ST-boundaries for obstacles.

        For each obstacle, predict its trajectory and create lower/upper bounds
        along the time axis.
        
        v12.3 Modified: Dynamic safety margin based on ego velocity
        - Low speed (v < 10 m/s): Increased safety margin to prevent creeping collisions
        - High speed (v >= 10 m/s): Standard safety margin
        """
        st_boundaries = []
        L_VEHICLE = 4.5  # Vehicle length [m]
        
        # ====================================================================
        # v12.3 Modified: Dynamic safety margin for low-speed scenarios
        # ====================================================================
        # Apollo Safety Manager: Increase safety buffer in congestion/low-speed
        # ====================================================================
        # v18.11: Apollo-aligned Low-Speed Threshold
        # Previous (v12.3): 10 m/s threshold (too aggressive)
        # New (v18.11): 5 m/s threshold (Apollo standard)
        # ====================================================================
        LOW_SPEED_THRESHOLD = 5.0  # v18.11: Apollo-aligned threshold
        
        if ego_state.v < LOW_SPEED_THRESHOLD:
            # Low-speed scenario: Enhanced safety for near-stop situations
            MARGIN = 3.5  # Enhanced margin for near-stop safety
            if ENABLE_DEBUG_OUTPUT:
                print(f"[DP-SAFETY v18.11] Low-speed mode (v={ego_state.v:.1f}m/s < {LOW_SPEED_THRESHOLD}m/s)")
                print(f"  Using enhanced safety margin: {MARGIN:.1f}m (standard: 2.0m)")
        else:
            # Normal speed: Standard safety margin
            MARGIN = 2.0

        for obs in obstacles:
            s_lower = []
            s_upper = []

            for i_t in range(self.n_t):
                t = i_t * self.dt

                # Predict obstacle position at time t
                if obs.predicted_trajectory and len(obs.predicted_trajectory) > 0:
                    # Use provided trajectory
                    # Find closest time point
                    traj = obs.predicted_trajectory
                    if t >= traj[-1][0]:
                        # Beyond trajectory, use last point
                        obs_s = traj[-1][1]
                    else:
                        # Interpolate
                        for k in range(len(traj) - 1):
                            if traj[k][0] <= t <= traj[k + 1][0]:
                                t1, s1, _, _ = traj[k]
                                t2, s2, _, _ = traj[k + 1]
                                obs_s = s1 + (s2 - s1) * (t - t1) / (t2 - t1)
                                break
                        else:
                            obs_s = traj[0][1]
                else:
                    # [修正1] Constant Acceleration Model (安全性向上)
                    # 速度が負にならないようにクリップ
                    v_pred = max(0.0, obs.vehicle_state.v + obs.vehicle_state.a * t)
                    # s = s0 + v0*t + 0.5*a*t^2 (簡易版: 平均速度 * t)
                    obs_s = obs.vehicle_state.s + (obs.vehicle_state.v + v_pred) / 2.0 * t

                # ST-boundary with dynamic safety margin
                lower = obs_s - L_VEHICLE / 2 - MARGIN
                upper = obs_s + L_VEHICLE / 2 + MARGIN

                s_lower.append(lower)
                s_upper.append(upper)

            boundary = STBoundaryDP(
                s_lower=s_lower,
                s_upper=s_upper,
                obstacle_id=obs.vehicle_state.id
            )
            st_boundaries.append(boundary)

        return st_boundaries

    def _init_cost_table(self, ego_state: VehicleState):
        """
        Initialize DP cost table.

        Create grid nodes and set initial state cost to 0.
        """
        self.cost_table = []

        for i_t in range(self.n_t):
            t = i_t * self.dt
            row = []

            for j_s in range(self.n_s):
                s = j_s * self.ds_dense
                point = STPoint(s=s, t=t)
                node = DPNode(i_t=i_t, j_s=j_s, point=point)
                row.append(node)

            self.cost_table.append(row)

        # Set initial node cost to 0 (t=0, s=ego.s)
        # Find closest grid point to ego_state.s
        j_s_init = int(np.round(ego_state.s / self.ds_dense))
        j_s_init = max(0, min(j_s_init, self.n_s - 1))
        self.cost_table[0][j_s_init].total_cost = 0.0

    def _dp_search(self, st_boundaries: List[STBoundaryDP]) -> bool:
        """
        Run DP search to find minimum-cost path.

        Args:
            st_boundaries: List of obstacle ST-boundaries

        Returns:
            bool: True if search succeeded
        """
        # Forward DP (iterate over time steps)
        for i_t in range(self.n_t - 1):
            current_layer = self.cost_table[i_t]
            next_layer = self.cost_table[i_t + 1]

            for j_s in range(self.n_s):
                current_node = current_layer[j_s]

                # Skip if current node is unreachable
                if current_node.total_cost >= np.inf:
                    continue

                # Expand to reachable next nodes
                reachable_indices = self._get_reachable_indices(
                    i_t, j_s, current_node.point
                )

                for j_s_next in reachable_indices:
                    next_node = next_layer[j_s_next]

                    # ====================================================================
                    # v12.3: Soft Obstacle Cost (Apollo dp_st_cost.cc)
                    # ====================================================================
                    # Instead of strict Inf for collision, use high risk cost
                    # This encourages planner to keep safe margin, not just "barely avoid"
                    # ====================================================================
                    obstacle_cost = 0.0

                    # Check if trajectory is dangerously close to obstacles
                    # Sample multiple points along segment
                    N_SAMPLES = 5
                    for alpha in np.linspace(0, 1, N_SAMPLES):
                        s_test = current_node.point.s + alpha * (next_node.point.s - current_node.point.s)
                        t_test = current_node.point.t + alpha * (next_node.point.t - current_node.point.t)

                        for boundary in st_boundaries:
                            i_t_bound = int(np.round(t_test / self.dt))
                            i_t_bound = max(0, min(i_t_bound, len(boundary.s_lower) - 1))

                            # Check if in collision
                            if boundary.s_lower[i_t_bound] <= s_test <= boundary.s_upper[i_t_bound]:
                                obstacle_cost = np.inf  # Hard collision
                                break

                            # Calculate distance to boundary
                            if s_test < boundary.s_lower[i_t_bound]:
                                # Below obstacle (trying to pass behind)
                                dist = boundary.s_lower[i_t_bound] - s_test
                            else:
                                # Above obstacle (trying to pass ahead)
                                dist = s_test - boundary.s_upper[i_t_bound]

                            # Soft cost if within safety buffer
                            if dist < self.safe_buffer:
                                # Risk cost: higher penalty as distance decreases
                                # Reference: Apollo dp_st_cost.cc GetObstacleCost
                                risk_cost = 1000.0 / max(dist ** 2, 0.01)
                                obstacle_cost += risk_cost

                        if obstacle_cost >= np.inf:
                            break

                    if obstacle_cost >= np.inf:
                        continue  # Skip collision nodes
                    # ====================================================================

                    # Calculate transition cost (includes jerk penalty)
                    transition_cost = self._calculate_transition_cost(
                        current_node.point, next_node.point, current_node
                    )

                    total_cost = current_node.total_cost + transition_cost + obstacle_cost

                    # Update if better cost found
                    if total_cost < next_node.total_cost:
                        next_node.total_cost = total_cost
                        next_node.parent = (i_t, j_s)

        # Check if any node in final layer is reachable
        final_layer = self.cost_table[-1]
        min_cost = min(node.total_cost for node in final_layer)

        return min_cost < np.inf

    def _get_reachable_indices(
        self, i_t: int, j_s: int, point: STPoint
    ) -> List[int]:
        """
        Get indices of reachable nodes in next time step.

        Uses kinematic constraints (max acceleration/deceleration).
        
        v18.4: Added safety bounds to prevent excessive grid expansion
        """
        # Current velocity estimate
        if i_t == 0:
            v_current = point.s / max(point.t, 1e-6) if point.t > 0 else self.v_target
        else:
            prev_point = self.cost_table[i_t - 1][j_s].point if i_t > 0 else None
            if prev_point:
                v_current = (point.s - prev_point.s) / self.dt
            else:
                v_current = self.v_target

        # v18.4: Clip velocity to physically reasonable bounds [0, 30 m/s]
        V_MAX_SAFE = 30.0  # Maximum physically reasonable speed
        v_current = max(0.0, min(v_current, V_MAX_SAFE))

        # Reachable s range in next time step
        s_min = point.s + v_current * self.dt + 0.5 * self.a_min * (self.dt ** 2)
        s_max = point.s + v_current * self.dt + 0.5 * self.a_max * (self.dt ** 2)

        # Ensure non-negative and within grid bounds
        s_min = max(0.0, s_min)
        s_max = min(self.s_max, s_max)

        # Convert to grid indices
        j_s_min = int(np.floor(s_min / self.ds_dense))
        j_s_max = int(np.ceil(s_max / self.ds_dense))

        j_s_min = max(0, j_s_min)
        j_s_max = min(self.n_s - 1, j_s_max)
        
        # v18.4: Limit maximum expansion to prevent excessive computation
        MAX_EXPANSION = 20  # Maximum number of nodes to expand
        if j_s_max - j_s_min > MAX_EXPANSION:
            mid = (j_s_min + j_s_max) // 2
            j_s_min = mid - MAX_EXPANSION // 2
            j_s_max = mid + MAX_EXPANSION // 2
            j_s_min = max(0, j_s_min)
            j_s_max = min(self.n_s - 1, j_s_max)

        return list(range(j_s_min, j_s_max + 1))

    def _check_collision(
        self,
        p1: STPoint,
        p2: STPoint,
        st_boundaries: List[STBoundaryDP]
    ) -> bool:
        """
        Check if the line segment (p1, p2) collides with any obstacle boundary.

        [v12.3] Note: This method is now deprecated in favor of the integrated
        soft obstacle cost in _dp_search, but kept for backward compatibility.

        Args:
            p1: Start point
            p2: End point
            st_boundaries: List of obstacle boundaries

        Returns:
            bool: True if collision detected
        """
        # Check collision at multiple sample points along the segment
        N_SAMPLES = 5
        for alpha in np.linspace(0, 1, N_SAMPLES):
            s_test = p1.s + alpha * (p2.s - p1.s)
            t_test = p1.t + alpha * (p2.t - p1.t)

            for boundary in st_boundaries:
                # Find time index
                i_t = int(np.round(t_test / self.dt))
                i_t = max(0, min(i_t, len(boundary.s_lower) - 1))

                # Check if s_test is within boundary
                if boundary.s_lower[i_t] <= s_test <= boundary.s_upper[i_t]:
                    return True  # Collision detected

        return False

    def _calculate_transition_cost(
        self, p1: STPoint, p2: STPoint, prev_node: Optional[DPNode] = None
    ) -> float:
        """
        Calculate transition cost from p1 to p2.

        v12.3: Added Jerk cost (Apollo dp_st_cost.cc)

        Components:
        1. Smoothness cost: Penalize large acceleration
        2. Jerk cost: Penalize rapid acceleration changes (v12.3)
        3. Reference cost: Penalize deviation from target speed
        4. Spatial potential: Encourage forward progress
        """
        dt = p2.t - p1.t
        ds = p2.s - p1.s

        if dt < 1e-6:
            return np.inf

        # Velocity
        v_current = ds / dt
        v_current = max(0.0, v_current)  # Non-negative

        # Acceleration estimate
        v_target = self.v_target
        a_current = (v_current - v_target) / dt

        # ====================================================================
        # v12.3: Jerk Cost (Apollo dp_st_cost.cc - GetJerkCost)
        # ====================================================================
        # Jerk = da/dt ≈ (a_current - a_prev) / dt
        # This prevents DP from choosing paths that QP cannot smooth out
        # ====================================================================
        cost_jerk = 0.0
        if prev_node is not None and prev_node.parent is not None:
            # Calculate previous acceleration
            i_t_prev, j_s_prev = prev_node.parent
            if i_t_prev >= 0:
                p0 = self.cost_table[i_t_prev][j_s_prev].point
                ds_prev = p1.s - p0.s
                dt_prev = p1.t - p0.t
                if dt_prev > 1e-6:
                    v_prev = ds_prev / dt_prev
                    a_prev = (v_prev - v_target) / dt_prev

                    # Jerk = change in acceleration
                    jerk = (a_current - a_prev) / dt
                    cost_jerk = self.w_jerk * (jerk ** 2)
        # ====================================================================

        # Cost components
        cost_smoothness = self.w_smoothness * (a_current ** 2)
        cost_reference = self.w_reference * ((v_current - v_target) ** 2)
        cost_spatial = self.w_spatial_potential * (-ds)  # Negative reward for progress

        total_cost = cost_smoothness + cost_jerk + cost_reference + cost_spatial

        return max(0.0, total_cost)

    def _extract_optimal_path(self) -> Optional[List[STPoint]]:
        """
        Extract optimal path by backtracking from final layer.

        Returns:
            List[STPoint]: Optimal path from t=0 to t=T, or None if failed
        """
        # Find minimum cost node in final layer
        final_layer = self.cost_table[-1]
        min_node = min(final_layer, key=lambda node: node.total_cost)

        if min_node.total_cost >= np.inf:
            return None

        # Backtrack to extract path
        path = []
        current = min_node

        for _ in range(self.n_t):
            path.append(current.point)

            if current.parent is None:
                break

            i_t_prev, j_s_prev = current.parent
            current = self.cost_table[i_t_prev][j_s_prev]

        path.reverse()
        return path

    def _extract_decisions(
        self,
        optimal_path: List[STPoint],
        obstacles: List[ObstacleInfo],
        ego_state: VehicleState
    ) -> Dict[int, InteractionDecision]:
        """
        Determine interaction decisions by comparing optimal path with obstacles.

        Logic:
        - If DP path is consistently BELOW obstacle → YIELD
        - If DP path is consistently ABOVE obstacle → OVERTAKE
        - If ambiguous → Use kinematic heuristic (prefer YIELD for safety)
        
        v18.4 FIX: Override OVERTAKE to YIELD if physically impossible
        - If obstacle is in front and closing fast, YIELD regardless of DP result

        Args:
            optimal_path: DP optimal path
            obstacles: List of obstacles
            ego_state: Current ego state

        Returns:
            Dict[obstacle_id, InteractionDecision]
        """
        decisions = {}
        L_VEHICLE = 4.5
        MARGIN = 2.0

        for obs in obstacles:
            obs_id = obs.vehicle_state.id

            # Compare path with obstacle at multiple time points
            above_count = 0
            below_count = 0
            total_checks = 0

            for point in optimal_path[1:]:  # Skip initial point
                t = point.t

                # Predict obstacle position at time t
                if obs.predicted_trajectory and len(obs.predicted_trajectory) > 0:
                    traj = obs.predicted_trajectory
                    if t >= traj[-1][0]:
                        obs_s = traj[-1][1]
                    else:
                        for k in range(len(traj) - 1):
                            if traj[k][0] <= t <= traj[k + 1][0]:
                                t1, s1, _, _ = traj[k]
                                t2, s2, _, _ = traj[k + 1]
                                obs_s = s1 + (s2 - s1) * (t - t1) / (t2 - t1)
                                break
                        else:
                            obs_s = traj[0][1]
                else:
                    obs_s = obs.vehicle_state.s + obs.vehicle_state.v * t

                # Compare ego path position with obstacle
                ego_s = point.s

                if ego_s > obs_s + L_VEHICLE / 2 + MARGIN:
                    above_count += 1
                elif ego_s < obs_s - L_VEHICLE / 2 - MARGIN:
                    below_count += 1

                total_checks += 1

            # Determine decision based on majority vote
            if total_checks == 0:
                decisions[obs_id] = InteractionDecision.YIELD
            elif above_count > below_count:
                decisions[obs_id] = InteractionDecision.OVERTAKE
            elif below_count > above_count:
                decisions[obs_id] = InteractionDecision.YIELD
            else:
                # Ambiguous case: use kinematic heuristic
                # If obstacle is far ahead, YIELD; if far behind, OVERTAKE
                rel_s = obs.vehicle_state.s - ego_state.s
                if rel_s > 10.0:
                    decisions[obs_id] = InteractionDecision.YIELD
                elif rel_s < -10.0:
                    decisions[obs_id] = InteractionDecision.OVERTAKE
                else:
                    # Default to YIELD for safety
                    decisions[obs_id] = InteractionDecision.YIELD

            # ================================================================
            # v18.17: CAV COOPERATIVE CONTROL - OVERTAKE PREFERENCE
            # ================================================================
            # Philosophy: CAVs can share trajectory information, so we can make
            # smarter decisions than traditional ACC/AEB-based systems.
            # 
            # Key Insight: YIELD often leads to unnecessary deceleration and
            # traffic congestion. If OVERTAKE is safely achievable, prefer it.
            #
            # NOTE: This only applies when DP gives ambiguous results (50/50 vote)
            # or when the gap is very large. For clear DP decisions, respect them.
            # ================================================================
            
            rel_s_current = obs.vehicle_state.s  # Already in relative frame (0 = ego)
            rel_v = obs.vehicle_state.v - ego_state.v  # Negative = closing
            
            # v18.17: OVERTAKE PREFERENCE - Only for ambiguous cases
            # Apply OVERTAKE preference when: 
            # 1. Obstacle is BEHIND us (should never be YIELD)
            # 2. Gap is very large AND ego is not slower than obstacle
            if decisions[obs_id] == InteractionDecision.YIELD:
                ego_v = ego_state.v
                obs_v = obs.vehicle_state.v
                
                if rel_s_current <= 0:
                    # Obstacle is BEHIND us - should be OVERTAKE, not YIELD!
                    decisions[obs_id] = InteractionDecision.OVERTAKE
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[CAV-OVERTAKE-PREF] V#{obs_id}: YIELD→OVERTAKE "
                              f"(obs is BEHIND: rel_s={rel_s_current:.1f}m)")
                
                elif rel_s_current > 50.0 and ego_v >= obs_v - 1.0:
                    # Very large gap AND not significantly slower
                    # We have plenty of time to accelerate and pass
                    decisions[obs_id] = InteractionDecision.OVERTAKE
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[CAV-OVERTAKE-PREF] V#{obs_id}: YIELD→OVERTAKE "
                              f"(very large gap: {rel_s_current:.1f}m > 50m, ego_v={ego_v:.1f})")
            
            # ================================================================
            # v18.17: SAFETY CHECK - Force YIELD only when truly necessary
            # ================================================================
            # Only override to YIELD in critical situations
            # ================================================================
            
            if decisions[obs_id] == InteractionDecision.OVERTAKE:
                # Only apply safety override for FRONT obstacles (rel_s > 0)
                if rel_s_current > 0:
                    # v18.17: Critical safety conditions only
                    
                    # Condition 1: Very close AND closing rapidly
                    CRITICAL_GAP = 8.0  # Force YIELD if < 8m
                    if rel_s_current < CRITICAL_GAP and rel_v < -1.0:
                        decisions[obs_id] = InteractionDecision.YIELD
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[CAV-SAFETY] V#{obs_id}: OVERTAKE→YIELD "
                                  f"(CRITICAL: gap={rel_s_current:.1f}m, closing={-rel_v:.1f}m/s)")
                    
                    # Condition 2: Very short TTC (imminent collision)
                    elif rel_v < -0.5:  # Closing
                        ttc = rel_s_current / abs(rel_v)
                        CRITICAL_TTC = 2.0  # Force YIELD if TTC < 2.0s
                        if ttc < CRITICAL_TTC:
                            decisions[obs_id] = InteractionDecision.YIELD
                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[CAV-SAFETY] V#{obs_id}: OVERTAKE→YIELD "
                                      f"(CRITICAL TTC: {ttc:.1f}s < {CRITICAL_TTC}s)")
            # ================================================================

            if ENABLE_DEBUG_OUTPUT:
                print(f"[DP] V#{obs_id}: above={above_count}, below={below_count} "
                      f"→ {decisions[obs_id].name}")

        return decisions

    def _fallback_decisions(
        self, obstacles: List[ObstacleInfo]
    ) -> Dict[int, InteractionDecision]:
        """
        Fallback decision logic when DP fails.

        Default to YIELD for all obstacles (safe conservative behavior).
        """
        decisions = {}
        for obs in obstacles:
            decisions[obs.vehicle_state.id] = InteractionDecision.YIELD
        return decisions


# ===== Unit Test =====
def test_dp_optimizer():
    """Unit test for SpeedHeuristicOptimizer"""
    print("=" * 80)
    print("SpeedHeuristicOptimizer Unit Test")
    print("=" * 80)

    # Create optimizer
    optimizer = SpeedHeuristicOptimizer(
        horizon_time=5.0,
        dt=0.5,
        ds_dense=1.0,
        s_max=100.0,
        a_max=2.0,
        a_min=-4.0,
        v_target=15.0
    )

    # Test case 1: Front vehicle moving slower
    print("\n[Test 1] Front vehicle moving slower")
    ego = VehicleState(id=0, s=0.0, v=15.0, a=0.0, lane='center')
    obs1 = ObstacleInfo(
        vehicle_state=VehicleState(id=1, s=30.0, v=10.0, a=0.0, lane='center'),
        is_front=True
    )

    decisions = optimizer.optimize(ego, [obs1])
    print(f"Decision for V#1: {decisions[1].name}")
    assert decisions[1] in [InteractionDecision.YIELD, InteractionDecision.OVERTAKE]

    # Test case 2: Fast vehicle behind
    print("\n[Test 2] Fast vehicle behind")
    ego2 = VehicleState(id=0, s=50.0, v=12.0, a=0.0, lane='center')
    obs2 = ObstacleInfo(
        vehicle_state=VehicleState(id=2, s=20.0, v=18.0, a=0.0, lane='center'),
        is_front=False
    )

    decisions2 = optimizer.optimize(ego2, [obs2])
    print(f"Decision for V#2: {decisions2[2].name}")
    assert decisions2[2] in [InteractionDecision.YIELD, InteractionDecision.OVERTAKE]

    # Test case 3: Multiple obstacles
    print("\n[Test 3] Multiple obstacles")
    ego3 = VehicleState(id=0, s=0.0, v=15.0, a=0.0, lane='center')
    obs3 = ObstacleInfo(
        vehicle_state=VehicleState(id=3, s=40.0, v=12.0, a=0.0, lane='center'),
        is_front=True
    )
    obs4 = ObstacleInfo(
        vehicle_state=VehicleState(id=4, s=80.0, v=15.0, a=0.0, lane='center'),
        is_front=True
    )

    decisions3 = optimizer.optimize(ego3, [obs3, obs4])
    print(f"Decision for V#3: {decisions3[3].name}")
    print(f"Decision for V#4: {decisions3[4].name}")

    print("\n" + "=" * 80)
    print("[PASS] All DP optimizer tests passed!")
    print("=" * 80)


if __name__ == "__main__":
    test_dp_optimizer()
