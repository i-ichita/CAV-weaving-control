# -*- coding: utf-8 -*-
# --- frenet_tools.py (ver.11.0 / 2025-12-17): Frenetフレームユーティリティ ---
"""
================================================================================
Frenetフレーム座標変換 & ST境界ユーティリティ
================================================================================

参照:
- Apollo: modules/planning/reference_line/reference_line.cc
- Werling et al. (2010): 動的道路シナリオのための最適軌道生成
- Fan et al. (2018): Baidu Apollo EMモーションプランナー

主要機能:
- XY ↔ SD (Frenet) 座標変換 (直線高速道路用)
- ST境界 (Station-Timeグラフ) 構築
- 高速道路織り込み用の簡略化実装 (直線参照経路)

バージョン: 11.0
日付: 2025-12-17
作者: Apolloオープンソース実装に基づく
================================================================================
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class PathPoint:
    """Reference path point in Cartesian coordinates (straight highway)"""
    x: float      # global X position [m]
    y: float      # global Y position [m] (constant for straight highway)
    theta: float  # heading angle [rad] (0 for straight highway)
    s: float      # accumulated arc length [m] (= x for straight highway)


@dataclass
class FrenetPoint:
    """Point in Frenet frame (relative to reference path)"""
    s: float      # longitudinal position [m]
    d: float      # lateral offset [m] (positive = left of centerline)
    s_dot: float  # longitudinal velocity [m/s]
    d_dot: float  # lateral velocity [m/s]
    s_ddot: float # longitudinal acceleration [m/s²]
    d_ddot: float # lateral acceleration [m/s²]


@dataclass
class STPoint:
    """Point in Station-Time (ST) graph"""
    s: float  # station (longitudinal position) [m]
    t: float  # time [s]
    
    
@dataclass
class STBoundary:
    """
    ST-Boundary for obstacle in Station-Time graph
    
    Reference: Apollo SpeedBoundsDecider
    modules/planning/tasks/deciders/speed_bounds_decider/
    """
    s_lower: np.ndarray  # s_lower[k] at time t_k
    s_upper: np.ndarray  # s_upper[k] at time t_k
    t_array: np.ndarray  # time array
    vehicle_id: int
    
    def __repr__(self):
        return f"STBoundary(vehicle={self.vehicle_id}, t_span={self.t_array[0]:.1f}-{self.t_array[-1]:.1f}s)"


class FrenetCoordinateTransform:
    """
    Frenet coordinate transformation for straight highway
    
    Reference: Werling et al. (2010) - Optimal Trajectory Generation
    
    For straight highway, the transformation simplifies to:
    - s = x (longitudinal position)
    - l = y - y_ref (lateral offset from centerline)
    - θ_ref = 0 (heading angle of reference path)
    """
    
    def __init__(self, lane_width: float = 3.5):
        """
        Args:
            lane_width: Width of each lane [m] (default: 3.5m, standard highway)
        """
        self.lane_width = lane_width
        
        # Lane centerlines (straight highway, y-coordinates)
        # Example: 4-lane highway with lanes numbered 0-3
        self.lane_centers = {
            'left': -1.5 * lane_width,     # Lane 0 (leftmost)
            'center_left': -0.5 * lane_width,  # Lane 1
            'center_right': 0.5 * lane_width,  # Lane 2
            'right': 1.5 * lane_width      # Lane 3 (rightmost)
        }
    
    def cartesian_to_frenet(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        ax: float,
        ay: float,
        lane: str
    ) -> FrenetPoint:
        """
        Convert Cartesian coordinates to Frenet frame
        
        For straight highway:
        s = x, l = y - y_lane_center
        
        Args:
            x, y: Cartesian position [m]
            vx, vy: Cartesian velocity [m/s]
            ax, ay: Cartesian acceleration [m/s²]
            lane: Current lane name
            
        Returns:
            FrenetPoint in Frenet coordinates
        """
        y_ref = self.lane_centers.get(lane, 0.0)
        
        # For straight highway: s = x, lateral_offset = y - y_ref
        s = x
        lateral_offset = y - y_ref
        
        # Velocities (assuming heading along x-axis)
        s_dot = vx
        lateral_velocity = vy
        
        # Accelerations
        s_ddot = ax
        lateral_accel = ay
        
        return FrenetPoint(s, lateral_offset, s_dot, lateral_velocity, s_ddot, lateral_accel)
    
    def frenet_to_cartesian(
        self,
        frenet_point: FrenetPoint,
        lane: str
    ) -> Tuple[float, float, float, float, float, float]:
        """
        Convert Frenet coordinates to Cartesian frame
        
        For straight highway:
        x = s, y = l + y_lane_center
        
        Args:
            frenet_point: Point in Frenet coordinates
            lane: Current lane name
            
        Returns:
            (x, y, vx, vy, ax, ay) in Cartesian coordinates
        """
        y_ref = self.lane_centers.get(lane, 0.0)
        
        x = frenet_point.s
        y = frenet_point.d + y_ref
        
        vx = frenet_point.s_dot
        vy = frenet_point.d_dot
        
        ax = frenet_point.s_ddot
        ay = frenet_point.d_ddot
        
        return x, y, vx, vy, ax, ay


class STBoundaryBuilder:
    """
    Build ST-Boundary from vehicle trajectories
    
    Reference: Apollo SpeedBoundsDecider
    modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.cc
    
    Key Concept:
    - Project surrounding vehicles into (s, t) space
    - Define safe region as s_lower(t) <= s_ego(t) <= s_upper(t)
    - QP solver enforces these bounds as hard constraints
    """
    
    def __init__(self, vehicle_length: float = 4.5, safety_buffer: float = 1.0):
        """
        Args:
            vehicle_length: Length of vehicle [m]
            safety_buffer: Additional safety buffer [m]
        """
        self.vehicle_length = vehicle_length
        self.safety_buffer = safety_buffer
    
    def build_boundary_from_vehicle(
        self,
        ego_s: float,
        obstacle_s: float,
        obstacle_v: float,
        horizon: int,
        dt: float
    ) -> STBoundary:
        """
        Build ST-Boundary for a single obstacle vehicle
        
        Args:
            ego_s: Ego vehicle current position [m]
            obstacle_s: Obstacle vehicle current position [m]
            obstacle_v: Obstacle vehicle velocity [m/s]
            horizon: Number of time steps
            dt: Time step [s]
            
        Returns:
            STBoundary with s_lower and s_upper arrays
        """
        t_array = np.arange(horizon) * dt
        
        # Predict obstacle trajectory (constant velocity assumption)
        s_obstacle = obstacle_s + obstacle_v * t_array
        
        # Determine if obstacle is in front or behind
        if obstacle_s > ego_s:
            # Obstacle in front: set upper bound
            s_upper = s_obstacle - self.vehicle_length - self.safety_buffer
            s_lower = np.zeros(horizon)  # No lower constraint
        else:
            # Obstacle behind: set lower bound
            s_lower = s_obstacle + self.vehicle_length + self.safety_buffer
            s_upper = np.full(horizon, np.inf)  # No upper constraint
        
        return STBoundary(
            s_lower=s_lower,
            s_upper=s_upper,
            t_array=t_array,
            vehicle_id=-1  # Unknown ID
        )
    
    def merge_boundaries(
        self,
        boundaries: List[STBoundary]
    ) -> STBoundary:
        """
        Merge multiple ST-Boundaries into tightest combined boundary
        
        Reference: Apollo SpeedBoundsDecider::MergeBoundaries
        
        Args:
            boundaries: List of STBoundary objects
            
        Returns:
            Combined STBoundary with tightest constraints
        """
        if not boundaries:
            # No constraints
            horizon = 30
            return STBoundary(
                s_lower=np.zeros(horizon),
                s_upper=np.full(horizon, np.inf),
                t_array=np.arange(horizon) * 0.1,
                vehicle_id=-1
            )
        
        # Use first boundary as template
        n = len(boundaries[0].s_lower)
        s_lower_combined = np.zeros(n)
        s_upper_combined = np.full(n, np.inf)
        
        for boundary in boundaries:
            # Take maximum of lower bounds (tightest lower constraint)
            s_lower_combined = np.maximum(s_lower_combined, boundary.s_lower)
            # Take minimum of upper bounds (tightest upper constraint)
            s_upper_combined = np.minimum(s_upper_combined, boundary.s_upper)
        
        return STBoundary(
            s_lower=s_lower_combined,
            s_upper=s_upper_combined,
            t_array=boundaries[0].t_array,
            vehicle_id=-1
        )


def compute_time_headway(
    gap: float,
    v_rear: float,
    L_vehicle: float = 4.5
) -> float:
    """
    Compute time headway from spatial gap
    
    Time headway = (gap - L_vehicle) / v_rear
    
    Args:
        gap: Spatial gap between vehicles [m]
        v_rear: Rear vehicle velocity [m/s]
        L_vehicle: Vehicle length [m]
        
    Returns:
        Time headway [s]
    """
    net_gap = max(0.0, gap - L_vehicle)
    if v_rear < 0.1:
        return np.inf
    return net_gap / v_rear


def compute_ttc(
    gap: float,
    v_rear: float,
    v_front: float,
    L_vehicle: float = 5.0
) -> float:
    """
    Compute Time-To-Collision (TTC)

    Reference: ISO 21202 (Advanced Driver Assistance Systems)

    TTC = net_gap / (v_rear - v_front)  if v_rear > v_front
          ∞                              otherwise

    【視点による理解】
    - 後続車から見ると、前方車の「後端」が見える
    - TTCは、後続車の「前端」が前方車の「後端」に到達するまでの時間

    【重要】引数 'gap' の定義:
    このgapは「車両中心間距離」を想定しています。
    したがって、物理的なheadway（前端-後端間距離）を得るためには
    L_vehicleを引く必要があります。

    net_gap = gap - L_vehicle
            = (x_front - x_rear) - L_vehicle
            = (x_front - L/2) - (x_rear + L/2)  [前端-後端距離]

    Args:
        gap: Spatial gap between vehicle centers [m] (NOT physical headway)
        v_rear: Rear vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        L_vehicle: Vehicle length [m] (default: 5.0m, updated from 4.5m for consistency)

    Returns:
        TTC [s], or np.inf if not approaching
    """
    delta_v = v_rear - v_front

    if delta_v <= 0:
        return np.inf

    # Convert center-to-center gap to physical headway (bumper-to-bumper)
    net_gap = max(0.0, gap - L_vehicle)
    return net_gap / delta_v


# ============ UTILITY FUNCTIONS ============

def visualize_st_graph(
    boundaries: List[STBoundary],
    ego_trajectory: Optional[np.ndarray] = None,
    save_path: Optional[str] = None
):
    """
    Visualize ST-graph with boundaries
    
    Args:
        boundaries: List of STBoundary objects
        ego_trajectory: Ego vehicle's s-t trajectory (optional)
        save_path: Path to save figure (optional)
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[Warning] matplotlib not available, skipping visualization")
        return
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot boundaries
    for boundary in boundaries:
        t = boundary.t_array
        ax.fill_between(t, boundary.s_lower, boundary.s_upper,
                        alpha=0.3, label=f'Vehicle {boundary.vehicle_id}')
        ax.plot(t, boundary.s_lower, 'r-', linewidth=1)
        ax.plot(t, boundary.s_upper, 'g-', linewidth=1)
    
    # Plot ego trajectory
    if ego_trajectory is not None:
        ax.plot(ego_trajectory[:, 1], ego_trajectory[:, 0], 'b-',
                linewidth=2, label='Ego trajectory')
    
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Station (s) [m]')
    ax.set_title('ST-Graph with Boundaries')
    ax.legend()
    ax.grid(True)
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[ST-Graph] Saved to {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    # Test case: Frenet coordinate transformation
    print("=" * 80)
    print("Frenet Tools Test")
    print("=" * 80)
    
    # Initialize transformer
    transformer = FrenetCoordinateTransform(lane_width=3.5)
    
    # Test vehicle at (x=100m, y=0m, lane=center_right)
    x, y, vx, vy = 100.0, 1.75, 15.0, 0.0
    ax, ay = 0.5, 0.0
    lane = 'center_right'
    
    # Convert to Frenet
    frenet = transformer.cartesian_to_frenet(x, y, vx, vy, ax, ay, lane)
    print(f"\nCartesian: x={x}, y={y}, vx={vx}")
    print(f"Frenet:    s={frenet.s:.2f}, d={frenet.d:.2f}, s_dot={frenet.s_dot:.2f}")
    
    # Convert back to Cartesian
    x2, y2, vx2, vy2, ax2, ay2 = transformer.frenet_to_cartesian(frenet, lane)
    print(f"Recovered: x={x2}, y={y2}, vx={vx2}")
    print(f"Error: {abs(x-x2):.6f} m")
    
    # Test ST-Boundary builder
    print("\n" + "=" * 80)
    print("ST-Boundary Test")
    print("=" * 80)
    
    builder = STBoundaryBuilder(vehicle_length=4.5, safety_buffer=1.0)
    
    # Ego at s=0, obstacle at s=30m, v=10m/s
    boundary = builder.build_boundary_from_vehicle(
        ego_s=0.0,
        obstacle_s=30.0,
        obstacle_v=10.0,
        horizon=30,
        dt=0.1
    )
    
    print(f"\nBoundary: {boundary}")
    print(f"s_upper at t=0s: {boundary.s_upper[0]:.2f} m")
    print(f"s_upper at t=2s: {boundary.s_upper[20]:.2f} m")
    
    # Test TTC computation
    print("\n" + "=" * 80)
    print("TTC Test")
    print("=" * 80)
    
    gap = 20.0
    v_rear = 15.0
    v_front = 10.0
    ttc = compute_ttc(gap, v_rear, v_front)
    print(f"\nGap: {gap}m, v_rear: {v_rear}m/s, v_front: {v_front}m/s")
    print(f"TTC: {ttc:.2f} s")
    
    print("\n✓ All tests passed!")
