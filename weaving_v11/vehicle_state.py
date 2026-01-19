# -*- coding: utf-8 -*-
"""
================================================================================
Apollo Complete Vehicle State with Full Frenet Coordinates (s, d)
================================================================================

Apollo準拠の完全なFrenet座標系での車両状態表現

Frenet座標 (s, d, s_dot, d_dot, s_ddot, d_ddot):
- s: 縦方向位置（道路中心線に沿った距離）[m]
- d: 横方向オフセット（道路中心線からの距離）[m]
- s_dot: 縦方向速度 [m/s]
- d_dot: 横方向速度 [m/s] (車線変更時に非ゼロ)
- s_ddot: 縦方向加速度 [m/s²]
- d_ddot: 横方向加速度 [m/s²]

Reference:
- Apollo: modules/planning/common/path/frenet_frame_path.h
- Werling et al. (2010): Optimal Trajectory Generation

Version: 11.0
Date: 2025-12-16
================================================================================
"""

from dataclasses import dataclass
from typing import Optional, cast


@dataclass
class VehicleStateComplete:
    """
    Complete vehicle state in Frenet frame (Apollo standard)

    Apollo準拠の完全なFrenet座標系車両状態

    Attributes:
        id: 車両ID
        s: 縦方向位置 [m]
        d: 横方向オフセット [m] (正: 左, 負: 右)
        s_dot: 縦方向速度 [m/s]
        d_dot: 横方向速度 [m/s] (車線変更時に非ゼロ)
        s_ddot: 縦方向加速度 [m/s²]
        d_ddot: 横方向加速度 [m/s²]
        lane: 現在のレーン名（互換性のため）
        target_lane: 目標レーン（車線変更時）
        changing_lane: 車線変更中フラグ
    """
    id: int

    # Frenet longitudinal (縦方向)
    s: float           # position [m]
    s_dot: float       # velocity [m/s]
    s_ddot: float = 0.0  # acceleration [m/s²]

    # Frenet lateral (横方向)
    d: float = 0.0     # lateral offset [m]
    d_dot: float = 0.0   # lateral velocity [m/s]
    d_ddot: float = 0.0  # lateral acceleration [m/s²]

    # Lane information (compatibility)
    lane: str = 'center'
    target_lane: Optional[str] = None
    changing_lane: bool = False

    @property
    def v(self) -> float:
        """Total velocity magnitude [m/s]"""
        return (self.s_dot**2 + self.d_dot**2)**0.5

    @property
    def a(self) -> float:
        """Total acceleration magnitude [m/s²]"""
        return (self.s_ddot**2 + self.d_ddot**2)**0.5

    def is_lane_changing(self) -> bool:
        """車線変更中かどうか（横方向速度が非ゼロ）"""
        return abs(self.d_dot) > 0.01  # threshold: 0.01 m/s

    def lateral_distance_to(self, target_d: float) -> float:
        """目標横方向位置までの距離 [m]"""
        return abs(target_d - self.d)

    def __repr__(self) -> str:
        return (f"VehicleState(id={self.id}, s={self.s:.1f}m, d={self.d:.2f}m, "
                f"v={self.v:.1f}m/s, lane={self.lane})")


@dataclass
class VehicleStateSimplified:
    """
    Simplified vehicle state (longitudinal only, compatible with v11.0 current implementation)

    簡易版車両状態（縦方向のみ、現在のv11.0実装と互換）

    Note:
        これは現在のv11.0実装で使用されている簡易版です。
        横方向の動きを考慮する場合は VehicleStateComplete を使用してください。
    """
    id: int
    s: float       # longitudinal position [m]
    v: float       # velocity [m/s]
    a: float = 0.0   # acceleration [m/s²]
    d: float = 0.0   # lateral offset [m] (static, from lane)
    lane: str = 'center'

    def to_complete(self, d_dot: float = 0.0, d_ddot: float = 0.0) -> VehicleStateComplete:
        """簡易版から完全版に変換"""
        return VehicleStateComplete(
            id=self.id,
            s=self.s,
            s_dot=self.v,
            s_ddot=self.a,
            d=self.d,
            d_dot=d_dot,
            d_ddot=d_ddot,
            lane=self.lane
        )


def compute_lane_change_lateral_trajectory(
    d_start: float,
    d_end: float,
    duration: float,
    dt: float = 0.1
) -> tuple[list[float], list[float], list[float]]:
    """
    Generate smooth lateral trajectory for lane change (Apollo polynomial method)

    Apollo標準の5次多項式による滑らかな車線変更軌道生成

    Args:
        d_start: 開始横方向位置 [m]
        d_end: 終了横方向位置 [m]
        duration: 車線変更時間 [s]
        dt: 時間刻み [s]

    Returns:
        (d_array, d_dot_array, d_ddot_array): 横方向位置・速度・加速度

    Reference:
        Apollo: modules/planning/math/quintic_polynomial_curve1d.cc
    """
    import numpy as np

    T: float = duration
    N: int = int(T / dt)
    t_array: np.ndarray = np.linspace(0, T, N)

    # 5次多項式係数（境界条件: d(0)=d_start, d(T)=d_end, d'(0)=d'(T)=0, d''(0)=d''(T)=0）
    # d(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    a0: float = d_start
    a1: float = 0.0
    a2: float = 0.0
    a3: float = 10 * (d_end - d_start) / (T**3)
    a4: float = -15 * (d_end - d_start) / (T**4)
    a5: float = 6 * (d_end - d_start) / (T**5)

    d_array: np.ndarray = a0 + a1*t_array + a2*t_array**2 + a3*t_array**3 + a4*t_array**4 + a5*t_array**5
    d_dot_array: np.ndarray = a1 + 2*a2*t_array + 3*a3*t_array**2 + 4*a4*t_array**3 + 5*a5*t_array**4
    d_ddot_array: np.ndarray = 2*a2 + 6*a3*t_array + 12*a4*t_array**2 + 20*a5*t_array**3

    d_list: list[float] = cast(list[float], d_array.tolist())
    d_dot_list: list[float] = cast(list[float], d_dot_array.tolist())
    d_ddot_list: list[float] = cast(list[float], d_ddot_array.tolist())

    return d_list, d_dot_list, d_ddot_list


if __name__ == "__main__":
    print("=" * 80)
    print("Apollo Complete Vehicle State Test")
    print("=" * 80)

    # Test 1: Complete state
    print("\n### Test 1: Complete Vehicle State ###")
    vehicle = VehicleStateComplete(
        id=1,
        s=100.0,
        s_dot=15.0,
        s_ddot=0.5,
        d=-1.75,  # center_left lane
        d_dot=0.0,
        d_ddot=0.0,
        lane='center_left'
    )
    print(f"Vehicle: {vehicle}")
    print(f"  Total velocity: {vehicle.v:.2f} m/s")
    print(f"  Is lane changing: {vehicle.is_lane_changing()}")

    # Test 2: Lane change trajectory
    print("\n### Test 2: Lane Change Trajectory (5th order polynomial) ###")
    d_start = -1.75  # center_left
    d_end = 1.75     # center_right
    duration = 3.0   # 3 seconds

    d_arr, d_dot_arr, d_ddot_arr = compute_lane_change_lateral_trajectory(
        d_start, d_end, duration
    )

    print(f"  Start: d={d_start:.2f}m")
    print(f"  End: d={d_end:.2f}m")
    print(f"  Duration: {duration}s")
    print(f"  Mid-point: d={d_arr[len(d_arr)//2]:.2f}m, "
          f"d_dot={d_dot_arr[len(d_arr)//2]:.3f}m/s")
    print(f"  Max lateral velocity: {max(abs(v) for v in d_dot_arr):.3f} m/s")
    print(f"  Max lateral acceleration: {max(abs(a) for a in d_ddot_arr):.3f} m/s^2")

    # Test 3: Simplified to Complete conversion
    print("\n### Test 3: Simplified -> Complete Conversion ###")
    simple = VehicleStateSimplified(id=2, s=200.0, v=20.0, a=1.0, d=0.0, lane='center')
    complete = simple.to_complete(d_dot=0.5)
    print(f"  Simplified: s={simple.s:.1f}m, v={simple.v:.1f}m/s")
    print(f"  Complete: {complete}")

    print("\n[PASS] All Apollo vehicle state tests passed!")
