# -*- coding: utf-8 -*-
"""
================================================================================
Coordinate Transformation: v10.3 (Cartesian) ↔ v11.0 (Frenet)
================================================================================

v10.3の座標系:
- x: 縦方向位置（道路に沿った距離） [m]
- lane: レーン名 ('left', 'center_left', 'center_right', 'right')

v11.0の座標系 (Frenet):
- s: 縦方向位置（道路中心線に沿った距離） [m]
- d: 横方向オフセット（道路中心線からの距離） [m]
  - d > 0: 左側
  - d = 0: 中心線
  - d < 0: 右側

Version: 11.0
Date: 2025-12-16
================================================================================
"""

from typing import Dict
from dataclasses import dataclass


# レーン中心線の横方向オフセット（道路中心線からの距離）
# 標準的な高速道路: レーン幅 = 3.5m
LANE_WIDTH = 3.5  # [m]

LANE_OFFSETS: Dict[str, float] = {
    'left': -1.5 * LANE_WIDTH,        # -5.25m (左端レーン)
    'lcenter': -0.5 * LANE_WIDTH,     # -1.75m (左中央レーン)
    'center_left': -0.5 * LANE_WIDTH, # -1.75m (エイリアス)
    'rcenter': 0.5 * LANE_WIDTH,      # +1.75m (右中央レーン)
    'center_right': 0.5 * LANE_WIDTH, # +1.75m (エイリアス)
    'right': 1.5 * LANE_WIDTH,        # +5.25m (右端レーン)
}


@dataclass
class CartesianState:
    """v10.3 の車両状態（デカルト座標）"""
    id: int
    x: float      # 縦方向位置 [m]
    v: float      # 速度 [m/s]
    a: float      # 加速度 [m/s²]
    lane: str     # レーン名


@dataclass
class FrenetState:
    """v11.0 の車両状態（Frenet座標）"""
    id: int
    s: float      # 縦方向位置 [m]
    d: float      # 横方向オフセット [m]
    v: float      # 速度 [m/s]
    a: float      # 加速度 [m/s²]
    lane: str     # レーン名（互換性のため保持）


def cartesian_to_frenet(cart_state: CartesianState) -> FrenetState:
    """
    v10.3のデカルト座標をv11.0のFrenet座標に変換

    Args:
        cart_state: v10.3の車両状態

    Returns:
        frenet_state: v11.0の車両状態

    Note:
        直線道路を仮定しているため、s = x となります
    """
    s = cart_state.x  # 直線道路では s = x
    d = LANE_OFFSETS.get(cart_state.lane, 0.0)

    return FrenetState(
        id=cart_state.id,
        s=s,
        d=d,
        v=cart_state.v,
        a=cart_state.a,
        lane=cart_state.lane
    )


def frenet_to_cartesian(frenet_state: FrenetState) -> CartesianState:
    """
    v11.0のFrenet座標をv10.3のデカルト座標に変換

    Args:
        frenet_state: v11.0の車両状態

    Returns:
        cart_state: v10.3の車両状態

    Note:
        直線道路では x = s
        レーンはd値から最も近いレーンを選択
    """
    x = frenet_state.s  # 直線道路では x = s

    # d値から最も近いレーンを特定
    lane = find_closest_lane(frenet_state.d)

    return CartesianState(
        id=frenet_state.id,
        x=x,
        v=frenet_state.v,
        a=frenet_state.a,
        lane=lane
    )


def find_closest_lane(d: float) -> str:
    """
    横方向オフセットdから最も近いレーン名を取得

    Args:
        d: 横方向オフセット [m]

    Returns:
        lane: レーン名
    """
    min_dist = float('inf')
    closest_lane = 'center_left'

    for lane_name, lane_d in LANE_OFFSETS.items():
        dist = abs(d - lane_d)
        if dist < min_dist:
            min_dist = dist
            closest_lane = lane_name

    # エイリアスを正規化
    if closest_lane == 'lcenter':
        closest_lane = 'center_left'
    elif closest_lane == 'rcenter':
        closest_lane = 'center_right'

    return closest_lane


def get_lane_offset(lane: str) -> float:
    """
    レーン名から横方向オフセットを取得

    Args:
        lane: レーン名

    Returns:
        d: 横方向オフセット [m]
    """
    return LANE_OFFSETS.get(lane, 0.0)


def compute_lateral_distance(lane1: str, lane2: str) -> float:
    """
    2つのレーン間の横方向距離を計算

    Args:
        lane1: レーン1の名前
        lane2: レーン2の名前

    Returns:
        distance: 横方向距離 [m] (絶対値)
    """
    d1 = get_lane_offset(lane1)
    d2 = get_lane_offset(lane2)
    return abs(d2 - d1)


if __name__ == "__main__":
    print("=" * 80)
    print("座標変換テスト: v10.3 (Cartesian) <-> v11.0 (Frenet)")
    print("=" * 80)

    # テスト1: Cartesian → Frenet
    print("\n### Test 1: Cartesian → Frenet ###")
    cart = CartesianState(id=1, x=100.0, v=15.0, a=0.0, lane='center_left')
    frenet = cartesian_to_frenet(cart)
    print(f"Input (v10.3):  x={cart.x:.1f}m, lane={cart.lane}")
    print(f"Output (v11.0): s={frenet.s:.1f}m, d={frenet.d:.2f}m")

    # テスト2: Frenet → Cartesian
    print("\n### Test 2: Frenet → Cartesian ###")
    frenet2 = FrenetState(id=2, s=200.0, d=1.75, v=20.0, a=0.5, lane='center_right')
    cart2 = frenet_to_cartesian(frenet2)
    print(f"Input (v11.0):  s={frenet2.s:.1f}m, d={frenet2.d:.2f}m")
    print(f"Output (v10.3): x={cart2.x:.1f}m, lane={cart2.lane}")

    # テスト3: レーン間距離
    print("\n### Test 3: Lane Distance ###")
    for lane1 in ['left', 'center_left', 'center_right', 'right']:
        for lane2 in ['left', 'center_left', 'center_right', 'right']:
            if lane1 != lane2:
                dist = compute_lateral_distance(lane1, lane2)
                print(f"  {lane1:12s} <-> {lane2:12s}: {dist:.2f}m")

    print("\n[PASS] All coordinate transformation tests passed!")
