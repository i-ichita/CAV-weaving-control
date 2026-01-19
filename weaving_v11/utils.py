# -*- coding: utf-8 -*-
"""
weaving_v11/utils.py (ver.11.0 / 2025-12-17)

織り込み区間制御システムのユーティリティクラスとヘルパー関数:
- Logger: シミュレーション用ログ出力ハンドラ
- SCRIPT_NAME: スクリプトバージョン識別子
- ENABLE_DEBUG_OUTPUT: デバッグ出力フラグ
- レーン隣接ヘルパー関数
"""

import sys
import os
from typing import Optional, List
from datetime import datetime

# スクリプト識別
SCRIPT_NAME = "weaving_v11"
SCRIPT_VERSION = SCRIPT_NAME

# デバッグフラグ（インポート元モジュールでオーバーライド可能）
ENABLE_DEBUG_OUTPUT = True
ENABLE_VERBOSE_DEBUG = False


class Logger:
    """
    シミュレーション結果用デュアル出力ロガー。

    ターミナルとログファイルの両方に即時フラッシュで出力。
    """

    def __init__(self, filename: str, script_name: str = SCRIPT_VERSION):
        self.terminal = sys.stdout
        # buffering=1 for line buffering
        self.log = open(filename, 'w', encoding='utf-8', buffering=1)
        self.closed = False
        self.log.write(f"Simulation Log - {script_name}\n")
        self.log.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.log.write("="*80 + "\n")
        self.flush()

    def write(self, message):
        if self.closed:
            return
        try:
            self.terminal.write(message)
            self.terminal.flush()
        except Exception:
            pass  # Ignore console errors

        self.log.write(message)
        self.log.flush()  # Flush Python buffer
        os.fsync(self.log.fileno())  # Force OS-level write (important)

    def flush(self):
        if not self.closed:
            self.terminal.flush()
            self.log.flush()
            os.fsync(self.log.fileno())

    def close(self):
        if not self.closed:
            self.flush()
            self.closed = True
            self.log.close()


# ============================================================================
# Lane Adjacency Helper Functions
# ============================================================================

def get_next_adjacent_lane(current_lane: str, target_area: str) -> Optional[str]:
    """
    Get the next adjacent lane moving from current_lane towards target_area.

    Args:
        current_lane: Current lane name ("left", "lcenter", "rcenter", "right")
        target_area: Target area ("LEFT" or "RIGHT")

    Returns:
        Next adjacent lane to move to, or None if already at target or invalid input

    Examples:
        >>> get_next_adjacent_lane("rcenter", "LEFT")
        "lcenter"
        >>> get_next_adjacent_lane("left", "LEFT")
        None
        >>> get_next_adjacent_lane("right", "RIGHT")
        None
    """
    lane_index = {"left": 0, "lcenter": 1, "rcenter": 2, "right": 3}
    index_to_lane = {0: "left", 1: "lcenter", 2: "rcenter", 3: "right"}

    if current_lane not in lane_index:
        return None

    current_idx = lane_index[current_lane]

    if target_area == "LEFT":
        # Already at leftmost lane
        if current_lane == "left":
            return None
        # Move one step left
        target_idx = current_idx - 1
    elif target_area == "RIGHT":
        # Already at rightmost lane
        if current_lane == "right":
            return None
        # Move one step right
        target_idx = current_idx + 1
    else:
        return None

    return index_to_lane.get(target_idx)


def get_adjacent_lanes(lane: str) -> List[str]:
    """
    Get list of adjacent lanes for a given lane.

    Args:
        lane: Lane name ("left", "lcenter", "rcenter", or "right")

    Returns:
        List of adjacent lane names

    Examples:
        >>> get_adjacent_lanes("lcenter")
        ["left", "rcenter"]
        >>> get_adjacent_lanes("left")
        ["lcenter"]
    """
    adjacency = {
        "left": ["lcenter"],
        "lcenter": ["left", "rcenter"],
        "rcenter": ["lcenter", "right"],
        "right": ["rcenter"]
    }

    return adjacency.get(lane, [])


def is_adjacent_lane(lane1: str, lane2: str) -> bool:
    """
    Check if two lanes are adjacent.

    Args:
        lane1: First lane name
        lane2: Second lane name

    Returns:
        True if lanes are adjacent, False otherwise
    """
    try:
        lane_order = ["left", "lcenter", "rcenter", "right"]
        idx1 = lane_order.index(lane1)
        idx2 = lane_order.index(lane2)
        return abs(idx1 - idx2) == 1
    except ValueError:
        return False
