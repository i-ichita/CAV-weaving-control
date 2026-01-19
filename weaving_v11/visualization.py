# -*- coding: utf-8 -*-
"""
weaving_v11/visualization.py

シミュレーション結果の可視化関数:
- plot_lc_spatiotemporal_distribution: LC時空間分布プロット
- export_video: シミュレーションアニメーションのMP4エクスポート
"""

from typing import TYPE_CHECKING, List, Dict, Tuple
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

from .coordinate_transform import LANE_OFFSETS, LANE_WIDTH

if TYPE_CHECKING:
    from .simulator import IntegratedZoneSimulator


def plot_lc_spatiotemporal_distribution(simulator: 'IntegratedZoneSimulator',
                                        output_filename: str):
    """
    Plot lane change spatiotemporal distribution (v6.47).

    Only uses data where x >= 0 (excludes warmup zone).

    Args:
        simulator: Integrated zone simulator instance
        output_filename: Output plot file path

    Note:
        This function creates a 2x2 subplot with:
        - (a) Spatiotemporal scatter plot
        - (b) Spatial distribution histogram
        - (c) Temporal distribution histogram
        - (d) Cell-wise LC distribution bar chart
    """
    if not simulator.lc_history:
        print("[WARNING] No LC history to plot")
        return

    # Filter: x >= 0 (exclude warmup zone)
    lc_df = pd.DataFrame(simulator.lc_history)
    lc_df = lc_df[lc_df['x'] >= 0.0]

    if len(lc_df) == 0:
        print("[WARNING] No valid LC data (all x < 0)")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Plot 1: Spatiotemporal scatter plot
    ax1 = axes[0, 0]
    ax1.scatter(lc_df['x'], lc_df['t'], alpha=0.6, s=30)
    ax1.set_xlabel('Position (m)', fontsize=12)
    ax1.set_ylabel('Time (s)', fontsize=12)
    ax1.set_title(f'(a) Spatiotemporal Distribution - {simulator.mode.upper()}',
                  fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, simulator.params.total_length])

    # Preparation-Weave boundary
    ax1.axvline(x=simulator.params.prep_zone_length,
                color='r', linestyle='--', linewidth=2,
                label='Prep-Weave Boundary')
    ax1.legend()

    # Plot 2: Spatial distribution histogram
    ax2 = axes[0, 1]
    ax2.hist(lc_df['x'], bins=40, alpha=0.7, edgecolor='black')
    ax2.set_xlabel('Position (m)', fontsize=12)
    ax2.set_ylabel('Frequency', fontsize=12)
    ax2.set_title('(b) Spatial Distribution', fontsize=14, fontweight='bold')
    ax2.grid(axis='y', alpha=0.3)
    ax2.axvline(x=simulator.params.prep_zone_length,
                color='r', linestyle='--', linewidth=2)

    # Plot 3: Temporal distribution histogram
    ax3 = axes[1, 0]
    ax3.hist(lc_df['t'], bins=40, alpha=0.7, color='green', edgecolor='black')
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Frequency', fontsize=12)
    ax3.set_title('(c) Temporal Distribution', fontsize=14, fontweight='bold')
    ax3.grid(axis='y', alpha=0.3)

    # Plot 4: Cell-wise LC count
    ax4 = axes[1, 1]
    cell_counts = {}
    for x in lc_df['x']:
        cell = int(x / simulator.params.cell_length)
        cell_counts[cell] = cell_counts.get(cell, 0) + 1

    if cell_counts:
        ax4.bar(cell_counts.keys(), cell_counts.values(), alpha=0.7, color='orange', edgecolor='black')
        ax4.set_xlabel('Cell Index')
        ax4.set_ylabel('LC Count')
        ax4.set_title('(d) Cell-wise LC Distribution', fontsize=14, fontweight='bold')
        ax4.grid(axis='y', alpha=0.3)
        ax4.axvline(x=simulator.params.num_cells_prep,
                    color='r', linestyle='--', linewidth=2)
    else:
        ax4.text(0.5, 0.5, 'No LC recorded', va='center', ha='center', transform=ax4.transAxes)

    plt.tight_layout()
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"[PASS] Spatiotemporal plot saved to {output_filename}")
    plt.close()


def export_video(simulator: 'IntegratedZoneSimulator',
                 output_filename: str,
                 fps: int = 10) -> None:
    """Export a simple trajectory animation video (mp4) using vehicle histories.

    Shows vehicles as points moving along x (longitudinal) with lane center offsets (d) on y.

    Args:
        simulator: Integrated zone simulator instance
        output_filename: Output video file path (e.g., outputs/sim_anim.mp4)
        fps: Frames per second for the video (default: 10)
    """
    # Collect histories from both active and completed vehicles
    vehicles = list(simulator.vehicles) + list(simulator.completed_vehicles)
    if not vehicles:
        print("[WARNING] No vehicles to animate")
        return

    # Determine time range
    def max_time(v) -> float:
        ht = getattr(v, 'history_time', [])
        return ht[-1] if ht else 0.0
    t_end = max([max_time(v) for v in vehicles] + [0.0])
    if t_end <= 0.0:
        print("[WARNING] No history_time recorded; skipping video export")
        return

    # Frame times
    dt_frame = 1.0 / max(1, fps)
    t_values = np.arange(0.0, t_end + dt_frame * 0.5, dt_frame)

    # Prepare figure
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(0, simulator.params.total_length)
    # Y range to include all lane centers with margin
    d_vals = list(LANE_OFFSETS.values())
    d_min, d_max = min(d_vals) - 0.75, max(d_vals) + 0.75
    ax.set_ylim(d_min, d_max)
    ax.set_xlabel('Position x (m)')
    ax.set_ylabel('Lateral offset d (m)')
    ax.set_title(f'Weaving Animation ({simulator.mode.upper()})')
    ax.grid(True, alpha=0.2)

    # Draw lane center lines
    for lane_name, d in LANE_OFFSETS.items():
        ax.plot([0, simulator.params.total_length], [d, d], linestyle='--', linewidth=1, alpha=0.4)
        ax.text(2, d + 0.2, lane_name, fontsize=8, alpha=0.6)

    # Scatter handle
    scat = ax.scatter([], [], s=20, c='tab:blue', alpha=0.9)

    # Build quick index for per-vehicle histories
    histories: List[Tuple[List[float], List[float], List[str]]] = []
    for v in vehicles:
        histories.append((getattr(v, 'history_time', []), getattr(v, 'history_x', []), getattr(v, 'history_lane', [])))

    def get_positions_at_time(t: float) -> Tuple[np.ndarray, np.ndarray]:
        xs: List[float] = []
        ds: List[float] = []
        for (ht, hx, hl) in histories:
            if not ht:
                continue
            # Find last index with time <= t
            # Histories are appended in order; use binary search or linear scan fallback
            # Simple linear scan from end for robustness (histories are short per vehicle)
            idx = None
            for i in range(len(ht) - 1, -1, -1):
                if ht[i] <= t:
                    idx = i
                    break
            if idx is None:
                continue
            x = hx[idx]
            lane = hl[idx] if idx < len(hl) else None
            lane_key = lane if lane is not None else 'lcenter'
            d = LANE_OFFSETS.get(lane_key, 0.0)
            # Filter: within scene bounds
            if 0.0 <= x <= simulator.params.total_length:
                xs.append(x)
                ds.append(d)
        return np.array(xs), np.array(ds)

    def init():
        scat.set_offsets(np.empty((0, 2)))
        return (scat,)

    def update(frame_idx: int):
        t = t_values[frame_idx]
        xs, ds = get_positions_at_time(t)
        if len(xs) > 0:
            scat.set_offsets(np.column_stack((xs, ds)))
        else:
            scat.set_offsets(np.empty((0, 2)))
        ax.set_title(f'Weaving Animation ({simulator.mode.upper()}) - t={t:.1f}s')
        return (scat,)

    ani = animation.FuncAnimation(fig, update, frames=len(t_values), init_func=init, blit=True, interval=1000.0 / max(1, fps))

    try:
        writer = animation.FFMpegWriter(fps=fps, bitrate=1800)
        ani.save(output_filename, writer=writer)
        print(f"[PASS] Video saved to {output_filename}")
    except Exception as e:
        # Fallback: save as GIF using PillowWriter
        try:
            gif_out = output_filename.rsplit('.', 1)[0] + '.gif'
            writer = animation.PillowWriter(fps=fps)
            ani.save(gif_out, writer=writer)
            print(f"[WARNING] FFmpeg unavailable, saved GIF instead: {gif_out}")
        except Exception as e2:
            print(f"[ERROR] Failed to export video: {e2}")
    plt.close(fig)
