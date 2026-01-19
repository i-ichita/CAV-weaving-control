"""
CAV Weaving Zone Control System - v11.0
=======================================

Apollo-based Frenet QP Controller with Urgency-based MPC

Modules:
    - vehicle: Vehicle dataclass and related models
    - parameters: IntegratedZoneParameters
    - controllers: Preparation/Weaving/Integrated zone controllers
    - simulator: IntegratedZoneSimulator
    - visualization: Plotting functions
    - utils: Logger and utilities
    - frenet_qp_apollo: Apollo-style Piecewise Jerk QP controller
    - mpc_controller: Urgency-based hierarchical MPC
    - coordinate_transform: Frenet coordinate transformation utilities
    - main: Main simulation entry point

Version: v11.0
Date: 2025-12-16
"""

# Core simulation components
from .vehicle import (
    Vehicle,
    SafetyMetrics,
    SafetyAnalyzer,
    IDMModel,
    ObservedDistributionModel,
    ComputationMetrics,
    # determine_target_lanes は削除
)
from .parameters import IntegratedZoneParameters
from .controllers import (
    IntegratedZoneController,
)
from .simulator import IntegratedZoneSimulator
from .visualization import plot_lc_spatiotemporal_distribution
from .utils import Logger, SCRIPT_NAME, SCRIPT_VERSION, is_adjacent_lane, get_next_adjacent_lane

# Advanced controllers
from .frenet_qp_apollo import FrenetQPController, VehicleState, ObstacleInfo
from .mpc_controller import (
    UrgencyPlanner,
    FrenetQPControllerWithRelaxation,
    compute_lane_density,
    compute_urgency_distribution_metrics
)
from .coordinate_transform import (
    CartesianState,
    FrenetState,
    cartesian_to_frenet,
    frenet_to_cartesian,
    get_lane_offset,
    compute_lateral_distance,
    LANE_WIDTH,
    LANE_OFFSETS
)

__version__ = "11.0.0"
__author__ = "CAV Research Group"
__all__ = [
    # Vehicle and models
    "Vehicle",
    "SafetyMetrics",
    "SafetyAnalyzer",
    "IDMModel",
    "ObservedDistributionModel",
    "ComputationMetrics",
    "is_adjacent_lane",
    "get_next_adjacent_lane",
    # Parameters
    "IntegratedZoneParameters",
    # Controllers
    "IntegratedZoneController",
    # Simulator
    "IntegratedZoneSimulator",
    # Visualization
    "plot_lc_spatiotemporal_distribution",
    # Utils
    "Logger",
    "SCRIPT_NAME",
    "SCRIPT_VERSION",
    # Advanced controllers
    "FrenetQPController",
    "VehicleState",
    "ObstacleInfo",
    "UrgencyPlanner",
    "FrenetQPControllerWithRelaxation",
    "compute_lane_density",
    "compute_urgency_distribution_metrics",
    # Coordinate transformation
    "CartesianState",
    "FrenetState",
    "cartesian_to_frenet",
    "frenet_to_cartesian",
    "get_lane_offset",
    "compute_lateral_distance",
    "LANE_WIDTH",
    "LANE_OFFSETS",
]
