"""Single source of truth for every user-defined experiment parameter.

Edit this file before using either ``phase0.py`` or ``main.py``.  Neither
program accepts command-line overrides for experiment values; the CLI only
selects a safe operation (validate, prepare, summarize, or execute).
"""

from pathlib import Path


# Paths ---------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent
DATA_ROOT = PROJECT_ROOT / "data"
PHASE0_ROOT = DATA_ROOT / "phase0"
PHASE1_ROOT = DATA_ROOT / "phase1"
PHASE0_RESULTS_CSV = PHASE0_ROOT / "phase0_results.csv"
PROBLEMS_CSV = PHASE1_ROOT / "problems.csv"
NOTES_FILE = PHASE1_ROOT / "notes.txt"
NOVA_DIR = Path.home() / "nova"
NOVA_ENV_FILE = NOVA_DIR / "env.bash"


# Safety --------------------------------------------------------------------
# Both this value and --execute must be supplied before any robot command can
# be published.  Leave False until the complete pipeline has been inspected.
ALLOW_HARDWARE_EXECUTION = True
REQUIRE_OPERATOR_CONFIRMATION = False
TRACKING_START_TIMEOUT_S = 10.0
MOCAP_LEAD_TIME_S = 0.5
MOCAP_TRAIL_TIME_S = 0.5
MAX_TRACKING_GAP_S = 0.10

# Live safety stop. While a command sequence runs, the robot is stopped (and
# the run fails) if its body center comes within EDGE_STOP_MARGIN_M of the
# workspace edge, or if mocap has not tracked it for LIVE_TRACKING_TIMEOUT_S.
EDGE_STOP_MARGIN_M = None
LIVE_TRACKING_TIMEOUT_S = 0.50

# Before every Phase 1 candidate the robot must stand this close to the
# planned start pose, checked live with mocap.
START_POSITION_TOLERANCE_M = 0.15
START_HEADING_TOLERANCE_RAD = 0.17  # about 10 degrees

# Autonomous repositioning between candidates. This motion is recorded in a
# separate reset folder and is never included in deviation analysis. When
# False, the operator walks the robot back to the start between candidates.
AUTONOMOUS_RESET_ENABLED = False
RESET_POSITION_TOLERANCE_M = 0.10
RESET_HEADING_TOLERANCE_RAD = 0.20
RESET_COMMAND_DT_S = 2.0
RESET_VX_MAX_MPS = 0.15
RESET_VY_MAX_MPS = 0.10
RESET_OMEGA_MAX_RADPS = 0.25
RESET_MAX_ATTEMPTS = 8
RESET_SETTLE_TIME_S = 0.75


# ROS / Unitree Go2 ---------------------------------------------------------
CMD_VEL_TOPIC = "/cmd_vel_out"
STOP_TOPIC = "/webrtc_req"
COMMAND_HZ = 50.0
STOP_REPEATS = 5
STOP_INTERVAL_S = 0.05


# OptiTrack / Motive --------------------------------------------------------
MOCAP_CLIENT_IP = "192.168.0.241"
MOCAP_SERVER_IP = "192.168.0.245"
MOCAP_USE_MULTICAST = True
RIGID_BODY_ID = 1006
# Motive is Y-up. Experiment world x=Motive X and y=-Motive Z.
MOCAP_GROUND_X_SIGN = 1.0
MOCAP_GROUND_Y_SIGN = -1.0


# Measured calibration (replace after Phase 0) ------------------------------
YAW_OFFSET_RAD = 2.40395144164075
MARKER_TO_BODY_X_M = 0.0
MARKER_TO_BODY_Y_M = 0.0
POSITIVE_YAW_COMMAND_SIGN = 1.0
POSITIVE_LATERAL_COMMAND_SIGN = 1.0


# Workspace in mocap world coordinates (replace after mapping capture area) --
# Largest axis-aligned rectangle inside the four measured mat corners:
# start (2.454, -3.039), (2.231, 2.903), goal (-3.587, 2.776), (-3.362, -2.408).
WORKSPACE_X_MIN_M = -3.362
WORKSPACE_X_MAX_M = 2.231
WORKSPACE_Y_MIN_M = -2.408
WORKSPACE_Y_MAX_M = 2.776

# Conservative planar footprint radius of the robot. Measure from the robot's
# body/control center to its furthest occupied point and add any desired fixed
# mechanical clearance. The planner adds the learned rho separately.
ROBOT_RADIUS_M = 0.30


# Nominal model and command limits ------------------------------------------
DT_S = 2.0
T_SEGMENTS = 15
VX_MAX_MPS = 0.5
VY_MAX_MPS = 0.5
OMEGA_MAX_RADPS = 1.0
# Optional combined translational limit. Set None if the interface has none.
TRANSLATIONAL_SPEED_MAX_MPS = None


# Planning ------------------------------------------------------------------
GOAL_RADIUS_M = 0.50
START_REGION_RADIUS_M = 0.50
# Fixed start and goal centers for every instance (only obstacles are random).
# Each is 0.9 m in from its workspace corner, which leaves room for the robot
# radius plus any viable rho (< GOAL_RADIUS_M). Set START_X_M = None to sample
# start and goal at random instead.
START_X_M = 1.331
START_Y_M = -1.508
GOAL_X_M = -2.462
GOAL_Y_M = 1.876
# None makes the robot start facing the goal.
START_THETA_RAD = None
# PHASE0_RESULTS_CSV supplies the frozen rho used by main.py. This fallback is
# only written into planning templates before the Phase 0 summary exists.
RHO_INITIAL_GUESS_M = 0.20
CANDIDATES_PER_INSTANCE = 5
N_POOL = 100
MASTER_RANDOM_SEED = 20261009
# Uploaded planner entry point. Expected callable contract is documented in
# planner_adapter.py. Example: "my_rrt:create_candidates".
PLANNER_ENTRY_POINT = ""
OBSTACLE_COUNT_MIN = 4
OBSTACLE_COUNT_MAX = 6
OBSTACLE_RADIUS_MIN_M = 0.10
OBSTACLE_RADIUS_MAX_M = 0.30
RRT_MAX_ITERATIONS = 10000
RRT_GOAL_SAMPLE_RATE = 0.20
RRT_CONTROL_SAMPLES_PER_EXPANSION = 30
RRT_COLLISION_SAMPLE_SPACING_M = 0.025
RRT_POSITION_WEIGHT = 1.0
RRT_HEADING_WEIGHT = 0.10


# Phase 0 verification and pilot --------------------------------------------
PIPELINE_TEST_DURATION_S = 5.0
PIPELINE_FORWARD_VX_MPS = 0.25
PIPELINE_ARC_VX_MPS = 0.20
PIPELINE_ARC_OMEGA_RADPS = 0.25
PIPELINE_LATERAL_VY_MPS = 0.15
PILOT_RUNS_VX_ONLY = 3
PILOT_RUNS_VY_ONLY = 3
PILOT_RUNS_OMEGA_ONLY = 3
PILOT_RUNS_MIXED = 6
PILOT_HIGH_QUANTILE = 0.90
RHO_MULTIPLIER = 1.75


# Session metadata ----------------------------------------------------------
SESSION_ID = "S01"
OPERATOR = ""
SURFACE = ""
EXPECTED_MOCAP_RATE_HZ = 120.0


def validate() -> None:
    """Reject unsafe or internally inconsistent configuration."""
    import math

    positive = {
        "DT_S": DT_S,
        "T_SEGMENTS": T_SEGMENTS,
        "VX_MAX_MPS": VX_MAX_MPS,
        "VY_MAX_MPS": VY_MAX_MPS,
        "OMEGA_MAX_RADPS": OMEGA_MAX_RADPS,
        "ROBOT_RADIUS_M": ROBOT_RADIUS_M,
        "GOAL_RADIUS_M": GOAL_RADIUS_M,
        "CANDIDATES_PER_INSTANCE": CANDIDATES_PER_INSTANCE,
        "N_POOL": N_POOL,
        "COMMAND_HZ": COMMAND_HZ,
    }
    for name, value in positive.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if WORKSPACE_X_MIN_M >= WORKSPACE_X_MAX_M or WORKSPACE_Y_MIN_M >= WORKSPACE_Y_MAX_M:
        raise ValueError("Workspace minimums must be smaller than maximums")
    if POSITIVE_YAW_COMMAND_SIGN not in (-1.0, 1.0):
        raise ValueError("POSITIVE_YAW_COMMAND_SIGN must be -1.0 or 1.0")
    if POSITIVE_LATERAL_COMMAND_SIGN not in (-1.0, 1.0):
        raise ValueError("POSITIVE_LATERAL_COMMAND_SIGN must be -1.0 or 1.0")
    if not 0.0 < PILOT_HIGH_QUANTILE < 1.0:
        raise ValueError("PILOT_HIGH_QUANTILE must be between zero and one")
    if PILOT_RUNS_VX_ONLY + PILOT_RUNS_VY_ONLY + PILOT_RUNS_OMEGA_ONLY + PILOT_RUNS_MIXED < 1:
        raise ValueError("At least one pilot run is required")
    reset_positive = {
        "RESET_POSITION_TOLERANCE_M": RESET_POSITION_TOLERANCE_M,
        "RESET_HEADING_TOLERANCE_RAD": RESET_HEADING_TOLERANCE_RAD,
        "RESET_COMMAND_DT_S": RESET_COMMAND_DT_S,
        "RESET_VX_MAX_MPS": RESET_VX_MAX_MPS,
        "RESET_VY_MAX_MPS": RESET_VY_MAX_MPS,
        "RESET_OMEGA_MAX_RADPS": RESET_OMEGA_MAX_RADPS,
        "RESET_MAX_ATTEMPTS": RESET_MAX_ATTEMPTS,
    }
    for name, value in reset_positive.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if RESET_VX_MAX_MPS > VX_MAX_MPS or RESET_VY_MAX_MPS > VY_MAX_MPS:
        raise ValueError("Reset translation limits cannot exceed experiment limits")
    if RESET_OMEGA_MAX_RADPS > OMEGA_MAX_RADPS:
        raise ValueError("Reset yaw limit cannot exceed experiment limit")
    if EDGE_STOP_MARGIN_M is not None and not (
            isinstance(EDGE_STOP_MARGIN_M, (int, float)) and EDGE_STOP_MARGIN_M >= 0):
        raise ValueError("EDGE_STOP_MARGIN_M must be None or a non-negative number")
    safety_positive = {
        "LIVE_TRACKING_TIMEOUT_S": LIVE_TRACKING_TIMEOUT_S,
        "START_POSITION_TOLERANCE_M": START_POSITION_TOLERANCE_M,
        "START_HEADING_TOLERANCE_RAD": START_HEADING_TOLERANCE_RAD,
    }
    for name, value in safety_positive.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if START_X_M is not None:
        # Any viable rho is below GOAL_RADIUS_M, so this clearance always suffices.
        clearance = ROBOT_RADIUS_M + GOAL_RADIUS_M
        for name, x, y in (("start", START_X_M, START_Y_M), ("goal", GOAL_X_M, GOAL_Y_M)):
            if not (WORKSPACE_X_MIN_M + clearance <= x <= WORKSPACE_X_MAX_M - clearance
                    and WORKSPACE_Y_MIN_M + clearance <= y <= WORKSPACE_Y_MAX_M - clearance):
                raise ValueError(f"Fixed {name} must be at least {clearance:.2f} m "
                                 "inside the workspace")
