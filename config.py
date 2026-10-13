from pathlib import Path


# Paths
PROJECT_ROOT = Path(__file__).resolve().parent
DATA_ROOT = PROJECT_ROOT / "data"
PHASE0_ROOT = DATA_ROOT / "phase0"
PHASE1_ROOT = DATA_ROOT / "phase1"
PHASE0_RESULTS_CSV = PHASE0_ROOT / "phase0_results.csv"
PROBLEMS_CSV = PHASE1_ROOT / "problems.csv"
NOTES_FILE = PHASE1_ROOT / "notes.txt"
NOVA_DIR = Path.home() / "nova"
NOVA_ENV_FILE = NOVA_DIR / "env.bash"


# Safety
ALLOW_HARDWARE_EXECUTION = True
REQUIRE_OPERATOR_CONFIRMATION = False
TRACKING_START_TIMEOUT_S = 10.0
MOCAP_LEAD_TIME_S = 0.5
MOCAP_TRAIL_TIME_S = 0.5
MAX_TRACKING_GAP_S = 0.20
EDGE_STOP_MARGIN_M = None
LIVE_TRACKING_TIMEOUT_S = 0.50


START_POSITION_TOLERANCE_M = 0.15
START_HEADING_TOLERANCE_RAD = 0.17  # about 10 degrees

# Autonomous repositioning between candidates. This motion is recorded in a
# separate reset folder and is never included in deviation analysis. When
# False, the operator walks the robot back to the start between candidates.
AUTONOMOUS_RESET_ENABLED = False
RESET_POSITION_TOLERANCE_M = 0.10
RESET_HEADING_TOLERANCE_RAD = 0.15
RESET_COMMAND_DT_S = 0.5
RESET_VX_MAX_MPS = 0.15
RESET_VY_MAX_MPS = 0.10
RESET_OMEGA_MAX_RADPS = 0.25
RESET_MAX_ATTEMPTS = 10
RESET_SETTLE_TIME_S = 0.75


# ROS / Unitree Go2
CMD_VEL_TOPIC = "/cmd_vel_out"
STOP_TOPIC = "/webrtc_req"
COMMAND_HZ = 50.0
STOP_REPEATS = 5
STOP_INTERVAL_S = 0.05


# OptiTrack / Motive
MOCAP_CLIENT_IP = "192.168.0.241"
MOCAP_SERVER_IP = "192.168.0.245"
MOCAP_USE_MULTICAST = True
RIGID_BODY_ID = 1006
# Motive is Y-up. Experiment world x=Motive X and y=-Motive Z.
MOCAP_GROUND_X_SIGN = 1.0
MOCAP_GROUND_Y_SIGN = -1.0


# Measured calibration
YAW_OFFSET_RAD = 2.40395144164075
MARKER_TO_BODY_X_M = -0.07
MARKER_TO_BODY_Y_M = -0.01
POSITIVE_YAW_COMMAND_SIGN = 1.0
POSITIVE_LATERAL_COMMAND_SIGN = 1.0
# yaw_offset_calibration.py drives straight forward with this single vx-only
# command and prints the YAW_OFFSET_RAD it measures.
YAW_CALIBRATION_ROOT = DATA_ROOT / "yaw_calibration"
YAW_CALIBRATION_VX_MPS = 0.25
YAW_CALIBRATION_DURATION_S = 5.0


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



DT_S = 0.5
T_SEGMENTS = 40
VX_MAX_MPS = 1.0
VY_MAX_MPS = 1.0
OMEGA_MAX_RADPS = 0.4
# Optional combined translational limit. Set None if the interface has none.
TRANSLATIONAL_SPEED_MAX_MPS = None

MODEL_GAIN_VX_POS = 1.05     # forward
MODEL_GAIN_VX_NEG = 1.00    # backward
MODEL_GAIN_VY_POS = 1.00     # sideways to the robot's left
MODEL_GAIN_VY_NEG = 0.74     # sideways to the robot's right
MODEL_GAIN_OMEGA_POS = 0.82  # turning left (counterclockwise)
MODEL_GAIN_OMEGA_NEG = 0.79  # turning right (clockwise)
# Offsets of the same model, per command segment: modelled motion = gain *
# commanded motion + offset, in the robot's frame (m, or rad for omega). 
MODEL_OFFSET_VX_POS_M = 0.0
MODEL_OFFSET_VY_POS_M = 0.0
MODEL_OFFSET_VY_NEG_M = 0.0
MODEL_OFFSET_OMEGA_POS_RAD = 0.0
MODEL_OFFSET_OMEGA_NEG_RAD = 0.0


# Planning ------------------------------------------------------------------
GOAL_RADIUS_M = 1.00
START_REGION_RADIUS_M = 0.50
# Fixed start and goal centers for every Phase 1 instance (only obstacles are
# random). Each is 1.35 m in from its workspace corner, which leaves room for
# the robot radius plus any viable rho (< GOAL_RADIUS_M). Set START_X_M = None
# to sample start and goal at random instead. Phase 0 pilots use the same goal
# and GOAL_RADIUS_M, but start wherever the robot stands.
START_X_M = 0.881
START_Y_M = -1.058
GOAL_X_M = -2.012
GOAL_Y_M = 1.426
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
OBSTACLE_COUNT_MIN = 2
OBSTACLE_COUNT_MAX = 5
OBSTACLE_RADIUS_MIN_M = 0.10
OBSTACLE_RADIUS_MAX_M = 0.30
RRT_MAX_ITERATIONS = 10000
RRT_GOAL_SAMPLE_RATE = 0.30
RRT_CONTROL_SAMPLES_PER_EXPANSION = 30
RRT_COLLISION_SAMPLE_SPACING_M = 0.025
RRT_POSITION_WEIGHT = 1.0
RRT_HEADING_WEIGHT = 0.10


# Phase 0 pilot Minimizer
PHASE0_FIT_ITERATIONS = 50000
PILOT_HIGH_QUANTILE = 0.90
RHO_MULTIPLIER = 1.05


# Training data for a learned dynamics model ---------------------------------
# collect_training.py drives one random command sequence per call and saves it
# as the next free trial_NNN. These runs only train a model; they are never
# used for Phase 0 calibration or Phase 1.
TRAINING_ROOT = DATA_ROOT / "training"
TRAINING_SEGMENTS = 20
# Trials come in blocks of this many mixed (vx, vy, omega) runs plus one
# vx-only, one vy-only and one omega-only run, in a seeded random order.
# 12 makes 12 of every 15 trials mixed (80 %).
TRAINING_MIXED_PER_BLOCK = 12
TRAINING_RANDOM_SEED = 20261010
# Commands are drawn from the measured start pose so that the nominal path
# stays at least this far inside the workspace. The real path drifts from the
# nominal one, so keep EDGE_STOP_MARGIN_M set as well.
TRAINING_PLAN_MARGIN_M = 0.50


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
    if not (0 < YAW_CALIBRATION_VX_MPS <= VX_MAX_MPS) or YAW_CALIBRATION_DURATION_S <= 0:
        raise ValueError("Yaw calibration needs 0 < vx <= VX_MAX_MPS and a positive duration")
    if WORKSPACE_X_MIN_M >= WORKSPACE_X_MAX_M or WORKSPACE_Y_MIN_M >= WORKSPACE_Y_MAX_M:
        raise ValueError("Workspace minimums must be smaller than maximums")
    if POSITIVE_YAW_COMMAND_SIGN not in (-1.0, 1.0):
        raise ValueError("POSITIVE_YAW_COMMAND_SIGN must be -1.0 or 1.0")
    if POSITIVE_LATERAL_COMMAND_SIGN not in (-1.0, 1.0):
        raise ValueError("POSITIVE_LATERAL_COMMAND_SIGN must be -1.0 or 1.0")
    if not 0.0 < PILOT_HIGH_QUANTILE < 1.0:
        raise ValueError("PILOT_HIGH_QUANTILE must be between zero and one")
    if not (WORKSPACE_X_MIN_M <= GOAL_X_M <= WORKSPACE_X_MAX_M
            and WORKSPACE_Y_MIN_M <= GOAL_Y_M <= WORKSPACE_Y_MAX_M):
        raise ValueError("The goal must be inside the workspace")
    if not isinstance(PHASE0_FIT_ITERATIONS, int) or PHASE0_FIT_ITERATIONS < 1:
        raise ValueError("PHASE0_FIT_ITERATIONS must be a positive integer")
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
    for name in ("MODEL_GAIN_VX_POS", "MODEL_GAIN_VX_NEG", "MODEL_GAIN_VY_POS",
                 "MODEL_GAIN_VY_NEG", "MODEL_GAIN_OMEGA_POS", "MODEL_GAIN_OMEGA_NEG"):
        value = globals()[name]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    for name in ("MODEL_OFFSET_VX_POS_M", "MODEL_OFFSET_VY_POS_M", "MODEL_OFFSET_VY_NEG_M",
                 "MODEL_OFFSET_OMEGA_POS_RAD", "MODEL_OFFSET_OMEGA_NEG_RAD"):
        value = globals()[name]
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    safety_positive = {
        "LIVE_TRACKING_TIMEOUT_S": LIVE_TRACKING_TIMEOUT_S,
        "START_POSITION_TOLERANCE_M": START_POSITION_TOLERANCE_M,
        "START_HEADING_TOLERANCE_RAD": START_HEADING_TOLERANCE_RAD,
    }
    for name, value in safety_positive.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not isinstance(TRAINING_SEGMENTS, int) or TRAINING_SEGMENTS < 1:
        raise ValueError("TRAINING_SEGMENTS must be a positive integer")
    if not isinstance(TRAINING_MIXED_PER_BLOCK, int) or TRAINING_MIXED_PER_BLOCK < 0:
        raise ValueError("TRAINING_MIXED_PER_BLOCK must be a non-negative integer")
    if not (isinstance(TRAINING_PLAN_MARGIN_M, (int, float)) and TRAINING_PLAN_MARGIN_M >= 0
            and 2 * TRAINING_PLAN_MARGIN_M < min(WORKSPACE_X_MAX_M - WORKSPACE_X_MIN_M,
                                                 WORKSPACE_Y_MAX_M - WORKSPACE_Y_MIN_M)):
        raise ValueError("TRAINING_PLAN_MARGIN_M must leave room inside the workspace")
    if START_X_M is not None:
        # Any viable rho is below GOAL_RADIUS_M, so this clearance always suffices.
        clearance = ROBOT_RADIUS_M + GOAL_RADIUS_M
        for name, x, y in (("start", START_X_M, START_Y_M), ("goal", GOAL_X_M, GOAL_Y_M)):
            if not (WORKSPACE_X_MIN_M + clearance <= x <= WORKSPACE_X_MAX_M - clearance
                    and WORKSPACE_Y_MIN_M + clearance <= y <= WORKSPACE_Y_MAX_M - clearance):
                raise ValueError(f"Fixed {name} must be at least {clearance:.2f} m "
                                 "inside the workspace")
