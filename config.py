from pathlib import Path


# Paths
PROJECT_ROOT = Path(__file__).resolve().parent
DATA_ROOT = PROJECT_ROOT / "data"
PHASE0_ROOT = DATA_ROOT / "phase0"
PHASE1_ROOT = DATA_ROOT / "phase1"
PHASE0_RESULTS_CSV = PHASE0_ROOT / "phase0_results.csv"
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
MOCAP_GROUND_X_SIGN = 1.0
MOCAP_GROUND_Y_SIGN = -1.0


# Measured calibration
YAW_OFFSET_RAD = 2.3810
MARKER_TO_BODY_X_M = -0.07
MARKER_TO_BODY_Y_M = -0.01
POSITIVE_YAW_COMMAND_SIGN = 1.0
POSITIVE_LATERAL_COMMAND_SIGN = 1.0
YAW_CALIBRATION_ROOT = DATA_ROOT / "yaw_calibration"
YAW_CALIBRATION_VX_MPS = 0.25
YAW_CALIBRATION_DURATION_S = 5.0


#Map setting-------------------------------------------
WORKSPACE_X_MIN_M = -2.826
WORKSPACE_X_MAX_M = 1.830
WORKSPACE_Y_MIN_M = -2.492
WORKSPACE_Y_MAX_M = 2.358
ROBOT_RADIUS_M = 0.35
PHASE1_ROBOT_RADIUS_M = 0.35


#Velocity command setting-------------------------------------------
DT_S = 2.0
T_SEGMENTS = 15
VX_MAX_MPS = 0.5
VY_MAX_MPS = 0.2
OMEGA_MAX_RADPS = 0.2
DIAG_SPEED_SUM_MAX_MPS = 0.2
ARC_VX_MAX_MPS = 0.2
ARC_OMEGA_MAX_RADPS = 0.2
TRANSLATIONAL_SPEED_MAX_MPS = None

#Tuning parameters for gain and offset---------------------------------------
#Vx Setting--------
MODEL_GAIN_VX_POS = 1.05     # forward
MODEL_OFFSET_VX_POS_M = 0.0
MODEL_GAIN_VX_NEG = 0.0      # backward set

#Vy seeting--------
MODEL_GAIN_VY_POS = 1.00     # sideways to the robot's left
MODEL_OFFSET_VY_POS_M = 0.0
MODEL_GAIN_VY_NEG = 0.74     # sideways to the robot's right
MODEL_OFFSET_VY_NEG_M = 0.0

#Omega Seeting------------
MODEL_GAIN_OMEGA_POS = 0.70  # turning left (counterclockwise)
MODEL_OFFSET_OMEGA_POS_RAD = 0.0
MODEL_GAIN_OMEGA_NEG = 0.70  # turning right (clockwise)
MODEL_OFFSET_OMEGA_NEG_RAD = 0.0

#Diagnal Velocity Setting------
MODEL_GAIN_DIAG_VX = 1.05
MODEL_OFFSET_DIAG_VX_M = 0.0
MODEL_GAIN_DIAG_VY_POS = 1.00  # diagonal to the robot's left
MODEL_OFFSET_DIAG_VY_POS_M = 0.0
MODEL_GAIN_DIAG_VY_NEG = 0.74  # diagonal to the robot's right
MODEL_OFFSET_DIAG_VY_NEG_M = 0.0

#Arc Seeting
MODEL_GAIN_ARC_VX = 1.05
MODEL_OFFSET_ARC_VX_M = 0.0
MODEL_GAIN_ARC_OMEGA_POS = 0.80  # arcing left
MODEL_OFFSET_ARC_OMEGA_POS_RAD = 0.0
MODEL_GAIN_ARC_OMEGA_NEG = 0.80  # arcing right
MODEL_OFFSET_ARC_OMEGA_NEG_RAD = 0.0


MODEL_TURN_LAG_S = 0.30
MODEL_TURN_LAG_AFTER_WALK_S = 0.10
MODEL_WALK_YAW_RAD_PER_M = 0.05



# Planning ------------------------------------------------------------------
GOAL_RADIUS_M = 1.00
START_REGION_RADIUS_M = 0.50
START_X_M = 0.881
START_Y_M = -1.058
GOAL_X_M = -2.364
GOAL_Y_M = 2.067
RHO_INITIAL_GUESS_M = 0.20
MASTER_RANDOM_SEED = 20261009

PLANNER_ENTRY_POINT = ""
OBSTACLE_COUNT_MIN = 2
OBSTACLE_COUNT_MAX = 4
OBSTACLE_RADIUS_MIN_M = 0.10
OBSTACLE_RADIUS_MAX_M = 0.30
RRT_MAX_ITERATIONS = 10000
PLANNER_TURN_THEN_STRAIGHT = True
PLANNER_LEG_KINDS = ("vy", "vx+vy", "vx+omega")
PLANNER_LEG_SHARES = (0.25, 0.5, 0.75, 1.0)
PLANNER_MIXED_ATTEMPTS = 40
PLANNER_ARC_BEARING_MIN_RAD = 0.17  # about 10 degrees
PLANNER_ARC_BEARING_MAX_RAD = 0.70  # about 40 degrees
RRT_STEP_M = 0.5
RRT_GOAL_SAMPLE_RATE = 0.30
RRT_CONTROL_SAMPLES_PER_EXPANSION = 30
RRT_COLLISION_SAMPLE_SPACING_M = 0.025
RRT_POSITION_WEIGHT = 1.0
RRT_HEADING_WEIGHT = 0.10
# Phase 0 pilot Minimizer
PHASE0_FIT_ITERATIONS = 50000
PILOT_HIGH_QUANTILE = 0.90
RHO_MULTIPLIER = 1.05


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
        "OMEGA_MAX_RADPS": OMEGA_MAX_RADPS,
        "ROBOT_RADIUS_M": ROBOT_RADIUS_M,
        "PHASE1_ROBOT_RADIUS_M": PHASE1_ROBOT_RADIUS_M,
        "GOAL_RADIUS_M": GOAL_RADIUS_M,
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
    # 0 turns that command off: plans then never use it.
    for name in ("VY_MAX_MPS", "DIAG_SPEED_SUM_MAX_MPS", "ARC_VX_MAX_MPS",
                 "ARC_OMEGA_MAX_RADPS"):
        value = globals()[name]
        if not (isinstance(value, (int, float)) and math.isfinite(value) and value >= 0):
            raise ValueError(f"{name} must be finite and zero or positive")
    if not PLANNER_LEG_KINDS or not set(PLANNER_LEG_KINDS) <= {"vx", "vy", "vx+vy", "vx+omega"}:
        raise ValueError('PLANNER_LEG_KINDS must hold some of "vx", "vy", "vx+vy" and "vx+omega"')
    if not PLANNER_LEG_SHARES or not all(0 < share <= 1 for share in PLANNER_LEG_SHARES):
        raise ValueError("PLANNER_LEG_SHARES must hold shares in (0, 1]")
    if not (isinstance(PLANNER_MIXED_ATTEMPTS, int) and PLANNER_MIXED_ATTEMPTS >= 0):
        raise ValueError("PLANNER_MIXED_ATTEMPTS must be a non-negative integer")
    if not 0 < PLANNER_ARC_BEARING_MIN_RAD <= PLANNER_ARC_BEARING_MAX_RAD < math.pi / 2:
        raise ValueError("Need 0 < PLANNER_ARC_BEARING_MIN_RAD <= "
                         "PLANNER_ARC_BEARING_MAX_RAD < pi/2")
    if EDGE_STOP_MARGIN_M is not None and not (
            isinstance(EDGE_STOP_MARGIN_M, (int, float)) and EDGE_STOP_MARGIN_M >= 0):
        raise ValueError("EDGE_STOP_MARGIN_M must be None or a non-negative number")
    for name in ("MODEL_GAIN_VX_POS", "MODEL_GAIN_VY_POS", "MODEL_GAIN_VY_NEG",
                 "MODEL_GAIN_OMEGA_POS", "MODEL_GAIN_OMEGA_NEG",
                 "MODEL_GAIN_DIAG_VX", "MODEL_GAIN_DIAG_VY_POS", "MODEL_GAIN_DIAG_VY_NEG",
                 "MODEL_GAIN_ARC_VX", "MODEL_GAIN_ARC_OMEGA_POS", "MODEL_GAIN_ARC_OMEGA_NEG"):
        value = globals()[name]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    # 0 marks a direction the model does not use.
    if not (isinstance(MODEL_GAIN_VX_NEG, (int, float)) and math.isfinite(MODEL_GAIN_VX_NEG)
            and MODEL_GAIN_VX_NEG >= 0):
        raise ValueError("MODEL_GAIN_VX_NEG must be finite and zero or positive")
    for name in ("MODEL_OFFSET_VX_POS_M", "MODEL_OFFSET_VY_POS_M", "MODEL_OFFSET_VY_NEG_M",
                 "MODEL_OFFSET_OMEGA_POS_RAD", "MODEL_OFFSET_OMEGA_NEG_RAD",
                 "MODEL_OFFSET_DIAG_VX_M", "MODEL_OFFSET_DIAG_VY_POS_M",
                 "MODEL_OFFSET_DIAG_VY_NEG_M", "MODEL_OFFSET_ARC_VX_M",
                 "MODEL_OFFSET_ARC_OMEGA_POS_RAD", "MODEL_OFFSET_ARC_OMEGA_NEG_RAD",
                 "MODEL_WALK_YAW_RAD_PER_M"):
        value = globals()[name]
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    for name in ("MODEL_TURN_LAG_S", "MODEL_TURN_LAG_AFTER_WALK_S"):
        value = globals()[name]
        if not (isinstance(value, (int, float)) and 0 <= value < DT_S):
            raise ValueError(f"{name} must be at least zero and shorter than DT_S")
    safety_positive = {
        "LIVE_TRACKING_TIMEOUT_S": LIVE_TRACKING_TIMEOUT_S,
        "START_POSITION_TOLERANCE_M": START_POSITION_TOLERANCE_M,
        "START_HEADING_TOLERANCE_RAD": START_HEADING_TOLERANCE_RAD,
    }
    for name, value in safety_positive.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not (isinstance(OBSTACLE_COUNT_MIN, int) and isinstance(OBSTACLE_COUNT_MAX, int)
            and 0 <= OBSTACLE_COUNT_MIN <= OBSTACLE_COUNT_MAX):
        raise ValueError("Need integers 0 <= OBSTACLE_COUNT_MIN <= OBSTACLE_COUNT_MAX")
    if not 0 < OBSTACLE_RADIUS_MIN_M <= OBSTACLE_RADIUS_MAX_M:
        raise ValueError("Need 0 < OBSTACLE_RADIUS_MIN_M <= OBSTACLE_RADIUS_MAX_M")
