"""Full-holonomic kinodynamic RRT used by the collection protocol.

The implementation retains the uploaded homework RRT's seeded sampling,
nearest-node expansion, parent-chain path recovery, and conservative sampled
collision checks.  It intentionally does not retain its online lidar/replanning
loop or unicycle converter because the hardware protocol uses static virtual
obstacles and controls (vx, vy, omega) held for a fixed DT_S.
"""

from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass

import config
from core import Command, nominal_rollout, nominal_step, turn_lag, wrap_angle


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Problem:
    instance: int
    start_x: float
    start_y: float
    start_theta: float
    goal_x: float
    goal_y: float
    obstacles: tuple[Circle, ...]
    problem_seed: int


@dataclass
class Node:
    state: tuple[float, float, float]
    parent: "Node | None"
    command: Command | None
    depth: int


def _inside_workspace(x: float, y: float, clearance: float = 0.0) -> bool:
    return (
        config.WORKSPACE_X_MIN_M + clearance <= x <= config.WORKSPACE_X_MAX_M - clearance
        and config.WORKSPACE_Y_MIN_M + clearance <= y <= config.WORKSPACE_Y_MAX_M - clearance
    )


def _point_is_free(x: float, y: float, obstacles: tuple[Circle, ...],
                   clearance: float, edge_clearance: float) -> bool:
    if not _inside_workspace(x, y, edge_clearance):
        return False
    return all(math.hypot(x-obstacle.x, y-obstacle.y)
               > obstacle.radius + clearance
               for obstacle in obstacles)


def _command_is_free(start, vx: float, vy: float, omega: float,
                     obstacles: tuple[Circle, ...],
                     clearance: float, edge_clearance: float,
                     previous: Command | None) -> bool:
    """Sample the exact curved nominal trajectory, not its endpoint chord.

    `previous` is the command before this one (None: the robot stands still).
    """
    path_length_bound = math.hypot(vx, vy) * config.DT_S
    samples = max(1, math.ceil(path_length_bound / config.RRT_COLLISION_SAMPLE_SPACING_M))
    for index in range(samples + 1):
        elapsed = config.DT_S * index / samples
        x, y, _ = nominal_step(*start, Command(0, vx, vy, omega, config.DT_S), config, elapsed,
                               previous)
        if not _point_is_free(x, y, obstacles, clearance, edge_clearance):
            return False
    return True


def _path_is_free(start, commands: list[Command], previous: Command | None,
                  obstacles: tuple[Circle, ...], clearance: float,
                  edge_clearance: float) -> bool:
    """True if the nominal trajectory of `commands`, run from `start`, stays free."""
    state = start
    for command in commands:
        if not _command_is_free(state, command.vx, command.vy, command.omega,
                                obstacles, clearance, edge_clearance, previous):
            return False
        state = nominal_step(*state, command, config, previous=previous)
        previous = command
    return True


def _leg_kinds() -> tuple[str, ...]:
    """PLANNER_LEG_KINDS without those a zero speed limit turns off."""
    available = {
        "vx": True,
        "vy": config.VY_MAX_MPS > 0,
        "vx+vy": config.VY_MAX_MPS > 0 and config.DIAG_SPEED_SUM_MAX_MPS > 0,
        "vx+omega": config.ARC_VX_MAX_MPS > 0 and config.ARC_OMEGA_MAX_RADPS > 0,
    }
    return tuple(kind for kind in config.PLANNER_LEG_KINDS if available[kind])


def _sample_obstacles(rng: random.Random, start, goal, goal_radius: float,
                      rho: float, robot_radius: float) -> tuple[Circle, ...] | None:
    """Draw one obstacle layout clear of the start and goal regions, or None if it does not fit."""
    count = rng.randint(config.OBSTACLE_COUNT_MIN, config.OBSTACLE_COUNT_MAX)
    obstacles: list[Circle] = []
    for _ in range(count):
        for _ in range(1000):
            radius = rng.uniform(config.OBSTACLE_RADIUS_MIN_M,
                                 config.OBSTACLE_RADIUS_MAX_M)
            x = rng.uniform(config.WORKSPACE_X_MIN_M + radius,
                            config.WORKSPACE_X_MAX_M - radius)
            y = rng.uniform(config.WORKSPACE_Y_MIN_M + radius,
                            config.WORKSPACE_Y_MAX_M - radius)
            protected_radius = radius + robot_radius + rho
            protects_start = math.hypot(x-start[0], y-start[1]) <= protected_radius + config.START_REGION_RADIUS_M
            protects_goal = math.hypot(x-goal[0], y-goal[1]) <= protected_radius + goal_radius
            overlaps = any(math.hypot(x-o.x, y-o.y) <= radius + o.radius for o in obstacles)
            if not (protects_start or protects_goal or overlaps):
                obstacles.append(Circle(x, y, radius))
                break
        else:
            return None
    return tuple(obstacles)


def sample_pilot_problem(number: int, seed: int,
                         start: tuple[float, float, float]) -> Problem:
    """Phase 0 problem: the measured start pose, the configured goal, random obstacles.

    rho is not known in Phase 0, so obstacles keep clear of the regions by the
    robot radius only.
    """
    return _sample_from_pose(number, seed, start, 0.0, config.ROBOT_RADIUS_M)


def sample_trial_problem(number: int, seed: int, start: tuple[float, float, float],
                         rho: float) -> Problem:
    """Phase 1 problem: like a pilot, but obstacles keep clear of the start and
    goal regions by PHASE1_ROBOT_RADIUS_M + rho."""
    return _sample_from_pose(number, seed, start, rho, config.PHASE1_ROBOT_RADIUS_M)


def _sample_from_pose(number: int, seed: int, start: tuple[float, float, float],
                      rho: float, robot_radius: float) -> Problem:
    rng = random.Random(seed)
    goal = (config.GOAL_X_M, config.GOAL_Y_M)
    for _ in range(10000):
        obstacles = _sample_obstacles(rng, start, goal, config.GOAL_RADIUS_M, rho,
                                      robot_radius)
        if obstacles is not None:
            return Problem(number, *start, *goal, obstacles, seed)
    raise RuntimeError("Could not sample a valid planning problem")


def _distance(state, target) -> float:
    position = math.hypot(state[0]-target[0], state[1]-target[1])
    heading = abs(wrap_angle(state[2]-target[2]))
    return config.RRT_POSITION_WEIGHT*position + config.RRT_HEADING_WEIGHT*heading


def _sample_target(rng: random.Random, problem: Problem):
    if rng.random() < config.RRT_GOAL_SAMPLE_RATE:
        return problem.goal_x, problem.goal_y, rng.uniform(-math.pi, math.pi)
    return (
        rng.uniform(config.WORKSPACE_X_MIN_M, config.WORKSPACE_X_MAX_M),
        rng.uniform(config.WORKSPACE_Y_MIN_M, config.WORKSPACE_Y_MAX_M),
        rng.uniform(-math.pi, math.pi),
    )


def _sample_control(rng: random.Random) -> tuple[float, float, float]:
    # Include zero and limit values regularly while preserving continuous variety.
    def component(limit: float, lowest: float) -> float:
        choices = [value for value in (-limit, -0.5*limit, 0.0, 0.5*limit, limit)
                   if value >= lowest]
        return rng.choice(choices) if rng.random() < 0.65 else rng.uniform(lowest, limit)
    # Every segment is one allowed command: omega alone, or one of the leg
    # kinds. vx is never negative: the robot only walks forward and turns to go back.
    kinds = ("omega", *_leg_kinds())
    for _ in range(100):
        kind = rng.choice(kinds)
        if kind == "omega":
            control = 0.0, 0.0, component(config.OMEGA_MAX_RADPS, -config.OMEGA_MAX_RADPS)
        elif kind == "vx":
            control = component(config.VX_MAX_MPS, 0.0), 0.0, 0.0
        elif kind == "vy":
            control = 0.0, component(config.VY_MAX_MPS, -config.VY_MAX_MPS), 0.0
        elif kind == "vx+vy":
            vx = rng.uniform(0.0, config.DIAG_SPEED_SUM_MAX_MPS)
            vy = rng.uniform(-1.0, 1.0) * min(config.VY_MAX_MPS,
                                              config.DIAG_SPEED_SUM_MAX_MPS - vx)
            control = vx, vy, 0.0
        else:
            control = (component(config.ARC_VX_MAX_MPS, 0.0), 0.0,
                       component(config.ARC_OMEGA_MAX_RADPS, -config.ARC_OMEGA_MAX_RADPS))
        limit = config.TRANSLATIONAL_SPEED_MAX_MPS
        if limit is None or math.hypot(control[0], control[1]) <= limit:
            return control
    return 0.0, 0.0, 0.0


def _extract(node: Node) -> list[Command]:
    reverse = []
    current = node
    while current.parent is not None:
        reverse.append(current.command)
        current = current.parent
    reverse.reverse()
    commands = [Command(index, c.vx, c.vy, c.omega, config.DT_S)
                for index, c in enumerate(reverse)]
    while len(commands) < config.T_SEGMENTS:
        commands.append(Command(len(commands), 0.0, 0.0, 0.0, config.DT_S))
    return commands


def plan_candidate(problem: Problem, seed: int, rho: float) -> list[Command] | None:
    """Phase 1 plan, or None: obstacles are kept PHASE1_ROBOT_RADIUS_M + rho away
    and the goal radius shrinks by rho. As in Phase 0, the body center may reach
    the workspace edge."""
    if not 0.0 < rho < config.GOAL_RADIUS_M:
        raise ValueError("rho must satisfy 0 < rho < GOAL_RADIUS_M")
    clearance = config.PHASE1_ROBOT_RADIUS_M + rho
    planner = _plan_turn_then_straight if config.PLANNER_TURN_THEN_STRAIGHT else _plan
    return planner(problem, seed, config.GOAL_RADIUS_M - rho, clearance, 0.0)


def plan_pilot(problem: Problem, seed: int) -> list[Command] | None:
    """Phase 0 plan, or None: no rho yet, and the body center may reach the workspace edge."""
    planner = _plan_turn_then_straight if config.PLANNER_TURN_THEN_STRAIGHT else _plan
    return planner(problem, seed, config.GOAL_RADIUS_M, config.ROBOT_RADIUS_M, 0.0)


def _line_is_free(a, b, obstacles, clearance: float, edge_clearance: float) -> bool:
    samples = max(1, math.ceil(math.dist(a, b) / config.RRT_COLLISION_SAMPLE_SPACING_M))
    return all(_point_is_free(a[0] + (b[0]-a[0])*i/samples, a[1] + (b[1]-a[1])*i/samples,
                              obstacles, clearance, edge_clearance)
               for i in range(samples + 1))


def _waypoints(problem: Problem, rng: random.Random, goal_radius: float,
               clearance: float, edge_clearance: float) -> list[tuple[float, float]] | None:
    """RRT over positions from the start to the goal region, shortcut to few corners."""
    def free(a, b):
        return _line_is_free(a, b, problem.obstacles, clearance, edge_clearance)
    start, goal = (problem.start_x, problem.start_y), (problem.goal_x, problem.goal_y)
    points, parents = [start], [None]
    for _ in range(config.RRT_MAX_ITERATIONS):
        target = (goal if rng.random() < config.RRT_GOAL_SAMPLE_RATE else
                  (rng.uniform(config.WORKSPACE_X_MIN_M, config.WORKSPACE_X_MAX_M),
                   rng.uniform(config.WORKSPACE_Y_MIN_M, config.WORKSPACE_Y_MAX_M)))
        nearest = min(range(len(points)), key=lambda i: math.dist(points[i], target))
        distance = math.dist(points[nearest], target)
        if distance < 1e-9:
            continue
        fraction = min(1.0, config.RRT_STEP_M / distance)
        new = (points[nearest][0] + (target[0]-points[nearest][0])*fraction,
               points[nearest][1] + (target[1]-points[nearest][1])*fraction)
        if not free(points[nearest], new):
            continue
        points.append(new)
        parents.append(nearest)
        if math.dist(new, goal) <= goal_radius:
            path, index = [], len(points) - 1
            while index is not None:
                path.append(points[index])
                index = parents[index]
            path.reverse()
            # Walk on to the goal center when nothing is in the way.
            if free(path[-1], goal):
                path.append(goal)
            corners, current = [path[0]], 0
            while current < len(path) - 1:
                current = next(j for j in range(len(path) - 1, current, -1)
                               if free(path[current], path[j]))
                corners.append(path[current])
            return corners
    return None


def _equal_steps(amount: float, rate_limit: float, fraction: float,
                 gain: float, offset: float, dt: float,
                 lag: float = 0.0) -> tuple[int, float] | None:
    """(segments, command) whose modelled motion covers `amount` > 0 in equal segments.

    The model moves at gain*command + offset/dt for all but the first `lag`
    seconds of the segments, so the command is solved from it; the commanded
    rate stays within (0, rate_limit].
    """
    per_segment = gain*fraction*rate_limit*dt + offset
    first = max(1, math.ceil(amount / per_segment)) if per_segment > 0 else 1
    for count in range(first, first + 100):
        rate = (amount/(count*dt - lag) - offset/dt) / gain
        if 0.0 < rate <= rate_limit:
            return count, rate
    return None


def _diagonal_steps(distance: float, share: float, side: float, fraction: float,
                    dt: float) -> tuple[int, float, float, float] | None:
    """(segments, vx, vy, direction) of a diagonal walk covering `distance` > 0 in equal segments.

    vx is `share` of vx + |vy|, vy goes to `side` (+1 left, -1 right), and the
    sum stays within (0, DIAG_SPEED_SUM_MAX_MPS]. `direction` is the angle of
    the modelled motion from the robot's heading.
    """
    cfg = config
    if side > 0:
        gain_y, offset_y = cfg.MODEL_GAIN_DIAG_VY_POS, cfg.MODEL_OFFSET_DIAG_VY_POS_M
    else:
        gain_y, offset_y = cfg.MODEL_GAIN_DIAG_VY_NEG, cfg.MODEL_OFFSET_DIAG_VY_NEG_M
    # Modelled motion per segment at speed sum s: (ax*s + bx, ay*s + by).
    ax, bx = cfg.MODEL_GAIN_DIAG_VX*share*dt, cfg.MODEL_OFFSET_DIAG_VX_M
    ay, by = side*gain_y*(1.0 - share)*dt, offset_y
    limit = cfg.DIAG_SPEED_SUM_MAX_MPS
    per_segment = math.hypot(ax*fraction*limit + bx, ay*fraction*limit + by)
    first = max(1, math.ceil(distance / per_segment)) if per_segment > 0 else 1
    for count in range(first, first + 100):
        # Solve |(ax*s + bx, ay*s + by)| = distance/count for the larger root s.
        a, b = ax*ax + ay*ay, ax*bx + ay*by
        c = bx*bx + by*by - (distance/count)**2
        if b*b - a*c < 0:
            continue
        s = (-b + math.sqrt(b*b - a*c)) / a
        if (0.0 < s <= limit and share*s <= cfg.VX_MAX_MPS
                and (1.0 - share)*s <= cfg.VY_MAX_MPS):
            return count, share*s, side*(1.0 - share)*s, math.atan2(ay*s + by, ax*s + bx)
    return None


def _arc_steps(length: float, turn: float, fraction: float,
               dt: float) -> tuple[int, float, float] | None:
    """(segments, vx, omega) of an arc whose modelled motion covers `length` > 0
    while turning by `turn` != 0, in equal segments within the arc limits."""
    cfg = config
    sign = 1.0 if turn > 0 else -1.0
    if turn > 0:
        gain_w, offset_w = cfg.MODEL_GAIN_ARC_OMEGA_POS, cfg.MODEL_OFFSET_ARC_OMEGA_POS_RAD
    else:
        gain_w, offset_w = cfg.MODEL_GAIN_ARC_OMEGA_NEG, -cfg.MODEL_OFFSET_ARC_OMEGA_NEG_RAD
    gain_x, offset_x = cfg.MODEL_GAIN_ARC_VX, cfg.MODEL_OFFSET_ARC_VX_M
    per_x = gain_x*fraction*cfg.ARC_VX_MAX_MPS*dt + offset_x
    per_w = gain_w*fraction*cfg.ARC_OMEGA_MAX_RADPS*dt + offset_w
    first = max(1, math.ceil(length / per_x) if per_x > 0 else 1,
                math.ceil(abs(turn) / per_w) if per_w > 0 else 1)
    for count in range(first, first + 100):
        vx = (length/(count*dt) - offset_x/dt) / gain_x
        omega = (abs(turn)/(count*dt) - offset_w/dt) / gain_w
        if 0.0 < vx <= cfg.ARC_VX_MAX_MPS and 0.0 < omega <= cfg.ARC_OMEGA_MAX_RADPS:
            return count, vx, sign*omega
    return None


def _turn_in_place(angle: float, previous: Command | None, fraction: float,
                   dt: float) -> list[tuple[float, float, float]] | None:
    """Equal in-place turn commands whose modelled turning covers `angle`; None if none fit."""
    cfg = config
    if abs(angle) <= 1e-6:
        return []
    if angle > 0:
        gain, offset, sign = cfg.MODEL_GAIN_OMEGA_POS, cfg.MODEL_OFFSET_OMEGA_POS_RAD, 1.0
    else:
        gain, offset, sign = cfg.MODEL_GAIN_OMEGA_NEG, -cfg.MODEL_OFFSET_OMEGA_NEG_RAD, -1.0
    lag = turn_lag(Command(0, 0.0, 0.0, sign, dt), previous, cfg)
    steps = _equal_steps(abs(angle), cfg.OMEGA_MAX_RADPS, fraction, gain, offset, dt, lag)
    if steps is None:
        return None
    count, rate = steps
    return [(0.0, 0.0, sign*rate)] * count


def _leg(kind: str, state, corner, previous: Command | None, rng: random.Random,
         full_speed: bool, first_k: int) -> list[Command] | None:
    """Commands that turn in place, then reach `corner` with a leg of `kind`.

    Every motion is solved through the nominal model, so the modelled robot
    ends at the corner. Commands are numbered from `first_k`. None if the
    speeds do not fit the limits.
    """
    cfg, dt = config, config.DT_S
    turn_fraction = 1.0 if full_speed else rng.uniform(0.5, 1.0)
    walk_fraction = 1.0 if full_speed else rng.uniform(0.5, 1.0)
    distance = math.dist(state[:2], corner)
    if distance <= 1e-6:
        return []
    bearing = math.atan2(corner[1]-state[1], corner[0]-state[0])
    # Sideways, diagonal and arc legs go to the side the corner is on: the smaller turn.
    side = 1.0 if wrap_angle(bearing - state[2]) >= 0 else -1.0
    if kind == "vx":
        steps = _equal_steps(distance, cfg.VX_MAX_MPS, walk_fraction, cfg.MODEL_GAIN_VX_POS,
                             cfg.MODEL_OFFSET_VX_POS_M, dt)
        if steps is None:
            return None
        count, rate = steps
        # Walking curves left by MODEL_WALK_YAW_RAD_PER_M, so the turn stops
        # short by half the leg's yaw and the curved walk ends at the corner.
        facing = bearing - cfg.MODEL_WALK_YAW_RAD_PER_M * count * rate * dt / 2
        motion = [(rate, 0.0, 0.0)] * count
    elif kind == "vy":
        if side > 0:
            gain, offset = cfg.MODEL_GAIN_VY_POS, cfg.MODEL_OFFSET_VY_POS_M
        else:
            gain, offset = cfg.MODEL_GAIN_VY_NEG, -cfg.MODEL_OFFSET_VY_NEG_M
        steps = _equal_steps(distance, cfg.VY_MAX_MPS, walk_fraction, gain, offset, dt)
        if steps is None:
            return None
        count, rate = steps
        facing = bearing - side * math.pi / 2
        motion = [(0.0, side * rate, 0.0)] * count
    elif kind == "vx+vy":
        steps = _diagonal_steps(distance, rng.uniform(0.3, 0.7), side, walk_fraction, dt)
        if steps is None:
            return None
        count, vx, vy, direction = steps
        facing = bearing - direction
        motion = [(vx, vy, 0.0)] * count
    else:
        # Put the corner `alpha` to the side, then arc to it: under the model the
        # arc is the circle tangent to the heading through the corner, turning
        # by 2*alpha over a length of distance*alpha/sin(alpha).
        alpha = side * rng.uniform(cfg.PLANNER_ARC_BEARING_MIN_RAD,
                                   cfg.PLANNER_ARC_BEARING_MAX_RAD)
        steps = _arc_steps(distance * abs(alpha) / math.sin(abs(alpha)), 2 * alpha,
                           walk_fraction, dt)
        if steps is None:
            return None
        count, vx, omega = steps
        facing = bearing - alpha
        motion = [(vx, 0.0, omega)] * count
    turn = _turn_in_place(wrap_angle(facing - state[2]), previous, turn_fraction, dt)
    if turn is None:
        return None
    return [Command(first_k + index, *control, dt)
            for index, control in enumerate(turn + motion)]


def _commands_along(corners, start_theta: float, rng: random.Random, full_speed: bool,
                    kinds: tuple[str, ...], leg_is_free) -> list[Command] | None:
    """Reach each corner in turn, starting each leg with a kind drawn from `kinds`.

    A "vy", "vx+vy" or "vx+omega" start covers one of PLANNER_LEG_SHARES of
    the way, and a "vx" leg (turn to face the corner, walk straight) the rest.
    A start that does not fit the limits, or that `leg_is_free(state, leg,
    previous)` rejects, is left out; "vx" has no start.
    """
    state = (*corners[0], start_theta)
    commands: list[Command] = []

    def add(leg):
        nonlocal state
        previous = commands[-1] if commands else None
        for command in leg:
            state = nominal_step(*state, command, config, previous=previous)
            previous = command
        commands.extend(leg)

    for corner in corners[1:]:
        kind = rng.choice(kinds)
        if kind != "vx":
            share = rng.choice(config.PLANNER_LEG_SHARES)
            target = (state[0] + share*(corner[0]-state[0]), state[1] + share*(corner[1]-state[1]))
            previous = commands[-1] if commands else None
            leg = _leg(kind, state, target, previous, rng, full_speed, len(commands))
            if leg is not None and leg_is_free(state, leg, previous):
                add(leg)
        leg = _leg("vx", state, corner, commands[-1] if commands else None, rng, full_speed,
                   len(commands))
        if leg is None:
            return None
        add(leg)
    return commands


def _plan_turn_then_straight(problem: Problem, seed: int, goal_radius: float,
                             clearance: float, edge_clearance: float) -> list[Command] | None:
    """Polyline plan: per leg, turn in place, then walk or arc to the next corner;
    None if it does not fit."""
    rng = random.Random(seed)
    corners = _waypoints(problem, rng, goal_radius, clearance, edge_clearance)
    if corners is None:
        return None

    def leg_is_free(state, leg, previous):
        return _path_is_free(state, leg, previous, problem.obstacles, clearance, edge_clearance)

    start = (problem.start_x, problem.start_y, problem.start_theta)
    kinds = _leg_kinds() or ("vx",)
    # Random leg kinds, shares and speeds vary the data; forward-only legs,
    # then at full speed, are the fallbacks for long paths.
    tries = ([(kinds, attempt % 2 == 1) for attempt in range(config.PLANNER_MIXED_ATTEMPTS)]
             + [(("vx",), False), (("vx",), True)])
    for leg_kinds, full_speed in tries:
        commands = _commands_along(corners, problem.start_theta, rng, full_speed, leg_kinds,
                                   leg_is_free)
        if (commands is None or len(commands) > config.T_SEGMENTS
                or not leg_is_free(start, commands, None)):
            continue
        end = nominal_rollout(commands, start, config)[-1]
        if math.hypot(end[0]-problem.goal_x, end[1]-problem.goal_y) <= goal_radius:
            break
    else:
        return None
    while len(commands) < config.T_SEGMENTS:
        commands.append(Command(len(commands), 0.0, 0.0, 0.0, config.DT_S))
    return commands


def _plan(problem: Problem, seed: int, goal_radius: float,
          clearance: float, edge_clearance: float) -> list[Command] | None:
    rng = random.Random(seed)
    root = Node((problem.start_x, problem.start_y, problem.start_theta), None, None, 0)
    nodes = [root]
    for _ in range(config.RRT_MAX_ITERATIONS):
        target = _sample_target(rng, problem)
        expandable = [node for node in nodes if node.depth < config.T_SEGMENTS]
        if not expandable:
            break
        nearest = min(expandable, key=lambda node: _distance(node.state, target))
        best = None
        for _ in range(config.RRT_CONTROL_SAMPLES_PER_EXPANSION):
            vx, vy, omega = _sample_control(rng)
            nxt = nominal_step(*nearest.state, Command(0, vx, vy, omega, config.DT_S), config,
                               previous=nearest.command)
            if not _command_is_free(nearest.state, vx, vy, omega,
                                    problem.obstacles, clearance, edge_clearance,
                                    nearest.command):
                continue
            score = _distance(nxt, target)
            if best is None or score < best[0]:
                best = (score, nxt, vx, vy, omega)
        if best is None:
            continue
        _, nxt, vx, vy, omega = best
        command = Command(nearest.depth, vx, vy, omega, config.DT_S)
        node = Node(nxt, nearest, command, nearest.depth + 1)
        nodes.append(node)
        if math.hypot(nxt[0]-problem.goal_x, nxt[1]-problem.goal_y) <= goal_radius:
            return _extract(node)
    return None


def obstacles_json(problem: Problem) -> str:
    return json.dumps([{"x": o.x, "y": o.y, "radius": o.radius}
                       for o in problem.obstacles], separators=(",", ":"))
