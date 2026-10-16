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
from core import Command, nominal_step, turn_lag, wrap_angle


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
    # Each segment either turns in place (omega only) or translates without
    # turning (vx, vy only), never both.
    if rng.random() < 0.5:
        return 0.0, 0.0, component(config.OMEGA_MAX_RADPS, -config.OMEGA_MAX_RADPS)
    for _ in range(100):
        # vx is never negative: the robot only walks forward and turns to go back.
        vx = component(config.VX_MAX_MPS, 0.0)
        vy = component(config.VY_MAX_MPS, -config.VY_MAX_MPS)
        limit = config.TRANSLATIONAL_SPEED_MAX_MPS
        if limit is None or math.hypot(vx, vy) <= limit:
            return vx, vy, 0.0
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
    """Phase 1 plan, or None: obstacles and the workspace edge are kept
    PHASE1_ROBOT_RADIUS_M + rho away, and the goal radius shrinks by rho."""
    if not 0.0 < rho < config.GOAL_RADIUS_M:
        raise ValueError("rho must satisfy 0 < rho < GOAL_RADIUS_M")
    clearance = config.PHASE1_ROBOT_RADIUS_M + rho
    planner = _plan_turn_then_straight if config.PLANNER_TURN_THEN_STRAIGHT else _plan
    return planner(problem, seed, config.GOAL_RADIUS_M - rho, clearance, clearance)


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


def _commands_along(corners, start_theta: float, rng: random.Random,
                    full_speed: bool) -> list[Command] | None:
    """Turn in place to face each corner, then walk straight to it.

    Walking curves left by MODEL_WALK_YAW_RAD_PER_M, so each turn stops short
    by half the leg's yaw and the curved walk ends at the corner.
    """
    cfg, dt = config, config.DT_S
    state = (*corners[0], start_theta)
    commands: list[Command] = []
    for corner in corners[1:]:
        turn_fraction = 1.0 if full_speed else rng.uniform(0.5, 1.0)
        walk_fraction = 1.0 if full_speed else rng.uniform(0.5, 1.0)
        distance = math.dist(state[:2], corner)
        walk = (_equal_steps(distance, cfg.VX_MAX_MPS, walk_fraction, cfg.MODEL_GAIN_VX_POS,
                             cfg.MODEL_OFFSET_VX_POS_M, dt)
                if distance > 1e-6 else (0, 0.0))
        if walk is None:
            return None
        walk_yaw = cfg.MODEL_WALK_YAW_RAD_PER_M * walk[0] * walk[1] * dt
        turn = wrap_angle(math.atan2(corner[1]-state[1], corner[0]-state[0])
                          - state[2] - walk_yaw/2)
        legs = []
        if abs(turn) > 1e-6:
            if turn > 0:
                gain, offset, sign = cfg.MODEL_GAIN_OMEGA_POS, cfg.MODEL_OFFSET_OMEGA_POS_RAD, 1.0
            else:
                gain, offset, sign = cfg.MODEL_GAIN_OMEGA_NEG, -cfg.MODEL_OFFSET_OMEGA_NEG_RAD, -1.0
            lag = turn_lag(Command(0, 0.0, 0.0, sign, dt), commands[-1] if commands else None,
                           cfg)
            steps = _equal_steps(abs(turn), cfg.OMEGA_MAX_RADPS, turn_fraction, gain, offset, dt,
                                 lag)
            if steps is None:
                return None
            legs.append(("omega", sign, steps))
        legs.append(("vx", 1.0, walk))
        for axis, sign, (count, rate) in legs:
            for _ in range(count):
                command = Command(len(commands), sign*rate if axis == "vx" else 0.0, 0.0,
                                  sign*rate if axis == "omega" else 0.0, dt)
                state = nominal_step(*state, command, cfg,
                                     previous=commands[-1] if commands else None)
                commands.append(command)
    return commands


def _plan_turn_then_straight(problem: Problem, seed: int, goal_radius: float,
                             clearance: float, edge_clearance: float) -> list[Command] | None:
    """Polyline plan: turn in place, walk straight, repeat; None if it does not fit."""
    rng = random.Random(seed)
    corners = _waypoints(problem, rng, goal_radius, clearance, edge_clearance)
    if corners is None:
        return None
    # Random speeds per leg vary the data; full speed is the fallback for long paths.
    for full_speed in (False, True):
        commands = _commands_along(corners, problem.start_theta, rng, full_speed)
        if commands is not None and len(commands) <= config.T_SEGMENTS:
            break
    else:
        return None
    state = (problem.start_x, problem.start_y, problem.start_theta)
    previous = None
    for command in commands:
        if not _command_is_free(state, command.vx, command.vy, command.omega,
                                problem.obstacles, clearance, edge_clearance, previous):
            return None
        state = nominal_step(*state, command, config, previous=previous)
        previous = command
    if math.hypot(state[0]-problem.goal_x, state[1]-problem.goal_y) > goal_radius:
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
