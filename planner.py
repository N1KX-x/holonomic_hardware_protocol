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
from core import Command, step_holonomic, wrap_angle


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


def _point_is_free(x: float, y: float, obstacles: tuple[Circle, ...], rho: float) -> bool:
    total_clearance = config.ROBOT_RADIUS_M + rho
    if not _inside_workspace(x, y, total_clearance):
        return False
    return all(math.hypot(x-obstacle.x, y-obstacle.y)
               > obstacle.radius + total_clearance
               for obstacle in obstacles)


def _command_is_free(start, vx: float, vy: float, omega: float,
                     obstacles: tuple[Circle, ...], rho: float) -> bool:
    """Sample the exact curved nominal trajectory, not its endpoint chord."""
    path_length_bound = math.hypot(vx, vy) * config.DT_S
    samples = max(1, math.ceil(path_length_bound / config.RRT_COLLISION_SAMPLE_SPACING_M))
    for index in range(samples + 1):
        elapsed = config.DT_S * index / samples
        x, y, _ = step_holonomic(*start, vx, vy, omega, elapsed)
        if not _point_is_free(x, y, obstacles, rho):
            return False
    return True


def sample_problem(instance: int, seed: int, rho: float) -> Problem:
    """Sample one reproducible problem without conditioning on RRT success."""
    rng = random.Random(seed)
    margin = (max(config.GOAL_RADIUS_M, config.OBSTACLE_RADIUS_MAX_M)
              + config.ROBOT_RADIUS_M + rho)
    for _ in range(10000):
        if config.START_X_M is None:
            start_x = rng.uniform(config.WORKSPACE_X_MIN_M + margin,
                                  config.WORKSPACE_X_MAX_M - margin)
            start_y = rng.uniform(config.WORKSPACE_Y_MIN_M + margin,
                                  config.WORKSPACE_Y_MAX_M - margin)
            goal_x = rng.uniform(config.WORKSPACE_X_MIN_M + margin,
                                 config.WORKSPACE_X_MAX_M - margin)
            goal_y = rng.uniform(config.WORKSPACE_Y_MIN_M + margin,
                                 config.WORKSPACE_Y_MAX_M - margin)
            if math.hypot(goal_x-start_x, goal_y-start_y) < 2.0*config.GOAL_RADIUS_M:
                continue
            start_theta = 0.0
        else:
            # Fixed start and goal: only the obstacle layout differs per instance.
            start_x, start_y = config.START_X_M, config.START_Y_M
            goal_x, goal_y = config.GOAL_X_M, config.GOAL_Y_M
            start_theta = (math.atan2(goal_y-start_y, goal_x-start_x)
                           if config.START_THETA_RAD is None else config.START_THETA_RAD)
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
                circle = Circle(x, y, radius)
                protected_radius = radius + config.ROBOT_RADIUS_M + rho
                protects_start = math.hypot(x-start_x, y-start_y) <= protected_radius + config.START_REGION_RADIUS_M
                protects_goal = math.hypot(x-goal_x, y-goal_y) <= protected_radius + config.GOAL_RADIUS_M
                overlaps = any(math.hypot(x-o.x, y-o.y) <= radius + o.radius for o in obstacles)
                if not (protects_start or protects_goal or overlaps):
                    obstacles.append(circle)
                    break
            else:
                break
        if len(obstacles) == count:
            return Problem(instance, start_x, start_y, start_theta, goal_x, goal_y,
                           tuple(obstacles), seed)
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
    def component(limit: float) -> float:
        choices = (-limit, -0.5*limit, 0.0, 0.5*limit, limit)
        return rng.choice(choices) if rng.random() < 0.65 else rng.uniform(-limit, limit)
    for _ in range(100):
        vx = component(config.VX_MAX_MPS)
        vy = component(config.VY_MAX_MPS)
        omega = component(config.OMEGA_MAX_RADPS)
        limit = config.TRANSLATIONAL_SPEED_MAX_MPS
        if limit is None or math.hypot(vx, vy) <= limit:
            return vx, vy, omega
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
    """Return a fixed-horizon control sequence, or None without resampling."""
    if not 0.0 < rho < config.GOAL_RADIUS_M:
        raise ValueError("rho must satisfy 0 < rho < GOAL_RADIUS_M")
    goal_radius = config.GOAL_RADIUS_M - rho
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
            nxt = step_holonomic(*nearest.state, vx, vy, omega, config.DT_S)
            if not _command_is_free(nearest.state, vx, vy, omega,
                                    problem.obstacles, rho):
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
