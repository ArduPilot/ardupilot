#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Simulate decentralized CBF navigation into a bitmap-word formation.

Each agent has exact knowledge of its own state, but receives the other agents'
position and velocity only at ``--state-rate``.  Between broadcasts it predicts
peer positions with a constant velocity model and enlarges the safety constraint
using the error reachable under the configured acceleration limit.

Agents assigned to less accessible northern rows depart first in short priority
waves.  This is the trajectory reservation layer: it prevents an acceleration-
limited safety filter from being handed an avoidable, already infeasible encounter.

The local controller is a two dimensional quadratic program: choose the velocity
closest to the formation guidance command while satisfying speed, acceleration,
and pairwise control-barrier-function (CBF) constraints.  The tiny QP is solved without
external packages by enumerating the possible boundary intersections.

Example:
    python3 swarm_cbf_sim.py --word GSoC --csv /tmp/cbf.csv
"""

import argparse
import csv
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path


GLYPH_WIDTH = 5
GLYPH_HEIGHT = 7
GLYPH_GAP = 1
DEFAULT_APPLET = Path(__file__).resolve().parents[2] / "AP_Scripting" / "applets" / "swarm_speller.lua"
EPSILON = 1.0e-9
FEASIBILITY_TOLERANCE = 1.0e-7


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scale):
        return Vec2(self.x * scale, self.y * scale)

    __rmul__ = __mul__

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def norm_squared(self):
        return self.dot(self)

    def norm(self):
        return math.sqrt(self.norm_squared())


ZERO = Vec2(0.0, 0.0)


@dataclass
class Agent:
    agent_id: int
    position: Vec2
    velocity: Vec2
    target: Vec2
    launch_time: float = 0.0
    command: Vec2 = ZERO


@dataclass(frozen=True)
class BroadcastState:
    position: Vec2
    velocity: Vec2
    timestamp: float


@dataclass(frozen=True)
class CircleConstraint:
    center: Vec2
    radius: float


@dataclass
class SimulationResult:
    converged: bool
    convergence_time: float
    minimum_distance: float
    infeasible_steps: int
    collisions: int
    final_max_error: float


def clamp_norm(vector, limit):
    length = vector.norm()
    if length <= limit or length <= EPSILON:
        return vector
    return vector * (limit / length)


def cross(a, b):
    return a.x * b.y - a.y * b.x


def parse_font(applet_path):
    """Read the bitmap font from the Lua applet, the formation source of truth."""
    text = applet_path.read_text(encoding="utf-8")
    font = {}
    row_group = rf'((?:"[.#]{{{GLYPH_WIDTH}}}",?\s*)+)'
    for match in re.finditer(rf"^\s{{2}}(\S)\s*=\s*\{{{row_group}\}},", text, re.MULTILINE):
        rows = re.findall(rf'"([.#]{{{GLYPH_WIDTH}}})"', match.group(2))
        if len(rows) == GLYPH_HEIGHT:
            font[match.group(1)] = rows
    if not font:
        raise ValueError(f"could not parse the bitmap font from {applet_path}")
    return font


def build_slots(word, font, spacing):
    """Return target positions as east/north coordinates in metres."""
    total_width = len(word) * GLYPH_WIDTH + (len(word) - 1) * GLYPH_GAP
    east_origin = -(total_width - 1) * 0.5
    north_origin = (GLYPH_HEIGHT - 1) * 0.5
    slots = []
    for char_index, char in enumerate(word):
        if char not in font:
            raise ValueError(f"glyph {char!r} is not present in {DEFAULT_APPLET}")
        east_offset = char_index * (GLYPH_WIDTH + GLYPH_GAP)
        for row in range(GLYPH_HEIGHT):
            for column in range(GLYPH_WIDTH):
                if font[char][row][column] == "#":
                    slots.append(Vec2(
                        (east_origin + east_offset + column) * spacing,
                        (north_origin - row) * spacing,
                    ))
    return slots


def hungarian_assignment(cost):
    """Return the minimum-cost column for every row, for rows <= columns."""
    row_count = len(cost)
    column_count = len(cost[0]) if row_count else 0
    if row_count > column_count:
        raise ValueError("Hungarian implementation requires no more agents than slots")

    row_potential = [0.0] * (row_count + 1)
    column_potential = [0.0] * (column_count + 1)
    matched_row = [0] * (column_count + 1)
    predecessor = [0] * (column_count + 1)

    for row in range(1, row_count + 1):
        matched_row[0] = row
        minimum = [math.inf] * (column_count + 1)
        used = [False] * (column_count + 1)
        column = 0
        while True:
            used[column] = True
            active_row = matched_row[column]
            delta = math.inf
            next_column = 0
            for candidate in range(1, column_count + 1):
                if used[candidate]:
                    continue
                reduced = (cost[active_row - 1][candidate - 1]
                           - row_potential[active_row] - column_potential[candidate])
                if reduced < minimum[candidate]:
                    minimum[candidate] = reduced
                    predecessor[candidate] = column
                if minimum[candidate] < delta:
                    delta = minimum[candidate]
                    next_column = candidate
            for candidate in range(column_count + 1):
                if used[candidate]:
                    row_potential[matched_row[candidate]] += delta
                    column_potential[candidate] -= delta
                else:
                    minimum[candidate] -= delta
            column = next_column
            if matched_row[column] == 0:
                break
        while True:
            previous = predecessor[column]
            matched_row[column] = matched_row[previous]
            column = previous
            if column == 0:
                break

    assignment = [-1] * row_count
    for column in range(1, column_count + 1):
        if matched_row[column] != 0:
            assignment[matched_row[column] - 1] = column - 1
    return assignment


def line_intersection(first, second):
    first_normal, first_offset = first
    second_normal, second_offset = second
    determinant = cross(first_normal, second_normal)
    if abs(determinant) <= EPSILON:
        return []
    return [Vec2(
        (first_offset * second_normal.y - first_normal.y * second_offset) / determinant,
        (first_normal.x * second_offset - first_offset * second_normal.x) / determinant,
    )]


def line_circle_intersections(line, circle):
    normal, offset = line
    normal_squared = normal.norm_squared()
    if normal_squared <= EPSILON:
        return []
    signed_offset = offset - normal.dot(circle.center)
    foot = circle.center + normal * (signed_offset / normal_squared)
    distance_squared = signed_offset * signed_offset / normal_squared
    if distance_squared > circle.radius * circle.radius + FEASIBILITY_TOLERANCE:
        return []
    tangent_distance = math.sqrt(max(0.0, circle.radius * circle.radius - distance_squared))
    tangent = Vec2(-normal.y, normal.x) * (tangent_distance / math.sqrt(normal_squared))
    return [foot + tangent, foot - tangent]


def circle_intersections(first, second):
    offset = second.center - first.center
    distance = offset.norm()
    if distance <= EPSILON:
        return []
    if distance > first.radius + second.radius + FEASIBILITY_TOLERANCE:
        return []
    if distance < abs(first.radius - second.radius) - FEASIBILITY_TOLERANCE:
        return []
    along = (first.radius * first.radius - second.radius * second.radius + distance * distance) / (2.0 * distance)
    height_squared = first.radius * first.radius - along * along
    if height_squared < -FEASIBILITY_TOLERANCE:
        return []
    midpoint = first.center + offset * (along / distance)
    perpendicular = Vec2(-offset.y, offset.x) * (math.sqrt(max(0.0, height_squared)) / distance)
    return [midpoint + perpendicular, midpoint - perpendicular]


def feasible(point, halfplanes, circles):
    for normal, offset in halfplanes:
        if normal.dot(point) < offset - FEASIBILITY_TOLERANCE:
            return False
    for circle in circles:
        if (point - circle.center).norm() > circle.radius + FEASIBILITY_TOLERANCE:
            return False
    return True


def solve_velocity_qp(nominal, halfplanes, circles):
    """Project nominal onto an intersection of halfplanes and disks in 2D."""
    candidates = [nominal]

    for normal, offset in halfplanes:
        normal_squared = normal.norm_squared()
        if normal_squared > EPSILON:
            candidates.append(nominal + normal * ((offset - normal.dot(nominal)) / normal_squared))

    for index, first in enumerate(halfplanes):
        for second in halfplanes[index + 1:]:
            candidates.extend(line_intersection(first, second))
        for circle in circles:
            candidates.extend(line_circle_intersections(first, circle))

    for index, first in enumerate(circles):
        radial = nominal - first.center
        if radial.norm() > EPSILON:
            candidates.append(first.center + radial * (first.radius / radial.norm()))
        else:
            candidates.extend([
                first.center + Vec2(first.radius, 0.0),
                first.center - Vec2(first.radius, 0.0),
            ])
        for second in circles[index + 1:]:
            candidates.extend(circle_intersections(first, second))

    valid = [candidate for candidate in candidates if feasible(candidate, halfplanes, circles)]
    if not valid:
        return None
    return min(valid, key=lambda candidate: (candidate - nominal).norm_squared())


def continuous_pair_distance(first_start, first_end, second_start, second_end):
    relative_start = first_start - second_start
    relative_delta = (first_end - first_start) - (second_end - second_start)
    denominator = relative_delta.norm_squared()
    if denominator <= EPSILON:
        return relative_start.norm()
    fraction = max(0.0, min(1.0, -relative_start.dot(relative_delta) / denominator))
    return (relative_start + relative_delta * fraction).norm()


class CBFSimulation:
    def __init__(self, args, slots):
        self.args = args
        self.slots = slots
        self.time = 0.0
        self.minimum_distance = math.inf
        self.infeasible_steps = 0
        self.first_infeasible = []
        self.collisions = 0
        self.broadcasts = {}
        self.rows = []
        self.agents = self._build_agents()
        self._broadcast_state()

    def _build_agents(self):
        count = len(self.slots)
        columns = math.ceil(math.sqrt(count))
        rows = math.ceil(count / columns)
        start_spacing = max(self.args.safe_distance * 1.25, self.args.start_spacing)
        start_north = min(slot.y for slot in self.slots) - self.args.standoff
        rng = random.Random(self.args.seed)
        positions = []
        for index in range(count):
            row, column = divmod(index, columns)
            positions.append(Vec2(
                (column - (columns - 1) * 0.5) * start_spacing + rng.uniform(-0.05, 0.05),
                start_north - (rows - 1 - row) * start_spacing + rng.uniform(-0.05, 0.05),
            ))

        costs = []
        for agent_id, position in enumerate(positions):
            costs.append([
                (position - slot).norm_squared() + 1.0e-9 * (agent_id * count + slot_id)
                for slot_id, slot in enumerate(self.slots)
            ])
        assignment = hungarian_assignment(costs)
        maximum_north = max(slot.y for slot in self.slots)
        return [
            Agent(
                agent_id=index + 1,
                position=position,
                velocity=ZERO,
                target=self.slots[assignment[index]],
                launch_time=(maximum_north - self.slots[assignment[index]].y) / self.args.spacing
                * self.args.wave_interval,
            )
            for index, position in enumerate(positions)
        ]

    def _broadcast_state(self):
        self.broadcasts = {
            agent.agent_id: BroadcastState(agent.position, agent.velocity, self.time)
            for agent in self.agents
        }

    def _barrier_constraints(self, agent):
        halfplanes = []
        for neighbor in self.agents:
            if neighbor.agent_id == agent.agent_id:
                continue
            state = self.broadcasts[neighbor.agent_id]
            age = max(0.0, self.time - state.timestamp)
            predicted_position = state.position + state.velocity * age
            relative_position = agent.position - predicted_position
            distance = relative_position.norm()

            position_error = 0.5 * self.args.max_acceleration * age * age
            guard_radius = self.args.safe_distance + position_error
            if distance > self.args.neighbor_radius + guard_radius:
                continue

            h = relative_position.norm_squared() - guard_radius * guard_radius
            velocity_error = self.args.max_acceleration * age
            robust_margin = 2.0 * distance * velocity_error
            normal = relative_position * 2.0
            offset = (2.0 * relative_position.dot(state.velocity)
                      - self.args.cbf_alpha * h + robust_margin)
            halfplanes.append((normal, offset))
        return halfplanes

    def _nominal_velocity(self, agent):
        if self.time < agent.launch_time:
            return ZERO
        error = agent.target - agent.position
        if error.norm() <= self.args.position_tolerance:
            return ZERO
        return clamp_norm(error * self.args.position_gain, self.args.max_speed)

    def _emergency_velocity(self, agent):
        escape = ZERO
        for neighbor in self.agents:
            if neighbor.agent_id == agent.agent_id:
                continue
            relative = agent.position - neighbor.position
            distance = relative.norm()
            if EPSILON < distance < self.args.neighbor_radius:
                escape = escape + relative * (1.0 / (distance * distance))
        desired = clamp_norm(escape, self.args.max_speed)
        acceleration_delta = clamp_norm(desired - agent.velocity, self.args.max_acceleration * self.args.control_dt)
        return clamp_norm(agent.velocity + acceleration_delta, self.args.max_speed)

    def _commands(self):
        commands = []
        for agent in self.agents:
            nominal = self._nominal_velocity(agent)
            halfplanes = self._barrier_constraints(agent)
            circles = [
                CircleConstraint(ZERO, self.args.max_speed),
                CircleConstraint(agent.velocity, self.args.max_acceleration * self.args.control_dt),
            ]
            command = solve_velocity_qp(nominal, halfplanes, circles)
            if command is None:
                self.infeasible_steps += 1
                if len(self.first_infeasible) < 10:
                    self.first_infeasible.append((self.time, agent.agent_id, len(halfplanes)))
                command = self._emergency_velocity(agent)
            commands.append(command)
        return commands

    def _advance(self, commands):
        old_positions = [agent.position for agent in self.agents]
        new_positions = [
            agent.position + command * self.args.control_dt
            for agent, command in zip(self.agents, commands)
        ]

        for first in range(len(self.agents)):
            for second in range(first + 1, len(self.agents)):
                distance = continuous_pair_distance(
                    old_positions[first], new_positions[first],
                    old_positions[second], new_positions[second],
                )
                self.minimum_distance = min(self.minimum_distance, distance)
                if distance < self.args.body_diameter - FEASIBILITY_TOLERANCE:
                    self.collisions += 1

        for agent, command, position in zip(self.agents, commands, new_positions):
            agent.command = command
            agent.velocity = command
            agent.position = position
        self.time += self.args.control_dt

    def _record(self):
        for agent in self.agents:
            self.rows.append((
                self.time,
                agent.agent_id,
                agent.position.x,
                agent.position.y,
                agent.velocity.x,
                agent.velocity.y,
                agent.target.x,
                agent.target.y,
                (agent.target - agent.position).norm(),
            ))

    def _has_converged(self):
        return all(
            (agent.target - agent.position).norm() <= self.args.position_tolerance
            and agent.velocity.norm() <= self.args.velocity_tolerance
            for agent in self.agents
        )

    def run(self):
        state_period_steps = round(1.0 / (self.args.state_rate * self.args.control_dt))
        log_period_steps = max(1, round(1.0 / (self.args.log_rate * self.args.control_dt)))
        max_steps = math.ceil(self.args.duration / self.args.control_dt)
        convergence_time = math.inf
        settled_steps = 0
        settle_steps_required = math.ceil(self.args.settle_time / self.args.control_dt)

        self._record()
        for step in range(max_steps):
            if step % state_period_steps == 0:
                self._broadcast_state()

            commands = self._commands()
            self._advance(commands)

            if (step + 1) % log_period_steps == 0:
                self._record()

            if self._has_converged():
                settled_steps += 1
                if settled_steps >= settle_steps_required:
                    convergence_time = self.time - self.args.settle_time
                    break
            else:
                settled_steps = 0

        final_max_error = max((agent.target - agent.position).norm() for agent in self.agents)
        return SimulationResult(
            converged=math.isfinite(convergence_time),
            convergence_time=convergence_time,
            minimum_distance=self.minimum_distance,
            infeasible_steps=self.infeasible_steps,
            collisions=self.collisions,
            final_max_error=final_max_error,
        )

    def write_csv(self, path):
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "time_s", "agent_id", "east_m", "north_m", "velocity_east_m_s", "velocity_north_m_s",
                "target_east_m", "target_north_m", "target_error_m",
            ])
            writer.writerows(self.rows)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--word", default="GSoC")
    parser.add_argument("--applet", type=Path, default=DEFAULT_APPLET)
    parser.add_argument("--spacing", type=float, default=4.0, help="bitmap-cell spacing in metres")
    parser.add_argument("--state-rate", type=float, default=5.0, help="peer state broadcasts per second")
    parser.add_argument("--control-dt", type=float, default=0.05, help="local controller timestep in seconds")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--max-speed", type=float, default=3.0)
    parser.add_argument("--max-acceleration", type=float, default=2.0)
    parser.add_argument("--body-diameter", type=float, default=1.0)
    parser.add_argument("--safety-margin", type=float, default=0.75)
    parser.add_argument("--neighbor-radius", type=float, default=12.0)
    parser.add_argument("--cbf-alpha", type=float, default=1.5)
    parser.add_argument("--position-gain", type=float, default=0.8)
    parser.add_argument("--position-tolerance", type=float, default=0.15)
    parser.add_argument("--velocity-tolerance", type=float, default=0.05)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--start-spacing", type=float, default=2.5)
    parser.add_argument("--standoff", type=float, default=25.0)
    parser.add_argument("--wave-interval", type=float, default=0.6,
                        help="delay between target rows, filling the least-accessible row first (default: 0.6)")
    parser.add_argument("--log-rate", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--csv", default="swarm_cbf_track.csv")
    args = parser.parse_args()

    args.safe_distance = args.body_diameter + args.safety_margin
    ratio = 1.0 / (args.state_rate * args.control_dt)
    if abs(ratio - round(ratio)) > 1.0e-8:
        parser.error("--state-rate period must be an integer number of --control-dt steps")
    if args.spacing <= args.safe_distance:
        parser.error("--spacing must exceed body diameter plus safety margin")
    if args.control_dt <= 0.0 or args.state_rate <= 0.0 or args.log_rate <= 0.0:
        parser.error("control timestep, state rate, and log rate must be positive")
    if ratio < 1.0:
        parser.error("--state-rate cannot exceed the local control rate")
    if args.max_speed <= 0.0 or args.max_acceleration <= 0.0:
        parser.error("speed and acceleration limits must be positive")
    if args.body_diameter <= 0.0 or args.safety_margin < 0.0:
        parser.error("body diameter must be positive and safety margin cannot be negative")
    if args.wave_interval < 0.0:
        parser.error("--wave-interval cannot be negative")
    return args


def main():
    args = parse_args()
    try:
        font = parse_font(args.applet)
        slots = build_slots(args.word, font, args.spacing)
    except (OSError, ValueError) as error:
        raise SystemExit(f"ERROR: {error}") from error

    simulation = CBFSimulation(args, slots)
    result = simulation.run()
    simulation.write_csv(args.csv)

    status = "converged" if result.converged else "timed out"
    convergence = f"{result.convergence_time:.2f} s" if result.converged else "n/a"
    print(f"CBF swarm simulation: {status}")
    print(f"  word / agents:       {args.word} / {len(slots)}")
    print(f"  state / control:     {args.state_rate:g} Hz / {1.0 / args.control_dt:g} Hz")
    print(f"  convergence time:    {convergence}")
    print(f"  minimum separation:  {result.minimum_distance:.3f} m")
    print(f"  physical overlap:    {result.collisions} integration intervals")
    print(f"  infeasible QPs:      {result.infeasible_steps}")
    if simulation.first_infeasible:
        samples = ", ".join(
            f"t={time:.2f}/id={agent_id}/n={neighbors}"
            for time, agent_id, neighbors in simulation.first_infeasible
        )
        print(f"  first infeasible:    {samples}")
    print(f"  final maximum error: {result.final_max_error:.3f} m")
    print(f"  track:               {Path(args.csv).resolve()}")

    if not result.converged or result.collisions or result.infeasible_steps:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
