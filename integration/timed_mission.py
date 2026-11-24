"""Feedback-driven RPYD targets, timed translation/settling, hard mission deadline.

Adapted from AUV-IITK/ATAL_controller (MIT License, Copyright (c) AUV-IITK Team):
src/auv_control_ros2/auv_control_ros2/timed_mission.py
"""

import copy
import math

RPYD = ("roll", "pitch", "yaw", "depth")
AXES = ("forward", "lateral")


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be a finite number in {low}..{high}")
    return float(value)


def validate_mission(data):
    if not isinstance(data, dict) or data.get("version") != 2 or type(data.get("version")) is not int:
        raise ValueError("Use a version 2 target mission. Legacy throttle-input missions cannot be converted automatically.")
    if set(data) != {"version", "name", "repeat", "max_time_s", "blocks"}:
        raise ValueError("Expected version, name, repeat, max_time_s and blocks")
    if not isinstance(data["name"], str) or not 1 <= len(data["name"].strip()) <= 80:
        raise ValueError("Mission name must contain 1–80 characters")
    if type(data["repeat"]) is not int or not 1 <= data["repeat"] <= 20:
        raise ValueError("Repeat must be an integer from 1 to 20")
    if not isinstance(data["blocks"], list) or not 1 <= len(data["blocks"]) <= 100:
        raise ValueError("A mission needs 1–100 blocks")
    result = dict(version=2, name=data["name"].strip(), repeat=data["repeat"],
                  max_time_s=number(data["max_time_s"], "Overall maximum time", .1, 900), blocks=[])
    for b in data["blocks"]:
        if not isinstance(b, dict) or b.get("type") not in ("target", "move", "break"):
            raise ValueError("Block type must be target, move or break")
        if not isinstance(b.get("label"), str) or not 1 <= len(b.get("label").strip()) <= 60:
            raise ValueError("Each block needs a label of 1–60 characters")
        clean = dict(type=b["type"], label=b["label"].strip())
        if b["type"] == "target":
            fields = {
                "type", "label", *RPYD, "roll_tol", "pitch_tol", "yaw_tol",
                "depth_tol", "stable_s", "max_time_s", "timeout_action",
            }
            if set(b) != fields:
                raise ValueError("Invalid RPYD target fields")
            for key, low, high in (("roll", -45, 45), ("pitch", -45, 45), ("yaw", -180, 180), ("depth", 0, 30),
                                   ("roll_tol", .1, 15), ("pitch_tol", .1, 15), ("yaw_tol", .1, 15),
                                   ("depth_tol", .005, 1), ("stable_s", .1, 10), ("max_time_s", .1, 120)):
                clean[key] = number(b[key], key, low, high)
            if b["stable_s"] >= b["max_time_s"]:
                raise ValueError("Target deadline must exceed the stable dwell")
            if b["timeout_action"] not in ("abort", "continue"):
                raise ValueError("Target timeout action must be abort or continue")
            clean["timeout_action"] = b["timeout_action"]
        else:
            fields = {"type", "label", "duration_s"} | (set(AXES) if b["type"] == "move" else set())
            if set(b) != fields:
                raise ValueError("Invalid movement/break fields")
            clean["duration_s"] = number(b["duration_s"], "Duration", .1, 120)
            if b["type"] == "move":
                for key in AXES:
                    clean[key] = number(b[key], key, -.5, .5)
        result["blocks"].append(clean)
    return result


def block_budget(block):
    return block["max_time_s"] if block["type"] == "target" else block["duration_s"]


def feedback_values(feedback):
    if not isinstance(feedback, dict) or not all(
            isinstance(feedback.get(k), (float, int)) and not isinstance(feedback[k], bool)
            and math.isfinite(feedback[k]) for k in (*RPYD, "sample_time")):
        raise ValueError("Fresh finite RPYD feedback is required")
    return feedback


class TimedMission:
    def __init__(self):
        self.status, self.reason, self.mission = "IDLE", "", None
        self.elapsed = self.execution = self.step_elapsed = self.stable_elapsed = 0.
        self.events = []
        self.targets = dict.fromkeys(RPYD, 0.)
        self.index = self.cycle = 0
        self.started = self.last_tick = 0.
        self.stable_since = self.last_sample = None
        self.errors = {}

    @property
    def active(self):
        return self.status in ("RUNNING", "PAUSED")

    def start(self, mission, now, feedback=None):
        if self.active:
            raise ValueError("Abort the existing mission first")
        mission = validate_mission(mission)
        if mission["blocks"][0]["type"] == "target":
            initial = mission["blocks"][0]
        else:
            if feedback is None:
                raise ValueError("Begin with an RPYD Target when initial feedback is unavailable")
            initial = feedback_values(feedback)
        self.mission = mission
        self.targets = {key: float(initial[key]) for key in RPYD}
        self.elapsed = self.execution = self.step_elapsed = 0.
        self.started = self.last_tick = now
        self.index, self.cycle, self.events = 0, 0, []
        self.status, self.reason = "RUNNING", "Executing controller targets"
        self._enter()

    def _enter(self):
        self.step_elapsed = self.stable_elapsed = 0.
        self.stable_since = self.last_sample = None
        self.errors = {}
        block = self.mission["blocks"][self.index]
        if block["type"] == "target":
            self.targets = {key: block[key] for key in RPYD}

    def _advance(self, result):
        self.events.append(dict(cycle=self.cycle + 1, block_index=self.index, result=result, elapsed_s=self.elapsed))
        self.events = self.events[-100:]
        self.index += 1
        if self.index == len(self.mission["blocks"]):
            self.index = 0
            self.cycle += 1
        if self.cycle == self.mission["repeat"]:
            self.index = len(self.mission["blocks"]) - 1
            self.cycle -= 1
            self.status, self.reason = "COMPLETE", "Mission completed; disarming"
        else:
            self.reason = result
            self._enter()

    def abort(self, reason="Operator aborted"):
        self.status, self.reason = "ABORTED", reason

    def tick(self, now, fault="", feedback=None):
        if not self.active:
            return self.output()
        delta = now - self.last_tick
        self.last_tick = now
        self.elapsed = max(0., now - self.started)
        if fault or not math.isfinite(delta) or delta < 0 or delta > .25:
            self.abort(fault or "Executor deadline missed; mission stopped")
        elif self.elapsed >= self.mission["max_time_s"]:
            self.abort("Overall mission maximum time exceeded")
        else:
            try:
                feedback = feedback_values(feedback)
            except ValueError:
                feedback = None
                self.stable_since = self.last_sample = None
                self.stable_elapsed = 0.
            self.errors = {key: abs(math.degrees(math.atan2(
                math.sin(math.radians(self.targets[key] - feedback[key])),
                math.cos(math.radians(self.targets[key] - feedback[key])))))
                if key != "depth" else abs(self.targets[key] - feedback[key]) for key in RPYD} if feedback is not None else {}
            if self.status == "RUNNING":
                self.execution += delta
                self.step_elapsed += delta
                block = self.mission["blocks"][self.index]
                if block["type"] != "target":
                    if self.step_elapsed + 1e-9 >= block["duration_s"]:
                        self._advance("Duration completed")
                else:
                    if self.step_elapsed >= block["max_time_s"]:
                        if block["timeout_action"] == "continue":
                            self._advance("Target timed out; continued by explicit policy")
                        else:
                            self.abort("RPYD target maximum time exceeded")
                    elif feedback is None:
                        self.reason = "Waiting for RPYD feedback; target deadline remains active"
                    elif feedback["sample_time"] != self.last_sample:
                        sample = feedback["sample_time"]
                        within = all(self.errors[key] <= block[key + "_tol"] + 1e-9 for key in RPYD)
                        if not within:
                            self.stable_since = None
                        elif (self.stable_since is None or
                                (self.last_sample is not None and not 0 < sample - self.last_sample <= .25)):
                            self.stable_since = sample
                        self.last_sample = sample
                        self.stable_elapsed = max(0., sample - self.stable_since) if self.stable_since is not None else 0.
                        if self.stable_elapsed + 1e-9 >= block["stable_s"]:
                            self._advance("RPYD target reached and stable")
        return self.output()

    def pause(self, now, feedback):
        if self.status != "RUNNING":
            raise ValueError("Only a running mission can be paused")
        self.tick(now, feedback=feedback)
        if self.status == "RUNNING":
            self.status, self.reason = "PAUSED", "Holding RPYD targets; step clock paused, overall deadline still active"
            self.stable_since = self.last_sample = None
            self.stable_elapsed = 0.

    def resume(self, now):
        if self.status != "PAUSED":
            raise ValueError("Only a paused mission can be resumed")
        self.elapsed = max(0., now - self.started)
        if self.elapsed >= self.mission["max_time_s"]:
            self.abort("Overall mission maximum time exceeded")
            return
        self.last_tick = now
        self.status, self.reason = "RUNNING", "Executing controller targets"

    def output(self):
        command = dict(self.targets, forward=0., lateral=0.)
        if self.status == "RUNNING":
            block = self.mission["blocks"][self.index]
            if block["type"] == "move":
                command.update({key: block[key] for key in AXES})
        return command

    def snapshot(self):
        result = dict(status=self.status, reason=self.reason, active=self.active,
                      elapsed_s=self.elapsed, execution_s=self.execution, command=self.output(),
                      mission=copy.deepcopy(self.mission), events=copy.deepcopy(self.events),
                      errors=dict(self.errors), stable_elapsed_s=self.stable_elapsed,
                      step_elapsed_s=self.step_elapsed, block_index=self.index, cycle=self.cycle + 1,
                      total_s=self.mission["max_time_s"] if self.mission else 0.)
        if self.mission:
            block = self.mission["blocks"][self.index]
            result.update(block_type=block["type"], block_remaining_s=max(0., block_budget(block) - self.step_elapsed),
                          planned_budget_s=sum(map(block_budget, self.mission["blocks"])) * self.mission["repeat"])
        return result
