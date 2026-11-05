"""smoothgaze - make a robot head follow a face smoothly.

This module does one thing: turn "how far off-centre the face is in the frame"
into "what angle the head should be at", and get there continuously, without
jitter and without overshoot.

It depends on no robot SDK. You supply three things:

  1. the face position for each frame, normalised to -1..1, where 0 is centre
  2. the timestamp of the moment that frame was captured
  3. a loop at roughly 40 Hz that asks for the pose to apply

Your own driver does the rest.

The point is not face detection. It is these four decisions:

  1. Close the full error every frame, not half of it.
     Correcting half hides your convergence rate inside the gain. Standing
     still you cannot tell. The moment the person walks, you fall behind.

  2. Compute the target from where the head was when the frame was CAPTURED,
     not from where it is now.
     Capture to result takes tens of milliseconds and the head is still turning
     during them. Using the current angle as the baseline pushes a little too
     far every frame, and the visible result is overshoot then oscillation.

  3. Adaptive smoothing.
     The target moving fast in the frame means the person is walking, so use a
     large factor and track tightly. Barely moving means filter out detection
     noise with a small factor.

  4. Turn the head first, and only involve the body when the head runs out.
     The body has hysteresis and a dwell time, and never recenters itself.

Author: JARVIS / Ethan
License: MIT
"""

from __future__ import annotations

import math
import time
from collections import deque

__all__ = [
    "pixels_to_degrees",
    "PoseHistory",
    "GazeFilter",
    "Axis",
    "BodyGate",
    "SmoothGazeTracker",
]

__version__ = "1.0.0"


def clamp(v: float, limit: float) -> float:
    """Clamp v into -limit..limit."""
    return limit if v > limit else (-limit if v < -limit else v)


def pixels_to_degrees(nx: float, half_fov_deg: float) -> float:
    """Convert a normalised pixel offset into an angular offset.

    nx is the -1..1 normalised offset, half_fov_deg is the camera's horizontal
    half field of view.

    Why not simply nx * half_fov: that assumes angle and pixel are linear inside
    the frame, which only holds for a narrow field of view. A real camera is a
    pinhole model and only tan lines up. At a 44 degree half field of view the
    linear approximation is off by more than 15 percent at the edges, and the
    symptom is "tracking gets less accurate toward the edges".
    """
    nx = clamp(float(nx), 1.0)
    return math.degrees(math.atan(nx * math.tan(math.radians(half_fov_deg))))


def degrees_to_pixels(deg: float, half_fov_deg: float) -> float:
    """Inverse of the function above. Used by the simulation and for debugging."""
    return math.tan(math.radians(deg)) / math.tan(math.radians(half_fov_deg))


class PoseHistory:
    """Ring buffer of head poses, used for latency compensation.

    Stores (time, yaw, pitch) for the last few seconds. Lookups return the
    entry closest to a given timestamp.
    """

    def __init__(self, maxlen: int = 80) -> None:
        self._q: deque = deque(maxlen=maxlen)
        self._last = (0.0, 0.0)

    def push(self, t: float, yaw: float, pitch: float) -> None:
        self._q.append((t, yaw, pitch))
        self._last = (yaw, pitch)

    def clear(self) -> None:
        self._q.clear()

    def _at(self, t: float):
        if not self._q:
            return self._last
        best = self._q[0]
        for item in self._q:
            if abs(item[0] - t) <= abs(best[0] - t):
                best = item
            elif item[0] > t:
                break
        return best

    def yaw_at(self, t: float) -> float:
        return self._at(t)[1]

    def pitch_at(self, t: float) -> float:
        return self._at(t)[2]

    def __len__(self) -> int:
        return len(self._q)


class GazeFilter:
    """First-order adaptive low-pass on the target angle, plus a per-step limit.

    The two factors come from the official Reachy Mini SDK's
    _AdaptiveCenterFilter: when the target moves more than move_threshold in the
    frame, use fast and track tightly; otherwise use slow and filter jitter.
    A fixed factor forces a choice: track tightly and jitter, or stay stable and
    lag behind.
    """

    def __init__(
        self,
        slow: float = 0.30,
        fast: float = 0.60,
        move_threshold: float = 0.15,
        dead_zone: float = 0.02,
        max_step: float = 15.0,
        yaw_limit: float = 90.0,
        pitch_limit: float = 28.5,
    ) -> None:
        self.slow = slow
        self.fast = fast
        self.move_threshold = move_threshold
        self.dead_zone = dead_zone
        self.max_step = max_step
        self.yaw_limit = yaw_limit
        self.pitch_limit = pitch_limit

        self.yaw = 0.0
        self.pitch = 0.0
        self.active = False
        self.last_nx = 0.0
        self.last_ny = 0.0

    def reset(self, yaw: float = 0.0, pitch: float = 0.0) -> None:
        self.yaw = yaw
        self.pitch = pitch
        self.active = False

    def update(
        self,
        raw_yaw: float,
        raw_pitch: float,
        nx: float,
        ny: float,
        seed_yaw: float = None,
        seed_pitch: float = None,
    ):
        """Feed in the absolute target angle from this frame, get back the smoothed one.

        seed_yaw / seed_pitch are used only on the first frame: start from where
        the head is now rather than jumping straight to the target, otherwise the
        first measurement snaps the head around.
        """
        if not self.active:
            self.yaw = self.yaw if seed_yaw is None else seed_yaw
            self.pitch = self.pitch if seed_pitch is None else seed_pitch
            self.last_nx, self.last_ny = nx, ny
            self.active = True

        move = math.hypot(nx - self.last_nx, ny - self.last_ny)
        self.last_nx, self.last_ny = nx, ny

        a = self.fast if move > self.move_threshold else self.slow
        self.yaw += clamp(a * (raw_yaw - self.yaw), self.max_step)
        self.pitch += clamp(a * (raw_pitch - self.pitch), self.max_step)
        self.yaw = clamp(self.yaw, self.yaw_limit)
        self.pitch = clamp(self.pitch, self.pitch_limit)
        return self.yaw, self.pitch


class Axis:
    """Motion on a single axis: proportional control, velocity and acceleration
    limits, plus a dead zone.

    All three are needed before it reads as smooth:
      velocity limit      the head never turns faster than the camera refreshes
      acceleration limit  starting and stopping have a ramp, so there is no jerk
      never overshoot     if this step would pass the target, land on it and
                          zero the velocity instead
    """

    def __init__(self, kp=4.0, vmax=70.0, amax=400.0, dead=0.4) -> None:
        self.kp = kp
        self.vmax = vmax
        self.amax = amax
        self.dead = dead
        self.pos = 0.0
        self.vel = 0.0

    def reset(self, pos: float = 0.0) -> None:
        self.pos = pos
        self.vel = 0.0

    def step(self, dt: float, target: float, limit: float = None) -> float:
        err = target - self.pos
        v_des = clamp(self.kp * err, self.vmax)
        # First-order velocity loop: approach the desired velocity at no more
        # than the acceleration limit.
        dv = v_des - self.vel
        self.vel += clamp(dv, self.amax * dt)

        step = self.vel * dt
        if abs(step) > abs(err) or (abs(err) < self.dead and abs(self.vel) < 4.0):
            self.pos, self.vel = target, 0.0
        else:
            self.pos += step

        if limit is not None:
            self.pos = clamp(self.pos, limit)
        return self.pos


class BodyGate:
    """Enable the body only when the head cannot reach on its own.

    The head is the wrist, the body is the waist. Solve it with the wrist.

    - enter: relative angle beyond which the dwell timer starts
    - dwell: how long that must hold before the body actually engages, so one
             stray wobble does not swing the whole torso
    - exit:  come back inside this angle and the body stops at once
    - lead:  while engaged the body target is head - lead, always lagging the
             head so the head keeps headroom in the direction of travel
    - after exit it freezes where it is and never recenters; recentering would
      lurch the whole view
    """

    def __init__(
        self,
        enter=45.0,
        exit=32.0,
        dwell=0.5,
        lead=34.0,
        rel_limit=50.0,
        body_limit=90.0,
        kp=4.0,
        vmax=30.0,
        amax=90.0,
        dead=0.3,
    ) -> None:
        self.enter = enter
        self.exit = exit
        self.dwell = dwell
        self.lead = lead
        self.rel_limit = rel_limit
        self.body_limit = body_limit
        self.axis = Axis(kp=kp, vmax=vmax, amax=amax, dead=dead)
        self.engaged = False
        self.enter_at = 0.0

    @property
    def pos(self) -> float:
        return self.axis.pos

    def reset(self, pos: float = 0.0) -> None:
        self.axis.reset(pos)
        self.engaged = False
        self.enter_at = 0.0

    def step(self, dt: float, now: float, head: float) -> float:
        rel = head - self.axis.pos

        if not self.engaged:
            if abs(rel) > self.enter:
                if not self.enter_at:
                    self.enter_at = now
                elif now - self.enter_at >= self.dwell:
                    self.engaged = True
            else:
                self.enter_at = 0.0
        elif abs(rel) < self.exit:
            self.engaged = False
            self.enter_at = 0.0

        if self.engaged:
            head_rel = head - self.axis.pos
            target = clamp(head - math.copysign(self.lead, head_rel), self.body_limit)
        else:
            target = self.axis.pos   # freeze in place, never recenter

        self.axis.step(dt, target, self.body_limit)
        return self.axis.pos

    def clamp_head(self, head: float) -> float:
        """The relative angle is a hard constraint: past rel_limit neither side moves."""
        rel = head - self.axis.pos
        if abs(rel) > self.rel_limit:
            head = self.axis.pos + math.copysign(self.rel_limit, rel)
        return head


class SmoothGazeTracker:
    """Wires the pieces above into one chain.

    Usage:

        tr = SmoothGazeTracker(half_fov_h=44.0, half_fov_v=28.5)

        # Vision thread: once per frame that contains a face
        tr.on_face(nx, ny, capture_ts)

        # Control thread: at 40 Hz, then apply the result
        yaw, pitch, body = tr.tick(dt)

    tick touches no hardware. It only does math. Drive it however you like.
    """

    def __init__(
        self,
        half_fov_h: float = 44.0,
        half_fov_v: float = 28.5,
        pipeline_ms: float = 55.0,
        yaw_limit: float = 90.0,
        pitch_limit: float = 28.5,
        lost_timeout: float = 1.5,
    ) -> None:
        self.half_fov_h = half_fov_h
        self.half_fov_v = half_fov_v
        self.pipeline_ms = pipeline_ms
        self.yaw_limit = yaw_limit
        self.pitch_limit = pitch_limit
        self.lost_timeout = lost_timeout

        self.history = PoseHistory()
        self.filter = GazeFilter(pitch_limit=pitch_limit)
        self.head_yaw = Axis(kp=4.0, vmax=70.0, amax=400.0, dead=0.4)
        self.head_pitch = Axis(kp=6.0, vmax=70.0, amax=400.0, dead=0.4)
        self.body = BodyGate()

        self.last_face_at = 0.0
        self.lost = True
        self.frames = 0

    # ---------- called from the vision thread ----------

    def on_face(self, nx: float, ny: float, capture_ts: float = None) -> tuple:
        """Feed in one frame's face. nx/ny are normalised offsets, capture_ts is
        when that frame was taken.

        Without capture_ts it is estimated as now minus the pipeline latency.
        """
        now = time.time()
        nx = clamp(float(nx), 1.0)
        ny = clamp(float(ny), 1.0)

        if capture_ts is None:
            capture_ts = now - self.pipeline_ms / 1000.0

        # The dead zone is in normalised pixels, not degrees: the frame is 2.0
        # wide, so 0.02 is one percent of it, which is under one degree. A face
        # inside that circle counts as dead centre; chasing further only tracks
        # detection noise.
        err_y = 0.0 if abs(nx) < self.filter.dead_zone else -pixels_to_degrees(nx, self.half_fov_h)
        err_p = 0.0 if abs(ny) < self.filter.dead_zone else pixels_to_degrees(ny, self.half_fov_v)

        # Baseline is where the head was when this frame was captured, not where
        # it is now. That is the latency compensation.
        raw_y = clamp(self.history.yaw_at(capture_ts) + err_y, self.yaw_limit)
        raw_p = clamp(self.history.pitch_at(capture_ts) + err_p, self.pitch_limit)

        ty, tp = self.filter.update(
            raw_y, raw_p, nx, ny,
            seed_yaw=self.head_yaw.pos, seed_pitch=self.head_pitch.pos,
        )
        self.last_face_at = now
        self.lost = False
        self.frames += 1
        return ty, tp

    def on_lost(self) -> None:
        """Lost the face: stop where you are. Do not drift back to centre."""
        self.lost = True
        self.filter.active = False

    # ---------- called from the control thread ----------

    def tick(self, dt: float = 0.025, now: float = None) -> tuple:
        """Advance one control period; returns (head yaw, head pitch, body yaw) in degrees."""
        now = time.time() if now is None else now

        if self.lost or (now - self.last_face_at) > self.lost_timeout:
            # No target: do not move, and leave no residual velocity behind.
            self.head_yaw.vel = self.head_pitch.vel = 0.0
            self.body.axis.vel = 0.0
            self.history.push(now, self.head_yaw.pos, self.head_pitch.pos)
            return self.head_yaw.pos, self.head_pitch.pos, self.body.pos

        y = self.head_yaw.step(dt, self.filter.yaw, self.yaw_limit)
        p = self.head_pitch.step(dt, self.filter.pitch, self.pitch_limit)

        b = self.body.step(dt, now, y)
        y = self.body.clamp_head(y)
        self.head_yaw.pos = y

        self.history.push(now, y, p)
        return y, p, b

    def recenter(self, yaw: float = 0.0, pitch: float = 0.0, body: float = 0.0) -> None:
        """Explicit recenter. This is the only path that moves the body back to zero."""
        self.head_yaw.reset(yaw)
        self.head_pitch.reset(pitch)
        self.body.reset(body)
        self.filter.reset(yaw, pitch)
        self.history.clear()
