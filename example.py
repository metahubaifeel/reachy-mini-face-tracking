"""Two demos, neither needs a robot.

    python example.py sim     offline closed-loop simulation, prints error data
    python example.py cam     opens a camera and draws a virtual head following your face

The simulation is the convincing part: the same person, the same motion, two
controllers, error side by side.
"""

from __future__ import annotations

import math
import random
import sys

from smoothgaze import SmoothGazeTracker, clamp, degrees_to_pixels


# ---------------------------------------------------------------- simulation

CTRL_HZ = 40.0
CAM_FPS = 30.0
DETECT_MS = 45.0          # roughly how long one detection takes


def run(kind: str, face_fn, secs: float = 14.0, seed: int = 1):
    """kind is 'smoothgaze' or 'naive'. 'naive' is the common wrong way."""
    rnd = random.Random(seed)
    tr = SmoothGazeTracker()
    tr.head_yaw.reset(0.0)

    naive_head = 0.0
    naive_vel = 0.0
    naive_target = 0.0
    next_naive_look = 0.0

    dt = 1.0 / CTRL_HZ
    t = 0.0
    next_cam = 0.0
    pending = []          # (deliver_t, nx, ny)
    errors = []

    while t < secs:
        # Grab a frame when it is time
        while t >= next_cam:
            face = face_fn(t)
            # What the camera sees is the error relative to the head
            seen = face - (tr.head_yaw.pos if kind == "smoothgaze" else naive_head)
            nx = -degrees_to_pixels(clamp(seen, 35.0), 44.0) + rnd.gauss(0, 0.004)
            ny = rnd.gauss(0, 0.004)
            delay = DETECT_MS / 1000.0 + 1.0 / CAM_FPS
            pending.append((t + delay, clamp(nx, 1.0), clamp(ny, 1.0)))
            next_cam += 1.0 / CAM_FPS

        due = [p for p in pending if p[0] <= t]
        pending = [p for p in pending if p[0] > t]
        for deliver, nx, ny in due:
            if kind == "smoothgaze":
                capture = deliver - DETECT_MS / 1000.0
                tr.on_face(nx, ny, capture_ts=capture)

        if kind == "smoothgaze":
            tr.tick(dt, now=t)
            head = tr.head_yaw.pos
        else:
            # The common wrong way: look once every 1.2 seconds, correct half of
            # what you see, then drive there with a large gain.
            if t >= next_naive_look:
                next_naive_look = t + 1.2
                seen = face_fn(t) - naive_head
                naive_target = clamp(naive_head + seen * 0.45, 90.0)
            err = naive_target - naive_head
            v_des = clamp(8.0 * err, 120.0)
            naive_vel += clamp(v_des - naive_vel, 700.0 * dt)
            step = naive_vel * dt
            if abs(step) > abs(err):
                naive_head, naive_vel = naive_target, 0.0
            else:
                naive_head += step
            head = naive_head

        errors.append((t, face_fn(t) - head))
        t += dt

    tail = [e for tt, e in errors if tt > secs * 0.25]
    absn = sorted(abs(e) for e in tail)
    med = absn[len(absn) // 2]
    p90 = absn[int(len(absn) * 0.9)]
    mx = absn[-1]
    return med, p90, mx


def face_walk(t):
    """The person walks past and comes back."""
    if t < 1.0:
        return 0.0
    if t < 4.0:
        return 40.0 * (t - 1.0) / 3.0
    if t < 7.0:
        return 40.0
    if t < 10.0:
        return 40.0 - 80.0 * (t - 7.0) / 3.0
    return -40.0


def demo_sim():
    scenarios = [
        ("standing still, 25 off-centre", lambda t: 25.0),
        ("walking across 40 and back", face_walk),
        ("pacing 30, 4 second period", lambda t: 30.0 * math.sin(2 * math.pi * t / 4.0)),
    ]
    print()
    print("  Following error in degrees. Lower is better.")
    print("  %-30s %-16s %-16s" % ("", "this module", "naive version"))
    print("  " + "-" * 64)
    for name, fn in scenarios:
        a = run("smoothgaze", fn)
        b = run("naive", fn)
        print("  %-30s %-16s %-16s" % (
            name,
            "med %.1f p90 %.1f" % (a[0], a[1]),
            "med %.1f p90 %.1f" % (b[0], b[1]),
        ))
    print()


# ---------------------------------------------------------------- camera

def demo_cam(cam_index: int = 0):
    import cv2
    import time
    from detect_yunet import YuNetFaceDetector

    model = "face_detection_yunet_2023mar.onnx"
    det = YuNetFaceDetector(model, width=800, height=450)
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 450)

    tr = SmoothGazeTracker()
    last = time.time()

    print("Press q in the window to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.resize(frame, (800, 450))
        found = det.detect(frame)
        if found:
            nx, ny, score = found
            tr.on_face(nx, ny)
            px = int(400 + nx * 400)
            py = int(225 + ny * 225)
            cv2.circle(frame, (px, py), 8, (0, 220, 160), 2)

        now = time.time()
        dt = now - last
        last = now
        y, p, b = tr.tick(dt, now=now)

        # Draw a virtual head near the bottom of the frame so you can see it follow
        cx, cy = 400, 520
        ang = math.radians(-y)
        tip = (int(cx + math.sin(ang) * 60), int(cy - math.cos(ang) * 60))
        cv2.line(frame, (cx, cy), tip, (90, 210, 255), 3)

        cv2.putText(frame, "yaw %.1f  pitch %.1f  body %.1f" % (y, p, b),
                    (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 220, 240), 1)
        cv2.imshow("smoothgaze", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "sim"
    if mode == "cam":
        demo_cam()
    else:
        demo_sim()
