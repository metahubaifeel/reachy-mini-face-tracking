"""两个演示，都不需要机器人。

    python example.py sim     离线闭环仿真，直接打印误差数据
    python example.py cam     打开摄像头，屏幕上画一个虚拟的头在跟你的脸

仿真是最有说服力的一段：同一个人在做同样的动作，跑两套控制器，对比误差。
"""

from __future__ import annotations

import math
import random
import sys

from smoothgaze import SmoothGazeTracker, clamp, pixels_to_degrees, degrees_to_pixels


# ---------------------------------------------------------------- 仿真

CTRL_HZ = 40.0
CAM_FPS = 30.0
DETECT_MS = 45.0          # 检测一次大概花多久


def run(kind: str, face_fn, secs: float = 14.0, seed: int = 1):
    """kind = 'smoothgaze' 或 'naive'。naive 是常见的错误写法。"""
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
        # 到点抓一帧
        while t >= next_cam:
            face = face_fn(t)
            # 相对头的误差才是相机看到的东西
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
            # 常见的错误写法：每 1.2 秒才看一次，看到偏了补一半，
            # 然后用一个大增益直接冲过去。
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
    """人走了过去又走回来。"""
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
        ("人站着不动，偏 25 度", lambda t: 25.0),
        ("人横穿画面 40 度再回来", face_walk),
        ("人来回走 30 度 4 秒一圈", lambda t: 30.0 * math.sin(2 * math.pi * t / 4.0)),
    ]
    print()
    print("  误差统计，单位度。越小越好。")
    print("  %-26s %-14s %-14s" % ("", "本模块", "常见错误写法"))
    print("  " + "-" * 56)
    for name, fn in scenarios:
        a = run("smoothgaze", fn)
        b = run("naive", fn)
        print("  %-26s %-14s %-14s" % (
            name,
            "中位 %.1f p90 %.1f" % (a[0], a[1]),
            "中位 %.1f p90 %.1f" % (b[0], b[1]),
        ))
    print()


# ---------------------------------------------------------------- 摄像头

def demo_cam(cam_index: int = 0):
    import cv2
    from detect_yunet import YuNetFaceDetector

    model = "face_detection_yunet_2023mar.onnx"
    det = YuNetFaceDetector(model, width=800, height=450)
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 450)

    tr = SmoothGazeTracker()
    import time
    last = time.time()

    print("打开窗口以后按 q 退出。")
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

        # 在画面下方画一个虚拟的头，看得见它在跟
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
