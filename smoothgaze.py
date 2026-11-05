"""smoothgaze —— 让机器人的头丝滑地跟着人脸转。

这个模块只做一件事：把「人脸在画面里偏了多少」变成「头该转到哪个角度」，
并且让这个角度以连续、不抖、不过冲的方式被追上。

它不依赖任何具体的机器人 SDK。你只要提供三样东西：
  1. 每帧的人脸位置，归一化到 -1..1，0 是画面正中
  2. 拍这一帧的时刻
  3. 一个 40Hz 左右的循环，每次回调取一次当前该到的姿态

剩下的交给你自己的驱动去执行。

核心不是人脸识别，是下面四件事：

  一、每帧把误差补满，不要补一半。
     补一半等于把收敛速度藏进增益里。人站着不动看不出，人一走就跟不上。

  二、算目标时用「拍照那一刻头在哪」，不是用现在的角度。
     抓帧到算出结果有几十毫秒，这段时间头还在转。拿当前角度当基准，
     每一帧都会多推一点，表现就是过冲之后来回摆。

  三、平滑系数自适应。
     目标在画面里动得快，说明人在走，系数给大；几乎不动时系数给小，压掉检测抖动。

  四、先转头，转不动了才动身子。身子带迟滞和驻留，而且永不自动回正。

作者：JARVIS / Ethan
许可：MIT
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
    """把 v 钳在 -limit..limit。"""
    return limit if v > limit else (-limit if v < -limit else v)


def pixels_to_degrees(nx: float, half_fov_deg: float) -> float:
    """归一化像素偏移换成角度偏移。

    nx 是 -1..1 的归一化偏移，half_fov_deg 是相机的水平半视场角。

    为什么不是简单地 nx * half_fov：那等于假设视场里角度和像素是线性的，
    只有小视场才近似成立。真实相机是针孔模型，tan 才能对上。44 度半视场下，
    边角处线性近似会差 15% 以上，表现就是「越往边上追得越不准」。
    """
    nx = clamp(float(nx), 1.0)
    return math.degrees(math.atan(nx * math.tan(math.radians(half_fov_deg))))


def degrees_to_pixels(deg: float, half_fov_deg: float) -> float:
    """上面那个函数的逆运算，调试和仿真时用。"""
    return math.tan(math.radians(deg)) / math.tan(math.radians(half_fov_deg))


class PoseHistory:
    """头部姿态环形缓冲，用来做延迟补偿。

    存最近若干秒的 (时刻, 偏航, 俯仰)。查的时候按时间戳找最接近的一条。
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
    """目标角度的一阶自适应低通，外加单步限幅。

    两档系数取自 Reachy Mini 官方 SDK 的 _AdaptiveCenterFilter：
    目标在画面里位移超过 move_threshold 就用 fast 追得紧，否则用 slow 压抖动。
    固定系数只能二选一：要么跟得紧但抖，要么稳但慢。
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
        """喂进这一帧算出来的绝对目标角，返回平滑后的目标角。

        seed_yaw / seed_pitch 只在第一帧用：从「头现在在哪」起步，不要一上来
        就跳到目标，否则画面上是一按就猛地扭头。
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
    """单个轴上的运动：比例控制 + 速度限幅 + 加速度限幅 + 死区。

    三件事合起来才叫「丝滑」：
      限速      —— 头不会为了追人而转得比画面刷新还快
      限加速度  —— 起步和刹车都有过程，没有顿挫
      不越过目标 —— 算出来这一拍会冲过头，就直接落到目标并归零速度
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
        # 一阶速度环：让速度按加速度上限逼近期望速度
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
    """底盘的门控。头是手腕，身子是腰，能用手腕解决的就别动腰。

    - enter：头相对身子超过这个角度才开始计时
    - dwell：要持续这么久才真正介入，防止偶尔晃一下身子就跟
    - exit：回到这个角度以内立刻退出
    - lead：介入时身子目标 = 头目标 - lead，永远比头慢一拍，给头留余量
    - 退出后原地冻结，不回正。回正会让画面明显晃一下
    """

    def __init__(
        self,
        enter=45.0,
        exit=32.0,
        dwell=0.8,
        lead=34.0,
        rel_limit=50.0,
        body_limit=90.0,
        kp=3.0,
        vmax=20.0,
        amax=40.0,
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
            target = self.axis.pos   # 就地冻结，不回正

        self.axis.step(dt, target, self.body_limit)
        return self.axis.pos

    def clamp_head(self, head: float) -> float:
        """相对角是硬约束：头相对身子超出 rel_limit 就谁也别想动。"""
        rel = head - self.axis.pos
        if abs(rel) > self.rel_limit:
            head = self.axis.pos + math.copysign(self.rel_limit, rel)
        return head


class SmoothGazeTracker:
    """把上面几块拼起来，形成完整的一条链路。

    用法：

        tr = SmoothGazeTracker(half_fov_h=44.0, half_fov_v=28.5)

        # 视觉线程：每拿到一帧人脸就调一次
        tr.on_face(nx, ny, capture_ts)

        # 控制线程：40Hz 调一次，拿回去执行
        yaw, pitch, body = tr.tick(dt)

    tick 里不碰任何硬件，只做数学。你想怎么驱动都行。
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

    # ---------- 视觉线程调用 ----------

    def on_face(self, nx: float, ny: float, capture_ts: float = None) -> tuple:
        """喂进一帧人脸。nx/ny 是归一化偏移，capture_ts 是这一帧被拍下的时刻。

        capture_ts 不给的话就用「现在减去管线延迟」估一个。
        """
        now = time.time()
        nx = clamp(float(nx), 1.0)
        ny = clamp(float(ny), 1.0)

        if capture_ts is None:
            capture_ts = now - self.pipeline_ms / 1000.0

        # 死区按归一化像素量取，不按角度取：
        # 整个画面宽 2.0，0.02 是画面的 1%，换算成角度只有 0.9 度。
        # 脸落在这圈里就是「正中间」，再微调只会跟着检测噪声抖。
        err_y = 0.0 if abs(nx) < self.filter.dead_zone else -pixels_to_degrees(nx, self.half_fov_h)
        err_p = 0.0 if abs(ny) < self.filter.dead_zone else pixels_to_degrees(ny, self.half_fov_v)

        # 基准取「拍这一帧时头在哪」，不是现在的角度。这就是延迟补偿。
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
        """跟丢了就停在原地，不要慢慢滑回中心。"""
        self.lost = True
        self.filter.active = False

    # ---------- 控制线程调用 ----------

    def tick(self, dt: float = 0.025, now: float = None) -> tuple:
        """推进一个控制周期，返回 (头偏航, 头俯仰, 底盘偏航)，单位度。"""
        now = time.time() if now is None else now

        if self.lost or (now - self.last_face_at) > self.lost_timeout:
            # 没目标就别动，也别留残余速度
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
        """显式回正。只有这里才会让底盘回中。"""
        self.head_yaw.reset(yaw)
        self.head_pitch.reset(pitch)
        self.body.reset(body)
        self.filter.reset(yaw, pitch)
        self.history.clear()
