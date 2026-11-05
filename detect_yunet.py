"""用 OpenCV 的 YuNet 做人脸检测，输出 smoothgaze 需要的归一化坐标。

YuNet 是 232 KB 的轻量人脸检测网络，纯 CPU 单帧 20 毫秒上下，小到 28 像素的
脸也能检出。模型来自 OpenCV Zoo，按 MIT 许可分发：

    https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet

下载：

    curl -LO https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx

依赖：opencv-python，numpy

两个容易踩的坑：
  一、cv2.dnn 默认按 CPU 核数开线程。800x450 的小模型根本用不上十六个线程，
      实测它会常驻吃满一个核做空转。第一件事就是 setNumThreads(2)。
  二、瞄准的点要用鼻尖，不要用人脸框中心。框中心受发际线和下巴影响，
      人一低头整体就偏，看着像「对着你但视线偏高」。
"""

from __future__ import annotations

import cv2
import numpy as np

__all__ = ["YuNetFaceDetector"]


class YuNetFaceDetector:
    def __init__(self, model_path: str, width: int = 800, height: int = 450,
                 score_threshold: float = 0.6, threads: int = 2):
        cv2.setNumThreads(threads)          # 见上面第一个坑
        self.width = width
        self.height = height
        self.det = cv2.FaceDetectorYN.create(
            model_path, "", (width, height), score_threshold, 0.3, 5000
        )
        self.det.setInputSize((width, height))

    def detect(self, frame_bgr: np.ndarray):
        """返回最大那张脸的 (nx, ny, confidence)，没有脸就返回 None。

        nx / ny 是鼻尖相对画面中心的归一化偏移，范围 -1..1，0 是正中。
        """
        h, w = frame_bgr.shape[:2]
        if (w, h) != (self.width, self.height):
            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))
        self.det.setInputSize((self.width, self.height))

        n, faces = self.det.detect(frame_bgr)
        if faces is None or n <= 0:
            return None

        best = None
        best_area = -1.0
        for f in faces:
            x, y, fw, fh = f[0], f[1], f[2], f[3]
            area = fw * fh
            if area > best_area:
                best_area = area
                best = f
        if best is None:
            return None

        x, y, fw, fh = best[0], best[1], best[2], best[3]
        # YuNet 输出的 15 个值里，第 4-5 个是右眼、6-7 是左眼、8-9 是鼻尖、10-11 是嘴
        nose = (float(best[8]), float(best[9]))
        if nose == (0.0, 0.0):              # 老版本模型可能没有关键点
            nose = (x + fw / 2.0, y + fh / 2.0)

        nx = (nose[0] - self.width / 2.0) / (self.width / 2.0)
        ny = (nose[1] - self.height / 2.0) / (self.height / 2.0)
        return nx, ny, float(best[-1])
