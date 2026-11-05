"""Face detection with OpenCV's YuNet, returning the normalised coordinates
smoothgaze needs.

YuNet is a 232 KB face detection network. On CPU one frame takes around 20 ms,
and it still finds faces as small as 28 pixels. The model comes from OpenCV Zoo
and is distributed under the MIT license:

    https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet

Download:

    curl -LO https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx

Requires: opencv-python, numpy

Two traps worth knowing about:

  1. cv2.dnn spawns one thread per CPU core by default. An 800x450 model has no
     use for sixteen threads; measured, it saturates a core spinning. First
     thing to do is setNumThreads(2).

  2. Aim at the nose tip, not the bounding box centre. The box centre depends on
     the hairline and the chin, so it drifts as the person nods, and the result
     looks like "facing you, but looking slightly up".
"""

from __future__ import annotations

import cv2
import numpy as np

__all__ = ["YuNetFaceDetector"]


class YuNetFaceDetector:
    def __init__(self, model_path: str, width: int = 800, height: int = 450,
                 score_threshold: float = 0.6, threads: int = 2):
        cv2.setNumThreads(threads)          # see trap 1 above
        self.width = width
        self.height = height
        self.det = cv2.FaceDetectorYN.create(
            model_path, "", (width, height), score_threshold, 0.3, 5000
        )
        self.det.setInputSize((width, height))

    def detect(self, frame_bgr: np.ndarray):
        """Returns (nx, ny, confidence) for the largest face, or None if there is none.

        nx / ny are the nose tip's offset from the frame centre, normalised to
        -1..1, where 0 is centre.
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
        # Of the 15 values YuNet returns, 4-5 are the right eye, 6-7 the left eye,
        # 8-9 the nose tip and 10-11 the mouth corners.
        nose = (float(best[8]), float(best[9]))
        if nose == (0.0, 0.0):              # older models may omit the landmarks
            nose = (x + fw / 2.0, y + fh / 2.0)

        nx = (nose[0] - self.width / 2.0) / (self.width / 2.0)
        ny = (nose[1] - self.height / 2.0) / (self.height / 2.0)
        return nx, ny, float(best[-1])
