#!/usr/bin/python3
# -*- coding: utf-8 -*-

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import numpy as np
import traceback
import cv2
import torch
import os
import ROBOCOMPMNIST
import sys

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        self.color = None
        self.last_roi = None
        self.last_result = ROBOCOMPMNIST.MNISTResult(number=-1, center=-1)

        # smoothing for center (reduces jitter so C++ can stabilize)
        self.smooth_center = None
        self.alpha = 0.6

        # debug windows
        self.debug_view = True

        if startup_check:
            self.startup_check()
        else:
            # ---------- Camera connection ----------
            started_camera = False
            while not started_camera:
                try:
                    print("Connecting to Camera360RGB...")
                    self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                    print("Camera specs:")
                    print(" width:", self.rgb_original.width)
                    print(" height:", self.rgb_original.height)
                    print(" focalx", self.rgb_original.focalx)
                    print(" focaly", self.rgb_original.focaly)
                    print(" period", self.rgb_original.period)
                    print(" ratio {:.2f}".format(self.rgb_original.width / self.rgb_original.height))
                    started_camera = True
                    print("Connected to Camera360RGB")
                except Ice.Exception as e:
                    traceback.print_exc()
                    print(e, "Trying again CAMERA...")

            # ---------- Load TorchScript model (force CPU) ----------
            model_path = os.path.join(os.path.dirname(__file__), "my_network.pt")
            self.device = torch.device("cpu")
            self.model = torch.jit.load(model_path, map_location=self.device)
            self.model.eval()
            print(f"✅ TorchScript MNIST model loaded from: {model_path}")

            # ---------- Timer ----------
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        pass

    # ------------------------------------------------------------
    # MAIN LOOP
    # ------------------------------------------------------------
    @QtCore.Slot()
    def compute(self):
        try:
            image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
            self.color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)

            rect = self.detect_sign_red_frame(self.color)   # <-- NEW: detects red frame
            self.last_roi = rect

            if rect is None:
                self.last_result = ROBOCOMPMNIST.MNISTResult(number=-1, center=-1)
                self.smooth_center = None
            else:
                x1, y1, x2, y2 = rect
                center_x = (x1 + x2) // 2

                roi = self.color[y1:y2, x1:x2]
                pred = self.classify_roi(roi)

                # Smooth center
                if self.smooth_center is None:
                    self.smooth_center = float(center_x)
                else:
                    self.smooth_center = self.alpha * float(center_x) + (1.0 - self.alpha) * self.smooth_center
                center_x_smooth = int(self.smooth_center)

                # Even if pred == -1, return center so C++ can keep centering
                self.last_result = ROBOCOMPMNIST.MNISTResult(number=pred, center=center_x_smooth)

            console.print(f"ROI={self.last_roi} -> number={self.last_result.number}, center={self.last_result.center}")

            # Debug visual
            if self.debug_view:
                dbg = self.color.copy()
                if self.last_roi is not None:
                    x1, y1, x2, y2 = self.last_roi
                    cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        dbg,
                        f"n={self.last_result.number} cx={self.last_result.center}",
                        (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                cv2.imshow("Camera360RGB", dbg)
                cv2.waitKey(1)

        except Exception as e:
            console.print("❌ Error in compute:", e)
            traceback.print_exc()
            self.last_result = ROBOCOMPMNIST.MNISTResult(number=-1, center=-1)
            self.smooth_center = None

    # ------------------------------------------------------------
    # NEW ROI detection: detect the RED FRAME of the sign
    # ------------------------------------------------------------
    def detect_sign_red_frame(self, bgr):
        """
        Detects the red frame around the digit sign.
        Returns [x1,y1,x2,y2] or None
        """
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # red = two HSV ranges
        lower1 = np.array([0, 80, 80])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 80, 80])
        upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        mask = cv2.medianBlur(mask, 7)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_score = -1.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 0.0015 * w * h:
                continue

            x, y, bw, bh = cv2.boundingRect(c)
            if bw < 30 or bh < 30:
                continue

            ar = bw / float(bh)
            if not (0.75 < ar < 1.33):
                continue

            squareness = 1.0 - abs(1.0 - ar)
            score = area * squareness

            if score > best_score:
                best_score = score
                best = (x, y, x + bw, y + bh)

        if best is None:
            return None

        x1, y1, x2, y2 = best

        # small padding
        pad = 8
        x1 = max(0, x1 - pad); y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad); y2 = min(h, y2 + pad)

        if x2 <= x1 or y2 <= y1:
            return None

        return [x1, y1, x2, y2]

    # ------------------------------------------------------------
    # Utils + classify ROI with model
    # ------------------------------------------------------------
    def crop_margin(self, img, margin_ratio):
        h, w = img.shape[:2]
        x_margin = int(w * margin_ratio)
        y_margin = int(h * margin_ratio)
        if (h - 2 * y_margin) <= 1 or (w - 2 * x_margin) <= 1:
            return img
        return img[y_margin:h - y_margin, x_margin:w - x_margin]

    def classify_roi(self, roi_bgr):
        """
        Returns digit 0-9, or -1 if cannot classify.
        """
        try:
            roi = self.crop_margin(roi_bgr, 0.08)

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            coords = cv2.findNonZero(bw)
            if coords is None or len(coords) < 30:
                return -1

            x, y, w, h = cv2.boundingRect(coords)
            digit = bw[y:y + h, x:x + w]

            side = int(max(w, h) * 1.25)
            if side < 10:
                return -1

            canvas = np.zeros((side, side), dtype=np.uint8)
            x_off = (side - w) // 2
            y_off = (side - h) // 2
            canvas[y_off:y_off + h, x_off:x_off + w] = digit

            mnist_img = cv2.resize(canvas, (28, 28))
            mnist_img = mnist_img.astype(np.float32) / 255.0

            input_tensor = torch.from_numpy(mnist_img).unsqueeze(0).unsqueeze(0).to(self.device)

            with torch.no_grad():
                out = self.model(input_tensor)
                pred = int(out.argmax(1).item())

            if self.debug_view:
                cv2.imshow("MNIST input (28x28)", mnist_img)
                cv2.waitKey(1)

            return pred

        except Exception:
            traceback.print_exc()
            return -1

    # ------------------------------------------------------------
    # Interface method
    # ------------------------------------------------------------
    def MNIST_getNumber(self):
        return self.last_result

    # extra safety depending on generated wrapper
    def getNumber(self):
        return self.last_result

    # ------------------------------------------------------------
    def startup_check(self):
        print(f"Testing RoboCompCamera360RGB.TRoi from ifaces.RoboCompCamera360RGB")
        _ = ifaces.RoboCompCamera360RGB.TRoi()
        print(f"Testing RoboCompCamera360RGB.TImage from ifaces.RoboCompCamera360RGB")
        _ = ifaces.RoboCompCamera360RGB.TImage()
        QTimer.singleShot(200, QApplication.instance().quit)

