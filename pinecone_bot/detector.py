"""
Detektor szyszek na obrazie kolorowym: prog HSV -> morfologia -> kontury -> filtr pola.

Zwraca liste Detection posortowana tak, ze pierwsza jest 'najblizsza' szyszka,
czyli ta najnizej w obrazie (najwieksze py). Przy kamerze patrzacej w dol
'nizej w obrazie' = 'blizej robota', bo szyszki leza na plaskiej ziemi.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import DetectorConfig


@dataclass
class Detection:
    px: float       # kolumna srodka (x w obrazie)
    py: float       # wiersz srodka (y w obrazie)
    area: float
    bbox: tuple     # (x, y, w, h)
    partial: bool = False   # uciety gorna/boczna krawedzia: px mowi tylko 'w ktora strone', py jest falszywe

    @property
    def bottom(self) -> float:
        """Dolna krawedz bboxa = punkt styku z ziemia. Stabilniejszy niz srodek."""
        x, y, w, h = self.bbox
        return float(y + h)


class HsvConeDetector:
    def __init__(self, cfg: DetectorConfig):
        self.cfg = cfg
        k = max(1, int(cfg.morph_ksize))
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.last_mask: np.ndarray | None = None

    def mask(self, bgr: np.ndarray) -> np.ndarray:
        img = bgr
        if self.cfg.blur_ksize and self.cfg.blur_ksize > 1:
            k = int(self.cfg.blur_ksize) | 1
            img = cv2.GaussianBlur(bgr, (k, k), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lo = np.array(self.cfg.hsv.lo, dtype=np.uint8)
        hi = np.array(self.cfg.hsv.hi, dtype=np.uint8)
        if lo[0] > hi[0]:
            # Zakres H przechodzi przez 180 (brazowo-czerwone szyszki maja odcien
            # 140..179 i 0..15 naraz): suma dwoch przedzialow, S i V wspolne.
            top = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]], dtype=np.uint8))
            bottom = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], dtype=np.uint8), hi)
            m = cv2.bitwise_or(top, bottom)
        else:
            m = cv2.inRange(hsv, lo, hi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._kernel)
        self.last_mask = m
        return m

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        m = self.mask(bgr)
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out: list[Detection] = []
        img_h, img_w = m.shape[:2]
        b = int(self.cfg.border_px)
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self.cfg.min_area_px or area > self.cfg.max_area_px:
                continue
            x, y, w, h = cv2.boundingRect(c)
            # obiekt uciety przez gorna lub boczna krawedz ma falszywy srodek: oznaczamy go jako czesciowy,
            # sterowanie uzywa wtedy tylko kierunku (px). Dolna krawedz nie liczy sie jako uciecie,
            # bo 'za blisko' i tak konczy sie cofaniem.
            partial = b > 0 and (y <= b or x <= b or x + w >= img_w - b)
            mom = cv2.moments(c)
            if mom["m00"] <= 0:
                continue
            px = mom["m10"] / mom["m00"]
            py = mom["m01"] / mom["m00"]
            out.append(Detection(px, py, area, (int(x), int(y), int(w), int(h)), partial))
        out.sort(key=lambda d: d.py, reverse=True)
        return out

    @staticmethod
    def draw_debug(bgr: np.ndarray, dets: list[Detection], cx: float | None = None,
                   target_row: float | None = None, text: str = "") -> np.ndarray:
        vis = bgr.copy()
        h, w = vis.shape[:2]
        if cx is not None:
            cv2.line(vis, (int(cx), 0), (int(cx), h), (255, 255, 0), 1)
        if target_row is not None:
            cv2.line(vis, (0, int(target_row)), (w, int(target_row)), (0, 255, 255), 1)
        for i, d in enumerate(dets):
            x, y, bw, bh = d.bbox
            color = (0, 0, 255) if i == 0 else (0, 165, 255)
            if d.partial:
                color = (255, 0, 255)
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), color, 2)
            cv2.circle(vis, (int(d.px), int(d.py)), 3, color, -1)
            cv2.putText(vis, f"{d.px:.0f},{d.py:.0f}", (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        if text:
            cv2.putText(vis, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return vis
