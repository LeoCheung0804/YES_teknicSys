import cv2
import numpy as np
from typing import Tuple, Optional

def draw_contour(img: np.ndarray, mask: np.ndarray) -> np.ndarray:
    img = cv2.drawContours(img, [mask.astype(int)], 0, (0, 255, 0), 2)
    return img

def draw_point(img: np.ndarray, point: Tuple[float, float]) -> np.ndarray:
    img = cv2.circle(img, (int(point[0]), int(point[1])), 2, (0, 0, 255), -1)
    return img

def draw_bounding_box(img: np.ndarray, box: np.ndarray) -> np.ndarray:
    x, y, w, h = box.astype(int)
    img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
    return img

def draw_text(img: np.ndarray, text: str, position: Tuple[int, int]) -> np.ndarray:
    cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img

def get_center(mask: np.ndarray) -> Optional[Tuple[int, int]]:
    moments = cv2.moments(mask.astype(int))
    if moments['m00'] != 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        return (cx, cy)
    return None

def get_orientation(mask: np.ndarray) -> float:
    # Calculate the minimum area rectangle
    (_, (w, h), angle) = cv2.minAreaRect(mask)
    if w < h:
        angle += 90
        if angle > 90:
            angle -= 180
    return angle

def get_simple_mask(mask: np.ndarray) -> np.ndarray:
    eps = 0.01 * cv2.arcLength(mask, True)
    approx = cv2.approxPolyDP(mask, eps, True)
    return approx

def get_longest_edge(mask: np.ndarray) -> float:
    if len(mask) < 2:
        return 0.0
    
    max_dist = 0.0
    for i in range(len(mask)):
        pt1 = mask[i][0]
        pt2 = mask[(i + 1) % len(mask)][0]
        dist = np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
        max_dist = max(max_dist, dist)
    
    return max_dist

def get_area(mask: np.ndarray) -> float:
    return cv2.contourArea(mask.astype(int))

def get_dist(mask: np.ndarray, area_1m: float) -> float:
    return (area_1m / get_area(mask)) ** 0.5