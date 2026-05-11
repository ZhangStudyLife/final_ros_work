#!/usr/bin/env python3

import os

import cv2
import numpy as np


def main():
    output = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "models",
        "aruco_marker_0",
        "materials",
        "textures",
        "marker_0.png",
    )
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker_size = 512
    border = 128
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, 0, marker_size)
    else:
        marker = aruco.drawMarker(dictionary, 0, marker_size)
    canvas = np.full((marker_size + border * 2, marker_size + border * 2), 255, dtype=np.uint8)
    canvas[border:border + marker_size, border:border + marker_size] = marker
    os.makedirs(os.path.dirname(output), exist_ok=True)
    cv2.imwrite(output, canvas)
    print(output)


if __name__ == "__main__":
    main()
