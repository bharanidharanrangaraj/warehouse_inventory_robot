"""Camera helpers for the tag scanner (no rclpy import, so they can be unit-tested).

* image_to_bgr()  converts a sensor_msgs/Image to a numpy BGR array without
                  cv_bridge (avoids the common cv_bridge / NumPy version clash).
* ArucoReader     wraps OpenCV's ArUco detector and works with both the old
                  (<= 4.6, Ubuntu 22.04's python3-opencv) and new (>= 4.7) APIs.
"""

import numpy as np

try:
    import cv2
    HAVE_ARUCO = hasattr(cv2, 'aruco')
except ImportError:            # pragma: no cover - handled by the caller
    cv2 = None
    HAVE_ARUCO = False


def image_to_bgr(msg) -> np.ndarray:
    """Convert a sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8) to BGR."""
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        if enc == 'rgb8':
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        img = img[:, :, [2, 1, 0]] if enc == 'rgba8' else img[:, :, :3]
        return np.ascontiguousarray(img)
    if enc in ('mono8', '8uc1'):
        gray = buf.reshape(h, step)[:, :w]
        return np.ascontiguousarray(np.repeat(gray[:, :, None], 3, axis=2))
    raise ValueError(f'Unsupported image encoding "{msg.encoding}"')


class ArucoReader:
    """Detects ArUco tags; also tries a mirrored image as a fallback.

    Depending on how a texture is mapped onto a box face, Gazebo can render a
    tag mirrored. A mirrored ArUco tag does not decode, so when nothing is
    found the image is flipped horizontally and detection is run again.
    """

    def __init__(self, dictionary_name: str = 'DICT_4X4_50'):
        if not HAVE_ARUCO:
            raise RuntimeError('OpenCV with the aruco module is not available '
                               '(sudo apt install python3-opencv)')
        aruco = cv2.aruco
        if not hasattr(aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary "{dictionary_name}"')
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        if hasattr(aruco, 'ArucoDetector'):                       # OpenCV >= 4.7
            detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())
            self._detect = detector.detectMarkers
        else:                                                     # OpenCV <= 4.6
            params = aruco.DetectorParameters_create()
            self._detect = lambda gray: aruco.detectMarkers(gray, self.dictionary,
                                                            parameters=params)

    def detect(self, gray: np.ndarray):
        """Return (corners, ids_array, id_list, mirrored)."""
        corners, ids, _ = self._detect(gray)
        mirrored = False
        if ids is None or len(ids) == 0:
            corners_f, ids_f, _ = self._detect(cv2.flip(gray, 1))
            if ids_f is not None and len(ids_f) > 0:
                width = gray.shape[1]
                corners = []
                for c in corners_f:
                    c = c.copy()
                    c[..., 0] = (width - 1) - c[..., 0]   # map back to original image
                    corners.append(c)
                ids, mirrored = ids_f, True
        id_list = [] if ids is None else [int(i) for i in np.asarray(ids).flatten()]
        return corners, ids, id_list, mirrored

    @staticmethod
    def draw(bgr: np.ndarray, corners, ids) -> np.ndarray:
        out = bgr.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(out, corners, ids)
        return out
