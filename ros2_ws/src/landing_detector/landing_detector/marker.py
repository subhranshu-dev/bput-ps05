"""AprilTag (36h11 via OpenCV ArUco) detection + PnP pose (Phase 15).

Works with the OpenCV 4.6 legacy ArUco API (Ubuntu 24.04 python3-opencv) and
the >= 4.7 ArucoDetector API.
"""
import cv2
import numpy as np


class MarkerDetector:
    def __init__(self, dictionary: str = 'DICT_APRILTAG_36h11', marker_id: int = 0, marker_size: float = 0.4):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV built without aruco module')
        self.dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.marker_id = int(marker_id)
        self.size = float(marker_size)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detector = cv2.aruco.ArucoDetector(self.dict, params)
            self._legacy_params = None
        else:
            self._detector = None
            self._legacy_params = cv2.aruco.DetectorParameters_create()
            self._legacy_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        h = self.size / 2.0
        # marker corners in the marker frame (x right, y up, z out of marker), TL, TR, BR, BL
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def detect(self, gray: np.ndarray, k: np.ndarray, dist: np.ndarray | None = None) -> dict | None:
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self._legacy_params)
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if int(i) != self.marker_id:
                continue
            img = c.reshape(4, 2).astype(np.float64)
            d = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(self.obj, img, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
            proj, _ = cv2.projectPoints(self.obj, rvec, tvec, k, d)
            reproj = float(np.linalg.norm(proj.reshape(4, 2) - img, axis=1).mean())
            center = img.mean(axis=0)
            return {
                'id': int(i),
                'corners': img,
                'center_px': center,
                'tvec': tvec.reshape(3),      # marker centre in camera optical frame (m)
                'rvec': rvec.reshape(3),
                'reprojection_px': reproj,
                'area_px': float(cv2.contourArea(img.astype(np.float32))),
            }
        return None


def pixel_error(center_px, k: np.ndarray) -> tuple:
    """Normalised image-centre error (error_x = target_x - cx, error_y = target_y - cy)."""
    return float(center_px[0] - k[0, 2]), float(center_px[1] - k[1, 2])


def make_tag_image(dictionary: str = 'DICT_APRILTAG_36h11', marker_id: int = 0, pixels: int = 800,
                   border_frac: float = 0.25) -> np.ndarray:
    """Render the tag with a white quiet zone (used to build the Gazebo texture)."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        tag = cv2.aruco.generateImageMarker(d, marker_id, pixels)
    else:
        tag = cv2.aruco.drawMarker(d, marker_id, pixels)
    pad = int(pixels * border_frac)
    return cv2.copyMakeBorder(tag, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
