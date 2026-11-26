"""Universal joint angle computation and privacy-safe landmark anonymization."""

import math
from typing import Any


# Standard MediaPipe Pose landmark indices
LANDMARK_INDICES = {
    "nose": 0,
    "left_shoulder": 11,
    "right_shoulder": 12,
    "left_elbow": 13,
    "right_elbow": 14,
    "left_wrist": 15,
    "right_wrist": 16,
    "left_hip": 23,
    "right_hip": 24,
}


def calculate_joint_angle_degrees(point_a, point_b, point_c) -> float | None:
    """Measure the angle (0..180 deg) at vertex `point_b` formed by A-B-C.

    Points may be (x, y) tuples or objects with `.x` and `.y` attributes.
    """
    ax, ay = _extract_xy(point_a)
    bx, by = _extract_xy(point_b)
    cx, cy = _extract_xy(point_c)

    ux, uy = ax - bx, ay - by
    vx, vy = cx - bx, cy - by

    len_u = math.hypot(ux, uy)
    len_v = math.hypot(vx, vy)
    if len_u < 1e-6 or len_v < 1e-6:
        return None

    cos_val = (ux * vx + uy * vy) / (len_u * len_v)
    cos_val = max(-1.0, min(1.0, cos_val))
    return math.degrees(math.acos(cos_val))


def _extract_xy(point) -> tuple[float, float]:
    if isinstance(point, (tuple, list)):
        return float(point[0]), float(point[1])
    return float(point.x), float(point.y)


def _landmark_vis(lm) -> float:
    vis = getattr(lm, "visibility", None)
    if vis is None:
        vis = getattr(lm, "presence", 1.0)
    return float(vis if vis is not None else 1.0)


def _angle_from_vertical_deg(shoulder, elbow) -> float | None:
    """Measure angle (0..180 deg) between upper arm vector (shoulder->elbow) and vertical down."""
    sx, sy = _extract_xy(shoulder)
    ex, ey = _extract_xy(elbow)
    ux, uy = ex - sx, ey - sy
    length = math.hypot(ux, uy)
    if length < 1e-6:
        return None
    cos_val = max(-1.0, min(1.0, uy / length))
    return math.degrees(math.acos(cos_val))


def build_anonymized_landmark_summary(
    landmarks: list[Any],
    focus_joints: list[int],
    min_visibility: float = 0.60,
) -> dict[str, Any]:
    """Extract ONLY anonymized numeric joint angles and posture metrics.

    Never includes raw images, pixel buffers, names, or user identifiers.
    """
    if not landmarks or len(landmarks) < 25:
        return {"detected": False}

    l_sh = landmarks[LANDMARK_INDICES["left_shoulder"]]
    r_sh = landmarks[LANDMARK_INDICES["right_shoulder"]]
    l_hip = landmarks[LANDMARK_INDICES["left_hip"]]
    r_hip = landmarks[LANDMARK_INDICES["right_hip"]]

    # Shoulder tilt angle from horizontal (measures torso lean / asymmetry)
    sh_dx = r_sh.x - l_sh.x
    sh_dy = r_sh.y - l_sh.y
    torso_tilt_deg = math.degrees(math.atan2(sh_dy, sh_dx)) if abs(sh_dx) > 1e-4 else 0.0
    # Normalize around 0 deg (level shoulders)
    if torso_tilt_deg > 90.0:
        torso_tilt_deg -= 180.0
    elif torso_tilt_deg < -90.0:
        torso_tilt_deg += 180.0

    # Seated posture lean (shoulder midpoint relative to hip midpoint)
    sh_mid_x = (l_sh.x + r_sh.x) / 2.0
    sh_mid_y = (l_sh.y + r_sh.y) / 2.0
    hip_mid_x = (l_hip.x + r_hip.x) / 2.0
    hip_mid_y = (l_hip.y + r_hip.y) / 2.0
    hip_visible = _landmark_vis(l_hip) >= 0.4 and _landmark_vis(r_hip) >= 0.4

    spine_lean_deg = 0.0
    if hip_visible and abs(hip_mid_y - sh_mid_y) > 1e-4:
        spine_lean_deg = math.degrees(math.atan2(sh_mid_x - hip_mid_x, hip_mid_y - sh_mid_y))

    # Focus joint angle for the active exercise
    focus_angle_deg = None
    focus_visible = False
    if len(focus_joints) == 3 and max(focus_joints) < len(landmarks):
        a, b, c = (landmarks[i] for i in focus_joints)
        if all(_landmark_vis(pt) >= min_visibility for pt in (a, b, c)):
            focus_visible = True
            focus_angle_deg = calculate_joint_angle_degrees(a, b, c)

    # Also compute both elbow angles as anonymized context for Gemini coach
    left_elbow_deg = None
    if all(_landmark_vis(landmarks[i]) >= min_visibility for i in (11, 13, 15)):
        left_elbow_deg = calculate_joint_angle_degrees(landmarks[11], landmarks[13], landmarks[15])

    right_elbow_deg = None
    if all(_landmark_vis(landmarks[i]) >= min_visibility for i in (12, 14, 16)):
        right_elbow_deg = calculate_joint_angle_degrees(landmarks[12], landmarks[14], landmarks[16])

    # Biomechanical anti-cheat metrics: upper-arm sideways flare & shoulder shrug ratio
    left_elbow_flare_deg = None
    if _landmark_vis(l_sh) >= min_visibility and _landmark_vis(landmarks[13]) >= min_visibility:
        left_elbow_flare_deg = _angle_from_vertical_deg(l_sh, landmarks[13])

    right_elbow_flare_deg = None
    if _landmark_vis(r_sh) >= min_visibility and _landmark_vis(landmarks[14]) >= min_visibility:
        right_elbow_flare_deg = _angle_from_vertical_deg(r_sh, landmarks[14])

    shoulder_shrug_ratio = None
    if _landmark_vis(l_sh) >= min_visibility and _landmark_vis(r_sh) >= min_visibility:
        shoulder_shrug_ratio = abs(l_sh.y - r_sh.y) / max(abs(r_sh.x - l_sh.x), 1e-4)

    return {
        "detected": True,
        "focus_joints": list(focus_joints),
        "focus_visible": focus_visible,
        "focus_angle_deg": round(focus_angle_deg, 1) if focus_angle_deg is not None else None,
        "left_elbow_deg": round(left_elbow_deg, 1) if left_elbow_deg is not None else None,
        "right_elbow_deg": round(right_elbow_deg, 1) if right_elbow_deg is not None else None,
        "left_elbow_flare_deg": round(left_elbow_flare_deg, 1) if left_elbow_flare_deg is not None else None,
        "right_elbow_flare_deg": round(right_elbow_flare_deg, 1) if right_elbow_flare_deg is not None else None,
        "shoulder_shrug_ratio": round(shoulder_shrug_ratio, 3) if shoulder_shrug_ratio is not None else None,
        "torso_tilt_deg": round(torso_tilt_deg, 1),
        "spine_lean_deg": round(spine_lean_deg, 1) if hip_visible else None,
        "hip_visible": hip_visible,
    }
