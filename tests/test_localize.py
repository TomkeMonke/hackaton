"""Testy pinecone_bot/localize.py - syntetyczna mapa, bez kamery."""
from __future__ import annotations

import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinecone_bot.localize import (  # noqa: E402
    MapLocalizer, Pose2D, _interp_pose, quat_to_R, read_poses, yaw_of,
)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
from record_rgbd import write_calibration  # noqa: E402

K = (600.0, 600.0, 320.0, 240.0)


def _texture(seed=3, w=640, h=480):
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w), np.uint8)
    for _ in range(400):  # rogi prostokatow = dobre cechy ORB
        x, y = int(rng.integers(0, w - 20)), int(rng.integers(0, h - 20))
        cv2.rectangle(img, (x, y), (x + int(rng.integers(5, 30)), y + int(rng.integers(5, 30))),
                      int(rng.integers(0, 255)), -1)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def _pose_line(stamp, x, y, yaw, kid):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    return "%.3f %f %f 0 0 0 %f %f %d\n" % (stamp, x, y, qz, qw, kid)


def _dataset(tmp_path, poses):
    """Katalog jak z record_rgbd + map_poses.txt; kazda klatka to inna tekstura na scianie 2 m."""
    for sub in ("rgb", "depth", "calib"):
        os.makedirs(tmp_path / sub, exist_ok=True)
    write_calibration(str(tmp_path / "calib" / "rs_color.yaml"), 640, 480, K[0], K[1], K[2], K[3])
    lines = ["#timestamp x y z qx qy qz qw id\n"]
    for kid, (x, y, yaw) in poses.items():
        cv2.imwrite(str(tmp_path / "rgb" / ("%06d.jpg" % kid)), _texture(seed=kid),
                    [cv2.IMWRITE_JPEG_QUALITY, 98])
        cv2.imwrite(str(tmp_path / "depth" / ("%06d.png" % kid)), np.full((480, 640), 2000, np.uint16))
        lines.append(_pose_line(kid, x, y, yaw, kid))
    (tmp_path / "map_poses.txt").write_text("".join(lines))
    return str(tmp_path)


def test_quat_and_yaw():
    for yaw in (0.0, 0.7, -2.5):
        R = quat_to_R(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
        T = np.eye(4)
        T[:3, :3] = R
        assert abs(yaw_of(T) - yaw) < 1e-9


def test_read_poses_and_interp(tmp_path):
    p = tmp_path / "map_poses.txt"
    p.write_text("#timestamp x y z qx qy qz qw id\n" + _pose_line(1, 0, 0, 0, 1)
                 + _pose_line(2, 1, 0, math.pi / 2, 11))
    poses = read_poses(str(p))
    assert sorted(poses) == [1, 11]
    mid = _interp_pose(poses, 6)
    assert abs(mid[0, 3] - 0.5) < 1e-9 and abs(yaw_of(mid) - math.pi / 4) < 1e-9
    assert _interp_pose(poses, 20) is None


def test_localize_recovers_keyframe_pose(tmp_path):
    ds = _dataset(tmp_path, {1: (0.0, 0.0, 0.0), 16: (1.0, 2.0, math.pi / 2), 31: (-1.0, 0.5, -2.0)})
    loc = MapLocalizer.build(ds, verbose=False)
    assert loc.ids == [1, 16, 31]
    npz = str(tmp_path / "map_features.npz")
    loc.save(npz)
    loc = MapLocalizer.load(npz)
    for kid, (x, y, yaw) in {16: (1.0, 2.0, math.pi / 2), 31: (-1.0, 0.5, -2.0)}.items():
        img = cv2.imread(os.path.join(ds, "rgb", "%06d.jpg" % kid))
        p = loc.localize(img)
        assert p is not None and p.keyframe == kid
        assert math.hypot(p.x - x, p.y - y) < 0.05
        assert abs(math.atan2(math.sin(p.yaw - yaw), math.cos(p.yaw - yaw))) < math.radians(2)


def test_localize_rotation_from_shifted_image(tmp_path):
    """Scena w obrazie przesunieta w lewo (homografia K R K^-1) = kamera obrocona w prawo o a."""
    ds = _dataset(tmp_path, {1: (0.0, 0.0, 0.0)})
    loc = MapLocalizer.build(ds, verbose=False)
    img = cv2.imread(os.path.join(ds, "rgb", "000001.jpg"))
    a = math.radians(5)
    Kmat = np.array([[K[0], 0, K[2]], [0, K[1], K[3]], [0, 0, 1]])
    # obrot kamery wokol osi pionowej (optyczne Y w dol): homografia K R K^-1
    Ry = np.array([[math.cos(a), 0, -math.sin(a)], [0, 1, 0], [math.sin(a), 0, math.cos(a)]])
    H = Kmat @ Ry @ np.linalg.inv(Kmat)
    rotated = cv2.warpPerspective(img, H, (640, 480))
    p = loc.localize(rotated)
    assert p is not None
    assert math.hypot(p.x, p.y) < 0.05
    assert abs(p.yaw + a) < math.radians(1.0)   # w prawo = kurs ujemny


def test_prior_limits_candidates(tmp_path):
    ds = _dataset(tmp_path, {1: (0.0, 0.0, 0.0), 16: (10.0, 0.0, 0.0)})
    loc = MapLocalizer.build(ds, verbose=False)
    img = cv2.imread(os.path.join(ds, "rgb", "000016.jpg"))
    assert loc.localize(img, prior=Pose2D(10.0, 0.0, 0.0)).keyframe == 16
    # prior daleko od obu -> brak kandydatow w promieniu, szuka we wszystkich
    assert loc.localize(img, prior=Pose2D(50.0, 50.0, 0.0)).keyframe == 16
