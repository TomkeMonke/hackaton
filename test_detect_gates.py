"""
Test bramek wymiarowych z detect_floor_objects.py, na wymyslonej scenie.

Bez kamery i bez czekania, az odpowiedni obiekt wejdzie w kadr - dlatego
`objects_from_cloud` jest osobno od `detect`. Uruchomienie:

    python test_detect_gates.py

Scena: "szyszka" miesci sie w pasmie wysokosci, "noga stolu" ma ten sam
przekroj, ale siega sufitu pasma - czyli idzie dalej w gore i nie jest celem.
"""

import numpy as np

from detect_floor_objects import FloorObjectDetector

H, W = 480, 640
M_PER_PX = 0.0015  # 1 px = 1.5 mm, zeby wymiary wyszly w skali szyszki


def make_scene():
    xyz = np.zeros((H, W, 3), dtype=np.float32)
    cols, rows = np.meshgrid(np.arange(W), np.arange(H))
    xyz[:, :, 0] = cols * M_PER_PX
    xyz[:, :, 1] = rows * M_PER_PX
    xyz[:, :, 2] = 1.0

    distance = np.zeros((H, W), dtype=np.float32)
    above = np.zeros((H, W), dtype=bool)

    # "Szyszka": 30 x 60 px = 4.5 x 9.0 cm, szczyt 6 cm, w pasmie 2-9 cm.
    above[100:160, 100:130] = True
    distance[100:160, 100:130] = 0.06

    # "Noga stolu": ten sam przekroj, ale siega sufitu pasma.
    above[300:330, 300:330] = True
    distance[300:330, 300:330] = 0.0895

    return xyz, distance, above


def make_detector():
    detector = FloorObjectDetector.for_target("szyszka")
    detector.normal = np.array([0.0, 0.0, 1.0])
    detector.offset = 0.0
    return detector


def test_szyszka_przechodzi_noga_odpada():
    xyz, distance, above = make_scene()
    detector = make_detector()

    found = detector.objects_from_cloud(xyz, distance, above)
    assert len(found) == 1, f"oczekiwano 1 obiektu, jest {len(found)}"
    assert detector.rejected["tall"] == 1, "noga stolu powinna odpasc jako wystajaca"

    obj = found[0]
    assert 0.03 <= obj["width_m"] <= 0.07, obj["width_m"]
    assert 0.03 <= obj["length_m"] <= 0.16, obj["length_m"]
    assert abs(obj["height_m"] - 0.06) < 1e-3, obj["height_m"]
    assert obj["fill"] > 0.9, obj["fill"]
    assert obj["distance_m"] > 1.0, obj["distance_m"]
    return found


def test_keep_tall_zwraca_oba():
    xyz, distance, above = make_scene()
    detector = make_detector()
    detector.reject_tall = False
    found = detector.objects_from_cloud(xyz, distance, above)
    assert len(found) == 2, f"bez odrzucania oczekiwano 2, jest {len(found)}"


def test_preset_any_nie_filtruje():
    xyz, distance, above = make_scene()
    detector = FloorObjectDetector.for_target("any")
    detector.normal = np.array([0.0, 0.0, 1.0])
    detector.offset = 0.0
    detector.max_height = 0.09
    found = detector.objects_from_cloud(xyz, distance, above)
    assert len(found) >= 1, "preset 'any' nie powinien odrzucac po wymiarach"


if __name__ == "__main__":
    found = test_szyszka_przechodzi_noga_odpada()
    obj = found[0]
    print(
        f"szyszka: {obj['length_m'] * 100:.1f} x {obj['width_m'] * 100:.1f} cm, "
        f"wys {obj['height_m'] * 100:.1f} cm, wypelnienie {obj['fill']:.2f}, "
        f"dystans {obj['distance_m']:.3f} m"
    )
    test_keep_tall_zwraca_oba()
    test_preset_any_nie_filtruje()
    print("OK - wszystkie testy przeszly")
