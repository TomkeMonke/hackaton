"""Kurs z zyroskopu telefonu (phyphox) i pasy po kursie w SEARCH (bez sprzetu)."""
from __future__ import annotations

import math

from pinecone_bot.brain import Brain, State
from pinecone_bot.config import Config, HeadingConfig
from pinecone_bot.detector import HsvConeDetector
from pinecone_bot.heading import OdometryHeading, PhyphoxGyro, make_heading
from pinecone_bot.sim import SimArmSimple, SimCamera, SimClock, SimDrive, SimWorld, calibrate_grasps


class FakePhone:
    """Udaje serwer phyphox: probki co 10 ms ze stala predkoscia katowa w_z."""

    def __init__(self, w_z=0.5, measuring=True):
        self.t = 10.0
        self.w_z = w_z
        self.measuring = measuring
        self.urls = []
        self.pending = []   # probki (t, w) od ostatniego zapytania
        self.fail = False

    def run(self, seconds):
        for _ in range(int(round(seconds / 0.01))):
            self.t += 0.01
            self.pending.append((self.t, self.w_z))

    def __call__(self, url):
        self.urls.append(url)
        if self.fail:
            raise OSError("brak sieci")
        if "/control?" in url:
            self.measuring = True
            return {"result": True}
        if "=" not in url:   # pierwsze zapytanie: tylko ostatnia probka
            samples = [(self.t, self.w_z)]
        else:
            samples = self.pending
        self.pending = []
        return {
            "buffer": {
                "gyr_time": {"buffer": [s[0] for s in samples]},
                "gyrZ": {"buffer": [s[1] for s in samples]},
            },
            "status": {"measuring": self.measuring},
        }


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_phyphox_integrates_rotation_rate():
    phone, clock = FakePhone(w_z=0.5), FakeClock()
    gyro = PhyphoxGyro(HeadingConfig(), fetch=phone, clock=clock)
    assert gyro.yaw() is None          # zanim cokolwiek przyszlo
    assert gyro.poll() == 1
    assert gyro.yaw() == 0.0           # calkujemy od pierwszej probki
    for _ in range(20):
        phone.run(0.05)
        clock.t += 0.05
        gyro.poll()
    assert abs(gyro.yaw() - 0.5) < 1e-6   # 0.5 rad/s przez 1 s
    assert "gyrZ=" in phone.urls[-1] and "|gyr_time" in phone.urls[-1]


def test_phyphox_sign_and_stale():
    phone, clock = FakePhone(w_z=1.0), FakeClock()
    gyro = PhyphoxGyro(HeadingConfig(sign=-1.0, stale_s=1.0), fetch=phone, clock=clock)
    gyro.poll()
    phone.run(0.5)
    gyro.poll()
    assert abs(gyro.yaw() + 0.5) < 1e-6
    clock.t += 1.5                      # brak nowych probek dluzej niz stale_s
    gyro.poll()
    assert gyro.yaw() is None


def test_phyphox_skips_nulls_gaps_and_network_errors():
    phone, clock = FakePhone(w_z=1.0), FakeClock()
    gyro = PhyphoxGyro(HeadingConfig(), fetch=phone, clock=clock)
    gyro.poll()
    phone.pending = [(10.01, None), (10.02, 1.0), (12.0, 1.0)]   # null i dziura 2 s (pauza pomiaru)
    gyro.poll()
    assert abs(gyro.yaw() - 0.02) < 1e-6   # tylko 10.00 -> 10.02; przez dziure nie calkujemy
    phone.fail = True
    assert gyro.poll() == 0
    assert isinstance(gyro.last_error, OSError)


def test_phyphox_starts_measurement_when_stopped():
    phone, clock = FakePhone(measuring=False), FakeClock()
    gyro = PhyphoxGyro(HeadingConfig(), fetch=phone, clock=clock)
    gyro.poll()
    assert any("/control?cmd=start" in u for u in phone.urls)


def test_make_heading_sources():
    cfg = Config()
    assert make_heading(cfg) is None
    cfg.heading.source = "odometry"
    base = SimDrive(cfg)
    base.theta = 0.3
    assert make_heading(cfg, base).yaw() == 0.3


# --- pasy po kursie w symulacji -------------------------------------------------

def run_lanes(heading_kind: str, turn_gain=0.85, drift_w=0.03, cones=()):
    """Pusty (albo prawie) trawnik, naped z poslizgiem na obrotach i znoszeniem na prostej."""
    cfg = Config()
    cfg.sim.turn_gain = turn_gain
    cfg.sim.drift_w = drift_w
    base = SimDrive(cfg)
    x0, y0 = base.x, base.y
    world = SimWorld(cfg, base.odometry, cones=list(cones))
    calibrate_grasps(cfg, world)
    clock = SimClock(base)
    heading = None
    if heading_kind == "odometry":
        heading = OdometryHeading(base)
    elif heading_kind == "dead":
        class Dead:
            def yaw(self):
                return None
        heading = Dead()
    elif heading_kind == "inverted":
        class Inverted:
            def yaw(self):
                return -base.theta
        heading = Inverted()
    brain = Brain(cfg, SimCamera(world), HsvConeDetector(cfg.detector), base,
                  SimArmSimple(world, clock), clock=clock, verbose=False, heading=heading)
    brain.run(max_seconds=600)
    # idealny koniec: 4 pasy po 3 m co 1 m -> z powrotem na x0, 4 m w lewo, przodem wzdluz x
    c = cfg.control
    end_err = math.hypot(base.x - x0, base.y - (y0 + c.lane_count * c.lane_spacing_m))
    heading_err = abs(math.atan2(math.sin(base.theta), math.cos(base.theta)))
    return brain, world, end_err, heading_err


def test_lanes_with_heading_stay_on_pattern_despite_slip():
    brain, _, end_err, heading_err = run_lanes("odometry")
    assert brain.state == State.DONE
    assert end_err < 0.25, end_err
    assert heading_err < math.radians(5), math.degrees(heading_err)


def test_lanes_by_time_drift_with_same_slip():
    """Kontrola: bez kursu te same niedoskonalosci psuja wzorzec (to jest powod calej zmiany)."""
    brain, _, end_err, _ = run_lanes("none")
    assert brain.state == State.DONE
    assert end_err > 1.0, end_err


def test_lanes_fall_back_to_time_when_heading_dead():
    brain, _, _, _ = run_lanes("dead", turn_gain=1.0, drift_w=0.0)
    assert brain.state == State.DONE
    assert brain.heading is None
    assert any("bez kursu" in t[3] for t in brain.stats.transitions)


def test_lanes_fall_back_when_heading_sign_is_wrong():
    """Zly znak kursu (telefon ekranem w dol): obrot nigdy nie dochodzi, bezpiecznik przelacza na czas."""
    brain, _, _, _ = run_lanes("inverted", turn_gain=1.0, drift_w=0.0)
    assert brain.state == State.DONE
    assert brain.heading is None
    assert any("trwa za dlugo" in t[3] for t in brain.stats.transitions)


def test_lanes_with_heading_resume_after_collecting_cone():
    """Szyszka na pierwszym pasie: po chwycie robot wraca na kurs pasa i konczy wzorzec."""
    y0 = -Config().sim.field_m / 2 + 0.2
    brain, world, end_err, heading_err = run_lanes("odometry", cones=[(1.5, y0 + 0.15)])
    assert world.collected == 1
    assert brain.state == State.DONE
    assert heading_err < math.radians(5), math.degrees(heading_err)


def test_first_lane_direction_is_taken_before_approach_turns_robot():
    """Szyszka widoczna od razu z boku: podjazd obraca robota, ale pasy ida wzdluz kursu startowego."""
    y0 = -Config().sim.field_m / 2 + 0.2
    brain, world, end_err, heading_err = run_lanes("odometry", cones=[(0.8, y0 + 0.35)])
    assert world.collected == 1
    assert brain._h0 == 0.0
    assert end_err < 0.6, end_err   # przesuniecie z podjazdu (0.35 m w bok) zostaje, kurs nie
    assert heading_err < math.radians(5), math.degrees(heading_err)
