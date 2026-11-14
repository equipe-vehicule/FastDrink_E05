"""Tests du pilote ultrason sur ordinateur, avec un faux GPIO qui imite le capteur.

    python3 -m pytest tests
"""
import time

import pytest

from fastdrink.capteurs import Ultrason, MESURE_RATEE

TRIG, ECHO = 12, 16


class FauxGPIO:
    """Imite RPi.GPIO : après chaque impulsion, ECHO passe à 1 pendant la durée
    correspondant à `distance_mm`. distance_mm = None : pas d'écho."""
    BCM, OUT, IN, LOW, HIGH = "BCM", "OUT", "IN", 0, 1

    def __init__(self, distances_mm, delai_s=0.0005):
        self.distances = list(distances_mm)  # une valeur par mesure, dans l'ordre
        self.delai_s = delai_s               # délai entre l'impulsion et le début de l'écho
        self.debut_echo = None
        self.duree_echo = 0.0
        self.trig_haut = False
        self.nettoye = None

    def setmode(self, mode): pass
    def setwarnings(self, flag): pass
    def setup(self, pin, mode, initial=None): pass

    def output(self, pin, valeur):
        if pin == TRIG and valeur == self.HIGH:
            self.trig_haut = True
        elif pin == TRIG and valeur == self.LOW and self.trig_haut:  # fin de l'impulsion
            self.trig_haut = False
            d = self.distances.pop(0)
            if d is None:
                self.debut_echo = None
            else:
                self.debut_echo = time.perf_counter() + self.delai_s
                self.duree_echo = 2 * d / 1000.0 / 343.0

    def input(self, pin):
        if pin != ECHO or self.debut_echo is None:
            return 0
        t = time.perf_counter()
        return 1 if self.debut_echo <= t < self.debut_echo + self.duree_echo else 0

    def cleanup(self, pins=None):
        self.nettoye = pins


def capteur(distances, **kw):
    kw.setdefault("periode_min_s", 0.0)
    return Ultrason(trig=TRIG, echo=ECHO, gpio=FauxGPIO(distances), **kw)


def test_mesure_correcte():
    c = capteur([300])
    assert c.mesurer_mm() == pytest.approx(300, abs=10)


def test_pas_d_echo_renvoie_mesure_ratee_dans_le_delai():
    c = capteur([None], timeout_s=0.02)
    t0 = time.perf_counter()
    assert c.mesurer_mm() == MESURE_RATEE
    assert time.perf_counter() - t0 < 0.1


def test_moyenne_exclut_les_mesures_ratees():
    c = capteur([300, None, 310, None, 320])
    assert c.moyenne_mm(5) == pytest.approx(310, abs=10)


def test_moyenne_sans_mesure_valide():
    c = capteur([None, None])
    assert c.moyenne_mm(2) == MESURE_RATEE


def test_periode_minimale_respectee():
    c = capteur([300, 300], periode_min_s=0.06)
    t0 = time.perf_counter()
    c.mesurer_mm()
    c.mesurer_mm()
    assert time.perf_counter() - t0 >= 0.06


def test_interface_du_code_de_depart_en_cm():
    c = capteur([300] * 5 + [300] * 5 + [None] * 5)
    assert c.get_distance() == pytest.approx(30, abs=1)
    assert c.less_than(40) == 1
    assert c.less_than(20) == -1  # cinq mesures ratées


def test_fermer_libere_les_deux_broches():
    with capteur([]) as c:
        gpio = c._gpio
    assert gpio.nettoye == [TRIG, ECHO]
