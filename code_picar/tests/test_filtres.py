"""Tests du filtre A et du filtre final sur des signaux connus (sans robot).

    python3 -m pytest tests
"""
import math
import random

from fastdrink.filtres import FiltreA, FiltreFinal

T = 0.062


def test_filtre_a_supprime_une_valeur_isolee():
    f = FiltreA(tau_s=0.17)
    for i, z in enumerate([300, 301, 299, 300, 900, 300, 301, 300]):
        y = f.mettre_a_jour(z, i * T)
    assert abs(y - 300) < 2


def test_filtre_a_ignore_un_echo_manque():
    f = FiltreA(tau_s=0.17)
    for i in range(10):
        f.mettre_a_jour(300.0, i * T)
    assert abs(f.mettre_a_jour(-1.0, 10 * T) - 300) < 1e-9


def test_filtre_a_retard_theorique():
    """Sur une distance qui baisse à vitesse constante, le retard vaut T (1 + (1 - a) / a)."""
    tau, v = 0.17, 120.0
    f = FiltreA(tau_s=tau)
    for i in range(200):
        y = f.mettre_a_jour(1500.0 - v * i * T, i * T)
    a = 1 - math.exp(-T / tau)
    retard = (y - (1500.0 - v * 199 * T)) / v
    assert abs(retard - T * (1 + (1 - a) / a)) < 0.005


def test_filtre_final_suit_le_kalman_en_mouvement():
    f = FiltreFinal()
    rng = random.Random(1)
    for i in range(80):
        vrai = 1000.0 - 112.0 * i * T
        d = f.mettre_a_jour(vrai + rng.gauss(0, 2.4), i * T, en_mouvement=True)
    assert abs(d - vrai) < 10.0
    assert abs(f.vitesse() + 112.0) < 30.0


def test_filtre_final_sans_saut_au_passage_a_l_arret():
    f = FiltreFinal()
    rng = random.Random(2)
    for i in range(80):
        d_mouvement = f.mettre_a_jour(1000.0 - 112.0 * i * T + rng.gauss(0, 2.4), i * T, en_mouvement=True)
    d_arret = f.mettre_a_jour(1000.0 - 112.0 * 79 * T + rng.gauss(0, 2.4), 80 * T, en_mouvement=False)
    assert abs(d_arret - d_mouvement) < 10.0


def test_filtre_final_stable_a_l_arret():
    f = FiltreFinal()
    rng = random.Random(3)
    sorties = []
    for i in range(300):
        d = f.mettre_a_jour(300.0 + rng.gauss(0, 2.5), i * T, en_mouvement=False)
        if i > 20:
            sorties.append(d)
    moy = sum(sorties) / len(sorties)
    ecart_type = (sum((s - moy) ** 2 for s in sorties) / len(sorties)) ** 0.5
    assert ecart_type < 1.5
