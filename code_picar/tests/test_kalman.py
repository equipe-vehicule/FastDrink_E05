"""Tests du filtre de Kalman sur des signaux connus (sans robot).

    python3 -m pytest tests
"""
import random

from fastdrink.filtres import KalmanDistance, MedianeExponentiel

T = 0.06


def approche(v=300.0, d0=1000.0, n=50, bruit=3.0, graine=1):
    rng = random.Random(graine)
    return [(i * T, d0 - v * i * T, d0 - v * i * T + rng.gauss(0, bruit)) for i in range(n)]


def test_suit_une_approche_sans_retard():
    kf = KalmanDistance(sigma_z=3.0, sigma_a=600.0)
    for t, vrai, z in approche():
        d, v = kf.mettre_a_jour(z, t)
    assert abs(d - vrai) < 8.0
    assert abs(v - (-300.0)) < 40.0


def test_echo_manque_donne_la_prediction():
    kf = KalmanDistance(sigma_z=3.0, sigma_a=600.0)
    for t, _, z in approche(n=30):
        kf.mettre_a_jour(z, t)
    d_avant = kf.d
    d, _ = kf.mettre_a_jour(-1.0, 30 * T)
    assert d < d_avant                      # a continué d'avancer avec la vitesse estimée


def test_mesure_aberrante_ignoree():
    kf = KalmanDistance(sigma_z=3.0, sigma_a=600.0)
    for t, _, z in approche(n=30):
        kf.mettre_a_jour(z, t)
    t, vrai, _ = approche(n=31)[-1]
    d, _ = kf.mettre_a_jour(vrai + 400.0, t)  # écho parasite
    assert not kf.derniere_mesure_acceptee
    assert abs(d - vrai) < 10.0


def test_reinitialisation_apres_rejets_consecutifs():
    kf = KalmanDistance(sigma_z=3.0, sigma_a=600.0, rejets_avant_reset=5)
    for i in range(20):
        kf.mettre_a_jour(500.0, i * T)
    for i in range(20, 25):                 # l'obstacle a vraiment changé (nouvel objet à 200 mm)
        d, _ = kf.mettre_a_jour(200.0, i * T)
    assert abs(d - 200.0) < 1.0


def test_delai_avant_arret():
    kf = KalmanDistance(sigma_z=3.0, sigma_a=600.0)
    kf.d, kf.v, kf.t = 160.0, -200.0, 0.0
    assert abs(kf.delai_avant(150.0, 0.1) - 0.05) < 1e-9
    assert kf.delai_avant(150.0, 0.01) is None   # pas avant la prochaine mesure


def test_mediane_exponentiel_ignore_une_valeur_isolee():
    f = MedianeExponentiel(n=3, alpha=0.3)
    for z in [300, 301, 299, 300, 900, 300, 301]:
        y = f.mettre_a_jour(z)
    assert abs(y - 300) < 2


def test_mediane_kalman_echos_alternes_sans_derive():
    """Obstacle fixe à ~550 mm, un écho sur deux manqué (virage) : la sortie ne dérive pas vers 0 (écart n° 1, 14 nov.)."""
    from fastdrink.filtres import MedianeKalman
    rng = random.Random(3)
    f = MedianeKalman()
    d = None
    for i in range(60):
        z = -1.0 if i % 2 else 550.0 + rng.gauss(0, 2.4)
        d, _ = f.mettre_a_jour(z, i * T)
    assert abs(d - 550.0) < 15.0


def test_mediane_kalman_vitesse_impossible_redemarre():
    """Un saut de 660 à 572 mm en 62 ms (-1400 mm/s) dépasse VITESSE_MAX_MM_S : le filtre repart de la mesure."""
    from fastdrink import config
    from fastdrink.filtres import MedianeKalman
    kf = KalmanDistance(sigma_z=2.4, sigma_a=1000.0, seuil_rejet=1e9, vitesse_max_mm_s=config.VITESSE_MAX_MM_S)
    for i in range(10):
        kf.mettre_a_jour(660.0, i * T)
    d, v = kf.mettre_a_jour(572.0, 10 * T)
    assert v == 0.0 and d == 572.0               # redémarré sur la mesure
    sans = KalmanDistance(sigma_z=2.4, sigma_a=1000.0, seuil_rejet=1e9)
    for i in range(10):
        sans.mettre_a_jour(660.0, i * T)
    _, v2 = sans.mettre_a_jour(572.0, 10 * T)
    assert abs(v2) > config.VITESSE_MAX_MM_S      # sans contrôle (None), la vitesse impossible est gardée
    assert MedianeKalman().kf.vitesse_max_mm_s == config.VITESSE_MAX_MM_S
