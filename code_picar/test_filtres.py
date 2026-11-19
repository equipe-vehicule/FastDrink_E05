"""Tests du filtre final sur des signaux connus (sans robot).

    python3 -m pytest tests -s        # -s est obligatoire pour voir les print
"""
import random

from fastdrink.filtres import FiltreFinal

T = 0.062


def test_filtre_final_suit_le_kalman_en_mouvement():
    print("\n--- Filtre final : en mouvement à -112 mm/s ---")
    f = FiltreFinal()
    rng = random.Random(1)
    for i in range(80):
        vrai = 1000.0 - 112.0 * i * T
        mesure = vrai + rng.gauss(0, 2.4)
        d = f.mettre_a_jour(mesure, i * T, en_mouvement=True)
        if i % 10 == 0 or i == 79:
            print(f"t={i * T:6.3f} s  vraie={vrai:8.2f}  mesure={mesure:8.2f}  filtré={d:8.2f} mm  "
                  f"v={f.vitesse():7.1f} mm/s")
    print(f"Écart final = {d - vrai:.2f} mm, vitesse estimée = {f.vitesse():.1f} mm/s (attendu -112)")
    assert abs(d - vrai) < 10.0
    assert abs(f.vitesse() + 112.0) < 30.0


def test_filtre_final_sans_saut_au_passage_a_l_arret():
    print("\n--- Filtre final : passage mouvement -> arrêt ---")
    f = FiltreFinal()
    rng = random.Random(2)
    for i in range(80):
        d_mouvement = f.mettre_a_jour(1000.0 - 112.0 * i * T + rng.gauss(0, 2.4), i * T, en_mouvement=True)
    d_arret = f.mettre_a_jour(1000.0 - 112.0 * 79 * T + rng.gauss(0, 2.4), 80 * T, en_mouvement=False)
    print(f"Dernière distance en mouvement = {d_mouvement:.2f} mm")
    print(f"Première distance à l'arrêt   = {d_arret:.2f} mm")
    print(f"Saut = {d_arret - d_mouvement:.2f} mm (doit être < 10)")
    assert abs(d_arret - d_mouvement) < 10.0


def test_filtre_final_stable_a_l_arret():
    print("\n--- Filtre final : stabilité à l'arrêt (300 mm) ---")
    f = FiltreFinal()
    rng = random.Random(3)
    sorties = []
    for i in range(300):
        d = f.mettre_a_jour(300.0 + rng.gauss(0, 2.5), i * T, en_mouvement=False)
        if i % 30 == 0:
            print(f"t={i * T:6.3f} s  filtré={d:7.2f} mm")
        if i > 20:
            sorties.append(d)
    moy = sum(sorties) / len(sorties)
    ecart_type = (sum((s - moy) ** 2 for s in sorties) / len(sorties)) ** 0.5
    print(f"Moyenne = {moy:.2f} mm, écart-type = {ecart_type:.2f} mm (doit être < 1.5)")
    assert ecart_type < 1.5
