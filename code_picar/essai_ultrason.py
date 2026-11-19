#!/usr/bin/env python3
"""Essai du capteur ultrason sur le robot : compare chaque mesure brute avec la
sortie du filtre final (FiltreFinal), puis affiche un bilan sans filtre / avec filtre.

Les distances affichées sont mesurées depuis l'AVANT du véhicule (mesure capteur - CAPTEUR_AVANT_MM).

    python3 essai_ultrason.py                       # 50 mesures, robot à l'arrêt (filtre A)
    python3 essai_ultrason.py -n 200 -d 300         # 200 mesures, obstacle à 300 mm de l'avant, à la règle
    python3 essai_ultrason.py -n 100 --mouvement    # robot en mouvement (filtre B, Kalman)
"""
import argparse
import statistics
import time

from fastdrink.capteurs import Ultrason, MESURE_RATEE
from fastdrink.config import CAPTEUR_AVANT_MM
from fastdrink.filtres import FiltreFinal

N_IGNORE = 20  # premières mesures ignorées dans le bilan (temps de convergence du filtre)


def bilan(nom, valeurs, d_reelle):
    if len(valeurs) < 2:
        print(f"{nom} : pas assez de mesures")
        return
    moy = statistics.mean(valeurs)
    ligne = (f"{nom} : moyenne {moy:7.1f} mm  écart type {statistics.stdev(valeurs):5.1f} mm  "
             f"min {min(valeurs):7.1f}  max {max(valeurs):7.1f}")
    if d_reelle is not None:
        ligne += f"  erreur {moy - d_reelle:+.1f} mm"
    print(ligne)


def main():
    p = argparse.ArgumentParser(description="Essai du capteur ultrason, avec et sans filtre")
    p.add_argument("-n", type=int, default=50, help="nombre de mesures")
    p.add_argument("-d", type=float, default=None,
                   help="distance réelle obstacle -> avant du véhicule, à la règle (mm)")
    p.add_argument("--mouvement", action="store_true",
                   help="le robot est en mouvement (sinon : à l'arrêt)")
    args = p.parse_args()

    filtre = FiltreFinal()
    bruts, filtrees = [], []
    nb_ratees = 0

    print(f"Mode : {'en mouvement (filtre B, Kalman)' if args.mouvement else 'arrêt (filtre A)'}")
    print(f"{'i':>4s} {'t (s)':>7s} {'brut (mm)':>10s} {'filtré (mm)':>12s} {'écart':>7s}")

    with Ultrason() as capteur:
        t0 = time.perf_counter()
        for i in range(1, args.n + 1):
            z = capteur.mesurer_mm()               # mesure brute du capteur, ou MESURE_RATEE
            t = time.perf_counter() - t0
            # Le filtre travaille sur la mesure brute (c'est sur elle qu'il a été réglé)
            # et gère lui-même les mesures ratées.
            sortie = filtre.mettre_a_jour(z, t, en_mouvement=args.mouvement)
            d_filtre = None if sortie is None else sortie - CAPTEUR_AVANT_MM
            txt_filtre = f"{d_filtre:12.1f}" if d_filtre is not None else f"{'-':>12s}"

            if z == MESURE_RATEE:
                nb_ratees += 1
                print(f"{i:4d} {t:7.3f} {'ratée':>10s} {txt_filtre}")
                continue

            d = z - CAPTEUR_AVANT_MM
            bruts.append(d)
            if d_filtre is not None:
                filtrees.append(d_filtre)
            ecart = f"{d_filtre - d:+7.1f}" if d_filtre is not None else ""
            print(f"{i:4d} {t:7.3f} {d:10.1f} {txt_filtre} {ecart}")

    print(f"\nMesures : {args.n}  ratées : {nb_ratees}")
    print(f"(bilan sans les {N_IGNORE} premières mesures valides, le temps que le filtre converge)")
    if args.mouvement:
        # La distance change pendant l'essai : moyenne et écart type n'ont pas de sens ici.
        print("En mouvement : comparer les colonnes brut / filtré ligne par ligne.")
        print(f"Vitesse estimée en fin d'essai : {filtre.vitesse():.1f} mm/s")
    else:
        bilan("Sans filtre", bruts[N_IGNORE:], args.d)
        bilan("Avec filtre", filtrees[N_IGNORE:], args.d)


if __name__ == "__main__":
    main()
