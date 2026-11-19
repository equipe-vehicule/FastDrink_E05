#!/usr/bin/env python3
"""Essai du capteur ultrason sur le robot : compare chaque mesure brute avec la
sortie du filtre final, puis affiche un bilan avec et sans filtre.

    python3 essai_ultrason.py                       # 50 mesures, robot à l'arrêt
    python3 essai_ultrason.py -n 200 -d 300         # 200 mesures, obstacle à 300 mm à la règle
    python3 essai_ultrason.py -n 100 --mouvement    # robot en mouvement (filtre en mode Kalman)
"""
import argparse
import statistics
import time

from fastdrink.capteurs import Ultrason, MESURE_RATEE
from fastdrink.config import ULTRASONIC_OFFSET
from fastdrink.filtres import FiltreFinal

N_IGNORE = 20  # premières sorties filtrées ignorées dans le bilan (temps de convergence)


def bilan(nom, valeurs, d_reelle):
    if len(valeurs) < 2:
        print(f"{nom:8s}: pas assez de mesures")
        return
    moy = statistics.mean(valeurs)
    ligne = (f"{nom:8s}: moyenne {moy:7.1f} mm  écart type {statistics.stdev(valeurs):5.1f} mm  "
             f"min {min(valeurs):7.1f}  max {max(valeurs):7.1f}")
    if d_reelle is not None:
        ligne += f"  erreur {moy - d_reelle:+.1f} mm"
    print(ligne)


def main():
    p = argparse.ArgumentParser(description="Essai du capteur ultrason, avec et sans filtre")
    p.add_argument("-n", type=int, default=50, help="nombre de mesures")
    p.add_argument("-d", type=float, default=None, help="distance réelle à la règle (mm)")
    p.add_argument("--mouvement", action="store_true",
                   help="le robot est en mouvement (sinon : à l'arrêt)")
    args = p.parse_args()

    filtre = FiltreFinal()
    bruts, filtrees = [], []
    d_filtre = None
    nb_ratees = 0

    print(f"Mode : {'en mouvement' if args.mouvement else 'arrêt'}")
    print(f"{'i':>4s} {'t (s)':>7s} {'brut (mm)':>10s} {'filtré (mm)':>12s} {'écart':>7s}")

    with Ultrason() as capteur:
        t0 = time.monotonic()
        for i in range(1, args.n + 1):
            brut = capteur.mesurer_mm()
            t = time.monotonic() - t0

            if brut == MESURE_RATEE:
                # Écho manqué : on ne donne rien au filtre, il garde sa dernière valeur.
                nb_ratees += 1
                txt_filtre = f"{d_filtre:12.1f}" if d_filtre is not None else f"{'-':>12s}"
                print(f"{i:4d} {t:7.3f} {'ratée':>10s} {txt_filtre}")
                continue

            d = brut - ULTRASONIC_OFFSET
            d_filtre = filtre.mettre_a_jour(d, t, en_mouvement=args.mouvement)
            bruts.append(d)
            filtrees.append(d_filtre)
            print(f"{i:4d} {t:7.3f} {d:10.1f} {d_filtre:12.1f} {d_filtre - d:+7.1f}")

    print(f"\nMesures : {args.n}  ratées : {nb_ratees}")
    print(f"(bilan calculé sans les {N_IGNORE} premières mesures valides, le temps que le filtre converge)")
    bilan("Sans filtre", bruts[N_IGNORE:], args.d)
    bilan("Avec filtre", filtrees[N_IGNORE:], args.d)
    if args.mouvement and filtrees:
        print(f"Vitesse estimée en fin d'essai : {filtre.vitesse():.1f} mm/s")


if __name__ == "__main__":
    main()
