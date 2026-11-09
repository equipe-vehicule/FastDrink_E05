#!/usr/bin/env python3
"""Essai du capteur ultrason sur le robot : affiche chaque mesure puis un bilan.

    python3 essai_ultrason.py                 # 50 mesures
    python3 essai_ultrason.py -n 200 -d 300   # 200 mesures, obstacle à 300 mm à la règle
"""
import argparse
import statistics

from fastdrink.capteurs import Ultrason, MESURE_RATEE


def main():
    p = argparse.ArgumentParser(description="Essai du capteur ultrason")
    p.add_argument("-n", type=int, default=50, help="nombre de mesures")
    p.add_argument("-d", type=float, default=None, help="distance réelle à la règle (mm)")
    args = p.parse_args()

    with Ultrason() as capteur:
        mesures = []
        for i in range(1, args.n + 1):
            d = capteur.mesurer_mm()
            mesures.append(d)
            # print(f"{i:4d}  " + ("mesure ratée (-1)" if d == MESURE_RATEE else f"{d:7.1f} mm"))
            print(f"{i:4d}"))

    valides = [d for d in mesures if d != MESURE_RATEE]
    print(f"\nMesures : {len(mesures)}  ratées : {len(mesures) - len(valides)}")
    if len(valides) >= 2:
        moy = statistics.mean(valides)
        print(f"Moyenne : {moy:.1f} mm  écart type : {statistics.stdev(valides):.1f} mm  "
              f"min : {min(valides):.1f} mm  max : {max(valides):.1f} mm")
        if args.d is not None:
            print(f"Erreur moyenne : {moy - args.d:+.1f} mm")


if __name__ == "__main__":
    main()
