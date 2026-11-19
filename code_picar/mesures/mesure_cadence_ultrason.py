#!/usr/bin/env python3
"""Mesure 4 : cadence réelle de l'ultrason (mesures par seconde) et part de mesures ratées.

Robot immobile. Chaque mesure brute est horodatée ; la fiche attend environ 16 Hz (seuil 15 %).

    python3 mesures/mesure_cadence_ultrason.py            # 300 mesures
    python3 mesures/mesure_cadence_ultrason.py -n 600
"""
import argparse
import statistics
import time

from commun import ouvrir_csv
from fastdrink.capteurs import Ultrason, MESURE_RATEE


def main():
    ap = argparse.ArgumentParser(description="Cadence de l'ultrason")
    ap.add_argument("-n", type=int, default=300, help="nombre de mesures")
    args = ap.parse_args()

    f, w = ouvrir_csv("cadence_ultrason", ["i", "t_s", "brut_mm", "periode_ms"])
    instants, ratees = [], 0
    try:
        with Ultrason() as capteur:
            t0 = time.perf_counter()
            t_prec = None
            for i in range(1, args.n + 1):
                z = capteur.mesurer_mm()
                t = time.perf_counter() - t0
                per = None if t_prec is None else (t - t_prec) * 1000
                t_prec = t
                if z == MESURE_RATEE:
                    ratees += 1
                instants.append(t)
                w.writerow([i, f"{t:.4f}", "" if z == MESURE_RATEE else f"{z:.1f}",
                            "" if per is None else f"{per:.2f}"])
    finally:
        f.close()
    per = [(b - a) * 1000 for a, b in zip(instants, instants[1:])]
    moy = statistics.mean(per)
    print(f"{args.n} mesures en {instants[-1]:.1f} s : période moyenne {moy:.1f} ms -> {1000/moy:.1f} Hz "
          f"(écart type {statistics.stdev(per):.1f} ms, pire {max(per):.1f} ms)")
    print(f"Mesures ratées : {ratees} ({100*ratees/args.n:.1f} %)")
    print(f"Fichier : {f.chemin}")


if __name__ == "__main__":
    main()
