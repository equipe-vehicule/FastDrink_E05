#!/usr/bin/env python3
"""Mesure 3 : fréquence réelle de la boucle de commande sur le Pi.

La simulation suppose 50 Hz (fiche : seuil de 20 %). On mesure la période de la boucle :
  - sans ultrason (boucle à vide : plafond du Pi) ;
  - avec une mesure ultrason bloquante par tour (cas réel, limité à ~16 Hz par les 60 ms minimales du capteur).
Aucun moteur n'est commandé : le robot reste immobile.

    python3 mesures/mesure_boucle.py            # 500 tours par cas
    python3 mesures/mesure_boucle.py -n 1000
"""
import argparse
import statistics
import time

from commun import ouvrir_csv
from fastdrink.capteurs import Ultrason
from fastdrink.filtres import FiltreFinal


def periodes(n, tour):
    p = []
    t_prec = time.perf_counter()
    for _ in range(n):
        tour()
        t = time.perf_counter()
        p.append(t - t_prec)
        t_prec = t
    return p


def resume(nom, p):
    moy = statistics.mean(p)
    print(f"{nom:28s} période moyenne {moy*1000:7.2f} ms -> {1/moy:6.1f} Hz ; "
          f"écart type {statistics.stdev(p)*1000:6.2f} ms ; pire {max(p)*1000:7.2f} ms")


def main():
    ap = argparse.ArgumentParser(description="Fréquence de la boucle de commande")
    ap.add_argument("-n", type=int, default=500, help="nombre de tours par cas")
    args = ap.parse_args()

    f, w = ouvrir_csv("boucle", ["cas", "tour", "periode_ms"])
    try:
        filtre = FiltreFinal()
        t0 = time.perf_counter()
        a_vide = periodes(args.n, lambda: filtre.mettre_a_jour(300.0, time.perf_counter() - t0, False))
        resume("Filtre seul (sans capteur)", a_vide)
        with Ultrason() as capteur:
            avec = periodes(args.n, lambda: filtre.mettre_a_jour(capteur.mesurer_mm(),
                                                                 time.perf_counter() - t0, False))
        resume("Avec mesure ultrason", avec)
        for cas, p in (("filtre_seul", a_vide), ("avec_ultrason", avec)):
            for i, v in enumerate(p, 1):
                w.writerow([cas, i, f"{v*1000:.3f}"])
    finally:
        f.close()
    print(f"Fichier : {f.chemin}")


if __name__ == "__main__":
    main()
