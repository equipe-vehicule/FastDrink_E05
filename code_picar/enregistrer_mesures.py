#!/usr/bin/env python3
"""Enregistre les mesures BRUTES du capteur ultrason dans un fichier CSV (sur le robot).

Format : t_s,z_mm,d_reelle_mm,commande,note   (z_mm = -1 si l'écho manque)

    python3 enregistrer_mesures.py -o donnees/statique_300.csv -n 300 -d 300 --note "statique 30 cm"
    python3 enregistrer_mesures.py -o donnees/approche_c30_1.csv --duree 8 -c 30 --note "approche cmd 30"

Ce script ne pilote pas les moteurs : -c sert seulement à noter la commande utilisée.
Ctrl+C arrête l'enregistrement proprement (le fichier est gardé).
"""
import argparse
import csv
import os
import time

from fastdrink.capteurs import Ultrason


def main():
    p = argparse.ArgumentParser(description="Enregistrement des mesures brutes de l'ultrason")
    p.add_argument("-o", "--sortie", required=True, help="fichier CSV à créer")
    p.add_argument("-n", type=int, default=None, help="nombre de mesures")
    p.add_argument("--duree", type=float, default=None, help="durée d'enregistrement (s)")
    p.add_argument("-d", type=float, default=None, help="distance réelle à la règle (mm), si fixe")
    p.add_argument("-c", "--commande", default="", help="commande moteur utilisée (pour mémoire)")
    p.add_argument("--note", default="", help="description de l'essai")
    args = p.parse_args()
    if args.n is None and args.duree is None:
        p.error("donner -n ou --duree")

    os.makedirs(os.path.dirname(args.sortie) or ".", exist_ok=True)
    d_reelle = "" if args.d is None else args.d
    n = 0
    with Ultrason() as capteur, open(args.sortie, "w", newline="") as f:
        ecrivain = csv.writer(f)
        ecrivain.writerow(["t_s", "z_mm", "d_reelle_mm", "commande", "note"])
        t0 = time.perf_counter()
        try:
            while True:
                z = capteur.mesurer_mm()
                t = time.perf_counter() - t0
                ecrivain.writerow([f"{t:.4f}", f"{z:.1f}", d_reelle, args.commande,
                                   args.note if n == 0 else ""])
                n += 1
                if (args.n is not None and n >= args.n) or (args.duree is not None and t >= args.duree):
                    break
        except KeyboardInterrupt:
            pass
    print(f"{n} mesures enregistrées dans {args.sortie}")


if __name__ == "__main__":
    main()
