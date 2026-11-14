#!/usr/bin/env python3
"""Approches SIMULÉES construites avec le bruit RÉEL du capteur (en attendant des enregistrements
en mouvement). Le bruit est tiré au hasard parmi les écarts mesurés robot immobile (bootstrap),
donc il garde la vraie forme du bruit, valeurs extrêmes comprises.

    python3 analyse/approches_bruit_mesure.py --statique donnees_reelles/statique_*.csv \
        --vitesses 48 112 256 -o approches_bruit_reel

Hypothèses (à remplacer dès qu'on a des CSV en mouvement) : départ arrêté avec accélération
sur 0,5 s, une mesure toutes les 62 ms, bruit indépendant de la vitesse.
"""
import argparse
import csv
import os
import random
import statistics

T_MOY, T_JITTER = 0.062, 0.002


def residus(fichiers):
    r = []
    for f in fichiers:
        with open(f, newline="") as fh:
            z = [float(l["z_mm"]) for l in csv.DictReader(fh) if float(l["z_mm"]) != -1]
        m = statistics.median(z)
        r += [x - m for x in z]
    return r


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--statique", nargs="+", required=True)
    p.add_argument("--vitesses", nargs="+", type=float, required=True, help="mm/s")
    p.add_argument("--depart", type=float, default=1000.0, help="distance capteur au départ (mm)")
    p.add_argument("--fin", type=float, default=40.0, help="distance capteur à la fin (mm)")
    p.add_argument("--essais", type=int, default=5)
    p.add_argument("-o", "--dossier", default="approches_bruit_reel")
    p.add_argument("--graine", type=int, default=5)
    a = p.parse_args()
    rng = random.Random(a.graine)
    r = residus(a.statique)
    os.makedirs(a.dossier, exist_ok=True)
    for v in a.vitesses:
        for essai in range(1, a.essais + 1):
            t, d, vit, lignes = 0.0, a.depart, 0.0, []
            while d > a.fin:
                lignes.append([f"{t:.4f}", f"{d + rng.choice(r):.1f}", f"{d:.1f}", v,
                               f"SIMULE bruit reel, approche {v:.0f} mm/s" if not lignes else ""])
                dt = rng.gauss(T_MOY, T_JITTER)
                vit = min(v, vit + v / 0.5 * dt)
                t, d = t + dt, d - vit * dt
            chemin = os.path.join(a.dossier, f"SIMULE_bruitreel_v{v:.0f}_{essai}.csv")
            with open(chemin, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "z_mm", "d_reelle_mm", "commande", "note"])
                w.writerows(lignes)
    print(f"{len(r)} écarts réels utilisés ; fichiers dans {a.dossier}/")


if __name__ == "__main__":
    main()
