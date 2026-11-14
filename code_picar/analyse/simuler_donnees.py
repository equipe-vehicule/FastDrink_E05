#!/usr/bin/env python3
"""Crée des fichiers CSV SIMULÉS au format du robot, pour préparer l'analyse avant les vraies mesures.

Les valeurs (bruit, taux d'échecs, vitesses) sont des HYPOTHÈSES tirées de la littérature
sur le HC-SR04 ; tout fichier produit porte « SIMULE » dans sa note et son nom.

    python3 analyse/simuler_donnees.py -o donnees_simulees
"""
import argparse
import csv
import os
import random

T_MOY, T_JITTER = 0.060, 0.002     # une mesure toutes les 60 ms environ
SIGMA_MM = 3.0                     # bruit (essais publiés : 1 à 5 mm)
P_ECHEC, P_ABERRANT = 0.03, 0.02   # écho manqué / écho parasite


def mesure(d, rng):
    u = rng.random()
    if u < P_ECHEC:
        return -1.0
    if u < P_ECHEC + P_ABERRANT:
        return d + rng.uniform(100, 600)   # écho d'un objet plus loin dans le cône
    return d + rng.gauss(0, SIGMA_MM)


def ecrire(chemin, lignes):
    with open(chemin, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "z_mm", "d_reelle_mm", "commande", "note"])
        w.writerows(lignes)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-o", "--dossier", default="donnees_simulees")
    p.add_argument("--graine", type=int, default=5)
    a = p.parse_args()
    rng = random.Random(a.graine)
    os.makedirs(a.dossier, exist_ok=True)

    for d in (150, 300, 600):                       # jeu statique
        t, lignes = 0.0, []
        for i in range(300):
            lignes.append([f"{t:.4f}", f"{mesure(d, rng):.1f}", d, 0, "SIMULE statique" if i == 0 else ""])
            t += rng.gauss(T_MOY, T_JITTER)
        ecrire(os.path.join(a.dossier, f"SIMULE_statique_{d}.csv"), lignes)

    for v in (150, 300):                            # approches à vitesse constante (mm/s)
        for essai in range(1, 6):
            t, d, vit, lignes = 0.0, 1000.0, 0.0, []
            while d > 30:                           # départ arrêté, accélération sur 0,5 s
                lignes.append([f"{t:.4f}", f"{mesure(d, rng):.1f}", f"{d:.1f}", v,
                               f"SIMULE approche {v} mm/s" if not lignes else ""])
                dt = rng.gauss(T_MOY, T_JITTER)
                vit = min(v, vit + v / 0.5 * dt)
                t, d = t + dt, d - vit * dt
            ecrire(os.path.join(a.dossier, f"SIMULE_approche_v{v}_{essai}.csv"), lignes)
    print(f"Fichiers simulés écrits dans {a.dossier}/")


if __name__ == "__main__":
    main()
