#!/usr/bin/env python3
"""Convertit la feuille Statique du classeur Mesures_Filtres_E05.xlsx en fichiers CSV pour l'analyse.

Les mesures du classeur sont comptées depuis l'AVANT du véhicule (décalage déjà retiré) ;
on remet le décalage pour revenir à la distance vue par le capteur, celle que filtrent les filtres.
Le classeur ne contient pas les instants de mesure : t_s est mis à la période nominale (T_NOMINAL).

    python3 analyse/xlsx_vers_csv.py Mesures_Filtres_E05.xlsx -o donnees_reelles
"""
import argparse
import csv
import os
import sys

import openpyxl

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from fastdrink import config  # noqa: E402

T_NOMINAL = 0.062   # s : 60 ms d'attente minimale + durée de la mesure (instants non enregistrés)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("classeur")
    p.add_argument("-o", "--dossier", default="donnees_reelles")
    a = p.parse_args()
    os.makedirs(a.dossier, exist_ok=True)
    ws = openpyxl.load_workbook(a.classeur, data_only=True)["Statique"]
    for col in "BCDE":
        d_avant = ws[f"{col}6"].value
        valeurs = [ws[f"{col}{r}"].value for r in range(9, 1009)]
        valeurs = [float(v) for v in valeurs if v not in (None, "")]   # nombres stockés en texte
        if d_avant is None or not valeurs:
            continue
        off = config.CAPTEUR_AVANT_MM
        chemin = os.path.join(a.dossier, f"statique_{int(d_avant)}mm_avant.csv")
        with open(chemin, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "z_mm", "d_reelle_mm", "commande", "note"])
            for i, z in enumerate(valeurs):
                z_capteur = -1.0 if z == -1 else z + off
                w.writerow([f"{i * T_NOMINAL:.4f}", f"{z_capteur:.1f}", float(d_avant) + off, 0,
                            f"statique {d_avant} mm avant (+{off} mm capteur), temps nominal" if i == 0 else ""])
        print(f"{chemin} : {len(valeurs)} mesures")


if __name__ == "__main__":
    main()
