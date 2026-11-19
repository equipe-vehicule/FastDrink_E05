#!/usr/bin/env python3
"""Trace un essai enregistré par essai_ultrason.py (-o) : mesures brutes contre sortie du filtre final.

    python3 tracer_essai.py essai.csv              # crée essai.png
    python3 tracer_essai.py essai.csv -d 300       # ajoute la distance réelle à la règle
    python3 tracer_essai.py essai.csv --afficher   # ouvre aussi la fenêtre (sur un PC avec écran)
"""
import argparse
import csv

import matplotlib


def main():
    p = argparse.ArgumentParser(description="Graphique brut / filtré d'un essai ultrason")
    p.add_argument("csv", help="fichier CSV produit par essai_ultrason.py -o")
    p.add_argument("-d", type=float, default=None, help="distance réelle à la règle (mm)")
    p.add_argument("--seuil", type=float, default=7.2, help="écart brut/filtré marqué comme pic (mm)")
    p.add_argument("--afficher", action="store_true", help="ouvrir la fenêtre en plus du PNG")
    p.add_argument("-o", default=None, help="image de sortie (par défaut : même nom que le CSV, en .png)")
    args = p.parse_args()

    if not args.afficher:
        matplotlib.use("Agg")  # pas besoin d'écran (Raspberry Pi en SSH)
    import matplotlib.pyplot as plt

    t_b, brut, t_f, filtre, t_r = [], [], [], [], []
    pics_t, pics_v = [], []
    with open(args.csv, newline="") as f:
        for ligne in csv.DictReader(f):
            t = float(ligne["t_s"])
            b = float(ligne["brut_mm"]) if ligne["brut_mm"] else None
            y = float(ligne["filtre_mm"]) if ligne["filtre_mm"] else None
            if b is None:
                t_r.append(t)
            else:
                t_b.append(t)
                brut.append(b)
            if y is not None:
                t_f.append(t)
                filtre.append(y)
            if b is not None and y is not None and abs(b - y) > args.seuil:
                pics_t.append(t)
                pics_v.append(b)

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t_b, brut, ".", color="0.6", markersize=4, label="mesure brute")
    ax.plot(t_f, filtre, "-", color="tab:blue", linewidth=2, label="filtre final")
    if pics_t:
        ax.plot(pics_t, pics_v, "o", color="tab:red", fillstyle="none", markersize=9,
                label=f"pics corrigés ({len(pics_t)})")
    if args.d is not None:
        ax.axhline(args.d, color="black", linestyle="--", linewidth=1, label=f"distance réelle ({args.d:g} mm)")
    for t in t_r:
        ax.axvline(t, color="tab:orange", alpha=0.3, linewidth=1)
    if t_r:
        ax.plot([], [], color="tab:orange", alpha=0.5, label=f"mesures ratées ({len(t_r)})")

    ax.set_xlabel("temps (s)")
    ax.set_ylabel("distance obstacle -> avant du véhicule (mm)")
    ax.set_title("Capteur ultrason : mesures brutes et sortie du filtre final")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()

    sortie = args.o or args.csv.rsplit(".", 1)[0] + ".png"
    fig.savefig(sortie, dpi=150)
    print(f"Graphique enregistré : {sortie}")
    if args.afficher:
        plt.show()


if __name__ == "__main__":
    main()
