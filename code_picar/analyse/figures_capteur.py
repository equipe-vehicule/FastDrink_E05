#!/usr/bin/env python3
"""Figures de caractérisation du capteur à partir des mesures RÉELLES du 9 nov. :
histogrammes du bruit, biais (mesure vs règle) et retard observé dans les essais en mouvement.

    python3 analyse/figures_capteur.py Mesures_Filtres_E05.xlsx -o figures
"""
import argparse
import os

import numpy as np
import openpyxl
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GRIS, BLEU, ROUGE = "#9a9a9a", "#1f5fbf", "#c0392b"


def lire_statique(ws):
    series = {}
    for col in "BCDE":
        d = ws[f"{col}6"].value
        z = [float(ws[f"{col}{r}"].value) for r in range(9, 1009) if ws[f"{col}{r}"].value not in (None, "")]
        if d is not None and z:
            series[float(d)] = np.array([v for v in z if v != -1])
    return series


def lire_mouvement(ws):
    essais = []
    for r in range(7, 40):
        cmd, dep, dem, fin, temps = (ws.cell(r, c).value for c in (2, 3, 4, 5, 7))
        if None in (cmd, dep, dem, fin, temps):
            continue
        v = (dep - fin) / temps                     # mm/s, moyenne sur tout l'essai
        essais.append({"cmd": cmd, "v": v, "depassement": dem - fin, "contact": fin == 0})
    return essais


def main():
    p = argparse.ArgumentParser()
    p.add_argument("classeur")
    p.add_argument("-o", "--sortie", default="figures")
    a = p.parse_args()
    os.makedirs(a.sortie, exist_ok=True)
    wb = openpyxl.load_workbook(a.classeur, data_only=True)
    st = lire_statique(wb["Statique"])

    # A) histogrammes du bruit autour de la médiane
    fig, axes = plt.subplots(1, len(st), figsize=(11, 3.4), sharey=True)
    for ax, (d, z) in zip(axes, sorted(st.items())):
        ecarts = z - np.median(z)
        ax.hist(ecarts, bins=np.arange(-21, 21, 1), color=BLEU, alpha=0.85)
        ax.set_yscale("log")
        ax.set_title(f"{d:.0f} mm : écart type {z.std(ddof=1):.1f} mm")
        ax.set_xlabel("écart à la médiane (mm)")
    axes[0].set_ylabel("nombre de mesures (échelle log)")
    fig.suptitle("Bruit du capteur, robot immobile, 1000 mesures par distance (mesures réelles)")
    fig.tight_layout(); fig.savefig(os.path.join(a.sortie, "figA_bruit.png"), dpi=160); plt.close(fig)

    # B) biais : moyenne mesurée en fonction de la distance à la règle
    d = np.array(sorted(st)); m = np.array([st[k].mean() for k in d])
    pente, ordo = np.polyfit(d, m, 1)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    axes[0].plot([0, 650], [0, 650], "k--", lw=0.8, label="mesure parfaite")
    axes[0].plot(d, m, "o", color=BLEU, label="moyenne mesurée")
    axes[0].plot([0, 650], [ordo, pente * 650 + ordo], color=BLEU, lw=1,
                 label=f"droite : {pente:.3f} × d + {ordo:.1f} mm")
    axes[0].set(xlabel="distance à la règle, avant du robot (mm)", ylabel="distance mesurée (mm)")
    axes[0].legend(fontsize=8)
    axes[1].axhline(0, color="k", lw=0.8)
    axes[1].bar(d, m - d, width=40, color=BLEU)
    for x, y in zip(d, m - d):
        axes[1].text(x, y + (0.5 if y > 0 else -1.2), f"{y:+.1f}", ha="center", fontsize=9)
    axes[1].set_ylim(min(m - d) - 3, max(m - d) + 3)
    axes[1].set(xlabel="distance à la règle (mm)", ylabel="erreur moyenne (mm)")
    fig.suptitle("Biais du capteur : corrigé par l'étalonnage, pas par le filtre (mesures réelles)")
    fig.tight_layout(); fig.savefig(os.path.join(a.sortie, "figB_biais.png"), dpi=160); plt.close(fig)

    # C) dépassement de la distance d'arrêt en fonction de la vitesse
    ess = lire_mouvement(wb["Mouvement"])
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for e in ess:
        ax.plot(e["v"], e["depassement"], "^" if e["contact"] else "o", color=ROUGE if e["contact"] else BLEU, ms=7)
    vv = np.linspace(0, 280, 50)
    ax.plot(vv, 0.3 * vv, color=GRIS, ls="--", label="retard de 0,3 s")
    ax.axhline(30, color=ROUGE, lw=0.8, ls=":", label="tolérance ±30 mm")
    ax.plot([], [], "^", color=ROUGE, label="contact avec le mur (dépassement minimal)")
    ax.set(xlabel="vitesse moyenne de l'essai (mm/s)", ylabel="dépassement de la distance demandée (mm)",
           title="Essais en mouvement : le dépassement croît avec la vitesse (mesures réelles)")
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(a.sortie, "figC_retard.png"), dpi=160); plt.close(fig)
    print(f"Figures dans {a.sortie}/ ; droite de biais : {pente:.4f} × d + {ordo:.2f} mm")


if __name__ == "__main__":
    main()
