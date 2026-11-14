#!/usr/bin/env python3
"""Analyse hors ligne (sur PC) : règle le Kalman et le compare à médiane + exponentiel.

Entrées : fichiers CSV du robot (enregistrer_mesures.py) ou simulés (simuler_donnees.py).
  - statiques : robot immobile, colonne d_reelle_mm remplie ;
  - approches : robot qui roule vers l'obstacle. Distance réelle = colonne d_reelle_mm si elle
    est remplie à chaque ligne, sinon une droite ajustée sur les mesures valides (vitesse constante).

    python3 analyse/analyse_filtres.py --statique donnees/statique_*.csv \
        --approche donnees/approche_*.csv --d-arret 150 -o resultats

Sorties dans le dossier -o : figures PNG et resultats.csv. Nécessite numpy et matplotlib.
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from fastdrink.filtres import KalmanDistance, MedianeExponentiel, MedianeKalman  # noqa: E402

SIGMAS_A = [100, 200, 400, 600, 1000, 2000, 3000]   # balayage du réglage (mm/s²)
T_IGNORE_S = 0.3                                    # premières mesures ignorées (le filtre démarre)


# ------------------------------------------------------------------ lecture
def lire(chemin):
    with open(chemin, newline="") as f:
        lignes = list(csv.DictReader(f))
    t = np.array([float(l["t_s"]) for l in lignes])
    z = np.array([float(l["z_mm"]) for l in lignes])
    d = np.array([float(l["d_reelle_mm"]) if l["d_reelle_mm"] not in ("", None) else np.nan for l in lignes])
    return {"nom": os.path.basename(chemin), "t": t, "z": z, "d": d}


def reference_approche(e):
    """Distance réelle à chaque instant : colonne fournie, sinon droite robuste sur les mesures."""
    if not np.isnan(e["d"]).any():
        return e["d"], "mesurée"
    ok = e["z"] > 0
    t, z = e["t"][ok], e["z"][ok]
    for _ in range(3):                              # on retire les points trop loin de la droite
        a, b = np.polyfit(t, z, 1)
        r = z - (a * t + b)
        garde = np.abs(r) < 3 * max(np.std(r), 1.0)
        t, z = t[garde], z[garde]
    a, b = np.polyfit(t, z, 1)
    return a * e["t"] + b, "droite ajustée"


# ------------------------------------------------------------------ filtres
def appliquer(filtre, e):
    sortie = []
    for t, z in zip(e["t"], e["z"]):
        r = filtre.mettre_a_jour(z, t)
        y = r[0] if isinstance(r, tuple) else r
        sortie.append(np.nan if y is None else y)
    return np.array(sortie)


FILTRES_KALMAN = {"Kalman": KalmanDistance, "Médiane 3 + Kalman": MedianeKalman}


def mediane_exp():
    return MedianeExponentiel(n=3, alpha=0.3)


# ------------------------------------------------------------------ critères
def bruit_capteur(e):
    """Écart type des mesures valides sans les valeurs aberrantes (robot immobile)."""
    z = e["z"][e["z"] > 0]
    med = np.median(z)
    mad = np.median(np.abs(z - med)) * 1.4826
    propres = z[np.abs(z - med) < 5 * max(mad, 1.0)]
    return float(np.std(propres, ddof=1)), 1 - len(z) / len(e["z"]), 1 - len(propres) / len(z)


def pire_ecart_approche(y, e):
    m = (e["t"] > e["t"][0] + T_IGNORE_S) & ~np.isnan(y)
    return float(np.max(np.abs(y[m] - e["ref"][m])))


def pire_ecart_statique(y, e):
    m = (e["t"] > e["t"][0] + T_IGNORE_S) & ~np.isnan(y)
    return float(np.max(np.abs(y[m] - np.mean(y[m]))))


def criteres_statique(y, e):
    ok = ~np.isnan(y)
    erreur = y[ok] - e["d"][ok]
    return {"rmse_statique_mm": float(np.sqrt(np.mean(erreur ** 2))),
            "ecart_type_sortie_mm": float(np.std(y[ok], ddof=1)),
            "max_erreur_statique_mm": float(np.max(np.abs(erreur)))}


def criteres_approche(y, e, ref, d_arret, t_arret=None):
    """t_arret : instant d'arrêt donné par une autre règle (arrêt programmé) ; sinon on prend
    la première mesure où la sortie passe sous d_arret. Erreur d'arrêt = vraie distance à cet
    instant - d_arret (négative = trop près de l'obstacle)."""
    t = e["t"]
    v_locale = np.abs(np.gradient(ref, t))
    m = (t > t[0] + T_IGNORE_S) & ~np.isnan(y) & (v_locale > 20)
    erreur = y[m] - ref[m]
    if t_arret is None:
        sous = np.where((y <= d_arret) & ~np.isnan(y))[0]
        t_arret = t[sous[0]] if len(sous) else None
    err_arret = float(np.interp(t_arret, t, ref) - d_arret) if t_arret is not None else np.nan
    return {"rmse_approche_mm": float(np.sqrt(np.mean(erreur ** 2))),
            "retard_ms": float(np.mean(erreur / v_locale[m]) * 1000),
            "erreur_arret_mm": err_arret,
            "vitesse_mm_s": float(np.max(v_locale))}


def arret_programme(filtre, e, d_arret, horizon_s=0.1):
    """Règle d'arrêt avec la vitesse estimée : dès que d_arret sera atteinte avant la prochaine
    mesure, l'arrêt est programmé à l'instant prévu. Renvoie (sortie du filtre, instant d'arrêt)."""
    sortie, t_arret = [], None
    for t, z in zip(e["t"], e["z"]):
        d, _ = filtre.mettre_a_jour(z, t)
        sortie.append(np.nan if d is None else d)
        if t_arret is None:
            delai = filtre.delai_avant(d_arret, horizon_s)
            if delai is not None:
                t_arret = t + delai
    return np.array(sortie), t_arret


# ------------------------------------------------------------------ programme
def main():
    p = argparse.ArgumentParser(description="Réglage et comparaison des filtres")
    p.add_argument("--statique", nargs="+", required=True)
    p.add_argument("--approche", nargs="+", required=True)
    p.add_argument("--d-arret", type=float, default=150.0, help="distance d'arrêt visée (mm)")
    p.add_argument("-o", "--sortie", default="resultats")
    a = p.parse_args()
    os.makedirs(a.sortie, exist_ok=True)

    statiques = [lire(f) for f in sorted(a.statique)]
    approches = [lire(f) for f in sorted(a.approche)]
    for e in approches:
        e["ref"], e["source_ref"] = reference_approche(e)

    # 1) R : bruit du capteur, robot immobile
    print("Jeu statique (capteur brut)")
    sigmas = []
    for e in statiques:
        s, echecs, aberr = bruit_capteur(e)
        sigmas.append(s)
        print(f"  {e['nom']:32s} écart type {s:5.2f} mm   échecs {echecs:5.1%}   aberrantes {aberr:5.1%}")
    sigma_z = max(sigmas)
    print(f"  -> sigma_z retenu = {sigma_z:.2f} mm (R = {sigma_z ** 2:.1f} mm²)\n")

    # 2) sigma_a : balayage sur la moitié des approches (réglage), validation sur l'autre moitié.
    # Critère : la PIRE erreur, car la tolérance de ±30 mm s'applique à chaque arrêt.
    #  - en approche : plus grand |sortie - distance réelle| (décroche en accélération si sigma_a est trop petit) ;
    #  - robot immobile : plus grand écart autour de la moyenne (valeur extrême acceptée si sigma_a est
    #    trop grand) ; le biais du capteur est exclu, il relève de l'étalonnage.
    reglage = approches[::2]
    validation = approches[1::2] or approches
    balayages, sigma_a = {}, {}
    for nom, classe in FILTRES_KALMAN.items():
        b = []
        for sa in SIGMAS_A:
            pa = max(pire_ecart_approche(appliquer(classe(sigma_z=sigma_z, sigma_a=sa), e), e) for e in reglage)
            ps = max(pire_ecart_statique(appliquer(classe(sigma_z=sigma_z, sigma_a=sa), e), e) for e in statiques)
            b.append((sa, pa, ps))
        balayages[nom] = b
        sigma_a[nom] = min(b, key=lambda x: max(x[1], x[2]))[0]
        print(f"{nom} – balayage de sigma_a (mm/s²) : pire écart en approche | pire écart robot immobile")
        for sa, pa, ps in b:
            print(f"  {sa:5d} : {pa:6.2f} mm | {ps:6.2f} mm" + ("   <- retenu" if sa == sigma_a[nom] else ""))
        print()

    # 3) comparaison sur les fichiers de validation et statiques
    fabriques = {"Médiane + exponentiel": mediane_exp}
    for nom, classe in FILTRES_KALMAN.items():
        fabriques[nom] = (lambda c=classe, sa=sigma_a[nom]: c(sigma_z=sigma_z, sigma_a=sa))
    variantes = [("Médiane + exponentiel, arrêt au seuil", "Médiane + exponentiel", False),
                 ("Kalman, arrêt au seuil", "Kalman", False),
                 ("Kalman, arrêt programmé", "Kalman", True),
                 ("Médiane 3 + Kalman, arrêt au seuil", "Médiane 3 + Kalman", False),
                 ("Médiane 3 + Kalman, arrêt programmé", "Médiane 3 + Kalman", True)]
    lignes = []
    for nom, nom_filtre, programme in variantes:
        fab = fabriques[nom_filtre]
        sorties_st = [appliquer(fab(), e) for e in statiques]
        ligne = {"filtre": nom,
                 "ecart_type_immobile_mm": np.mean([np.nanstd(y[5:], ddof=1) for y in sorties_st]),
                 "pire_ecart_immobile_mm": max(pire_ecart_statique(y, e) for y, e in zip(sorties_st, statiques)),
                 "biais_moyen_immobile_mm": np.mean([np.nanmean(y - e["d"]) for y, e in zip(sorties_st, statiques)])}
        ap, pires = [], []
        for e in validation:
            if programme:
                y, t_arr = arret_programme(fab(), e, a.d_arret)
            else:
                y, t_arr = appliquer(fab(), e), None
            ap.append(criteres_approche(y, e, e["ref"], a.d_arret, t_arr))
            pires.append(pire_ecart_approche(y, e))
        ligne["rmse_approche_mm"] = np.mean([c["rmse_approche_mm"] for c in ap])
        ligne["pire_ecart_approche_mm"] = max(pires)
        ligne["retard_ms"] = np.mean([c["retard_ms"] for c in ap])
        errs = [c["erreur_arret_mm"] for c in ap]
        if any(np.isnan(errs)):
            print(f"  ATTENTION : {nom} n'a pas atteint la distance d'arrêt dans {sum(np.isnan(errs))} essai(s)")
        ligne["erreur_arret_moy_mm"] = float(np.nanmean(errs))
        ligne["erreur_arret_max_mm"] = float(np.nanmax(np.abs(errs)))
        lignes.append(ligne)
    with open(os.path.join(a.sortie, "resultats.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(lignes[0]))
        w.writeheader()
        w.writerows({k: (f"{v:.2f}" if isinstance(v, float) else v) for k, v in l.items()} for l in lignes)
    print("Comparaison (fichiers de validation)")
    for l in lignes:
        print("  " + "  ".join(f"{k}={v:.1f}" if isinstance(v, float) else str(v) for k, v in l.items()))
    print(f"\nRéférence des approches : {', '.join(sorted({e['source_ref'] for e in approches}))}")
    print(f"Paramètres retenus : sigma_z = {sigma_z:.2f} mm, sigma_a = {sigma_a}")

    figures(a, statiques, validation, balayages, sigma_z, sigma_a, fabriques)
    print(f"Figures et resultats.csv dans {a.sortie}/")


# ------------------------------------------------------------------ figures
GRIS, BLEU, ORANGE = "#9a9a9a", "#1f5fbf", "#d9822b"


COULEURS = {"Médiane + exponentiel": ORANGE, "Kalman": "#8fb3e8", "Médiane 3 + Kalman": BLEU}


def etiquette(fichiers):
    if any("bruitreel" in e["nom"] for e in fichiers):
        return " (SIMULÉE avec le bruit réel)"
    return " (données SIMULÉES)" if any("SIMULE" in e["nom"] for e in fichiers) else " (mesures réelles)"


def figures(a, statiques, validation, balayages, sigma_z, sigma_a, fabriques):
    # 1) robot immobile : le fichier qui contient le plus de valeurs extrêmes
    e = max(statiques, key=lambda x: np.max(np.abs(x["z"][x["z"] > 0] - np.median(x["z"][x["z"] > 0]))))
    fig, ax = plt.subplots(figsize=(10, 4))
    ok = e["z"] > 0
    ax.plot(e["t"][ok], e["z"][ok], ".", color=GRIS, ms=3, label="mesure brute")
    if (~ok).any():
        ax.plot(e["t"][~ok], np.full((~ok).sum(), e["d"][0]), "x", color="k", ms=5, label="écho manqué (−1)")
    for nom, fab in fabriques.items():
        ax.plot(e["t"], appliquer(fab(), e), color=COULEURS[nom], lw=1.4, label=nom)
    ax.axhline(e["d"][0], color="k", lw=0.8, ls="--", label="distance réelle (règle)")
    centre = np.median(e["z"][ok])
    ax.set_ylim(min(centre, e["d"][0]) - 25, max(centre, e["d"][0]) + 25)
    ax.set_xlim(e["t"][0], e["t"][0] + 20)   # 20 premières secondes : lisible
    ax.set(xlabel="temps (s)", ylabel="distance capteur (mm)",
           title=f"Robot immobile, {e['d'][0]:.0f} mm du capteur, 20 premières secondes{etiquette([e])}")
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    fig.tight_layout(); fig.savefig(os.path.join(a.sortie, "fig1_statique.png"), dpi=160); plt.close(fig)

    # 2) approche la plus rapide
    e = max(validation, key=lambda x: np.max(np.abs(np.gradient(x["ref"], x["t"]))))
    v = np.max(np.abs(np.gradient(e["ref"], e["t"])))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={"width_ratios": [3, 2]})
    for ax in axes:
        ok = e["z"] > 0
        ax.plot(e["t"][ok], e["z"][ok], ".", color=GRIS, ms=4, label="mesure brute")
        ax.plot(e["t"], e["ref"], "k--", lw=0.9, label="distance réelle")
        for nom, fab in fabriques.items():
            ax.plot(e["t"], appliquer(fab(), e), color=COULEURS[nom], lw=1.4, label=nom)
        ax.axhline(a.d_arret, color="r", lw=0.8, label=f"distance d'arrêt {a.d_arret:.0f} mm")
        ax.set(xlabel="temps (s)", ylabel="distance capteur (mm)")
    axes[0].set_ylim(0, max(e["ref"]) * 1.1)
    axes[0].set_title(f"Approche à {v:.0f} mm/s{etiquette([e])}")
    i = np.argmin(np.abs(e["ref"] - a.d_arret))
    axes[1].set_xlim(e["t"][i] - 0.4, e["t"][i] + 0.4)
    axes[1].set_ylim(a.d_arret - 80, a.d_arret + 120)
    axes[1].set_title("Zoom sur la zone d'arrêt")
    axes[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(a.sortie, "fig2_approche.png"), dpi=160); plt.close(fig)

    # 3) réglage de sigma_a : pire écart (le plus grand des deux cas) pour chaque Kalman
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for nom, b in balayages.items():
        sa, pa, ps = zip(*b)
        ax.plot(sa, np.maximum(pa, ps), "o-", color=COULEURS[nom], label=f"{nom} (retenu : {sigma_a[nom]} mm/s²)")
    ax.axhline(30, color="r", lw=0.8, ls=":", label="tolérance client ±30 mm")
    ax.set_xscale("log")
    ax.set(xlabel="σ_a (mm/s²), échelle log", ylabel="pire écart (mm)",
           title=f"Réglage de σ_a, σ_z = {sigma_z:.1f} mm")
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(a.sortie, "fig3_reglage.png"), dpi=160); plt.close(fig)


if __name__ == "__main__":
    main()
