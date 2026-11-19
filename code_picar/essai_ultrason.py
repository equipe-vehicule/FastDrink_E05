#!/usr/bin/env python3
"""Essai du capteur ultrason sur le robot : compare chaque mesure brute avec la
sortie du filtre final (FiltreFinal), puis affiche un bilan sans filtre / avec filtre.

Les distances affichées sont mesurées depuis l'AVANT du véhicule (mesure capteur - CAPTEUR_AVANT_MM).
Une ligne marquée « <- pic corrigé » est une mesure brute aberrante que le filtre a écartée.

    python3 essai_ultrason.py                          # 50 mesures, robot à l'arrêt (filtre A)
    python3 essai_ultrason.py -n 300 -d 300            # 300 mesures, obstacle à 300 mm de l'avant, à la règle
    python3 essai_ultrason.py -n 300 -d 300 -o essai.csv   # idem + enregistrement pour tracer_essai.py
    python3 essai_ultrason.py -n 100 --mouvement       # robot en mouvement (filtre B, Kalman)
"""
import argparse
import csv
import statistics
import time

from fastdrink.capteurs import Ultrason, MESURE_RATEE
from fastdrink.config import CAPTEUR_AVANT_MM, KALMAN_SIGMA_Z_MM
from fastdrink.filtres import FiltreFinal

N_IGNORE = 20                    # premières mesures ignorées dans le bilan (convergence du filtre)
SEUIL_PIC_MM = 3 * KALMAN_SIGMA_Z_MM  # brut et filtré écartés de plus de 3 écarts types = pic corrigé


def bilan(nom, valeurs, d_reelle):
    if len(valeurs) < 2:
        print(f"{nom} : pas assez de mesures")
        return
    moy = statistics.mean(valeurs)
    ligne = (f"{nom} : moyenne {moy:7.1f} mm  écart type {statistics.stdev(valeurs):5.1f} mm  "
             f"min {min(valeurs):7.1f}  max {max(valeurs):7.1f}  (plage {max(valeurs) - min(valeurs):5.1f} mm)")
    if d_reelle is not None:
        pire = max(abs(v - d_reelle) for v in valeurs)
        ligne += f"\n{'':14s}erreur moyenne {moy - d_reelle:+.1f} mm  pire erreur {pire:.1f} mm"
    print(ligne)


def main():
    p = argparse.ArgumentParser(description="Essai du capteur ultrason, avec et sans filtre")
    p.add_argument("-n", type=int, default=50, help="nombre de mesures")
    p.add_argument("-d", type=float, default=None,
                   help="distance réelle obstacle -> avant du véhicule, à la règle (mm)")
    p.add_argument("--mouvement", action="store_true",
                   help="le robot est en mouvement (sinon : à l'arrêt)")
    p.add_argument("-o", default=None, help="fichier CSV où enregistrer l'essai (pour tracer_essai.py)")
    args = p.parse_args()

    filtre = FiltreFinal()
    bruts, filtrees, lignes_csv = [], [], []
    nb_ratees = nb_pics = 0

    print(f"Mode : {'en mouvement (filtre B, Kalman)' if args.mouvement else 'arrêt (filtre A)'}")
    print(f"{'i':>4s} {'t (s)':>7s} {'brut (mm)':>10s} {'filtré (mm)':>12s} {'écart':>7s}")

    with Ultrason() as capteur:
        t0 = time.perf_counter()
        for i in range(1, args.n + 1):
            z = capteur.mesurer_mm()               # mesure brute du capteur, ou MESURE_RATEE
            t = time.perf_counter() - t0
            # Le filtre travaille sur la mesure brute (c'est sur elle qu'il a été réglé)
            # et gère lui-même les mesures ratées.
            sortie = filtre.mettre_a_jour(z, t, en_mouvement=args.mouvement)
            d_filtre = None if sortie is None else sortie - CAPTEUR_AVANT_MM
            txt_filtre = f"{d_filtre:12.1f}" if d_filtre is not None else f"{'-':>12s}"
            d = None if z == MESURE_RATEE else z - CAPTEUR_AVANT_MM
            lignes_csv.append([i, f"{t:.4f}", "" if d is None else f"{d:.1f}",
                               "" if d_filtre is None else f"{d_filtre:.1f}", int(args.mouvement)])

            if d is None:
                nb_ratees += 1
                print(f"{i:4d} {t:7.3f} {'ratée':>10s} {txt_filtre}   <- mesure ratée, filtre inchangé")
                continue

            bruts.append(d)
            if d_filtre is None:
                print(f"{i:4d} {t:7.3f} {d:10.1f} {txt_filtre}")
                continue
            filtrees.append(d_filtre)
            ecart = d_filtre - d
            marque = ""
            if abs(ecart) > SEUIL_PIC_MM:
                marque = "   <- pic corrigé"
                if len(bruts) > N_IGNORE:
                    nb_pics += 1
            print(f"{i:4d} {t:7.3f} {d:10.1f} {txt_filtre} {ecart:+7.1f}{marque}")

    print(f"\nMesures : {args.n}  ratées : {nb_ratees}  pics corrigés par le filtre : {nb_pics}"
          f"  (écart brut/filtré > {SEUIL_PIC_MM:.1f} mm)")
    print(f"(bilan sans les {N_IGNORE} premières mesures valides, le temps que le filtre converge)")
    if args.mouvement:
        # La distance change pendant l'essai : moyenne et écart type n'ont pas de sens ici.
        print("En mouvement : comparer les colonnes brut / filtré ligne par ligne (ou tracer le CSV).")
        print(f"Vitesse estimée en fin d'essai : {filtre.vitesse():.1f} mm/s")
    else:
        bilan("Sans filtre", bruts[N_IGNORE:], args.d)
        bilan("Avec filtre", filtrees[N_IGNORE:], args.d)
        if len(bruts) > N_IGNORE + 1 and len(filtrees) > N_IGNORE + 1:
            gain = statistics.stdev(bruts[N_IGNORE:]) / max(statistics.stdev(filtrees[N_IGNORE:]), 1e-9)
            print(f"Le filtre divise le bruit par {gain:.1f}")

    if args.o:
        with open(args.o, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["i", "t_s", "brut_mm", "filtre_mm", "en_mouvement"])
            w.writerows(lignes_csv)
        print(f"Essai enregistré dans {args.o} ; graphique : python3 tracer_essai.py {args.o}"
              + (f" -d {args.d:g}" if args.d is not None else ""))


if __name__ == "__main__":
    main()
