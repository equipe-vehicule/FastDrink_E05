#!/usr/bin/env python3
"""Mesure 1 : arrêt devant l'obstacle à D_arret (précision de l'arrêt réel).

Le robot avance droit, le capteur est filtré par FiltreFinal (mode « en mouvement », filtre B).
Dès que la distance filtrée (depuis l'AVANT du robot) atteint D_arret, commande 0 puis arrêt.
Après chaque essai, on mesure à la règle la distance réelle avant du robot -> obstacle.
Le dépassement (D_arret - réel) est ce que la simulation doit reproduire (seuil de la fiche : 10 mm).

    python3 mesures/mesure_arret.py                  # 5 essais, D_arret 100 mm, commande 30 %
    python3 mesures/mesure_arret.py -d 100 -n 5 -v 30

Placer un obstacle (carton) droit devant le robot, à au moins 500 mm. Ctrl+C = arrêt immédiat.
Les mesures du capteur sont prises toutes les ~60 ms au minimum (limite du capteur, manuel v2.0).
"""
import argparse
import time

from commun import robot, ouvrir_csv, demander_mm, VITESSE_DEFAUT, attendre
from fastdrink.capteurs import Ultrason
from fastdrink.config import CAPTEUR_AVANT_MM
from fastdrink.filtres import FiltreFinal

DUREE_MAX_S = 8.0   # sécurité : arrêt si l'obstacle n'est jamais atteint


def un_essai(bw, capteur, d_arret, vitesse):
    filtre = FiltreFinal()
    # Amorçage immobile : quelques mesures pour que le filtre ait une distance de départ.
    t0 = time.perf_counter()
    for _ in range(5):
        filtre.mettre_a_jour(capteur.mesurer_mm(), time.perf_counter() - t0, en_mouvement=False)

    bw.speed = vitesse
    bw.forward()
    t_depart = time.perf_counter()
    d_stop = None
    duree = None
    while time.perf_counter() - t_depart < DUREE_MAX_S:
        z = capteur.mesurer_mm()
        t = time.perf_counter() - t0
        sortie = filtre.mettre_a_jour(z, t, en_mouvement=True)
        if sortie is None:
            continue
        d = sortie - CAPTEUR_AVANT_MM
        if d <= d_arret:
            bw.speed = 0
            bw.stop()
            d_stop, duree = d, time.perf_counter() - t_depart
            break
    bw.speed = 0
    bw.stop()
    return d_stop, duree


def main():
    p = argparse.ArgumentParser(description="Mesure de l'arrêt devant l'obstacle")
    p.add_argument("-d", type=float, default=100.0, help="D_arret (mm), valeur de test 100")
    p.add_argument("-n", type=int, default=5, help="nombre d'essais")
    p.add_argument("-v", type=int, default=VITESSE_DEFAUT, help="commande moteur (%%)")
    args = p.parse_args()

    f, w = ouvrir_csv("arret", ["essai", "d_arret_mm", "commande_pct", "d_filtree_au_stop_mm",
                                "duree_s", "d_reelle_regle_mm", "depassement_mm"])
    resultats = []
    try:
        with robot() as (bw, fw), Ultrason() as capteur:
            for i in range(1, args.n + 1):
                input(f"\nEssai {i}/{args.n} : placer le robot à ~500 mm de l'obstacle, puis Entrée... ")
                d_stop, duree = un_essai(bw, capteur, args.d, args.v)
                if d_stop is None:
                    print("Obstacle non atteint dans le temps permis : essai ignoré.")
                    continue
                attendre(1.0)   # le robot finit de glisser
                d_reelle = demander_mm("Distance réelle avant du robot -> obstacle, à la règle")
                depassement = None if d_reelle is None else args.d - d_reelle
                w.writerow([i, args.d, args.v, f"{d_stop:.1f}", f"{duree:.2f}",
                            "" if d_reelle is None else d_reelle,
                            "" if depassement is None else f"{depassement:.1f}"])
                f.flush()
                if depassement is not None:
                    resultats.append(depassement)
                print(f"  filtre au stop : {d_stop:.1f} mm ; réel : {d_reelle} mm")
    finally:
        f.close()
    if resultats:
        print(f"\nRésumé ({len(resultats)} essais) : dépassement moyen {sum(resultats)/len(resultats):+.1f} mm, "
              f"pire {max(resultats, key=abs):+.1f} mm (positif = plus près que D_arret).")
    print(f"Fichier : {f.chemin}")


if __name__ == "__main__":
    main()
