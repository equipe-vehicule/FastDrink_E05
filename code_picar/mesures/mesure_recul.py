#!/usr/bin/env python3
"""Mesure 2 : distance reculée réelle (recul en boucle ouverte, sans suivi de ligne).

Le robot recule droit pendant une durée calculée = distance visée / vitesse (table mesurée par l'équipe,
logbooks GitHub, « idem à reculons »). On mesure à la règle la distance réellement reculée.
Écart visé (fiche) : moins de 15 mm. Limite du guide : moins de 200 mm.

    python3 mesures/mesure_recul.py                    # 5 essais, 150 mm, commande 30 %
    python3 mesures/mesure_recul.py -d 150 -v 30 --duree 1.4   # impose la durée (s) au lieu de la calculer

Marquer au sol la position de départ (avant du robot) ; la distance reculée = déplacement de ce repère.
Ctrl+C = arrêt immédiat. L'accélération du robot (environ 0,6 s de 0 à 100 %) n'est pas comptée dans
le calcul : l'écart mesuré montre justement son effet.
"""
import argparse
import time

from commun import robot, ouvrir_csv, demander_mm, vitesse_m_s, VITESSE_DEFAUT


def main():
    p = argparse.ArgumentParser(description="Mesure du recul en boucle ouverte")
    p.add_argument("-d", type=float, default=150.0, help="distance visée (mm), valeur de travail 150")
    p.add_argument("-n", type=int, default=5, help="nombre d'essais")
    p.add_argument("-v", type=int, default=VITESSE_DEFAUT, help="commande moteur (%%)")
    p.add_argument("--duree", type=float, default=None, help="durée du recul (s) ; sinon distance / vitesse")
    args = p.parse_args()

    duree = args.duree if args.duree is not None else args.d / 1000.0 / vitesse_m_s(args.v)
    print(f"Recul de {args.d:.0f} mm visés : commande {args.v} %, durée {duree:.2f} s "
          f"(vitesse table {vitesse_m_s(args.v):.4f} m/s)")

    f, w = ouvrir_csv("recul", ["essai", "d_visee_mm", "commande_pct", "duree_s",
                                "d_reelle_regle_mm", "ecart_mm"])
    ecarts = []
    try:
        with robot() as (bw, fw):
            for i in range(1, args.n + 1):
                input(f"\nEssai {i}/{args.n} : marquer le départ, dégager l'arrière, puis Entrée... ")
                bw.speed = args.v
                bw.backward()
                time.sleep(duree)
                bw.speed = 0
                bw.stop()
                time.sleep(1.0)
                d_reelle = demander_mm("Distance réellement reculée")
                ecart = None if d_reelle is None else d_reelle - args.d
                w.writerow([i, args.d, args.v, f"{duree:.3f}",
                            "" if d_reelle is None else d_reelle, "" if ecart is None else f"{ecart:.1f}"])
                f.flush()
                if ecart is not None:
                    ecarts.append(ecart)
    finally:
        f.close()
    if ecarts:
        print(f"\nRésumé ({len(ecarts)} essais) : écart moyen {sum(ecarts)/len(ecarts):+.1f} mm, "
              f"pire {max(ecarts, key=abs):+.1f} mm.")
    print(f"Fichier : {f.chemin}")


if __name__ == "__main__":
    main()
