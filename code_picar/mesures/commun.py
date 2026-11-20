"""Éléments communs aux scripts de mesure : robot, arrêt de sécurité, fichier CSV horodaté.

API moteur : bibliothèque SunFounder `picar` (04_Code-de-depart/Dossier_PiCar/SunFounder_PiCar/picar/)
    picar.setup()
    Back_Wheels(db='config')  : .ready()  .speed = 0..100 (propriété, %)  .forward()  .backward()  .stop()
    Front_Wheels(db='config') : .ready()  .turn_straight()  .turn(angle)  (90 = droit)
Même usage que reset.py et ultrasonic_stop_test.py.
"""
import csv
import datetime
import os
import sys
import time
from contextlib import contextmanager

# Permet de lancer « python3 mesures/xxx.py » depuis code_picar/ (le paquet fastdrink est dans code_picar/).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DOSSIER_MESURES = os.path.dirname(os.path.abspath(__file__))   # les CSV sont écrits ici (code_picar/mesures/)

VITESSE_DEFAUT = 30   # % de commande : basse, pour qu'une erreur ne casse rien (choix de prudence)

# Vitesse mesurée par l'équipe (logbooks GitHub, commande % -> m/s ; « idem à reculons »).
# Rien ne bouge sous 15 %.
TABLE_VITESSE = [(20, 0.0516), (30, 0.1279), (40, 0.2121), (50, 0.2673),
                 (60, 0.3371), (80, 0.4881), (100, 0.6550)]


def vitesse_m_s(commande):
    """Vitesse (m/s) pour une commande en %, par interpolation linéaire de TABLE_VITESSE."""
    t = TABLE_VITESSE
    if commande <= t[0][0]:
        return t[0][1] * commande / t[0][0]
    for (c0, v0), (c1, v1) in zip(t, t[1:]):
        if commande <= c1:
            return v0 + (v1 - v0) * (commande - c0) / (c1 - c0)
    return t[-1][1]


# Décalage du servo des roues avant pour rouler droit (degrés ; positif = vers la droite).
# Valeur de départ 17 = FW_WHEEL_OFFSET (config.py, Sarah) ; 18 après un essai le 19 nov. où le robot
# déviait encore un peu à gauche sur longue distance ; 23 (+5) après un second essai où 18 ne suffisait pas ;
# 21 après un troisième essai où 23 déviait à droite.
# Valeur d'essai de Mehdi, à confirmer.
DECALAGE_ROUES_MESURES = 21


@contextmanager
def robot():
    """Donne (roues arrière, roues avant). À la sortie, quoi qu'il arrive (erreur, Ctrl+C) :
    commande 0, moteurs arrêtés, servo droit."""
    import picar
    from picar import back_wheels, front_wheels
    picar.setup()
    bw = back_wheels.Back_Wheels(db='config')
    fw = front_wheels.Front_Wheels(db='config')
    bw.ready()
    fw.ready()
    # Le fichier « config » de picar est à 0 : sans ce décalage, le robot dérive (constaté le 19 nov.).
    # ready() remet le décalage à 0 : on l'applique donc après.
    fw.wheel.offset = DECALAGE_ROUES_MESURES
    fw.turn_straight()
    try:
        yield bw, fw
    finally:
        bw.speed = 0
        bw.stop()
        fw.turn_straight()


def ouvrir_csv(nom, entetes):
    """Crée mesures/<nom>_AAAAMMJJ_HHMMSS.csv et renvoie (fichier, écrivain). Fermer le fichier à la fin."""
    horodatage = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    chemin = os.path.join(DOSSIER_MESURES, f"{nom}_{horodatage}.csv")
    f = open(chemin, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(entetes)
    f.chemin = chemin
    return f, w


def demander_mm(question):
    """Demande une valeur en mm à la règle ; Entrée seule = pas de valeur (renvoie None)."""
    while True:
        rep = input(question + " (mm, Entrée = ignorer) : ").strip().replace(",", ".")
        if rep == "":
            return None
        try:
            return float(rep)
        except ValueError:
            print("Nombre attendu.")


def attendre(s):
    time.sleep(s)
