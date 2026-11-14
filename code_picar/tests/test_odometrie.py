"""Odométrie Python contre les traces Godot.

Dans Controleur._appliquer (Godot), à chaque tour et dans cet ordre : suivre_servo(angle envoyé, dt),
integrer_global(vitesse_odometrie_mm_s(cmd), dt), puis integrer(...) pendant les manœuvres ; dt = maxf(dt, 0.001).
L'odométrie globale n'est jamais remise à zéro : on la rejoue sur toute la trace, cumulée.
"""
import pytest

from fastdrink.conduite import godot as gd
from fastdrink.conduite.odometrie import Odometrie
from fastdrink.conduite.reglages import ReglagesRobot
from fastdrink.conduite.rejeu import TOLERANCE_ABS, TOLERANCE_REL, lire_trace, proches
from outils_gd import traces_reference

ETATS_MANOEUVRE_ODO = ("CONTOURNEMENT", "RECUL_LIGNE", "RATTRAPAGE", "CONTOURNEMENT_FLUIDE", "RECUL_OBSTACLE")


@pytest.mark.parametrize("chemin", traces_reference(), ids=lambda c: c.split("traces")[-1][1:])
def test_odometrie_globale_cumulee(chemin):
    t = lire_trace(chemin)
    robot = ReglagesRobot()
    robot.appliquer_champs(t.entete["robot"])
    odo = Odometrie(robot)
    pire = 0.0
    for i, tour in enumerate(t.tours):
        dt = gd.maxf(tour["dt"], 0.001)
        odo.suivre_servo(float(tour["angle"]), dt)
        odo.integrer_global(robot.vitesse_odometrie_mm_s(tour["cmd"]), dt)
        assert odo.servo_estime == tour["servo_estime"], i
        for nom, godot in (("x_global", tour["glob_x"]), ("y_global", tour["glob_y"]), ("cap_global", tour["glob_cap"])):
            python = getattr(odo, nom)
            assert proches(python, godot), "tour %d %s : godot %r python %r" % (i, nom, godot, python)
            pire = max(pire, abs(python - godot) / (TOLERANCE_ABS + TOLERANCE_REL * abs(godot)))
    print("%s : écart max = %.2g x la tolérance" % (t.nom, pire))


@pytest.mark.parametrize("chemin", traces_reference(), ids=lambda c: c.split("traces")[-1][1:])
def test_odometrie_de_manoeuvre_pas_a_pas(chemin):
    """Pendant une manœuvre, un pas d'intégration depuis l'état Godot du tour précédent (ou depuis zéro, quand un
    module vient d'appeler reinitialiser()) redonne l'état Godot du tour."""
    t = lire_trace(chemin)
    robot = ReglagesRobot()
    robot.appliquer_champs(t.entete["robot"])
    n = 0
    for i in range(1, len(t.tours)):
        prec, tour = t.tours[i - 1], t.tours[i]
        if tour["etat"] not in ETATS_MANOEUVRE_ODO:
            continue
        dt = gd.maxf(tour["dt"], 0.001)
        v = robot.vitesse_odometrie_mm_s(tour["cmd"])
        resultats = []
        for depart in ((prec["odo_x"], prec["odo_y"], prec["odo_cap"], prec["odo_parcours"]), (0.0, 0.0, 0.0, 0.0)):
            odo = Odometrie(robot)
            odo.x, odo.y, odo.cap, odo.parcours = depart
            odo.servo_estime = tour["servo_estime"]   # suivre_servo est appelé avant integrer
            odo.integrer(v, dt)
            resultats.append((odo.x, odo.y, odo.cap, odo.parcours))
        attendu = (tour["odo_x"], tour["odo_y"], tour["odo_cap"], tour["odo_parcours"])
        assert any(all(proches(a, b) for a, b in zip(r, attendu)) for r in resultats), \
            "tour %d (%s) : godot %r python %r" % (i, tour["etat"], attendu, resultats)
        n += 1
    # Traces sans manoeuvre odometrique (suivi, rattrapage ou RETROUVER seulement) : rien a comparer.
    assert n > 0 or any(nom in chemin for nom in ("virage_serre", "parcours_fourni", "epingle_180_retrouver"))
