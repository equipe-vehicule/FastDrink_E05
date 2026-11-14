"""Tests unitaires du lot C : MemoireParcours, RetrouverLigne, DemiTour (le Contrôleur est vérifié par test_rejeu.py).

Valeurs de réglage : celles de reglages_algorithmes.json (lues, pas recopiées) ; robot : valeurs d'essai, sans
signification physique hors de ces tests.
"""
import math

import pytest

from fastdrink.conduite import godot as gd
from fastdrink.conduite.constantes import ERREUR_MAX_MM
from fastdrink.conduite.demi_tour import DemiTour
from fastdrink.conduite.godot import Vec2
from fastdrink.conduite.memoire_parcours import MemoireParcours
from fastdrink.conduite.odometrie import Odometrie
from fastdrink.conduite.reglages import ReglagesRobot, lire_reglages_algorithmes
from fastdrink.conduite.retrouver_ligne import RetrouverLigne
from outils_gd import schemas_gd

VALEURS = lire_reglages_algorithmes(schemas_gd())


def _robot():
    r = ReglagesRobot()
    r.distance_sondes_mm = 100.0
    r.longueur_avant_mm = 150.0
    r.angle_roues_max_deg = 30.0
    return r


def _regler(objet, nom):
    for cle, v in VALEURS[nom].items():
        setattr(objet, cle, v)
    return objet


def _memoire():
    robot = _robot()
    odo = Odometrie(robot)
    return _regler(MemoireParcours(robot, odo), "memoire"), odo


def _avancer(m, odo, n, pas=5.0):
    """Roule en ligne droite (cap courant), en mémorisant la ligne centrée à chaque pas de 5 mm."""
    for _ in range(n):
        odo.x_global += pas * math.sin(odo.cap_global)
        odo.y_global += pas * math.cos(odo.cap_global)
        m.memoriser(0.0)


def test_memoriser_un_point_tous_les_20_mm_et_borne():
    m, odo = _memoire()
    _avancer(m, odo, 10)                        # 50 mm par pas de 5 : points après 5 et 30 mm (distance > 20 mm)
    assert len(m.ligne) == 2
    assert all(isinstance(p, Vec2) for p in m.ligne)
    assert m.ligne[0] == Vec2(0.0, 105.0)       # les sondes sont à distance_sondes_mm devant l'essieu
    _avancer(m, odo, 1100)
    assert len(m.ligne) == MemoireParcours.POINTS_LIGNE
    assert len(m._chemin) == MemoireParcours.POINTS_CHEMIN


def test_annuler_passage_retire_les_points_du_passage():
    m, odo = _memoire()
    _avancer(m, odo, 20)
    avant = len(m._chemin)
    m.nouveau_passage()
    _avancer(m, odo, 20)
    assert m._points_passage > 0
    m.annuler_passage()
    assert len(m._chemin) == avant and m._points_passage == 0


def test_derniere_droite_et_sens_ligne_sur_une_droite():
    m, odo = _memoire()
    odo.cap_global = math.radians(30.0)
    _avancer(m, odo, 60)
    centre, sens = m.derniere_droite()
    assert abs(sens.x - math.sin(odo.cap_global)) < 1e-4 and abs(sens.y - math.cos(odo.cap_global)) < 1e-4
    assert abs(m.sens_ligne().x - sens.x) < 1e-3
    assert m.vers_repere_manoeuvre()[-1].x == pytest.approx(0.0, abs=1e-3)   # la ligne est dans l'axe de la voiture
    # dernier point mémorisé : 4 pas de 5 mm derrière la position actuelle, donc à 100 - 20 mm devant l'essieu
    assert m.vers_repere_manoeuvre()[-1].y == pytest.approx(80.0, abs=1e-3)


def test_derniere_droite_s_arrete_au_virage():
    m, odo = _memoire()
    _avancer(m, odo, 100)                       # droite vers l'avant (y)
    odo.cap_global = math.pi / 2                # puis virage à 90 deg : droite vers x
    _avancer(m, odo, 40)
    _, sens = m.derniere_droite()
    assert sens.x > 0.99                        # seulement la dernière droite, pas le coin


def test_contresens_sur_le_chemin_parcouru():
    m, odo = _memoire()
    _avancer(m, odo, 300)                       # 1500 mm vers l'avant
    # revenir en arrière sur le chemin : cap inversé, position 600 mm plus bas
    odo.cap_global = math.pi
    odo.y_global -= 600.0
    assert m.remonte_le_chemin(0.0)
    assert not m.en_avant_sur_chemin(0.0, 1)
    assert m.sur_ligne_a_contresens(0.0)
    # dans le bon sens : pas de contresens, et la voiture « en avant sur le chemin »
    odo.cap_global = 0.0
    assert not m.remonte_le_chemin(0.0)
    assert m.en_avant_sur_chemin(0.0, 1) and not m.en_avant_sur_chemin(0.0, 0)
    # ligne sous les sondes hors du chemin : NaN
    odo.x_global = 5000.0
    assert gd.is_nan(m.sens_sur_chemin(0.0))


def test_obstacles_memorises_et_deja_contourne():
    m, odo = _memoire()
    m.noter_obstacle(Vec2(0.0, 500.0))
    assert m.obstacle_deja_contourne(500.0 - 150.0)         # echo = (longueur_avant + d) devant l'essieu
    assert not m.obstacle_deja_contourne(100.0)
    for i in range(m.obstacles_memorises + 3):
        m.noter_obstacle(Vec2(1000.0 + i, 0.0))
    assert len(m.obstacles) == m.obstacles_memorises


def test_demi_tours_autorises_puis_refuses():
    m, odo = _memoire()
    for _ in range(m.demi_tours_max_au_meme_endroit - 1):
        m.noter_demi_tour()
    assert m.demi_tour_autorise()
    m.noter_demi_tour()
    assert not m.demi_tour_autorise()
    odo.x_global = 10000.0                                   # ailleurs
    assert m.demi_tour_autorise()
    for _ in range(10):
        m.noter_demi_tour()
    assert len(m._demi_tours) == MemoireParcours.DEMI_TOURS_MEMORISES


def test_retrouver_cible_devant_le_dernier_point_et_conduite():
    m, odo = _memoire()
    _avancer(m, odo, 60)
    r = _regler(RetrouverLigne(_robot(), odo), "retrouver")
    r.demarrer(m, 3.0)
    assert r._cote_spirale == 1 and r._t_spirale < 0.0
    cible = r.cible_ligne(m)
    assert cible.x == pytest.approx(0.0, abs=1e-2)
    assert cible.y == pytest.approx(m.ligne[-1].y + r.avance_cible_mm, abs=1e-2)
    # cible droit devant, obstacle absent (INF) : servo droit, vitesse demandée
    cmd = r.conduire(Vec2(gd.INF, gd.INF), 0.0, 0.01)
    assert cmd == Vec2(90.0, r.vitesse_mm_s)
    # au-delà de approche_max_s : spirale, qui se resserre... puis s'ouvre moins vite avec le temps
    a1 = r.conduire(Vec2(gd.INF, gd.INF), r.approche_max_s + 1.0, 0.01)
    assert r._t_spirale > 0.0
    a2 = r.conduire(Vec2(gd.INF, gd.INF), r.approche_max_s + 2.0, 1.0)
    assert a1.x > 90.0 and 90.0 < a2.x < a1.x


def test_retrouver_s_ecarte_d_un_obstacle_proche():
    m, odo = _memoire()
    _avancer(m, odo, 60)
    r = _regler(RetrouverLigne(_robot(), odo), "retrouver")
    r.demarrer(m, -1.0)
    assert r._cote_spirale == -1
    obstacle = Vec2(odo.x_global + 50.0, odo.y_global + 100.0)   # à droite, devant, tout près
    cmd = r.conduire(obstacle, 0.0, 0.01)
    assert cmd == Vec2(90.0 - 45.0, r.vitesse_mm_s)            # braque à gauche, du côté opposé à l'obstacle
    # obstacle « hors ligne » proche de la ligne et devant : devient la base de la cible
    m.obstacle_hors_ligne = Vec2(0.0, m.ligne[-1].y + 100.0)
    assert r.cible_ligne(m).y == pytest.approx(m.ligne[-1].y + 100.0 + r.avance_cible_mm, abs=1e-2)


def test_demi_tour_alterne_avant_arriere_puis_termine():
    d = _regler(DemiTour(), "demi_tour")
    d.demarrer(0.0, 1)
    assert d.ecart_reprise_mm() == ERREUR_MAX_MM
    assert d.calculer(0, 0.0, 0.0) == Vec2(135.0, d.vitesse_mm_s)                  # marche avant, braquage à droite
    assert d.calculer(0, 0.1, d.duree_avant_s + 0.1) == Vec2(45.0, -d.vitesse_mm_s)  # marche arrière, braquage opposé
    assert d.calculer(0, 0.1, d.duree_avant_s + d.duree_arriere_s + 0.1).x == 135.0   # période suivante
    assert not d.termine
    # 150 deg sans ligne sous les sondes : continue ; avec la ligne : fini ; 210 deg : fini quoi qu'il arrive
    assert d.calculer(0, math.radians(160.0), 0.1) is not None and not d.termine
    assert d.calculer(2, math.radians(160.0), 0.1) == Vec2(90.0, 0.0) and d.termine
    d.demarrer(0.0, -1)
    assert not d.termine and d.ecart_reprise_mm() == -ERREUR_MAX_MM
    assert d.calculer(0, math.radians(-211.0), 0.1) == Vec2(90.0, 0.0) and d.termine
    d.demarrer(0.0, 1)
    assert d.calculer(0, 0.0, d.duree_max_s + 1.0) == Vec2(90.0, 0.0) and d.termine
