"""Lot A (ligne et capteur) : FiltreUltrason contre les traces Godot, tests unitaires des autres modules.

Filtre : entrée = robot.corriger_ultrason_cm(d_cm), dt = maxf(dt, 0.001), nouvelle (Controleur.calculer) ;
sortie comparée à la colonne d_filtre_mm (exactement, puis avec la tolérance du rejeu). Les modules de Godot
n'utilisant ni avance_mm ni projection_odometrie (faux dans les traces), on ne les fournit pas.
"""
import pytest

from fastdrink.conduite import constantes
from fastdrink.conduite import godot as gd
from fastdrink.conduite.arret_obstacle import ArretObstacle
from fastdrink.conduite.filtre_ultrason import FiltreUltrason
from fastdrink.conduite.godot import Vec2
from fastdrink.conduite.odometrie import Odometrie
from fastdrink.conduite.rattrapage import Rattrapage
from fastdrink.conduite.recul_ligne import ReculLigne
from fastdrink.conduite.reglages import ReglagesRobot, convertir
from fastdrink.conduite.rejeu import lire_trace, proches
from fastdrink.conduite.reprise_douce import RepriseDouce
from fastdrink.conduite.suivi_ligne import SuiviLigne
from outils_gd import traces_reference


# ----- FiltreUltrason contre les traces ------------------------------------------------------------------------

@pytest.mark.parametrize("chemin", traces_reference(), ids=lambda c: c.split("traces")[-1][1:])
def test_filtre_ultrason_contre_trace(chemin):
    t = lire_trace(chemin)
    robot = ReglagesRobot()
    robot.appliquer_champs(t.entete["robot"])
    filtre = FiltreUltrason(robot)
    for cle, v in t.entete["reglages"]["ultrason"].items():
        setattr(filtre, cle, convertir(v, FiltreUltrason.REGLAGES[cle]))
    for i, tour in enumerate(t.tours):
        d = filtre.filtrer(robot.corriger_ultrason_cm(tour["d_cm"]), gd.maxf(tour["dt"], 0.001), tour["nouvelle"])
        attendu = tour["d_filtre_mm"]
        assert proches(d, attendu), "%s tour %d : python %r, godot %r" % (t.nom, i, d, attendu)


def test_filtre_ultrason_exact_sur_trace_realiste():
    """Sans aucune tolérance (Kalman : mêmes opérations que le .gd dans le même ordre)."""
    t = lire_trace([c for c in traces_reference() if "realiste_parcours_fourni" in c][0])
    robot = ReglagesRobot()
    robot.appliquer_champs(t.entete["robot"])
    assert robot.filtre_ultrason == "mediane_kalman"
    filtre = FiltreUltrason(robot)
    for cle, v in t.entete["reglages"]["ultrason"].items():
        setattr(filtre, cle, convertir(v, FiltreUltrason.REGLAGES[cle]))
    for i, tour in enumerate(t.tours):
        d = filtre.filtrer(robot.corriger_ultrason_cm(tour["d_cm"]), gd.maxf(tour["dt"], 0.001), tour["nouvelle"])
        attendu = tour["d_filtre_mm"]
        assert d == attendu or (gd.is_nan(d) and gd.is_nan(attendu)), "tour %d : %r != %r" % (i, d, attendu)


def _filtre_mediane3():
    f = FiltreUltrason(None)
    f.portee_max_mm = 1500.0
    f.absence_s = 0.15
    f.zone_proche_mm = 60.0
    f.absence_proche_s = 1.0
    return f


def test_mediane3_un_ou_deux_mesures_puis_mediane_et_absence():
    f = _filtre_mediane3()
    assert f.filtrer(50.0, 0.02) == 500.0
    assert f.filtrer(40.0, 0.02) == 400.0            # 2 mesures : la plus proche
    assert f.filtrer(60.0, 0.02) == 500.0            # 3 : médiane
    assert f.filtrer(-1.0, 0.1) == 500.0             # absence trop courte : on garde
    assert f.filtrer(-1.0, 0.1) == -1.0              # absence >= absence_s : plus d'obstacle
    assert f.filtrer(200.0, 0.02) == -1.0             # 2000 mm au-delà de portee_max_mm : ignoré
    f.reinitialiser()
    assert f.filtrer(-1.0, 0.02) == -1.0


def test_mediane3_absence_longue_si_obstacle_proche():
    f = _filtre_mediane3()
    f.filtrer(4.0, 0.02)                              # 40 mm < zone_proche
    assert f.filtrer(-1.0, 0.5) == 40.0               # 0,5 s < absence_proche_s
    assert f.filtrer(-1.0, 0.6) == -1.0


# ----- SuiviLigne ----------------------------------------------------------------------------------------------

def _suivi():
    s = SuiviLigne()
    s.kp, s.ki, s.kd = 1.0, 0.0, 0.0
    s.vitesse_max_mm_s, s.vitesse_virage_mm_s = 100.0, 20.0
    s.delai_entre_sondes_s = 0.1
    s.abandon_s = 8.0
    s.aligne_avant_arrivee_s = 0.5
    s.pas_arrivee = 3
    return s


def test_suivi_enums_partagees():
    assert SuiviLigne.Ligne is constantes.Ligne
    assert SuiviLigne.ERREUR_MAX_MM == 38.0
    assert ReculLigne.Fin is constantes.Fin and Rattrapage.Reaction is constantes.Reaction


def test_lire_somme_et_nombre():
    assert SuiviLigne.lire([0, 1, 1, 0, 0]) == Vec2(-19.0, 2)
    assert SuiviLigne.lire([0, 0, 0, 0, 0]) == Vec2(0.0, 0)


def test_suivi_ligne_centree_et_decentree():
    s = _suivi()
    r = s.calculer([0, 0, 1, 0, 0], 0.02)
    assert r == Vec2(90.0, 100.0) and s.ligne == SuiviLigne.Ligne.VUE and s.nom_ligne() == "VUE"
    r = s.calculer([0, 0, 0, 1, 0], 0.02)             # erreur +19 : angle 90 + kp x 19, vitesse à mi-chemin
    assert r == Vec2(109.0, 60.0)
    assert s.derniere_erreur == 19.0 and s.position_avant_perte() == 19.0


def test_suivi_entre_sondes_puis_perdue():
    s = _suivi()
    s.calculer([0, 0, 0, 1, 0], 0.02)
    sortie = s.calculer([0, 0, 0, 0, 0], 0.05)
    assert s.ligne == SuiviLigne.Ligne.ENTRE_SONDES and sortie == Vec2(109.0, 60.0)
    s.calculer([0, 0, 0, 0, 0], 0.06)                 # 0,11 s > delai : PERDUE, braquage max à droite
    assert s.ligne == SuiviLigne.Ligne.PERDUE
    assert s._derniere_sortie == Vec2(ReglagesRobot.ANGLE_MAX, 20.0)
    assert abs(s.temps_sans_ligne() - 0.11) < 1e-12


def test_suivi_motif_incoherent_ignore_et_quatre_sondes_voisines():
    s = _suivi()
    s.calculer([0, 0, 1, 0, 0], 0.02)
    avant = s.calculer([1, 0, 1, 0, 0], 0.02)         # sondes non voisines : on garde la sortie
    assert avant == Vec2(90.0, 100.0)
    s.calculer([1, 1, 1, 1, 0], 0.02)                 # milieu de -38 et 19 = -9,5
    assert s._erreur == -9.5


def test_suivi_arrivee_et_reinitialiser():
    s = _suivi()
    for _ in range(30):
        s.calculer([0, 0, 1, 0, 0], 0.02)
    assert not s.arrivee
    for _ in range(3):
        s.calculer([1, 1, 1, 1, 1], 0.02)
    assert s.arrivee
    s.reinitialiser()
    assert s.ligne == SuiviLigne.Ligne.PERDUE and s.temps_sans_ligne() == 0.0


def test_suivi_abandon():
    s = _suivi()
    s.calculer([0, 0, 1, 0, 0], 0.02)
    for _ in range(100):
        r = s.calculer([0, 0, 0, 0, 0], 0.1)
    assert r == Vec2(90.0, 0.0)


# ----- ArretObstacle, ReglagesRobot ------------------------------------------------------------------------------

def _robot():
    r = ReglagesRobot()
    r.ultrason_avant_mm = 10.0
    r.deceleration_freinage_mm_s2 = 200.0
    r.marge_freinage = 0.5
    r.vitesse_approche_min_mm_s = 5.0
    r.distance_sondes_mm = 217.0
    r.empattement_mm = 138.0
    r.angle_roues_max_deg = 30.0
    return r


def test_arret_obstacle():
    a = ArretObstacle(_robot())
    a.recul_autorise, a.seuil_recul_mm, a.vitesse_recul_mm_s = True, 15.0, 8.0
    # restante = 400 - 10 - 100 = 290 > 30 : pas arrêté, vitesse limitée par la cible
    assert a.limiter(400.0, 50.0, 100.0) == 50.0 and not a.arrete
    # restante = 100 : vitesse de freinage sqrt(2 x 200 x 0,5 x 100) = sqrt(20000)
    assert a.limiter(210.0, 1000.0, 100.0) == gd.sqrt(20000.0)
    # restante = 1 : arrêt déclenché ; entre 1 et 30 mm il reste arrêté ; au-delà de 30 mm il repart
    assert a.limiter(111.0, 50.0, 100.0) == 0.0 and a.arrete
    assert a.limiter(130.0, 50.0, 100.0) == 0.0
    assert a.limiter(150.0, 50.0, 100.0) == 50.0 and not a.arrete     # restante 40 > 30
    a.limiter(111.0, 50.0, 100.0)
    a.relacher()
    assert not a.arrete and not a.en_recul


def test_arret_obstacle_trop_pres_recule():
    a = ArretObstacle(_robot())
    a.recul_autorise, a.seuil_recul_mm, a.vitesse_recul_mm_s = True, 15.0, 8.0
    assert a.limiter(90.0, 50.0, 100.0) == -8.0 and a.en_recul        # restante = -20 < -15
    assert a.limiter(105.0, 50.0, 100.0) == -8.0                      # restante = -5 : continue de reculer
    assert a.limiter(110.0, 50.0, 100.0) == 0.0 and a.arrete and not a.en_recul   # restante = 0 : arrêt
    b = ArretObstacle(_robot())
    b.recul_autorise, b.seuil_recul_mm, b.vitesse_recul_mm_s = False, 15.0, 8.0
    assert b.limiter(90.0, 50.0, 100.0) == 0.0                        # recul interdit : seulement l'arrêt


# ----- RepriseDouce ----------------------------------------------------------------------------------------------

def test_reprise_douce():
    r = RepriseDouce()
    r.duree_s, r.vitesse_max_mm_s, r.braquage_max_deg_s = 0.1, 40.0, 100.0
    assert not r.en_cours(True, 0.05)                 # pas démarrée
    r.demarrer()
    assert not r.en_cours(False, 0.05)                # pas en suivi : le temps ne décompte pas
    assert r.en_cours(True, 0.06) and r.en_cours(True, 0.06)
    assert not r.en_cours(True, 0.06)                 # 0,1 - 0,06 - 0,06 < 0
    assert r.limiter_vitesse(80.0) == 40.0 and r.limiter_vitesse(30.0) == 30.0
    assert r.limiter_angle(135.0, 0.1) == 100.0       # 90 -> 135 limité à 10 deg
    r.memoriser_angle(100.0)
    assert r.limiter_angle(80.0, 0.05) == 95.0
    assert r.limiter_angle(98.0, 0.1) == 98.0


# ----- ReculLigne ------------------------------------------------------------------------------------------------

def _recul(finir_sans_ligne=False):
    robot = _robot()
    odo = Odometrie(robot)
    suivi = _suivi()
    rl = ReculLigne(robot, odo, suivi)
    rl.vitesse_recul_ligne_mm_s, rl.point_virtuel_mm, rl.gain_recul = 30.0, 100.0, 2.0
    rl.ligne_perdue_recul_s, rl.finir_sans_ligne = 0.5, finir_sans_ligne
    return rl, odo, suivi


def test_recul_ligne_distance_bornee_et_fin():
    rl, odo, suivi = _recul()
    rl.demarrer(500.0)                                # borné à RECUL_MAX_MM = 200
    assert rl._distance_mm == 200.0 and rl.fin == ReculLigne.Fin.EN_COURS
    odo.parcours = 200.0
    assert rl.calculer(0.0, 1, 0.02, -30.0) == Vec2(90.0, 0.0) and rl.fin == ReculLigne.Fin.TERMINE


def test_recul_ligne_droit_sur_la_ligne_et_freinage():
    rl, odo, suivi = _recul()
    rl.demarrer(150.0)
    odo.parcours = 50.0                               # restante 100
    r = rl.calculer(0.0, 1, 0.02, -30.0)              # n = 1, somme = 0 : erreur 0, cap 0 -> angle 90
    assert r.x == 90.0
    v_frein = gd.sqrt(2.0 * 200.0 * 0.5 * 100.0)      # 141 > 30 : vitesse de recul limitée à 30
    assert r.y == -30.0 and v_frein > 30.0
    odo.parcours = 148.0                              # restante 2 : freinage = max(sqrt(400), 5) = 20
    assert rl.calculer(0.0, 1, 0.02, -30.0).y == -20.0


def test_recul_ligne_erreur_donne_un_braquage_et_ligne_perdue():
    rl, odo, suivi = _recul()
    rl.demarrer(150.0)
    r = rl.calculer(19.0, 1, 0.02, -30.0)             # ligne à droite de 19 mm, cap nul : braquage saturé
    assert r.x == 135.0
    r = rl.calculer(2.0, 1, 0.02, -30.0)              # 2 mm : delta = L / B x (cap + gain x e / |v|)
    delta = 138.0 / 100.0 * (0.0 + 2.0 * 2.0 / 30.0)  # servo = delta (deg) x 45 / angle_roues_max
    assert r.x == pytest.approx(90.0 + gd.rad_to_deg(delta) * 45.0 / 30.0, abs=1e-4)
    # ligne perdue : plus aucune sonde pendant plus de ligne_perdue_recul_s
    for _ in range(30):
        r = rl.calculer(0.0, 0, 0.02, -30.0)
    assert rl.fin == ReculLigne.Fin.LIGNE_PERDUE and r == Vec2(90.0, 0.0)
    rl2, odo2, _ = _recul(finir_sans_ligne=True)
    rl2.demarrer(150.0)
    for _ in range(30):
        r = rl2.calculer(0.0, 0, 0.02, -30.0)
    assert rl2.fin == ReculLigne.Fin.EN_COURS and rl2.ligne_perdue_en_route and r.x == 90.0
    rl3, _, _ = _recul()
    rl3.demarrer(150.0, attente_ligne_s=1.0)          # attente tolérée au début
    for _ in range(30):
        rl3.calculer(0.0, 0, 0.02, -30.0)
    assert rl3.fin == ReculLigne.Fin.EN_COURS


# ----- Rattrapage ------------------------------------------------------------------------------------------------

def _rattrapage():
    suivi = _suivi()
    odo = Odometrie(_robot())
    r = Rattrapage(odo, suivi)
    r.actif, r.apres_s, r.pause_s, r.vitesse_mm_s, r.distance_max_mm = True, 0.1, 0.2, 30.0, 100.0
    r.tentatives_max, r.ligne_retrouvee_s, r.arret_si_introuvable = 2, 0.5, True
    r.fin_ligne_ecart_mm, r.fin_ligne_vue_s, r.garder_cote = 10.0, 1.0, True
    return r, odo, suivi


def test_rattrapage_reaction_virage_et_fin_de_ligne():
    r, odo, suivi = _rattrapage()
    r.nouveau_suivi()
    assert r.reaction() == Rattrapage.Reaction.AUCUNE and not r.ligne_perdue()
    for _ in range(10):                               # ligne vue à droite (19 mm) pendant 0,2 s
        suivi.calculer([0, 0, 0, 1, 0], 0.02)
        r.observer(0.02)
    # 1re absence (0,02 s) : ENTRE_SONDES (ligne pas au bord) ; ensuite > delai_entre_sondes_s : PERDUE
    for _ in range(7):                                # ligne perdue 0,14 s >= apres_s
        suivi.calculer([0, 0, 0, 0, 0], 0.02)
        r.observer(0.02)
    assert r.ligne_perdue() and r.reaction() == Rattrapage.Reaction.RATTRAPER
    # fin de ligne : perdue alors qu'elle était centrée (|0| <= 10) et suivie depuis >= 1 s
    r2, odo2, suivi2 = _rattrapage()
    r2.nouveau_suivi()
    for _ in range(60):
        suivi2.calculer([0, 0, 1, 0, 0], 0.02)
        r2.observer(0.02)
    for _ in range(40):
        suivi2.calculer([0, 0, 0, 0, 0], 0.02)
        r2.observer(0.02)
    assert r2.reaction() == Rattrapage.Reaction.ARRETER
    r2.arret_si_introuvable = False
    assert r2.reaction() == Rattrapage.Reaction.AUCUNE


def test_rattrapage_manoeuvre_complete():
    r, odo, suivi = _rattrapage()
    r.demarrer(1)
    # 1) arrêt, roues braquées du côté opposé : attend v < 2 et pause_s
    assert r.calculer([0, 0, 0, 0, 0], 0.0, 0, 10.0, 0.5, 0.02) == Vec2(45.0, 0.0)
    assert r.calculer([0, 0, 0, 0, 0], 0.0, 0, 0.0, 0.1, 0.02) == Vec2(45.0, 0.0) and not r._recule
    assert r.calculer([0, 0, 0, 0, 0], 0.0, 0, 0.0, 0.2, 0.02) == Vec2(45.0, 0.0) and r._recule
    # 2) marche arrière
    assert r.calculer([0, 0, 0, 0, 0], 0.0, 0, 0.0, 0.3, 0.02) == Vec2(45.0, -30.0)
    suivi.derniere_erreur = 5.0
    r.calculer([0, 1, 0, 0, 0], -19.0, 1, -30.0, 0.4, 0.02)
    assert suivi.derniere_erreur == -19.0
    # sonde centrale : fin du recul, puis pause_s à l'arrêt, puis termine
    assert r.calculer([0, 0, 1, 0, 0], 0.0, 1, -30.0, 0.5, 0.02) == Vec2(90.0, 0.0) and r._fini
    for _ in range(5):
        r.calculer([0, 0, 1, 0, 0], 0.0, 1, 0.0, 0.6, 0.02)
    assert not r.termine                              # 0,1 s < pause_s
    for _ in range(10):
        r.calculer([0, 0, 1, 0, 0], 0.0, 1, 0.0, 0.6, 0.02)
    assert r.termine


def test_rattrapage_garder_cote_et_tentatives():
    r, odo, suivi = _rattrapage()
    r.demarrer(-1)
    assert r._cote == -1 and r._tentatives == 1
    r.demarrer(1)                                     # garder_cote : même côté que la première tentative
    assert r._cote == -1 and r._tentatives == 2
    r.garder_cote = False
    r.demarrer(1)
    assert r._cote == 1
    r.nouveau_suivi()
    suivi.calculer([0, 0, 1, 0, 0], 0.02)
    r.observer(0.6)                                   # ligne suivie >= ligne_retrouvee_s : compteur à zéro
    assert r._tentatives == 0
