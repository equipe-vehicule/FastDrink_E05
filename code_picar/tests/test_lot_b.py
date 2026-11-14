"""Lot B : ContournementFluide et Contournement (et GeometrieVoiture) rendent EXACTEMENT ce que rend Godot.

Valeurs de référence : tests/donnees/lot_b_godot.json.gz, calculées par Godot (tests/outils/sonde_lot_b.gd) : choisir,
rejeu de la poursuite, profil quintique, choisir_cap, et des suites de calculer() sur des entrées enregistrées.
Pour les régénérer (depuis la racine du dépôt ; chemins absolus) :
    G --headless --path . --script <abs>/code_picar/tests/outils/sonde_lot_b.gd -- <abs>/code_picar/tests/donnees/lot_b_godot.json
    py -3 -c "import gzip,shutil;shutil.copyfileobj(open('<json>','rb'),gzip.open('<json>.gz','wb',9))"  (puis supprimer le .json)
Comparaison : booléens, textes et entiers exacts ; réels au bit près, sauf là où le sin/cos/tan de Godot (x87) peut
différer de 1 ulp de CPython (PORTAGE.md, piège 11) : tolérance relative de 1e-9 pour les valeurs qui en dépendent
(positions de l'odométrie intégrées pas à pas), exacte pour celles qui n'en dépendent pas (le nombre de décisions
identiques le prouve).
"""
import gzip
import json
import math
import os

import pytest

from fastdrink.conduite import godot as gd
from fastdrink.conduite import reglages as rg
from fastdrink.conduite.contournement import Contournement
from fastdrink.conduite.contournement_fluide import ContournementFluide
from fastdrink.conduite.geometrie_voiture import GeometrieVoiture
from fastdrink.conduite.godot import Vec2
from fastdrink.conduite.odometrie import Odometrie
from outils_gd import schemas_gd

DONNEES = os.path.join(os.path.dirname(__file__), "donnees", "lot_b_godot.json.gz")
with gzip.open(DONNEES, "rt", encoding="utf-8") as f:
    REF = json.load(f)

SCHEMAS = schemas_gd()
VALEURS = rg.lire_reglages_algorithmes(SCHEMAS)
VALEURS_ROBOT = rg.lire_reglages_robot()


class Banc:
    """robot, épreuve, odométrie et les deux contournements, réglés comme Simulation._controleur_regle (fichiers JSON du
    dépôt), puis les variantes `ov` de la sonde."""

    def __init__(self, ov):
        self.robot = rg.ReglagesRobot()
        self.epreuve = rg.ReglagesEpreuve()
        self.odo = Odometrie(self.robot)
        self.contournement = Contournement(self.robot, self.odo, self.epreuve)
        self.fluide = ContournementFluide(self.robot, self.odo, self.epreuve)
        for nom in ("robot", "epreuve", "contournement", "fluide"):
            for cle, v in VALEURS[nom].items():
                setattr(getattr(self, nom), cle, list(v) if isinstance(v, list) else v)
        self.robot.appliquer(VALEURS_ROBOT)
        for mod, valeurs in ov.items():
            if mod == "servo_estime":
                self.odo.servo_estime = valeurs
                continue
            for cle, v in valeurs.items():
                setattr(getattr(self, mod), cle, v)


def egal(a, b):
    """Exact ; None (NaN du JSON de Godot) = NaN."""
    if b is None:
        return a != a
    if isinstance(b, (bool, str)) or isinstance(a, (bool, str)):
        return a == b
    a = float(a)
    b = float(b)
    if a != a or b != b:
        return a != a and b != b
    return a == b


def proche(a, b, tol=1e-9):
    """Égalité à tol près (relative + absolue), pour les valeurs qui passent par sin/cos/tan intégrés pas à pas."""
    if b is None:
        return a != a
    a = float(a)
    b = float(b)
    if a != a or b != b:
        return a != a and b != b
    return abs(a - b) <= tol * (1.0 + abs(b))


# ----- Fonctions pures sur des entrées simples --------------------------------------------------------------------------

@pytest.mark.parametrize("r", REF["q"])
def test_profil_quintique_comme_godot(r):
    assert egal(ContournementFluide._q(r["t"]), r["q"])


def test_profil_quintique_valeurs_connues():
    q = ContournementFluide._q
    assert q(0.0) == 0.0 and q(1.0) == 1.0 and q(-3.0) == 0.0 and q(7.0) == 1.0
    assert abs(q(0.5) - 0.5) < 1e-15                       # 10/8 - 15/16 + 6/32 = 0.5
    assert abs(q(0.25) + q(0.75) - 1.0) < 1e-12            # symétrie du profil


def test_geometrie_voiture():
    obs_min, obs_max = Vec2(-40.0, 200.0), Vec2(40.0, 250.0)
    # voiture bien à gauche : pas de contact ; en plein dans l'obstacle : contact
    assert not GeometrieVoiture.touche(Vec2(-200.0, 100.0), 0.0, 180.0, 120.0, 55.0, obs_min, obs_max)
    assert GeometrieVoiture.touche(Vec2(0.0, 100.0), 0.0, 180.0, 120.0, 55.0, obs_min, obs_max)
    # coin de l'obstacle dans le flanc de la voiture, sans coin de voiture dans l'obstacle
    assert GeometrieVoiture.touche(Vec2(0.0, 225.0), 0.0, 10.0, 10.0, 100.0, Vec2(90.0, 220.0), Vec2(95.0, 230.0))


# ----- ContournementFluide ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("i", range(len(REF["fluide"])))
def test_fluide_comme_godot(i):
    r = REF["fluide"][i]
    b = Banc(r["ov"])
    ok = b.fluide.choisir(r["d"], r["cote"], r["x0"], r["cap0"])
    assert ok == r["ok"]
    for nom, attendu in (("rayon", b.fluide.rayon_choisi_mm), ("v", b.fluide.vitesse_mm_s), ("y1", b.fluide._y1),
                         ("y2", b.fluide._y2), ("y3", b.fluide._y3), ("y_face", b.fluide._y_face),
                         ("ecart", b.fluide._ecart), ("x0_int", b.fluide._x0), ("cap0_int", b.fluide._cap0),
                         ("ecart_mm", b.fluide.ecart_mm()), ("rayon_retour", b.fluide._rayon_retour_mm())):
        assert egal(attendu, r[nom]), nom
    for y, x in r["x_chemin"]:
        assert egal(b.fluide.x_chemin(y), x), y
    if not ok:
        return
    b.fluide.demarrer()
    dt = 0.02
    for k, pas in enumerate(r["pas"]):
        somme, n = pas[0], pas[1]
        sortie = b.fluide.calculer(somme, n)
        assert isinstance(sortie, Vec2)
        b.odo.suivre_servo(sortie.x, dt)
        b.odo.integrer(sortie.y, dt)
        info = "pas %d" % k
        assert egal(sortie.x, pas[2]), info
        assert egal(sortie.y, pas[3]), info
        assert b.fluide.termine == pas[4] and b.fluide.fin_sans_ligne == pas[5], info
        assert egal(b.fluide.e_reprise, pas[6]), info
        assert b.fluide.nom_phase() == pas[7], info
        assert proche(b.odo.x, pas[8]) and proche(b.odo.y, pas[9]) and proche(b.odo.cap, pas[10]), info


def test_fluide_couvre_succes_et_echecs():
    assert any(r["ok"] for r in REF["fluide"]) and any(not r["ok"] for r in REF["fluide"])
    assert any(r["ok"] and r["pas"][-1][4] and not r["pas"][-1][5] for r in REF["fluide"])


# ----- Contournement : choisir_cap -------------------------------------------------------------------------------------

@pytest.mark.parametrize("i", range(len(REF["choisir_cap"])))
def test_choisir_cap_comme_godot(i):
    r = REF["choisir_cap"][i]
    b = Banc(r["ov"])
    c = b.contournement
    c.demarrer(r["d0"], r["braq"])
    assert c.cote == r["cote"]
    assert egal(c._y_fin, r["y_fin"]) and egal(c._x_requis, r["x_requis"])
    assert egal(c._centre.x, r["centre"][0]) and egal(c._centre.y, r["centre"][1])
    assert [[z.x, z.y] for z in c._zones] == r["zones"]
    assert egal(c._largeur_obstacle(), r["largeur"]) and egal(c._longueur_obstacle(), r["longueur"])
    assert proche(c._rayon_virage_mm(), r["rayon_virage"], 1e-14)   # tan : 1 ulp possible
    assert c.choisir_cap(r["d0"], r["marge"]) == r["ok"]
    assert egal(c._cap_ecart_deg, r["cap"])


def test_choisir_cap_couvre_succes_et_echecs():
    oks = [r["ok"] for r in REF["choisir_cap"]]
    assert any(oks) and not all(oks)


# ----- Contournement : suites de calculer() -----------------------------------------------------------------------------

@pytest.mark.parametrize("i", range(len(REF["contournement"])))
def test_contournement_comme_godot(i):
    r = REF["contournement"][i]
    b = Banc(r["ov"])
    c = b.contournement
    c.chemin_entrant = [Vec2(x, y) for x, y in r["chemin"]]
    if r["hors"]:
        c.demarrer_hors_ligne(r["d0"], r["cote"])
    else:
        c.demarrer(r["d0"], r["braq"])
        if r["avec_choix"]:
            assert c.choisir_cap(r["d0"], b.fluide.marge_obstacle_mm) == r["ok_cap"]
    dt = 0.02
    v = c.vitesse_manoeuvre_mm_s
    for k, pas in enumerate(r["pas"]):
        somme, n, d, d_brut = pas[0], pas[1], pas[2], pas[3]
        sortie = c.calculer(somme, n, v, d, d_brut, dt)
        assert isinstance(sortie, Vec2)
        b.odo.suivre_servo(sortie.x, dt)
        b.odo.integrer(sortie.y * 4.0, dt)
        v = sortie.y
        info = "pas %d (%s)" % (k, pas[6])
        assert egal(sortie.x, pas[4]), info
        assert egal(sortie.y, pas[5]), info
        assert c.nom_phase() == pas[6], info
        assert c.termine == pas[7] and c.echec == pas[8], info
        assert proche(c.ecart_ligne_mm(), pas[9]), info
        assert proche(b.odo.x, pas[10]) and proche(b.odo.y, pas[11]) and proche(b.odo.cap, pas[12]), info
        assert proche(c._x_requis, pas[13]) and proche(c._e_vu, pas[14]) and proche(c._arc, pas[15]), info


def test_contournement_couvre_les_phases():
    vues = {pas[6].split(" ")[0] for r in REF["contournement"] for pas in r["pas"]}
    assert {"ECART", "LONGER", "RETOUR", "ORBITE", "TRAVERS", "RECHERCHE", "SORTIE_ORBITE", "CROISEMENT"} <= vues


def test_nom_phase_et_enumeration():
    b = Banc({})
    assert b.contournement.nom_phase() == "ECART (droite)"
    assert [p.name for p in Contournement.Phase] == ["ECART", "LONGER", "RETOUR", "RECHERCHE", "CROISEMENT",
                                                     "ALIGNEMENT", "ORBITE", "SORTIE_ORBITE", "TRAVERS"]
    assert b.fluide.nom_phase() == "SORTIE" or b.fluide.nom_phase() in ("LONGER", "RETOUR")
