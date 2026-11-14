"""Arrêt d'urgence pendant une manœuvre (14 nov.) : le contrôleur Python redonne EXACTEMENT ce que fait Godot, et la
manœuvre se termine quand l'écho sous distance_securite_mm dure plus de arret_urgence_max_s.

Données de référence : tests/donnees/arret_urgence_godot.json.gz, enregistrées par Godot (tests/outils/sonde_arret_urgence.gd) :
entrées de calculer(), sorties, état, durée continue de l'arrêt (_t_urgence) et distance filtrée, tour par tour.
Pour les régénérer (depuis la racine du dépôt ; chemins absolus) :
    G --headless --path . --script <abs>/code_picar/tests/outils/sonde_arret_urgence.gd -- <abs>/code_picar/tests/donnees/arret_urgence_godot.json
    py -3 -c "import gzip,shutil;shutil.copyfileobj(open('<json>','rb'),gzip.open('<json>.gz','wb',9))"  (puis supprimer le .json)
Séries : « continu » (4 s d'écho à 30 mm pendant le contournement fluide : BLOQUE), « court » (2,9 s puis écho lointain :
la manœuvre continue), « phases » (contournement par phases avec la ligne en mémoire : RETROUVER, aussitôt suivi de
DECISION parce que l'écho est toujours là).
"""
import gzip
import json
import os

import pytest

from fastdrink.conduite import reglages as rg
from fastdrink.conduite.controleur import Controleur

DONNEES = os.path.join(os.path.dirname(__file__), "donnees", "arret_urgence_godot.json.gz")
with gzip.open(DONNEES, "rt", encoding="utf-8") as f:
    REF = json.load(f)

DT = REF["dt"]
MANOEUVRES = ("CONTOURNEMENT", "CONTOURNEMENT_FLUIDE")
FLUIDE = {"continu": True, "court": True, "phases": False}


def rejouer(nom):
    """Rejoue les entrées de la série ; rend (états, t_urgence, tours où _lancer_retrouver a été appelé)."""
    c = Controleur()
    rg.regler_controleur(c)
    c.contournement_fluide = FLUIDE[nom]
    retrouver = []
    lancer = c._lancer_retrouver
    tour = [0]

    def espion():
        retrouver.append(tour[0])
        lancer()
    c._lancer_retrouver = espion
    etats = []
    for i, (sondes, d_cm, dt, nouvelle, vitesse, angle, etat, t_urgence, d_filtre) in enumerate(REF["series"][nom]["tours"]):
        tour[0] = i
        v, a = c.calculer(sondes, d_cm, dt, nouvelle)
        assert (v, a) == (vitesse, angle), "%s tour %d : sorties %s au lieu de %s" % (nom, i, (v, a), (vitesse, angle))
        assert c.etat_court() == etat, "%s tour %d : état %s au lieu de %s" % (nom, i, c.etat_court(), etat)
        assert c._t_urgence == pytest.approx(t_urgence, abs=1e-9), "%s tour %d : _t_urgence" % (nom, i)
        assert c.distance_filtree_mm == pytest.approx(d_filtre, rel=1e-9), "%s tour %d : distance filtrée" % (nom, i)
        etats.append(etat)
    return etats, retrouver


def debut_arret(nom):
    """Premier tour où l'arrêt d'urgence est en cours (durée continue > 0 après le tour)."""
    return next(i for i, t in enumerate(REF["series"][nom]["tours"]) if t[7] > 0.0)


@pytest.mark.parametrize("nom", ["continu", "court", "phases"])
def test_python_redonne_godot(nom):
    rejouer(nom)


@pytest.mark.parametrize("nom", ["continu", "phases"])
def test_la_manoeuvre_se_termine_apres_arret_urgence_max_s(nom):
    etats, retrouver = rejouer(nom)
    c = Controleur()
    rg.regler_controleur(c)
    maxi = c.arret_urgence_max_s
    assert maxi == 3.0
    debut = debut_arret(nom)
    assert etats[debut - 1] in MANOEUVRES and etats[debut] == etats[debut - 1]
    fin = next(i for i in range(debut, len(etats)) if etats[i] not in MANOEUVRES)
    duree = (fin - debut) * DT         # durée continue de l'arrêt au début du tour où la manœuvre prend fin
    assert maxi <= duree + 1e-9 and duree <= maxi + DT + 1e-9, "fin après %.3f s d'arrêt" % duree
    if nom == "continu":
        assert etats[fin] == "BLOQUE" and not retrouver
    else:
        assert retrouver == [fin]      # ligne en mémoire : recherche de la ligne (puis DECISION : l'écho persiste)


def test_un_arret_de_2_9_s_ne_termine_pas_la_manoeuvre():
    etats, _ = rejouer("court")
    tours = REF["series"]["court"]["tours"]
    assert 2.8 < max(t[7] for t in tours) < 3.0          # l'arrêt a bien duré presque 3 s
    assert all(e in MANOEUVRES for e in etats[debut_arret("court"):])
    assert tours[-1][7] == 0.0                           # remis à zéro dès que l'écho s'éloigne


def test_reglage_declare_et_lu_du_json():
    c = Controleur()
    assert c.arret_urgence_max_s == 0.0 and Controleur.REGLAGES["arret_urgence_max_s"] is float
    rg.regler_controleur(c)
    assert c.arret_urgence_max_s == 3.0
