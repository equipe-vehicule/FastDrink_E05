"""Chargement des réglages : mêmes valeurs et mêmes contrôles que la simulation Godot."""
import importlib
import json
import os

import pytest

from fastdrink.conduite import reglages as rg
from fastdrink.conduite.rejeu import lire_trace
from outils_gd import schemas_gd, traces_reference

SCHEMAS_GD = schemas_gd()


def test_chemins_par_defaut_a_la_racine_du_depot():
    assert os.path.isfile(rg.CHEMIN_ALGORITHMES)
    assert os.path.isfile(rg.CHEMIN_ROBOT)
    assert os.path.isdir(os.path.join(rg.RACINE_DEPOT, "simulation"))


def test_fichier_du_depot_conforme_aux_gd():
    valeurs = rg.lire_reglages_algorithmes(SCHEMAS_GD)
    assert set(valeurs) == set(SCHEMAS_GD)
    assert isinstance(valeurs["suivi"]["pas_arrivee"], int)
    assert isinstance(valeurs["controleur"]["contournement_fluide"], bool)
    assert valeurs["fluide"]["rayons_candidats_mm"][0] == 300.0


@pytest.mark.parametrize("nom", sorted(rg.MODULES))
def test_reglages_python_identiques_aux_gd(nom):
    """Chaque classe portée déclare EXACTEMENT les mêmes réglages (noms et types) que son .gd. Classe pas encore
    portée : test sauté."""
    _, module, classe = rg.MODULES[nom]
    try:
        cls = getattr(importlib.import_module("fastdrink.conduite." + module), classe)
    except (ImportError, AttributeError):
        pytest.skip("%s pas encore porté" % classe)
    assert cls.REGLAGES == SCHEMAS_GD[nom]


def _ecrire(tmp_path, donnees):
    chemin = tmp_path / "reglages.json"
    chemin.write_text(json.dumps(donnees), encoding="utf-8")
    return str(chemin)


def test_cle_manquante_en_trop_et_type_signales_ensemble(tmp_path):
    with open(rg.CHEMIN_ALGORITHMES, encoding="utf-8") as f:
        donnees = json.load(f)
    del donnees["suivi"]["kp"]
    donnees["suivi"]["kp_inconnu"] = 1.0
    donnees["suivi"]["pas_arrivee"] = 2.5
    donnees["contournement"]["actif"] = 1          # un entier n'est pas un booléen
    del donnees["memoire"]
    donnees["module_inconnu"] = {}
    with pytest.raises(rg.ErreurReglages) as e:
        rg.lire_reglages_algorithmes(SCHEMAS_GD, _ecrire(tmp_path, donnees))
    texte = str(e.value)
    for attendu in ("« suivi:kp » absent", "suivi:kp_inconnu", "suivi:pas_arrivee", "contournement:actif",
                    "« memoire » absent", "« module_inconnu » inconnu"):
        assert attendu in texte, attendu


def test_entier_accepte_pour_un_reel_et_inversement():
    assert rg.convertir(3, float) == 3.0 and isinstance(rg.convertir(3, float), float)
    assert rg.convertir(4.0, int) == 4 and isinstance(rg.convertir(4.0, int), int)
    with pytest.raises(ValueError):
        rg.convertir(True, float)
    assert rg.convertir([40, 50.5], list) == [40.0, 50.5]


def test_reglages_robot_du_depot():
    r = rg.ReglagesRobot()
    r.appliquer(rg.lire_reglages_robot())
    assert r.empattement_mm > 0.0 and not r.est_lineaire()
    # inverse de la table au-delà de la zone morte
    for v in (50.0, 100.0, 256.0, 400.0):
        assert abs(r.vitesse_mm_s(r.commande_pour_vitesse(v)) - v) < 1e-9
        assert abs(r.vitesse_mm_s(r.commande_pour_vitesse(-v)) + v) < 1e-9
    assert r.vitesse_max_mm_s() == r.table_vitesse_mm_s[-1][1]
    assert r.corriger_ultrason_cm(-1.0) == -1.0


def test_robot_cle_absente(tmp_path):
    r = rg.ReglagesRobot()
    with pytest.raises(rg.ErreurReglages):
        r.appliquer({"empattement_mm": 138.0})


def test_robot_statut_inconnu(tmp_path):
    chemin = _ecrire(tmp_path, {"x": {"valeur": 1, "unite": "mm", "statut": "devine", "source": "", "tache": ""}})
    with pytest.raises(rg.ErreurReglages):
        rg.lire_reglages_robot(chemin)


def _trace(nom):
    return lire_trace([t for t in traces_reference() if nom in t][0])


def test_memes_valeurs_que_le_controleur_godot():
    """Trace réaliste d'une piste sans réglage d'essai : le contrôleur Godot avait exactement les valeurs des deux
    fichiers du dépôt, lues ici par le Python. (Si ce test échoue après une retouche d'un fichier JSON : régénérer
    les traces, voir PORTAGE.md.)"""
    entete = _trace("realiste_parcours_fourni").entete
    valeurs = rg.lire_reglages_algorithmes(SCHEMAS_GD)
    for module, reglages in entete["reglages"].items():
        for cle, v in reglages.items():
            assert rg.convertir(v, SCHEMAS_GD[module][cle]) == valeurs[module][cle], "%s:%s" % (module, cle)
    r = rg.ReglagesRobot()
    r.appliquer(rg.lire_reglages_robot())
    for champ, v in entete["robot"].items():
        assert getattr(r, champ) == v, champ


def test_fonctions_robot_sur_les_sorties_godot():
    """vitesse_mm_s / commande_pour_vitesse rejouées sur les colonnes cmd et v_mm_s de traces Godot (modèle idéal :
    commande flottante = commande_pour_vitesse(v) ; réaliste : arrondie, roundf)."""
    from fastdrink.conduite.godot import roundf
    for nom in ("ideal_recul_court_debout", "realiste_reference_droite"):
        t = _trace(nom)
        r = rg.ReglagesRobot()
        r.appliquer_champs(t.entete["robot"])
        for tour in t.tours:
            c = r.commande_pour_vitesse(tour["v_mm_s"])
            assert (c if r.est_lineaire() else roundf(c)) == tour["cmd"]
