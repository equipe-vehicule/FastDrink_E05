"""Banc de rejeu : le contrôleur Python, nourri tour par tour avec les entrées d'une trace Godot, doit redonner la même
suite d'états (exactement) et les mêmes sorties (voir fastdrink/conduite/rejeu.py pour la tolérance).

Traces de référence : tests/traces/*.jsonl.gz (Godot --trace, voir PORTAGE.md). Tant que le contrôleur n'est pas
porté (fastdrink/conduite/controleur.py absent), test_controleur_python_rejoue_la_trace est sauté ; le banc lui-même
est vérifié dès maintenant par un faux contrôleur qui récite la trace.

Lancer un seul cas avec le détail : py -3 -m pytest -q tests/test_rejeu.py -k recul_court -s
"""
import importlib
from types import SimpleNamespace

import pytest

from fastdrink.conduite import reglages as rg
from fastdrink.conduite.constantes import Phase
from fastdrink.conduite.rejeu import lire_trace, regler_depuis_entete, rejouer
from outils_gd import schema_gd, traces_reference

TRACES = traces_reference()
IDS = [c.split("traces")[-1][1:].replace(".jsonl.gz", "") for c in TRACES]

class FauxControleur:
    """Récite la trace (même interface que le Controleur Python). fausser = {tour: {colonne: valeur}}."""

    def __init__(self, trace, fausser=None):
        self._tours = trace.tours
        self._fausser = fausser or {}
        self._i = -1
        self._t = None
        self.odo = SimpleNamespace()
        self.contournement = SimpleNamespace(phase=Phase.ECART)

    def calculer(self, sondes, distance_cm, dt, mesure_nouvelle=True):
        self._i += 1
        t = dict(self._tours[self._i])
        t.update(self._fausser.get(self._i, {}))
        self._t = t
        if t["phase"]:
            self.contournement.phase = Phase[t["phase"]]
        self._v_mm_s = t["v_mm_s"]
        self.distance_filtree_mm = t["d_filtre_mm"]
        o = self.odo
        o.x, o.y, o.cap, o.parcours = t["odo_x"], t["odo_y"], t["odo_cap"], t["odo_parcours"]
        o.x_global, o.y_global, o.cap_global, o.servo_estime = t["glob_x"], t["glob_y"], t["glob_cap"], t["servo_estime"]
        return t["vitesse"], t["angle"]

    def etat_court(self):
        return self._t["etat"]

    def _commande_envoyee(self):
        return self._t["cmd"]

    def braquage_moyen(self):
        return self._t["braquage_moyen"]


def test_traces_de_reference_presentes():
    assert len(TRACES) >= 4
    for chemin in TRACES:
        t = lire_trace(chemin)
        assert t.fin is not None and t.tours, chemin
        assert t.entete["frequence_controle_hz"] > 0.0


@pytest.mark.parametrize("chemin", TRACES, ids=IDS)
def test_banc_accepte_la_trace_elle_meme(chemin):
    t = lire_trace(chemin)
    r = rejouer(FauxControleur(t), t)
    assert r.identique, str(r)
    assert r.tours_rejoues == len(t.tours)


def test_banc_trouve_le_premier_tour_divergent():
    t = lire_trace(TRACES[0])
    n = len(t.tours) // 2
    r = rejouer(FauxControleur(t, {n: {"etat": "BLOQUE"}, n + 5: {"angle": 0}}), t)
    assert r.premier_ecart == n
    assert r.ecarts == [("etat", t.tours[n]["etat"], "BLOQUE")]
    assert "premier écart au tour %d" % n in str(r)
    assert "tour %d t=" % (n - 1) in str(r)   # contexte : tours précédents


def test_banc_tolerance_des_reels():
    t = lire_trace(TRACES[0])
    i = len(t.tours) // 3
    v = t.tours[i]["glob_x"]
    # oubli d'un arrondi float32 : ~1e-8 relatif, doit être vu
    r = rejouer(FauxControleur(t, {i: {"glob_x": v * (1.0 + 1e-8) + 1e-6}}), t)
    assert r.premier_ecart == i and r.ecarts[0][0] == "glob_x"
    # écart de dernier bit (libm) : accepté
    r = rejouer(FauxControleur(t, {i: {"glob_x": v * (1.0 + 2e-16)}}), t)
    assert r.identique


def test_banc_vitesse_mal_arrondie_detectee():
    """Même consigne et même commande, vitesse différente de 1 : arrondi faux (round() de Python au lieu de
    gd.roundf), signalé."""
    t = lire_trace(TRACES[0])
    i = next(k for k, x in enumerate(t.tours) if x["vitesse"] > 0)
    r = rejouer(FauxControleur(t, {i: {"vitesse": t.tours[i]["vitesse"] + 1}}), t)
    assert r.premier_ecart == i and r.ecarts == [("vitesse", t.tours[i]["vitesse"], t.tours[i]["vitesse"] + 1)]


def _faux_controleur_vide():
    """Contrôleur sans logique, mais avec les mêmes objets à réglages que controleur.gd (REGLAGES lus dans les .gd)."""
    attributs = {}
    for nom, (fichier, _, classe) in rg.MODULES.items():
        if nom in ("controleur", "robot", "epreuve"):
            continue
        attributs[nom] = type(classe, (), {"REGLAGES": schema_gd(fichier)})()
    c = type("Controleur", (), {"REGLAGES": schema_gd("controleur.gd")})()
    c.robot = rg.ReglagesRobot()
    c.epreuve = rg.ReglagesEpreuve()
    for nom, objet in attributs.items():
        setattr(c, nom, objet)
    return c


@pytest.mark.parametrize("chemin", TRACES, ids=IDS)
def test_reglage_depuis_l_entete(chemin):
    t = lire_trace(chemin)
    c = regler_depuis_entete(_faux_controleur_vide(), t.entete)
    for module, reglages in t.entete["reglages"].items():
        cible = c if module == "controleur" else getattr(c, module)
        for cle, v in reglages.items():
            obtenu = getattr(cible, cle)
            attendu_type = schema_gd(rg.MODULES[module][0])[cle]
            assert type(obtenu) is attendu_type, "%s:%s" % (module, cle)
            assert obtenu == v, "%s:%s" % (module, cle)
            if isinstance(obtenu, list):
                assert all(type(x) is float for x in obtenu)
    for champ, v in t.entete["robot"].items():
        assert getattr(c.robot, champ) == v, champ
    assert c.robot.est_lineaire() == (t.entete["modele"] == "ideal")
    if "recul_50" in chemin:
        assert c.epreuve.distance_recul_mm == 50.0


@pytest.mark.parametrize("chemin", TRACES, ids=IDS)
def test_controleur_python_rejoue_la_trace(chemin):
    try:
        module = importlib.import_module("fastdrink.conduite.controleur")
    except ImportError:
        pytest.skip("contrôleur Python pas encore porté (fastdrink/conduite/controleur.py)")
    t = lire_trace(chemin)
    c = regler_depuis_entete(module.Controleur(), t.entete)
    r = rejouer(c, t)
    print(r)
    assert r.identique, str(r)
