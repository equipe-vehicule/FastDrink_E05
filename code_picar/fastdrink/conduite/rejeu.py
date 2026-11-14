"""Rejeu d'une trace du contrôleur Godot (simulation/trace_controleur.gd, option --trace) par le contrôleur Python.

Chaque tour : mêmes entrées que Controleur.calculer() dans Godot, puis comparaison :
  - état et phase du contournement : EXACTEMENT ;
  - sorties (vitesse, angle, entiers envoyés au robot) : exactement (un écart de 1 avec une commande égale = mauvais
    arrondi, ex. round() de Python au lieu de gd.roundf) ;
  - grandeurs internes (commande flottante, consigne, distance filtrée, braquage moyen, odométries, servo estimé) :
    |python - godot| <= TOLERANCE_ABS + TOLERANCE_REL x |godot|.
Le premier tour divergent est rendu avec son contexte (entrées, tours précédents).

Pourquoi une tolérance, et pourquoi si petite (mesuré, tests/test_godot.py et tests/test_odometrie.py) :
  - sin, cos et tan de Godot sous Windows diffèrent de ceux de CPython de 1 ulp (~1e-16 relatif) pour ~5 % des
    valeurs d'un échantillon large ; sur les 6 traces de référence pourtant, l'odométrie globale Python, cumulée sur
    tous les tours, est identique au bit près. Sur le Pi (glibc), des écarts de cet ordre restent possibles ;
  - un oubli de l'arrondi float32 d'un Vector2 (piège principal du portage) donne un écart relatif de ~1e-8 à 6e-8 :
    la tolérance doit rester bien en dessous pour le détecter.
  D'où TOLERANCE_REL = 1e-12 et TOLERANCE_ABS = 1e-9 (mm, rad, deg, mm/s ; pour les valeurs proches de zéro). Si un
  écart de libm fait un jour changer un arrondi float32, l'écart apparaîtra ici : l'examiner avant d'élargir.

Interface attendue du contrôleur Python (mêmes noms que controleur.gd ; c'est ce que lit trace_controleur.gd) :
  calculer(sondes, distance_cm, dt, mesure_nouvelle=True) -> (vitesse, angle) ;
  etat_court() ; contournement.phase (IntEnum, mêmes noms que Contournement.Phase) ; _commande_envoyee() ; _v_mm_s ;
  distance_filtree_mm ; braquage_moyen() ; odo.x, odo.y, odo.cap, odo.parcours, odo.x_global, odo.y_global,
  odo.cap_global, odo.servo_estime.
"""
import gzip
import json
import math

from . import godot as gd
from .reglages import appliquer_reglages, convertir, schemas_du_controleur

FORMATS_CONNUS = (1,)
TOLERANCE_ABS = 1e-9
TOLERANCE_REL = 1e-12
## Colonnes comparées avec tolérance (flottants) ; les autres sorties sont comparées exactement.
COLONNES_REELLES = ("cmd", "v_mm_s", "d_filtre_mm", "braquage_moyen", "odo_x", "odo_y", "odo_cap", "odo_parcours",
                    "glob_x", "glob_y", "glob_cap", "servo_estime")
COLONNES_EXACTES = ("etat", "phase", "vitesse", "angle")


class Trace:
    """Trace lue : entete (dict), tours (liste de dicts {colonne: valeur}), fin (dict ou None si coupée)."""

    def __init__(self, entete, tours, fin, chemin=""):
        self.entete = entete
        self.tours = tours
        self.fin = fin
        self.chemin = chemin

    @property
    def nom(self):
        return self.entete.get("cas", "")


def _nombre(v):
    """null du JSON de Godot = NaN (JSON.stringify écrit NAN en null)."""
    return gd.NAN if v is None else v


def lire_trace(chemin):
    """Lit une trace .jsonl ou .jsonl.gz. Lève ValueError si le format n'est pas reconnu."""
    ouvrir = gzip.open if chemin.endswith(".gz") else open
    with ouvrir(chemin, "rt", encoding="utf-8") as f:
        lignes = [l for l in f.read().splitlines() if l.strip()]
    entete = json.loads(lignes[0])
    if entete.get("type") != "entete" or entete.get("format") not in FORMATS_CONNUS:
        raise ValueError("%s : en-tête de trace absent ou format inconnu" % chemin)
    colonnes = entete["colonnes"]
    tours = []
    fin = None
    for texte in lignes[1:]:
        donnees = json.loads(texte)
        if isinstance(donnees, dict):
            if donnees.get("type") == "fin":
                fin = donnees
            continue
        tour = dict(zip(colonnes, donnees))
        for c in ("d_cm", "dt") + COLONNES_REELLES:
            if c in tour:
                tour[c] = _nombre(tour[c])
        tours.append(tour)
    return Trace(entete, tours, fin, chemin)


def regler_depuis_entete(controleur, entete):
    """Règle un contrôleur neuf exactement comme celui de la trace : tous les réglages de l'en-tête (types convertis
    selon REGLAGES de chaque module) et les valeurs physiques du robot."""
    schemas = schemas_du_controleur(controleur)
    valeurs = {}
    for module, reglages in entete["reglages"].items():
        if module not in schemas:
            raise ValueError("trace : module « %s » absent du contrôleur Python" % module)
        valeurs[module] = {cle: convertir(v, schemas[module][cle]) for cle, v in reglages.items()}
    appliquer_reglages(controleur, valeurs)
    controleur.robot.appliquer_champs(entete["robot"])
    return controleur


def observer(c, vitesse, angle):
    """Ce que trace_controleur.gd écrit après un tour, lu sur le contrôleur Python."""
    etat = c.etat_court()
    return {
        "vitesse": int(vitesse), "angle": int(angle), "etat": etat,
        "phase": c.contournement.phase.name if etat == "CONTOURNEMENT" else "",
        "cmd": c._commande_envoyee(), "v_mm_s": c._v_mm_s, "d_filtre_mm": c.distance_filtree_mm,
        "braquage_moyen": c.braquage_moyen(), "odo_x": c.odo.x, "odo_y": c.odo.y, "odo_cap": c.odo.cap,
        "odo_parcours": c.odo.parcours, "glob_x": c.odo.x_global, "glob_y": c.odo.y_global,
        "glob_cap": c.odo.cap_global, "servo_estime": c.odo.servo_estime,
    }


def proches(python, godot, tol_abs=TOLERANCE_ABS, tol_rel=TOLERANCE_REL):
    """Égalité à la tolérance près ; NaN = NaN ; INF = INF."""
    if python == godot:
        return True
    if gd.is_nan(python) or gd.is_nan(godot) or math.isinf(python) or math.isinf(godot):
        return gd.is_nan(python) and gd.is_nan(godot)
    return abs(python - godot) <= tol_abs + tol_rel * abs(godot)


def comparer_tour(attendu, obtenu):
    """Liste des écarts [(colonne, godot, python)] d'un tour (vide = identique)."""
    ecarts = []
    for c in COLONNES_EXACTES:
        if obtenu[c] != attendu[c]:
            ecarts.append((c, attendu[c], obtenu[c]))
    for c in COLONNES_REELLES:
        if not proches(obtenu[c], attendu[c]):
            ecarts.append((c, attendu[c], obtenu[c]))
    return ecarts


class Rapport:
    """Résultat d'un rejeu : premier tour divergent (None = trace rejouée sans écart) et écart maximal relevé."""

    def __init__(self, trace):
        self.trace = trace
        self.tours_rejoues = 0
        self.premier_ecart = None     # index du tour
        self.ecarts = []              # [(colonne, godot, python)] du premier tour divergent
        self.contexte = ""
        self.ecart_max = {}           # {colonne: plus grand |python - godot|} avant le premier écart

    @property
    def identique(self):
        return self.premier_ecart is None

    def __str__(self):
        if self.identique:
            pire = ", ".join("%s %.3g" % (c, e) for c, e in sorted(self.ecart_max.items()) if e > 0.0)
            return "%s : %d tours identiques (écart max : %s)" % (self.trace.nom, self.tours_rejoues, pire or "aucun")
        return self.contexte


def _texte_tour(i, tour):
    return "  tour %d t=%.4f s sondes=%s d_cm=%r dt=%r nouvelle=%s -> v=%s a=%s %s %s v_mm_s=%r d_filtre=%r" % (
        i, tour["t"], tour["s"], tour["d_cm"], tour["dt"], tour["nouvelle"], tour["vitesse"], tour["angle"],
        tour["etat"], tour["phase"], tour["v_mm_s"], tour["d_filtre_mm"])


def rejouer(controleur, trace, arreter_au_premier_ecart=True):
    """Nourrit controleur (déjà réglé : regler_depuis_entete) avec les entrées de la trace, tour par tour."""
    rapport = Rapport(trace)
    for i, tour in enumerate(trace.tours):
        vitesse, angle = controleur.calculer(list(tour["s"]), tour["d_cm"], tour["dt"], tour["nouvelle"])
        obtenu = observer(controleur, vitesse, angle)
        rapport.tours_rejoues = i + 1
        ecarts = comparer_tour(tour, obtenu)
        if not ecarts:
            for c in COLONNES_REELLES:
                if not (gd.is_nan(tour[c]) or math.isinf(tour[c])):
                    rapport.ecart_max[c] = max(rapport.ecart_max.get(c, 0.0), abs(obtenu[c] - tour[c]))
            continue
        if rapport.premier_ecart is None:
            rapport.premier_ecart = i
            rapport.ecarts = ecarts
            lignes = ["%s (%s) : premier écart au tour %d (t = %.4f s)" % (trace.nom, trace.chemin, i, tour["t"])]
            lignes += ["  %s : godot %r, python %r" % e for e in ecarts]
            lignes.append("Tours précédents (Godot) :")
            lignes += [_texte_tour(j, trace.tours[j]) for j in range(max(0, i - 3), i + 1)]
            lignes.append("Python au tour %d : v=%s a=%s %s %s v_mm_s=%r d_filtre=%r" % (
                i, obtenu["vitesse"], obtenu["angle"], obtenu["etat"], obtenu["phase"], obtenu["v_mm_s"],
                obtenu["d_filtre_mm"]))
            rapport.contexte = "\n".join(lignes)
        if arreter_au_premier_ecart:
            break
    return rapport
