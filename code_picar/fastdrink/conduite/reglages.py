"""Réglages du contrôleur : ReglagesRobot, ReglagesEpreuve et lecture des deux fichiers JSON de la racine du dépôt.

Portage de simulation/reglages_robot.gd, reglages_epreuve.gd, fichier_reglages.gd et reglages_algorithmes.gd.
Sources des valeurs (une seule chacune, partagées avec la simulation Godot) :
  - reglages_algorithmes.json : réglages des algorithmes, un objet par module (même nom que la propriété du
    Contrôleur : suivi, ultrason, fluide...) ; « controleur » = réglages du Contrôleur lui-même ;
  - reglages_robot.json : valeurs physiques ; le contrôleur n'en lit que ReglagesRobot.CLES_JSON, la table de vitesse,
    vitesse_mm_s_par_cmd et filtre_ultrason (jamais le modèle de simulation : bruit, retard, bille...).
Le modèle IDÉAL de la simulation (FichierReglages.VALEURS_IDEALES) n'existe pas ici : une trace idéale porte dans son
en-tête les valeurs que son contrôleur avait vraiment (rejeu.py).

Différence voulue avec Godot : Godot signale chaque erreur (push_error) et continue ; ici, toutes les erreurs sont
réunies dans une seule ErreurReglages (le robot ne doit pas partir avec un réglage manquant).
"""
import json
import math
import os

from . import godot as gd

## Racine du dépôt (code_picar/fastdrink/conduite/ -> 3 niveaux plus haut) : là où sont les deux fichiers JSON.
RACINE_DEPOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CHEMIN_ALGORITHMES = os.path.join(RACINE_DEPOT, "reglages_algorithmes.json")
CHEMIN_ROBOT = os.path.join(RACINE_DEPOT, "reglages_robot.json")

## Objets du Contrôleur qui portent des réglages : nom (propriété du Contrôleur, objet de reglages_algorithmes.json)
## -> (fichier .gd de simulation/, module Python de fastdrink.conduite, classe). Sert aux tests (mêmes réglages
## qu'en GDScript) et au plan de portage (PORTAGE.md).
MODULES = {
    "controleur": ("controleur.gd", "controleur", "Controleur"),
    "robot": ("reglages_robot.gd", "reglages", "ReglagesRobot"),
    "epreuve": ("reglages_epreuve.gd", "reglages", "ReglagesEpreuve"),
    "suivi": ("suivi_ligne.gd", "suivi_ligne", "SuiviLigne"),
    "ultrason": ("filtre_ultrason.gd", "filtre_ultrason", "FiltreUltrason"),
    "arret": ("arret_obstacle.gd", "arret_obstacle", "ArretObstacle"),
    "contournement": ("contournement.gd", "contournement", "Contournement"),
    "fluide": ("contournement_fluide.gd", "contournement_fluide", "ContournementFluide"),
    "recul": ("recul_ligne.gd", "recul_ligne", "ReculLigne"),
    "rattrapage": ("rattrapage.gd", "rattrapage", "Rattrapage"),
    "retrouver": ("retrouver_ligne.gd", "retrouver_ligne", "RetrouverLigne"),
    "demi_tour": ("demi_tour.gd", "demi_tour", "DemiTour"),
    "reprise": ("reprise_douce.gd", "reprise_douce", "RepriseDouce"),
    "memoire": ("memoire_parcours.gd", "memoire_parcours", "MemoireParcours"),
}

CHAMPS_ROBOT_JSON = ("valeur", "unite", "statut", "source", "tache")
STATUTS = ("provisoire", "mesuré", "hypothèse")


class ErreurReglages(ValueError):
    """Fichier de réglages absent, invalide, ou qui ne correspond pas aux réglages déclarés."""


# =====================================================================================================================
# Conversion des types (comme ReglagesAlgorithmes._convertir)

def convertir(valeur, type_attendu):
    """Valeur convertie au type du réglage (JSON ne distingue pas entier et réel), ou lève ValueError.

    type_attendu : float, int, bool, str ou list (les types GDScript float, int, bool, String, Array).
    Attention : en Python, bool est un int ; on le teste en premier. Les nombres d'une liste deviennent des float,
    comme dans Godot (le JSON de Godot ne donne que des réels)."""
    est_bool = isinstance(valeur, bool)
    est_nombre = isinstance(valeur, (int, float)) and not est_bool
    if type_attendu is float:
        if est_nombre:
            return float(valeur)
    elif type_attendu is int:
        if est_nombre and float(valeur) == math.floor(valeur):
            return int(valeur)
    elif type_attendu is bool:
        if est_bool:
            return valeur
    elif type_attendu is str:
        if isinstance(valeur, str):
            return valeur
    elif type_attendu is list:
        if isinstance(valeur, list):
            return [float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else v for v in valeur]
    raise ValueError("type incompatible (attendu : %s)" % type_attendu.__name__)


# =====================================================================================================================
# ReglagesRobot (simulation/reglages_robot.gd)

class ReglagesRobot:
    """Caractéristiques du robot partagées par tous les algorithmes (géométrie, moteur, servo)."""

    ANGLE_MIN = 45.0           # servo, braquage maximal à gauche (commande)
    ANGLE_MAX = 135.0          # servo, braquage maximal à droite (commande)

    ## Champ de cette classe <- clé de reglages_robot.json (seul endroit où les noms se correspondent).
    CLES_JSON = {
        "empattement_mm": "empattement_mm",
        "longueur_avant_mm": "essieu_ultrason_mm",
        "distance_sondes_mm": "essieu_sondes_mm",
        "ultrason_avant_mm": "ultrason_avant_mm",
        "largeur_voiture_mm": "largeur_mm",
        "arriere_voiture_mm": "arriere_voiture_mm",
        "angle_roues_max_deg": "angle_roues_max_deg",
        "vitesse_servo_deg_s": "vitesse_servo_deg_s",
        "demi_angle_ultrason_deg": "demi_angle_ultrason_deg",
        "frequence_controle_hz": "frequence_controle_hz",
        "erreur_vitesse_odometrie": "erreur_vitesse_odometrie",
        "gain_ultrason": "gain_ultrason",
        "biais_ultrason_mm": "biais_ultrason_mm",
    }
    ## Champs physiques hors CLES_JSON (ReglagesRobot.appliquer de Godot).
    CHAMPS_PHYSIQUES = ("table_vitesse_mm_s", "mm_s_par_cmd", "filtre_ultrason")

    ## Réglages des algorithmes (@export du .gd ; valeurs : reglages_algorithmes.json, objet « robot »).
    REGLAGES = {
        "deceleration_freinage_mm_s2": float,
        "acceleration_montee_mm_s2": float,
        "marge_freinage": float,
        "vitesse_approche_min_mm_s": float,
    }

    def __init__(self):
        # ----- Robot (reglages_robot.json)
        self.empattement_mm = 0.0
        self.longueur_avant_mm = 0.0
        self.distance_sondes_mm = 0.0
        self.ultrason_avant_mm = 0.0
        self.largeur_voiture_mm = 0.0
        self.arriere_voiture_mm = 0.0
        self.angle_roues_max_deg = 0.0
        self.vitesse_servo_deg_s = 0.0
        self.demi_angle_ultrason_deg = 0.0
        self.erreur_vitesse_odometrie = 0.0
        self.table_vitesse_mm_s = []      # [[commande, mm/s], ...] ; vide = modèle linéaire
        self.mm_s_par_cmd = 0.0           # modèle linéaire seulement (clé vitesse_mm_s_par_cmd)
        self.filtre_ultrason = "mediane3"
        self.gain_ultrason = 1.0
        self.biais_ultrason_mm = 0.0
        self.frequence_controle_hz = 0.0
        # ----- Algorithmes (reglages_algorithmes.json)
        self.deceleration_freinage_mm_s2 = 0.0
        self.acceleration_montee_mm_s2 = 0.0
        self.marge_freinage = 0.0
        self.vitesse_approche_min_mm_s = 0.0

    def appliquer(self, valeurs):
        """Remplit les valeurs du robot depuis {clé JSON: valeur} (lire_reglages_robot). Lève ErreurReglages."""
        erreurs = []
        for champ, cle in self.CLES_JSON.items():
            if cle in valeurs:
                setattr(self, champ, float(valeurs[cle]))
            else:
                erreurs.append("« %s » absent de reglages_robot.json" % cle)
        self.table_vitesse_mm_s = [[float(p[0]), float(p[1])] for p in valeurs.get("table_vitesse_mm_s", [])]
        if "vitesse_mm_s_par_cmd" in valeurs:
            self.mm_s_par_cmd = float(valeurs["vitesse_mm_s_par_cmd"])
        elif len(self.table_vitesse_mm_s) < 2:
            erreurs.append("ni table_vitesse_mm_s (2 points au moins) ni vitesse_mm_s_par_cmd")
        self.filtre_ultrason = str(valeurs.get("filtre_ultrason", "mediane3"))
        if erreurs:
            raise ErreurReglages("ReglagesRobot : " + " ; ".join(erreurs))

    def appliquer_champs(self, champs):
        """Remplit les valeurs physiques depuis {champ de cette classe: valeur} (en-tête d'une trace Godot)."""
        for champ in list(self.CLES_JSON) + ["mm_s_par_cmd"]:
            setattr(self, champ, float(champs[champ]))
        self.table_vitesse_mm_s = [[float(p[0]), float(p[1])] for p in champs["table_vitesse_mm_s"]]
        self.filtre_ultrason = str(champs["filtre_ultrason"])

    def corriger_ultrason_cm(self, z_cm):
        """Compensation de l'étalonnage de l'ultrason : d = (z - biais) / gain, en cm. -1 n'est pas transformé."""
        if z_cm < 0.0 or (self.gain_ultrason == 1.0 and self.biais_ultrason_mm == 0.0):
            return z_cm
        return gd.maxf(gd.div_reel(z_cm * 10.0 - self.biais_ultrason_mm, self.gain_ultrason), 0.0) / 10.0

    def avant_voiture_mm(self):
        return self.longueur_avant_mm + self.ultrason_avant_mm

    def est_lineaire(self):
        return len(self.table_vitesse_mm_s) < 2

    def vitesse_max_mm_s(self):
        return self.vitesse_mm_s(100.0)

    def vitesse_mm_s(self, cmd):
        """Vitesse réelle (mm/s, signée) pour une commande de -100 à 100."""
        if self.est_lineaire():
            return cmd * self.mm_s_par_cmd
        c = abs(cmd)
        v = self._vitesse_table(c)
        return v if cmd >= 0.0 else -v

    def commande_pour_vitesse(self, mm_s):
        """Inverse de vitesse_mm_s (partie strictement croissante de la table)."""
        if self.est_lineaire():
            return gd.div_reel(mm_s, self.mm_s_par_cmd)
        t = self.table_vitesse_mm_s
        v = abs(mm_s)
        c = 0.0
        if v > 0.0:
            c = t[len(t) - 1][0]
            for i in range(1, len(t)):
                a = t[i - 1]
                b = t[i]
                if b[1] > a[1] and v <= b[1]:
                    c = a[0] + (v - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
                    break
        return c if mm_s >= 0.0 else -c

    def vitesse_odometrie_mm_s(self, cmd):
        return self.vitesse_mm_s(cmd) * (1.0 + self.erreur_vitesse_odometrie)

    def _vitesse_table(self, c):
        t = self.table_vitesse_mm_s
        n = len(t)
        if c <= t[0][0]:
            return t[0][1]
        for i in range(1, n):
            a = t[i - 1]
            b = t[i]
            if c <= b[0]:
                return a[1] + (c - a[0]) / (b[0] - a[0]) * (b[1] - a[1])
        return t[n - 1][1]

    def vitesse_freinage(self, restante_mm):
        return gd.sqrt(2.0 * self.deceleration_freinage_mm_s2 * self.marge_freinage * restante_mm)

    def servo_vers_roues_deg(self, ecart_servo):
        return ecart_servo / 45.0 * self.angle_roues_max_deg

    def roues_vers_servo(self, angle_roues_deg):
        return gd.div_reel(angle_roues_deg * 45.0, self.angle_roues_max_deg)


# =====================================================================================================================
# ReglagesEpreuve (simulation/reglages_epreuve.gd)

class ReglagesEpreuve:
    """Ce qu'impose l'épreuve (guide du cours et précisions du professeur, voir reglages_epreuve.gd)."""

    RECUL_MAX_MM = 200.0              # guide : recul sur moins de 200 mm
    TOLERANCE_ARRET_MM = 30.0         # guide : précision de l'arrêt

    REGLAGES = {
        "distance_arret_mm": float,
        "distance_recul_mm": float,
        "pause_avant_recul_s": float,
        "pause_avant_contournement_s": float,
        "largeur_obstacle_mm": float,
        "hauteur_obstacle_mm": float,
        "profondeur_obstacle_mm": float,
        "tolerance_centrage_mm": float,
    }

    def __init__(self):
        for nom in self.REGLAGES:
            setattr(self, nom, 0.0)

    def largeur_a_contourner_mm(self):
        return gd.maxf(self.largeur_obstacle_mm, gd.maxf(self.hauteur_obstacle_mm, self.profondeur_obstacle_mm))

    def profondeur_a_contourner_mm(self):
        return gd.maxf(self.largeur_obstacle_mm, gd.maxf(self.hauteur_obstacle_mm, self.profondeur_obstacle_mm))


# =====================================================================================================================
# Lecture des fichiers

def _lire_json(chemin, quoi):
    try:
        with open(chemin, encoding="utf-8") as f:
            donnees = json.load(f)
    except OSError:
        raise ErreurReglages("%s : fichier introuvable (%s)" % (quoi, chemin))
    except ValueError as e:
        raise ErreurReglages("%s : JSON invalide (%s) : %s" % (quoi, chemin, e))
    if not isinstance(donnees, dict):
        raise ErreurReglages("%s : JSON invalide (%s) : objet attendu" % (quoi, chemin))
    return donnees


def lire_reglages_robot(chemin=None):
    """{clé: valeur} de reglages_robot.json (comme FichierReglages.lire, sans le modèle idéal). Lève ErreurReglages."""
    chemin = chemin or CHEMIN_ROBOT
    donnees = _lire_json(chemin, "Réglages du robot")
    valeurs = {}
    erreurs = []
    for nom, entree in donnees.items():
        if nom.startswith("_"):
            continue   # _info : explications, pas une valeur
        if not isinstance(entree, dict):
            erreurs.append("%s n'est pas une entrée {valeur, unite, statut, source, tache}" % nom)
            continue
        for champ in CHAMPS_ROBOT_JSON:
            if champ not in entree:
                erreurs.append("champ « %s » manquant pour %s" % (champ, nom))
        if "statut" in entree and entree["statut"] not in STATUTS:
            erreurs.append("statut « %s » inconnu pour %s" % (entree["statut"], nom))
        if "valeur" in entree:
            valeurs[nom] = entree["valeur"]
    if erreurs:
        raise ErreurReglages("Réglages du robot (%s) : %s" % (chemin, " ; ".join(erreurs)))
    return valeurs


def lire_reglages_algorithmes(schemas, chemin=None):
    """{module: {réglage: valeur convertie}} de reglages_algorithmes.json, vérifié contre schemas.

    schemas : {module: {réglage: type}} (le REGLAGES de chaque classe ; schemas_du_controleur() pour un contrôleur).
    Mêmes contrôles que ReglagesAlgorithmes.appliquer : module inconnu ou absent, clé en trop ou manquante, type
    incompatible ; les clés qui commencent par « _ » sont ignorées. Lève ErreurReglages avec toutes les erreurs."""
    chemin = chemin or CHEMIN_ALGORITHMES
    donnees = _lire_json(chemin, "Réglages des algorithmes")
    erreurs = []
    res = {}
    for nom, valeurs in donnees.items():
        if nom.startswith("_"):
            continue
        if nom not in schemas:
            erreurs.append("module « %s » inconnu (modules : %s)" % (nom, ", ".join(schemas)))
            continue
        if not isinstance(valeurs, dict):
            erreurs.append("« %s » doit être un objet {réglage: valeur}" % nom)
            continue
        attendus = schemas[nom]
        module = {}
        for cle, v in valeurs.items():
            if cle not in attendus:
                erreurs.append("« %s:%s » n'est pas un réglage de ce module" % (nom, cle))
                continue
            try:
                module[cle] = convertir(v, attendus[cle])
            except ValueError as e:
                erreurs.append("« %s:%s » = %r : %s" % (nom, cle, v, e))
        for cle in attendus:
            if cle not in valeurs:
                erreurs.append("« %s:%s » absent du fichier" % (nom, cle))
        res[nom] = module
    for nom in schemas:
        if nom not in donnees:
            erreurs.append("module « %s » absent de %s" % (nom, chemin))
    if erreurs:
        raise ErreurReglages("Réglages des algorithmes : " + " ; ".join(erreurs))
    return res


def schemas_du_controleur(controleur):
    """{module: REGLAGES} des objets qui portent des réglages : le contrôleur (« controleur ») et ceux de ses attributs
    dont la classe déclare REGLAGES (comme ReglagesAlgorithmes._modules, qui prend les @export)."""
    schemas = {"controleur": type(controleur).REGLAGES}
    for nom, objet in vars(controleur).items():
        if not nom.startswith("_") and getattr(type(objet), "REGLAGES", None):
            schemas[nom] = type(objet).REGLAGES
    return schemas


def appliquer_reglages(controleur, valeurs):
    """Écrit {module: {réglage: valeur}} dans le contrôleur et ses modules (« controleur » = lui-même)."""
    for nom, module in valeurs.items():
        cible = controleur if nom == "controleur" else getattr(controleur, nom)
        for cle, v in module.items():
            setattr(cible, cle, list(v) if isinstance(v, list) else v)


def regler_controleur(controleur, chemin_algorithmes=None, chemin_robot=None, essai=None):
    """Règle un contrôleur neuf comme Simulation._controleur_regle : reglages_algorithmes.json, puis les valeurs du
    robot (reglages_robot.json), puis les réglages d'essai {"module:cle": valeur} (ou {"cle": valeur} pour le
    contrôleur lui-même) qui l'emportent."""
    appliquer_reglages(controleur, lire_reglages_algorithmes(schemas_du_controleur(controleur), chemin_algorithmes))
    controleur.robot.appliquer(lire_reglages_robot(chemin_robot))
    schemas = schemas_du_controleur(controleur)
    for chemin, v in (essai or {}).items():
        module, _, cle = chemin.rpartition(":")
        module = module or "controleur"
        if module not in schemas or cle not in schemas[module]:
            raise ErreurReglages("réglage d'essai « %s » inconnu" % chemin)
        appliquer_reglages(controleur, {module: {cle: convertir(v, schemas[module][cle])}})
    return controleur
