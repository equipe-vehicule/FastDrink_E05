"""Suivi de la ligne en reculant sur de courtes distances (< ReglagesEpreuve.RECUL_MAX_MM).
Portage de simulation/recul_ligne.gd.
Utilisé devant chaque obstacle (Contrôleur, état RECUL_OBSTACLE) et en fin de ligne (piste Recul).

Les sondes sont à l'avant, donc à l'arrière du mouvement. Garder les sondes sur la ligne serait
instable (l'essieu, qui mène, s'écarterait de plus en plus). On asservit plutôt un point virtuel
situé derrière l'essieu arrière, estimé avec l'écart lu par les sondes et le cap odométrique.
"""
from . import constantes
from . import godot as gd
from .godot import Vec2
from .arret_obstacle import ArretObstacle
from .reglages import ReglagesEpreuve


class ReculLigne:
    Fin = constantes.Fin              # enum Fin { EN_COURS, TERMINE, LIGNE_PERDUE }

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « recul »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "vitesse_recul_ligne_mm_s": float,  # (= ancienne commande 15 à 4 mm/s par commande) vitesse de recul, en valeur absolue
        "point_virtuel_mm": float,    # distance du point asservi derrière l'essieu arrière (plus grand = plus doux)
        "gain_recul": float,          # 1/s : vitesse de convergence du point virtuel vers la ligne
        "ligne_perdue_recul_s": float,  # ligne invisible plus longtemps : perdue
        ## Ligne perdue en cours de recul : True = finir la distance tout droit (odométrie), False = s'arrêter (Fin.LIGNE_PERDUE).
        ## Voiture arrêtée de travers en sortie de virage (jusqu'à 19 deg) : en reculant, les sondes quittent la ligne après
        ## ~110 mm (banc du 12 nov. : reculs de 78 à 108 mm pour 150 demandés). La distance de recul est imposée, et la ligne
        ## droite avant l'obstacle garantit la place.
        "finir_sans_ligne": bool,
        "reprendre_apres_recul": bool,  # True = repart en marche avant à la fin du recul, False = reste arrêté
    }

    def __init__(self, robot, odo, suivi):
        self.vitesse_recul_ligne_mm_s = 0.0
        self.point_virtuel_mm = 0.0
        self.gain_recul = 0.0
        self.ligne_perdue_recul_s = 0.0
        self.finir_sans_ligne = False
        self.ligne_perdue_en_route = False   # diagnostic : la fin du recul s'est faite sans la ligne
        self.reprendre_apres_recul = False

        self.fin = ReculLigne.Fin.EN_COURS

        self._robot = robot
        self._odo = odo
        self._suivi = suivi
        self._distance_mm = 0.0
        self._t_ligne_perdue = 0.0

    def demarrer(self, distance_mm, attente_ligne_s=0.0):
        """La voiture doit être sur la ligne et à peu près dans son axe : ce cap sert de référence (cap = 0).
        attente_ligne_s : durée tolérée sans ligne au début (recul lancé après la fin de la ligne, sondes au-delà du ruban)."""
        self._distance_mm = gd.clampf(distance_mm, 0.0, ReglagesEpreuve.RECUL_MAX_MM)
        self._odo.reinitialiser()
        self._t_ligne_perdue = -attente_ligne_s
        self.ligne_perdue_en_route = False
        self.fin = ReculLigne.Fin.EN_COURS

    def calculer(self, somme, n, dt, v_mm_s):
        """Retourne Vec2(angle servo, vitesse cible). Regarder `fin` ensuite."""
        restante = self._distance_mm - self._odo.parcours
        if n > 0:
            self._suivi.derniere_erreur = somme / n
            self._t_ligne_perdue = 0.0
        else:
            self._t_ligne_perdue += dt
        if restante <= ArretObstacle.ARRET_TOLERANCE_MM:
            self.fin = ReculLigne.Fin.TERMINE
            return Vec2(90.0, 0.0)
        angle = self._angle_recul(self._suivi.derniere_erreur, v_mm_s)
        if self._t_ligne_perdue > self.ligne_perdue_recul_s:
            if not self.finir_sans_ligne:
                self.fin = ReculLigne.Fin.LIGNE_PERDUE           # ligne perdue : on ne recule pas à l'aveugle
                return Vec2(90.0, 0.0)
            self.ligne_perdue_en_route = True
            angle = 90.0                         # fin du recul tout droit

        # même profil de freinage que devant un obstacle, pour s'arrêter à la distance demandée
        v_freinage = self._robot.vitesse_freinage(restante)
        cible = -gd.minf(self.vitesse_recul_ligne_mm_s, gd.maxf(v_freinage, self._robot.vitesse_approche_min_mm_s))
        return Vec2(angle, cible)

    def _angle_recul(self, erreur_mm, v_mm_s):
        """On asservit un point P situé point_virtuel_mm (B) derrière l'essieu :
          écart de P à la ligne : e_p = e + (D + B) * sin(cap)        (D = distance_sondes_mm)
          loi : delta = L / B * (cap + gain_recul * e_p / |v|)  =>  P converge vers la ligne
        et, contrairement aux sondes, garder P sur la ligne est stable en marche arrière."""
        e_p = erreur_mm + (self._robot.distance_sondes_mm + self.point_virtuel_mm) * gd.sin(self._odo.cap)
        v = gd.maxf(gd.absf(v_mm_s), 20.0)        # évite la division par ~0 au démarrage
        delta = gd.div_reel(self._robot.empattement_mm, self.point_virtuel_mm) * (self._odo.cap + self.gain_recul * e_p / v)
        servo = self._robot.roues_vers_servo(gd.rad_to_deg(delta))
        return 90.0 + gd.clampf(servo, -45.0, 45.0)
