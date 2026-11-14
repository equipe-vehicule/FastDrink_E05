"""Arrêt à une distance prédéterminée d'un obstacle (précision exigée +/- 30 mm). Portage de simulation/arret_obstacle.gd.
Profil de freinage à décélération constante ; recule si l'on s'est arrêté trop près.
La distance d'arrêt voulue (avant de la voiture -> face de l'obstacle) vient de ReglagesEpreuve.distance_arret_mm.
"""
from . import godot as gd


class ArretObstacle:
    ARRET_TOLERANCE_MM = 1.0   # arrêt déclenché à 1 mm de la cible
    HYSTERESIS_MM = 30.0       # l'obstacle doit s'éloigner de plus que ça pour repartir (= tolérance exigée)

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « arret »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "recul_autorise": bool,        # recule si on s'est arrêté trop près (dépassement)
        "seuil_recul_mm": float,       # recule si on est plus près que cible - seuil (moitié de la tolérance)
        "vitesse_recul_mm_s": float,   # (= ancienne commande 10 à 4 mm/s par commande) vitesse de recul, en valeur absolue
    }

    def __init__(self, robot):
        self.recul_autorise = False
        self.seuil_recul_mm = 0.0
        self.vitesse_recul_mm_s = 0.0

        self.arrete = False
        self.en_recul = False

        self._robot = robot

    def relacher(self):
        """Plus d'obstacle (ou retour en suivi) : on peut repartir."""
        self.arrete = False
        self.en_recul = False

    def limiter(self, d_mm, cible, d_voulue_mm):
        """Vitesse permettant de s'arrêter à d_voulue_mm de l'obstacle (au plus `cible`).
        Retourne une vitesse négative si l'on doit reculer (arrêt trop près)."""
        restante = d_mm - self._robot.ultrason_avant_mm - d_voulue_mm      # mm, < 0 = trop près
        if self.recul_autorise and restante < -self.seuil_recul_mm:
            self.en_recul = True
        if self.en_recul:
            if restante >= 0.0:
                self.en_recul = False
                self.arrete = True
            else:
                return -self.vitesse_recul_mm_s
        if restante <= ArretObstacle.ARRET_TOLERANCE_MM:
            self.arrete = True
        elif restante > ArretObstacle.HYSTERESIS_MM:
            self.arrete = False
        if self.arrete:
            return 0.0
        return gd.minf(cible, gd.maxf(self._robot.vitesse_freinage(restante), self._robot.vitesse_approche_min_mm_s))
