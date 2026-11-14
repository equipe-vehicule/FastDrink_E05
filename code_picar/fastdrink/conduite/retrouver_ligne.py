"""Recherche de la ligne perdue depuis longtemps (virage raté après un contournement...) : au lieu de s'arrêter, la
voiture va, toujours en avançant, devant le dernier point de ligne vu (dans le sens de marche), puis cherche en
spirale jusqu'à croiser la ligne. (Journal Godot du 6 novembre, 19 h 17 : la voiture tournait en rond 8 s puis
s'arrêtait seule.) Les transitions (ligne revue, obstacle, abandon) sont décidées par le Contrôleur.

Portage de simulation/retrouver_ligne.gd (lot C).
"""
from . import godot as gd
from .godot import Vec2


class RetrouverLigne:
    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « retrouver »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        # ligne perdue depuis ça en suivi. Pas moins : en virage serré, la voiture retrouve la ligne seule en
        # tournant (à 4 s, elle bouclait)
        "apres_s": float,
        "max_s": float,                     # ensuite BLOQUE
        "approche_max_s": float,            # ensuite, ou une fois la cible atteinte : recherche en spirale
        "avance_cible_mm": float,           # la cible est devant le dernier point vu, dans le sens de marche : jamais en arrière
        "rayon_cible_mm": float,            # cible considérée comme atteinte
        "spirale_s": float,                 # plus grand = la spirale s'ouvre plus lentement
        "vitesse_mm_s": float,              # (= ancienne commande 18 à 4 mm/s par commande)
        "gain": float,                      # degrés de roue par degré d'écart de direction
        "rayon_obstacle_recent_mm": float,  # si près de l'obstacle évité, on s'en écarte d'abord
    }

    def __init__(self, robot, odo):
        self.apres_s = 0.0
        self.max_s = 0.0
        self.approche_max_s = 0.0
        self.avance_cible_mm = 0.0
        self.rayon_cible_mm = 0.0
        self.spirale_s = 0.0
        self.vitesse_mm_s = 0.0
        self.gain = 0.0
        self.rayon_obstacle_recent_mm = 0.0

        self._robot = robot
        self._odo = odo
        self._cible = Vec2.ZERO
        self._t_spirale = -1.0             # < 0 : la voiture va encore vers la cible
        self._cote_spirale = 1

    def cible_ligne(self, memoire):
        """Où chercher la ligne : devant le dernier point vu (ou devant l'obstacle qui vient d'être passé), dans le
        sens de marche. Revenir au dernier point vu ramenait la voiture sur le chemin déjà parcouru."""
        base = memoire.ligne[-1]
        proches = [memoire.obstacle_hors_ligne]
        if memoire.obstacles:
            proches.append(memoire.obstacles[-1])
        for o in proches:
            if (o.is_finite() and o.distance_to(memoire.ligne[-1]) < self.rayon_obstacle_recent_mm
                    and (o - base).dot(memoire.sens_ligne()) > 0.0):
                base = o
        return base + memoire.sens_ligne() * self.avance_cible_mm

    def demarrer(self, memoire, derniere_erreur):
        """derniere_erreur : dernier écart de la ligne vu par le suivi (côté de la spirale par défaut)."""
        self._cible = self.cible_ligne(memoire)
        self._t_spirale = -1.0
        self._cote_spirale = 1 if derniere_erreur >= 0.0 else -1

    def conduire(self, obstacle_recent, t_etat, dt):
        """Retourne Vec2(angle servo, vitesse cible) pour aller vers la cible, puis tourner en spirale."""
        ici = Vec2(self._odo.x_global, self._odo.y_global)
        vers = self._cible - ici
        rel = gd.wrapf(gd.atan2(vers.x, vers.y) - self._odo.cap_global, -gd.PI, gd.PI)
        if ici.distance_to(obstacle_recent) < self.rayon_obstacle_recent_mm:
            # Tout près de l'obstacle évité, l'ultrason ne voit pas ses coins sur le côté : on s'en écarte d'abord.
            vers_obstacle = obstacle_recent - ici
            rel_obstacle = gd.wrapf(gd.atan2(vers_obstacle.x, vers_obstacle.y) - self._odo.cap_global, -gd.PI, gd.PI)
            if gd.absf(rel_obstacle) < gd.deg_to_rad(100.0):
                return Vec2(90.0 - 45.0 * gd.signf(rel_obstacle), self.vitesse_mm_s)
        if self._t_spirale < 0.0 and (vers.length() < self.rayon_cible_mm or t_etat > self.approche_max_s):
            self._t_spirale = 0.0
            self._cote_spirale = 1 if rel >= 0.0 else -1
        if self._t_spirale >= 0.0:
            self._t_spirale += dt
            return Vec2(90.0 + self._cote_spirale * 45.0 / (1.0 + self._t_spirale / self.spirale_s), self.vitesse_mm_s)
        servo = self._robot.roues_vers_servo(gd.rad_to_deg(rel) * self.gain)
        return Vec2(90.0 + gd.clampf(servo, -45.0, 45.0), self.vitesse_mm_s)
