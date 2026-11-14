"""Position estimée de l'essieu arrière (portage de simulation/odometrie.gd, modèle « bicyclette »).

Repère : origine = essieu arrière au début de la manœuvre, x = vers la droite, y = vers l'avant ; cap en rad, > 0 vers
la droite. Le servo ne braque pas instantanément : on suit l'angle qu'il a réellement atteint (servo_estime), en le
faisant tourner vers la commande à ReglagesRobot.vitesse_servo_deg_s.

Exemple de style pour le portage : mêmes noms qu'en GDScript, scalaires en float (double, comme GDScript), Vec2
(float32) partout où le .gd construit un Vector2.
"""
from . import godot as gd
from .godot import Vec2


class Odometrie:
    def __init__(self, robot):
        self.x = 0.0                      # mm
        self.y = 0.0                      # mm
        self.cap = 0.0                    # rad, > 0 = vers la droite
        self.parcours = 0.0               # distance parcourue en marche arrière (mm)
        self.servo_estime = 90.0          # angle réellement atteint par le servo (deg)
        # Position globale : suivie en permanence depuis le départ, jamais remise à zéro
        self.x_global = 0.0
        self.y_global = 0.0
        self.cap_global = 0.0
        self._robot = robot

    def reinitialiser(self):
        """Remet la position à zéro (le servo, lui, garde son angle)."""
        self.x = 0.0
        self.y = 0.0
        self.cap = 0.0
        self.parcours = 0.0

    def suivre_servo(self, commande, dt):
        """À appeler à chaque pas avec la commande envoyée au servo, même hors manœuvre."""
        v = self._robot.vitesse_servo_deg_s
        self.servo_estime = commande if v <= 0.0 else gd.move_toward(self.servo_estime, commande, v * dt)

    def duree_servo(self, cible):
        """Temps (s) pour que le servo passe de son angle actuel à cible."""
        v = self._robot.vitesse_servo_deg_s
        return 0.0 if v <= 0.0 else abs(cible - self.servo_estime) / v

    def integrer_global(self, v_mm_s, dt):
        """À appeler à chaque pas, dans tous les états : position globale."""
        delta = gd.deg_to_rad(self._robot.servo_vers_roues_deg(self.servo_estime - 90.0))
        self.cap_global += v_mm_s / self._robot.empattement_mm * gd.tan(delta) * dt
        self.x_global += v_mm_s * gd.sin(self.cap_global) * dt
        self.y_global += v_mm_s * gd.cos(self.cap_global) * dt

    def point_devant_global(self, distance_mm):
        """Point situé à distance_mm devant l'essieu arrière, dans le repère global (Vec2 : float32 comme Godot)."""
        return Vec2(self.x_global + distance_mm * gd.sin(self.cap_global),
                    self.y_global + distance_mm * gd.cos(self.cap_global))

    def integrer(self, v_mm_s, dt):
        delta = gd.deg_to_rad(self._robot.servo_vers_roues_deg(self.servo_estime - 90.0))
        self.cap += v_mm_s / self._robot.empattement_mm * gd.tan(delta) * dt
        self.x += v_mm_s * gd.sin(self.cap) * dt
        self.y += v_mm_s * gd.cos(self.cap) * dt
        self.parcours -= v_mm_s * dt          # > 0 en marche arrière
