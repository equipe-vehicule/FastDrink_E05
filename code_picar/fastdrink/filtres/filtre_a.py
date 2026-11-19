"""Filtre A pour l'ultrason du PiCar : mediane des 3 dernieres mesures, puis lissage exponentiel.

Utilisé par le filtre final (filtre_final.py) quand le robot est immobile.
Meme algorithme que _filtrer_distance() dans godot/controleur.gd, pour comparer simulation et robot.

    m_k = mediane(z_k, z_k-1, z_k-2)
    y_k = a * m_k + (1 - a) * y_k-1,   a = 1 - exp(-T / tau)

tau = 0 donne la mediane seule.

Utilisation sur le Pi :
    from fastdrink.capteurs import Ultrason
    from fastdrink.filtres import FiltreA
    f = FiltreA()
    with Ultrason() as capteur:
        while True:
            d_mm = f.mettre_a_jour(capteur.mesurer_mm(), time.perf_counter())   # -1 = pas d'obstacle
"""
import math
from collections import deque

from fastdrink import config


class FiltreA:
    def __init__(self, tau_s=config.FILTRE_A_TAU_S, taille=3, portee_max_mm=config.DISTANCE_MAX_MM,
                 saut_reinit_mm=100.0,
                 absence_s=0.15, zone_proche_mm=60.0, absence_proche_s=1.0):
        self.tau_s = tau_s
        self.portee_max_mm = portee_max_mm
        self.saut_reinit_mm = saut_reinit_mm
        self.absence_s = absence_s
        self.zone_proche_mm = zone_proche_mm
        self.absence_proche_s = absence_proche_s
        self._fenetre = deque(maxlen=taille)
        self.reinitialiser()

    def reinitialiser(self):
        self._fenetre.clear()
        self._y = -1.0           # sortie de l'exponentiel (-1 = pas initialisee)
        self.sortie = -1.0       # distance filtree (mm), -1 = pas d'obstacle
        self.mediane = -1.0
        self.retard_s = 0.0      # retard du filtre sur une distance qui varie a vitesse constante
        self._t_prec = None
        self._t_absence = 0.0

    def mettre_a_jour(self, z_mm, t_s):
        """z_mm : mesure brute en mm (-1 si pas d'echo). t_s : instant de la mesure (s). Retourne la sortie (mm)."""
        periode = 0.0 if self._t_prec is None else t_s - self._t_prec
        self._t_prec = t_s
        self.mediane = -1.0
        if 0.0 <= z_mm <= self.portee_max_mm:
            self._fenetre.append(z_mm)
            self._t_absence = 0.0
            tri = sorted(self._fenetre)
            # 1 ou 2 mesures : la plus proche (prudent) ; 3 : la mediane
            self.mediane = tri[1] if len(tri) >= 3 else tri[0]
            self.sortie = self._lisser(self.mediane, periode)
        else:
            self._t_absence += periode
            proche = 0.0 <= self.sortie < self.zone_proche_mm
            if self._t_absence >= (self.absence_proche_s if proche else self.absence_s):
                self._fenetre.clear()
                self._y = -1.0
                self.sortie = -1.0
        return self.sortie

    def _lisser(self, m, periode):
        if self.tau_s <= 0.0 or periode <= 0.0:
            self.retard_s = periode
            self._y = m
            return m
        alpha = 1.0 - math.exp(-periode / self.tau_s)
        self.retard_s = periode * (1.0 + (1.0 - alpha) / alpha)
        if self._y < 0.0 or abs(m - self._y) > self.saut_reinit_mm:
            self._y = m                      # premier echo ou nouvel obstacle
        else:
            self._y = alpha * m + (1.0 - alpha) * self._y
        return self._y
