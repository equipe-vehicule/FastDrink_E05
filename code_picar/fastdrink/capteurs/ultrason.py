"""Capteur de distance ultrason du PiCar-S.

Principe : une impulsion de 10 µs sur TRIG déclenche une salve d'ultrasons ; la broche ECHO
reste à 1 pendant le temps aller-retour du son. Distance = durée × vitesse du son / 2.

Exemple :
    from fastdrink.capteurs import Ultrason
    with Ultrason() as capteur:
        d = capteur.mesurer_mm()      # une mesure brute en mm, ou MESURE_RATEE (-1)

La classe garde aussi les noms du code de départ (distance, get_distance, less_than), en cm,
pour rester compatible avec les exemples SunFounder.
"""
import time
from typing import List

from fastdrink import config

MESURE_RATEE = -1.0  # valeur renvoyée quand aucun écho valide n'est reçu


class Ultrason:
    def __init__(self, trig=config.ULTRASON_TRIG, echo=config.ULTRASON_ECHO,
                 timeout_s=config.ULTRASON_TIMEOUT_S,
                 periode_min_s=config.ULTRASON_PERIODE_MIN_S,
                 vitesse_son=config.VITESSE_SON_M_S, gpio=None):
        """gpio : module GPIO à utiliser ; None = RPi.GPIO (sur le robot). Un faux module
        peut être passé pour tester le code sur un ordinateur."""
        if gpio is None:
            import RPi.GPIO as gpio
        self._gpio = gpio
        self.trig = trig
        self.echo = echo
        self.timeout_s = timeout_s
        self.periode_min_s = periode_min_s
        self.vitesse_son = vitesse_son
        self._t_derniere = None  # instant de la dernière impulsion

        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(trig, gpio.OUT, initial=gpio.LOW)  # broches configurées une seule fois
        gpio.setup(echo, gpio.IN)

    # ------------------------------------------------------------------ mesures (mm)
    def mesurer_mm(self) -> float:
        """Une mesure brute en mm, avec décimales, ou MESURE_RATEE si l'écho manque."""
        self._attendre_periode()
        g = self._gpio
        g.output(self.trig, g.HIGH)
        time.sleep(0.00001)  # impulsion de 10 µs
        g.output(self.trig, g.LOW)
        self._t_derniere = time.perf_counter()

        # 1) attendre que l'écho commence
        t0 = time.perf_counter()
        while g.input(self.echo) == 0:
            if time.perf_counter() - t0 > self.timeout_s:
                return MESURE_RATEE
        debut = time.perf_counter()
        # 2) mesurer combien de temps il dure
        while g.input(self.echo) == 1:
            if time.perf_counter() - debut > self.timeout_s:
                return MESURE_RATEE
        duree = time.perf_counter() - debut
        return duree * self.vitesse_son / 2.0 * 1000.0

    def mesurer_plusieurs_mm(self, n: int) -> List[float]:
        """n mesures brutes, MESURE_RATEE comprises (utile pour enregistrer et filtrer)."""
        return [self.mesurer_mm() for _ in range(n)]

    def moyenne_mm(self, n: int = 5) -> float:
        """Moyenne de n mesures SANS les mesures ratées ; MESURE_RATEE si aucune n'est valide."""
        valides = [d for d in self.mesurer_plusieurs_mm(n) if d != MESURE_RATEE]
        return sum(valides) / len(valides) if valides else MESURE_RATEE

    def _attendre_periode(self):
        """Respecte l'intervalle minimal entre deux mesures, pour ne pas capter un vieil écho."""
        if self._t_derniere is None:
            return
        reste = self.periode_min_s - (time.perf_counter() - self._t_derniere)
        if reste > 0:
            time.sleep(reste)

    # ------------------------------------------------- interface du code de départ (cm)
    def distance(self) -> float:
        """Une mesure en cm, ou -1 (même rôle que distance() du code de départ)."""
        d = self.mesurer_mm()
        return MESURE_RATEE if d == MESURE_RATEE else d / 10.0

    def get_distance(self, mount: int = 5) -> float:
        """Moyenne de `mount` mesures en cm, sans les mesures ratées, ou -1."""
        d = self.moyenne_mm(mount)
        return MESURE_RATEE if d == MESURE_RATEE else d / 10.0

    def less_than(self, alarm_gate: float) -> int:
        """1 si la distance (cm) est <= alarm_gate, 0 si elle est plus grande, -1 si erreur."""
        d = self.get_distance()
        if d == MESURE_RATEE:
            return -1
        return 1 if d <= alarm_gate else 0

    # ------------------------------------------------------------------ fermeture
    def fermer(self):
        """Libère les deux broches."""
        self._gpio.cleanup([self.trig, self.echo])

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.fermer()
