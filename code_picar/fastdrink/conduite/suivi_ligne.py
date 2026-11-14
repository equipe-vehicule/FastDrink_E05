"""Suivi de ligne PID à partir des 5 sondes (0/1, de gauche à droite), avec une détection robuste de la ligne.
Portage de simulation/suivi_ligne.gd.

DÉTECTION : avant le PID, on décide si la lecture est fiable.
  VUE           : des sondes voisines voient la ligne ; erreur = milieu des sondes allumées.
  ENTRE_SONDES  : plus aucune sonde, mais la ligne n'était pas au bord. Le ruban (18 mm) a pu passer entre
                  deux sondes (espacées de 19 mm), ou une sonde a « clignoté ». On garde la commande précédente
                  pendant delai_entre_sondes_s au lieu de braquer à fond.
  PERDUE        : plus aucune sonde depuis plus longtemps, ou la ligne était au bord : braquage maximal
                  du dernier côté vu (comme avant).
  4 ou 5 sondes voisines : la voiture coupe la ligne de face (après un contournement, un demi-tour) : ligne VUE,
  au milieu des sondes allumées. (Avant, ce motif était ignoré : la voiture se croyait perdue et continuait
  de chercher alors qu'elle était sur la ligne - journal Godot du 6 novembre.)
  ARRIVÉE : 5 sondes allumées pendant pas_arrivee mesures alors que la voiture suivait la ligne bien centrée depuis
  aligne_avant_arrivee_s : c'est la barre d'arrivée posée en travers de la piste (`arrivee` passe à True).
  Motif incohérent (sondes non voisines : reflet) : lecture ignorée, on garde la commande précédente.
Sans cette détection, une seule lecture « 0 0 0 0 0 » erronée déclenchait un braquage maximal.

COMMANDE : PID (erreur en mm -> angle servo), vitesse réduite quand on est décentré.
Essais du 12 nov. (parcours fourni), abandonnés : loi de braquage par table (roues plus calmes mais 2 rattrapages par
virage serré au lieu de 1) et ralentissement en entrée de virage (aucun rattrapage évité, tour plus lent).
Détail dans SUIVI_SIMULATION.md.
"""
from . import constantes
from . import godot as gd
from .godot import Vec2
from .reglages import ReglagesRobot


class SuiviLigne:
    Ligne = constantes.Ligne          # enum Ligne { VUE, ENTRE_SONDES, PERDUE }

    POSITIONS_SONDES_MM = [-38.0, -19.0, 0.0, 19.0, 38.0]
    ERREUR_MAX_MM = constantes.ERREUR_MAX_MM    # écart maximal mesurable avec la ligne visible
    INTEGRALE_MAX = 60.0              # anti-windup (mm*s)
    FILTRE_DERIVEE = 0.3              # 0..1 : plus petit = dérivée plus lissée

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « suivi »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        # ----- Gains PID (à régler dans cet ordre : kp, puis kd, puis ki)
        "kp": float,                  # degrés par mm d'écart
        "ki": float,                  # degrés par (mm*s). Essai à 0 (12 nov.) : plus de pertes de ligne dans les virages serrés
        "kd": float,                  # degrés par (mm/s)
        # Candidats C2 à C4 (limiteur, zone neutre, gain au centre) essayés et rejetés le 13 nov. : voir
        # LISEZMOI_SIMULATION.md, étape 5.
        # ----- Vitesses (mm/s ; le contrôleur les convertit en commande par la table du moteur)
        "vitesse_max_mm_s": float,    # (= ancienne commande 55 à 4 mm/s par commande) ligne droite, bien centré. Essais du 12 nov.
                                      # (parcours fourni) : commande 40 -> 55,9 s, 55 -> 44,4 s sans chute de bille, 70 -> 37,3 s mais la bille tombe
        "vitesse_virage_mm_s": float,  # (= ancienne commande 15 à 4 mm/s par commande) grand écart ou ligne perdue
        # ----- Détection
        "delai_entre_sondes_s": float,  # durée pendant laquelle une disparition de la ligne est tolérée
        ## Ligne perdue plus longtemps (fin du parcours) : arrêt (0 = ne jamais s'arrêter).
        ## Pas moins de 8 s : en virage serré, les sondes (217 mm devant l'essieu) sortent de la ligne et la voiture
        ## la retrouve en tournant, ce qui peut prendre plusieurs secondes.
        "abandon_s": float,
        # ----- Ligne d'arrivée (barre noire en travers de la piste)
        "aligne_avant_arrivee_s": float,  # la voiture doit suivre la ligne bien centrée depuis au moins ça
        "pas_arrivee": int,           # mesures consécutives avec les 5 sondes allumées
    }

    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.vitesse_max_mm_s = 0.0
        self.vitesse_virage_mm_s = 0.0
        self.delai_entre_sondes_s = 0.0
        self.abandon_s = 0.0
        self.aligne_avant_arrivee_s = 0.0
        self.pas_arrivee = 0
        self.arrivee = False

        ## Dernier écart non nul (mm) : sert à savoir de quel côté chercher la ligne quand on la perd.
        ## Mis à jour aussi par le recul et par la fin du contournement.
        self.derniere_erreur = 0.0
        self.ligne = SuiviLigne.Ligne.PERDUE

        self._erreur = 0.0                # dernière position valide (mm)
        self._t_sans_ligne = 0.0
        self._derniere_sortie = Vec2(90.0, 15.0)
        self._erreur_prec = 0.0
        self._integrale = 0.0
        self._derivee = 0.0
        self._ligne_visible_avant = False
        self._t_aligne = 0.0              # temps depuis lequel la ligne est vue bien centrée
        self._n_cinq = 0                  # mesures consécutives avec les 5 sondes allumées

    @staticmethod
    def lire(sondes):
        """Retourne Vec2(somme des positions des sondes allumées, nombre de sondes allumées)."""
        somme = 0.0
        n = 0
        for i in range(5):
            if sondes[i] == 1:
                somme += SuiviLigne.POSITIONS_SONDES_MM[i]
                n += 1
        return Vec2(somme, n)

    def geler(self):
        """Annule les termes I et D (arrêt devant un obstacle : pas d'accumulation)."""
        self._integrale = 0.0
        self._derivee = 0.0

    def reinitialiser(self):
        """Retour en suivi de ligne : tant qu'aucune sonde ne voit la ligne, on la cherche du côté de derniere_erreur."""
        self.geler()
        self._ligne_visible_avant = False
        self.ligne = SuiviLigne.Ligne.PERDUE
        self._t_sans_ligne = 0.0              # sinon RETROUVER repartait juste après un contournement (journal du 6 novembre)
        self._t_aligne = 0.0
        self._n_cinq = 0

    def position_avant_perte(self):
        """Dernière position de la ligne vue avant de la perdre (mm, > 0 = à droite). Proche de 0 : la ligne a disparu
        sous les sondes centrales, donc devant la voiture (fin de la ligne), et non sur le côté (virage)."""
        return self._erreur

    def temps_sans_ligne(self):
        """Temps (s) depuis lequel aucune sonde ne voit la ligne."""
        return self._t_sans_ligne

    def nom_ligne(self):
        return self.ligne.name

    def calculer(self, sondes, dt):
        """Retourne Vec2(angle servo, vitesse cible)."""
        e = self._position(sondes)
        if gd.is_inf(e):
            return self._derniere_sortie              # motif incohérent : lecture ignorée
        # Barre d'arrivée : 5 sondes alors que la voiture suivait la ligne bien centrée
        if sondes.count(1) == 5 and self._t_aligne >= self.aligne_avant_arrivee_s:
            self._n_cinq += 1
            if self._n_cinq >= self.pas_arrivee:
                self.arrivee = True
            return self._derniere_sortie              # on garde le cap en attendant la confirmation
        self._n_cinq = 0
        if gd.is_nan(e):
            self._t_sans_ligne += dt
            if (self.ligne != SuiviLigne.Ligne.PERDUE and gd.absf(self._erreur) < SuiviLigne.ERREUR_MAX_MM
                    and self._t_sans_ligne <= self.delai_entre_sondes_s):
                self.ligne = SuiviLigne.Ligne.ENTRE_SONDES
                return self._derniere_sortie
            self.ligne = SuiviLigne.Ligne.PERDUE
            if self.abandon_s > 0.0 and self._t_sans_ligne > self.abandon_s:
                self._derniere_sortie = Vec2(90.0, 0.0)   # on ne tourne pas indéfiniment en rond à l'aveugle
                return self._derniere_sortie
        else:
            self._t_sans_ligne = 0.0
            self._erreur = e
            self.ligne = SuiviLigne.Ligne.VUE
        self._derniere_sortie = self._pid(e, dt)
        self._t_aligne = (self._t_aligne + dt
                          if self.ligne == SuiviLigne.Ligne.VUE and gd.absf(self._erreur) <= 19.0 else 0.0)
        return self._derniere_sortie

    def _position(self, sondes):
        """Position de la ligne (mm, > 0 = à droite). NAN = aucune sonde ; INF = motif incohérent (sondes non voisines).
        4 ou 5 sondes voisines (ligne coupée de face) : position = milieu des sondes allumées."""
        premiere = -1
        derniere = -1
        n = 0
        for i in range(5):
            if sondes[i] == 1:
                if premiere < 0:
                    premiere = i
                derniere = i
                n += 1
        if n == 0:
            return gd.NAN
        if derniere - premiere + 1 != n:
            return gd.INF
        return (SuiviLigne.POSITIONS_SONDES_MM[premiere] + SuiviLigne.POSITIONS_SONDES_MM[derniere]) / 2.0

    def _pid(self, erreur, dt):
        """PID. erreur = NAN quand la ligne est perdue. Retourne Vec2(angle, vitesse cible)."""
        angle = 90.0
        cible = self.vitesse_virage_mm_s
        if not gd.is_nan(erreur):
            if erreur != 0.0:
                self.derniere_erreur = erreur

            # Terme I : remis à zéro quand la ligne change de côté (fin de courbe)
            if self._ligne_visible_avant and erreur * self._erreur_prec < 0.0:
                self._integrale = 0.0
            self._integrale = gd.clampf(self._integrale + erreur * dt, -SuiviLigne.INTEGRALE_MAX, SuiviLigne.INTEGRALE_MAX)

            # Terme D (filtre, car les sondes 0/1 donnent des sauts d'erreur)
            if self._ligne_visible_avant:
                brute = gd.div_reel(erreur - self._erreur_prec, dt)
                self._derivee = gd.lerpf(self._derivee, brute, SuiviLigne.FILTRE_DERIVEE)
            else:
                self._derivee = 0.0
            self._erreur_prec = erreur
            self._ligne_visible_avant = True

            angle = 90.0 + self.kp * erreur + self.ki * self._integrale + self.kd * self._derivee

            # Vitesse adaptative : plus on est décentré, plus on ralentit
            ratio = gd.clampf(gd.absf(erreur) / SuiviLigne.ERREUR_MAX_MM, 0.0, 1.0)
            cible = gd.lerpf(self.vitesse_max_mm_s, self.vitesse_virage_mm_s, ratio)
        else:
            # Ligne perdue : braquage maximal du dernier côté vu, vitesse réduite
            self.geler()
            self._ligne_visible_avant = False
            if self.derniere_erreur > 0.0:
                angle = ReglagesRobot.ANGLE_MAX
            elif self.derniere_erreur < 0.0:
                angle = ReglagesRobot.ANGLE_MIN
        return Vec2(angle, cible)
