"""Rattrapage d'un virage serré en marche arrière (méthode de line_follower.py, code de départ SunFounder).
Portage de simulation/rattrapage.gd.
Le rayon de braquage (138 mm à 45 deg) ne permet pas aux sondes, 217 mm devant l'essieu, de rester sur une courbe de
120 à 170 mm : un point de la barre décrit au moins racine(138^2 + 217^2) = 257 mm. Quand la ligne est perdue en
virage, la voiture s'arrête, braque du côté opposé, recule jusqu'à ce que la sonde centrale revoie la ligne, s'arrête
de nouveau, puis reprend le suivi : chaque marche arrière la fait tourner vers la courbe (manœuvre en plusieurs fois).

Ce module surveille aussi la ligne pendant le suivi pour distinguer un virage raté d'une fin de ligne : ligne perdue
alors qu'elle était centrée et suivie sans interruption depuis un moment = fin de ligne -> arrêt (pas de rattrapage).
"""
from . import constantes
from . import godot as gd
from .godot import Vec2
from .suivi_ligne import SuiviLigne


class Rattrapage:
    Reaction = constantes.Reaction    # enum Reaction { AUCUNE, RATTRAPER, ARRETER }

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « rattrapage »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "actif": bool,
        "apres_s": float,             # ligne perdue depuis ça (après l'avoir suivie) : on recule. Plus court, la voiture
                                      # reculait dans les virages de 90 deg qu'elle retrouve seule en 0,35 s
        "pause_s": float,             # arrêt avant et après la marche arrière (bille : pas d'aller-retour brusque)
        "vitesse_mm_s": float,        # (= ancienne commande 15 à 4 mm/s par commande) vitesse de recul, en valeur absolue
        "distance_max_mm": float,     # recul maximal par tentative
        ## Tentatives sans retrouver la ligne durablement. Étape 10 (14 nov., #75, retenu par l'équipe) : avec
        ## garder_cote = true et 4 tentatives (HYPOTHÈSE : plus petite valeur qui passe), la graine 47515 des pistes libres
        ## passe (épingles de rayon 125 à 140 mm, sous le rayon minimal de la voiture, 138 mm) ; garder_cote seul ou 5
        ## tentatives seules : BLOQUE. Change aussi le modèle idéal.
        "tentatives_max": int,
        "ligne_retrouvee_s": float,   # ligne suivie au moins ça : le compteur de tentatives repart à zéro
        "arret_si_introuvable": bool,  # après tentatives_max (ou fin de ligne) : arrêt de sécurité (BLOQUE)
        "fin_ligne_ecart_mm": float,  # ligne perdue alors qu'elle était à moins de ça du centre...
        "fin_ligne_vue_s": float,     # ... et suivie sans interruption au moins ça : fin de ligne
        ## Étape 10 (#75) : tant que la ligne n'a pas été suivie durablement (ligne_retrouvee_s), les tentatives suivantes
        ## reculent du même côté que la première (côté du virage). Pourquoi (graine 47515, journal du 14 nov.) : dans une
        ## épingle plus serrée que la voiture, la 2e marche arrière finit la barre presque parallèle à la ligne (cap -98 deg
        ## par rapport à elle) avec la ligne sous les sondes de gauche (11100) ; le PID braque à gauche, perd la ligne, et la
        ## 3e tentative, prise de ce côté, ramenait la voiture au point de la 1re (boucle, puis BLOQUE). Comportement
        ## d'avant : essai par --reglage=rattrapage:garder_cote=false --reglage=rattrapage:tentatives_max=3.
        "garder_cote": bool,
    }

    def __init__(self, odo, suivi):
        self.actif = False
        self.apres_s = 0.0
        self.pause_s = 0.0
        self.vitesse_mm_s = 0.0
        self.distance_max_mm = 0.0
        self.tentatives_max = 0
        self.ligne_retrouvee_s = 0.0
        self.arret_si_introuvable = False
        self.fin_ligne_ecart_mm = 0.0
        self.fin_ligne_vue_s = 0.0
        self.garder_cote = False

        self.termine = False                 # ligne revue et voiture arrêtée : le suivi peut reprendre

        self._odo = odo
        self._suivi = suivi
        self._tentatives = 0
        self._cote = 1                       # côté où la ligne a été vue en dernier (1 = droite)
        self._recule = False
        self._fini = False                   # ligne revue : arrêt avant de repartir en avant
        self._t_fin = 0.0
        # Surveillance de la ligne pendant le suivi
        self._ligne_suivie = False           # la ligne a été vue depuis le dernier passage en suivi
        self._t_ligne_vue = 0.0
        self._duree_derniere_vue = 0.0       # durée de la dernière période où la ligne a été vue sans interruption

    # ----- Pendant le suivi

    def nouveau_suivi(self):
        """À appeler à chaque reprise du suivi de ligne."""
        self._ligne_suivie = False
        self._t_ligne_vue = 0.0
        self._duree_derniere_vue = 0.0

    def observer(self, dt):
        """À appeler à chaque pas du suivi."""
        if self._suivi.ligne == SuiviLigne.Ligne.VUE:
            self._ligne_suivie = True
            self._t_ligne_vue += dt
            self._duree_derniere_vue = self._t_ligne_vue
            if self._t_ligne_vue >= self.ligne_retrouvee_s:
                self._tentatives = 0
        else:
            self._t_ligne_vue = 0.0

    def ligne_perdue(self):
        """Vrai si la ligne, suivie depuis la reprise du suivi, est maintenant perdue."""
        return self._ligne_suivie and self._suivi.ligne == SuiviLigne.Ligne.PERDUE

    def reaction(self):
        """Que faire de la ligne perdue : rien (pas encore), rattraper en marche arrière, ou s'arrêter (fin de ligne)."""
        if not (self.actif and self.ligne_perdue() and self._suivi.temps_sans_ligne() >= self.apres_s):
            return Rattrapage.Reaction.AUCUNE
        fin_de_ligne = (gd.absf(self._suivi.position_avant_perte()) <= self.fin_ligne_ecart_mm
                        and self._duree_derniere_vue >= self.fin_ligne_vue_s)
        if not fin_de_ligne and self._tentatives < self.tentatives_max:
            return Rattrapage.Reaction.RATTRAPER
        if self.arret_si_introuvable:
            return Rattrapage.Reaction.ARRETER
        return Rattrapage.Reaction.AUCUNE

    # ----- Manœuvre

    def demarrer(self, cote_ligne):
        if self.garder_cote and self._tentatives > 0:
            cote_ligne = self._cote
        self._tentatives += 1
        self._cote = cote_ligne
        self._recule = False
        self._fini = False
        self.termine = False

    def calculer(self, sondes, somme, n, v_mm_s, t_etat, dt):
        """v_mm_s : vitesse de consigne actuelle (mm/s) ; t_etat : temps depuis le début de la manœuvre.
        Retourne Vec2(angle servo, vitesse cible). Regarder `termine` ensuite."""
        if not self._recule:
            # 1) arrêt complet, roues braquées du côté opposé, pendant au moins pause_s
            if gd.absf(v_mm_s) < 2.0 and t_etat >= self.pause_s:
                self._recule = True
                self._odo.reinitialiser()             # odo.parcours = distance reculée
            return Vec2(90.0 - 45.0 * self._cote, 0.0)
        # 3) ligne revue : arrêt complet pendant pause_s, puis reprise du suivi en marche avant
        if self._fini:
            self._t_fin += 0.0 if gd.absf(v_mm_s) >= 2.0 else dt
            if self._t_fin >= self.pause_s:
                self.termine = True
            return Vec2(90.0, 0.0)
        # 2) marche arrière jusqu'à ce que la sonde centrale revoie la ligne (lf.wait_tile_center() du code de départ)
        if n > 0:
            self._suivi.derniere_erreur = somme / n if somme != 0.0 else self._suivi.derniere_erreur
        if sondes[2] == 1 or self._odo.parcours >= self.distance_max_mm:
            self._fini = True
            self._t_fin = 0.0
            return Vec2(90.0, 0.0)
        return Vec2(90.0 - 45.0 * self._cote, -self.vitesse_mm_s)
