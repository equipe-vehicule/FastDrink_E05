"""CONTRÔLEUR DU VÉHICULE : à chaque pas, choisit quel algorithme commande la voiture (machine à états).
Entrées  : sondes (5 valeurs 0/1, de gauche à droite), distance_cm (ultrason, -1 si rien), dt (s)
Sorties  : (vitesse -100..100, angle 45..135)  [le .gd rend {"vitesse": ..., "angle": ...}]
Aucune dépendance à Godot : la même logique que simulation/controleur.gd (portage, lot C).

Exigences du client et modules qui y répondent :
  - suivre la ligne-guide (ruban de 18 mm, courbes de 120 mm et plus)  -> SuiviLigne, Rattrapage
  - s'arrêter à une distance prédéterminée d'un obstacle (+/- 30 mm)   -> ArretObstacle
  - contourner des obstacles en quittant la ligne                      -> ContournementFluide, Contournement
  - suivre la ligne en reculant sur moins de 200 mm                    -> ReculLigne
Devant chaque obstacle (précision du professeur) : arrêt, recul en suivant la ligne, puis contournement.
Outils : FiltreUltrason, Odometrie, MemoireParcours, RepriseDouce.
Réglages : ReglagesRobot (dimensions du robot), ReglagesEpreuve (distances d'arrêt et de recul, obstacle).

États :
  SUIVI                : SuiviLigne commande. Obstacle sur le chemin : freinage et arrêt à
                         ReglagesEpreuve.distance_arret_mm, puis DECISION.
  DECISION             : immobile ReglagesEpreuve.pause_avant_recul_s, puis RECUL_OBSTACLE (cas normal),
                         évitement hors ligne (voiture pas sur la ligne) ou demi-tour (obstacle déjà contourné, vu
                         de dos : la voiture roule à l'envers).
  RECUL_OBSTACLE       : recul de ReglagesEpreuve.distance_recul_mm en suivant la ligne (ReculLigne), pause, puis
                         contournement depuis l'arrêt : fluide si la place mesurée le permet, sinon par phases.
  CONTOURNEMENT        : Contournement (par phases) commande -> SUIVI ou RETROUVER.
  CONTOURNEMENT_FLUIDE : ContournementFluide commande (trajectoire lisse depuis l'arrêt) -> SUIVI.
  RATTRAPAGE           : ligne perdue dans un virage serré : marche arrière (Rattrapage) -> SUIVI.
  RETROUVER            : ligne perdue depuis longtemps : va où elle doit être, puis spirale (RetrouverLigne).
  DEMI_TOUR            : pivot d'environ 180 deg (DemiTour) -> SUIVI ou RETROUVER.
  RECUL_LIGNE / ARRET  : recul en suivant la ligne (ReculLigne), puis arrêt.
  BLOQUE               : arrêt de sécurité ; repart si la ligne est revue.
  ARRIVEE              : barre d'arrivée franchie (5 sondes en travers) : arrêt définitif.
Après toute manœuvre, RepriseDouce limite vitesse et braquage pendant 1 s (sinon la bille tombe).

Sens de marche : après un contournement près d'une épingle, la ligne retrouvée peut partir dans l'autre sens et le
PID la suivre à l'envers (journal Godot du 6 novembre : 8 reprises sur 20). MemoireParcours garde le chemin parcouru
et les obstacles contournés : remonter le chemin sur contresens_max_mm, ou retrouver devant soi un obstacle déjà
contourné, déclenche un demi-tour.
Détection en virage : l'ultrason vise tout droit, hors de la piste ; un écho au-delà de la distance où la ligne sort
du cône vient d'un objet qui n'est pas sur le chemin : il est ignoré (_portee_utile_mm).
"""
import enum

from . import constantes
from . import godot as gd
from .arret_obstacle import ArretObstacle
from .contournement import Contournement
from .contournement_fluide import ContournementFluide
from .demi_tour import DemiTour
from .filtre_ultrason import FiltreUltrason
from .godot import Vec2
from .memoire_parcours import MemoireParcours
from .odometrie import Odometrie
from .rattrapage import Rattrapage
from .recul_ligne import ReculLigne
from .reglages import ReglagesEpreuve, ReglagesRobot
from .reprise_douce import RepriseDouce
from .retrouver_ligne import RetrouverLigne
from .suivi_ligne import SuiviLigne

Etat = enum.IntEnum("Etat", "SUIVI DECISION BLOQUE ARRET CONTOURNEMENT RECUL_LIGNE DEMI_TOUR ARRIVEE RETROUVER "
                            "RATTRAPAGE CONTOURNEMENT_FLUIDE RECUL_OBSTACLE", start=0)


class Controleur:
    Etat = Etat

    ## États après lesquels le suivi reprend en douceur (RepriseDouce).
    ETATS_MANOEUVRE = [Etat.CONTOURNEMENT, Etat.CONTOURNEMENT_FLUIDE, Etat.RATTRAPAGE, Etat.DEMI_TOUR, Etat.RETROUVER]
    TIMEOUT_ETAT_S = 10.0         # sécurité : une phase de manœuvre ne dure jamais plus longtemps
    SEUIL_MARCHE_MM_S = 4.0       # roule au-delà de cette vitesse (= 1 commande à 4 mm/s par commande)
    SEUIL_ARRET_MM_S = 2.0        # arrêtée sous cette vitesse (= 0,5 commande à 4 mm/s par commande)
    LISSAGE_BRAQUAGE_S = 0.5      # constante de temps du braquage moyen (détection en virage, choix du côté)

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « controleur »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "contournement_fluide": bool,   # false : toujours le contournement par phases (après l'arrêt et le recul)
        "recul_fin_ligne_mm": float,    # piste Recul : > 0 = à la fin de la ligne, s'arrêter puis reculer de ça
        "hors_ligne_apres_s": float,    # ligne perdue depuis ça : la voiture n'est plus sur la ligne
        "portee_hors_ligne_mm": float,  # hors ligne, tout écho plus proche que ça est sur son chemin
        "echos_hors_ligne": int,        # pas consécutifs avec écho brut avant d'interrompre une orbite ou une recherche
        "contresens_max_mm": float,     # distance parcourue à contresens sur la ligne avant le demi-tour
        # HYPOTHESE (14 nov., Mehdi) : doit depasser absence_proche_s (1,0 s : le filtre garde un obstacle a moins de
        # 60 mm jusqu'a 1 s sans echo, un delai plus court couperait une manoeuvre normale sur un echo perdu) ; 2 s de
        # plus pour un obstacle passager (main), sans mesure ; a verifier sur le robot. Raison : sur le robot, un faux
        # echo persistant sous 40 mm gelait la manoeuvre indefiniment (relecture du portage, 14 nov.).
        "arret_urgence_max_s": float,   # arrêt d'urgence continu (écho sous distance_securite_mm) au-delà de ça : la manœuvre se termine
    }

    def __init__(self):
        # ----- Réglages propres au contrôleur
        self.contournement_fluide = False
        self.recul_fin_ligne_mm = 0.0
        self.hors_ligne_apres_s = 0.0
        self.portee_hors_ligne_mm = 0.0
        self.echos_hors_ligne = 0
        self.contresens_max_mm = 0.0
        self.arret_urgence_max_s = 0.0

        # ----- Modules (leurs réglages s'écrivent par ex. controleur.suivi.kp = 2.0,
        # controleur.epreuve.distance_recul_mm = 150)
        self.robot = ReglagesRobot()
        self.epreuve = ReglagesEpreuve()
        self.odo = Odometrie(self.robot)
        self.suivi = SuiviLigne()
        self.ultrason = FiltreUltrason(self.robot)
        self.arret = ArretObstacle(self.robot)
        self.contournement = Contournement(self.robot, self.odo, self.epreuve)
        self.fluide = ContournementFluide(self.robot, self.odo, self.epreuve)
        self.recul = ReculLigne(self.robot, self.odo, self.suivi)
        self.rattrapage = Rattrapage(self.odo, self.suivi)
        self.retrouver = RetrouverLigne(self.robot, self.odo)
        self.demi_tour = DemiTour()
        self.reprise = RepriseDouce()
        self.memoire = MemoireParcours(self.robot, self.odo)

        # ----- Journal et diagnostic
        self.distance_filtree_mm = -1.0      # dernière distance ultrason après filtrage
        ## Banc d'essai : pourquoi le contournement fluide n'a pas été utilisé (nombre de fois par raison).
        ## N'influence pas la conduite.
        self.raisons_repli = {}

        self._etat = Etat.SUIVI
        self._t_etat = 0.0
        self._t_urgence = 0.0                # durée continue de l'arrêt d'urgence en cours (s)
        self._t_arret = 0.0
        self._v_mm_s = 0.0                   # vitesse de consigne actuelle (mm/s, après rampe) ; la commande en sort par la table
        self._braquage_moyen = 0.0           # servo - 90, moyenne glissante pendant le suivi (> 0 = virage à droite)
        self._evitement_hors_ligne = False   # la prochaine DECISION lance un évitement hors ligne
        self._n_echos_hors_ligne = 0
        self._contresens_mm = 0.0
        self._recul_fini = False             # RECUL_OBSTACLE : recul terminé, pause avant le contournement
        self._t_pause = 0.0
        self._avance_mm = 0.0                # avance cumulée selon l'odométrie (mm), pour FiltreUltrason.projection_odometrie

    # =================================================================================================================
    # Pas de contrôle

    def calculer(self, sondes, distance_cm, dt, mesure_nouvelle=True):
        """mesure_nouvelle : vrai si distance_cm est une mesure arrivée depuis le tour précédent (le filtre
        « mediane_kalman » n'est mis à jour qu'à ce moment-là ; « mediane3 » l'ignore).
        Retourne (vitesse, angle) : entiers envoyés au robot (le .gd rend {"vitesse", "angle"})."""
        dt = gd.maxf(dt, 0.001)
        # Étape 10 : on suppose l'étalonnage de l'ultrason connu ; le robot réel utilisera la droite du 19 nov.
        distance_cm = self.robot.corriger_ultrason_cm(distance_cm)
        ligne = SuiviLigne.lire(sondes)
        somme = ligne.x
        n = int(ligne.y)
        self.ultrason.avance_mm = self._avance_mm
        d_mm = self.ultrason.filtrer(distance_cm, dt, mesure_nouvelle)
        self.distance_filtree_mm = d_mm
        self._verifier_duree_manoeuvre()
        r = self._commande_selon_etat(sondes, somme, n, d_mm, distance_cm, dt)
        return self._appliquer(r.x, r.y, somme, n, d_mm, dt)

    def _verifier_duree_manoeuvre(self):
        """Sécurité : une phase de manœuvre trop longue (odométrie perdue...) -> recherche de la ligne ou arrêt."""
        if self._etat == Etat.CONTOURNEMENT and self.contournement.phase in [constantes.Phase.ORBITE,
                                                                              constantes.Phase.SORTIE_ORBITE]:
            # un tour d'orbite dure plus longtemps qu'une phase ordinaire
            self._t_etat = gd.minf(self._t_etat, self.TIMEOUT_ETAT_S / 2.0)
        if self._en_manoeuvre() and (self._t_etat > self.TIMEOUT_ETAT_S or self._t_urgence > self.arret_urgence_max_s):
            self._terminer_manoeuvre_trop_longue()

    def _terminer_manoeuvre_trop_longue(self):
        """Fin d'une manœuvre trop longue (timeout ou arrêt d'urgence trop long) : recherche de la ligne si on l'a en
        mémoire après un contournement, sinon arrêt de sécurité."""
        if self._etat == Etat.CONTOURNEMENT and self.memoire.ligne:
            self._lancer_retrouver()
        else:
            self._changer_etat(Etat.BLOQUE)

    def _commande_selon_etat(self, sondes, somme, n, d_mm, distance_cm, dt):
        """Retourne Vec2(angle servo, vitesse cible) demandés par l'algorithme de l'état courant."""
        e = self._etat
        if e == Etat.SUIVI:
            return self._etat_suivi(sondes, d_mm, dt)
        elif e == Etat.DECISION:
            self._etat_decision(somme, n, d_mm)
        elif e == Etat.CONTOURNEMENT:
            return self._etat_contournement(somme, n, d_mm, distance_cm, dt)
        elif e == Etat.CONTOURNEMENT_FLUIDE:
            r = self.fluide.calculer(somme, n)
            if self.fluide.termine:
                self.suivi.derniere_erreur = self.fluide.e_reprise
                self._changer_etat(Etat.SUIVI)
            return r
        elif e == Etat.RATTRAPAGE:
            r = self.rattrapage.calculer(sondes, somme, n, self._v_mm_s, self._t_etat, dt)
            if self.rattrapage.termine:
                self._changer_etat(Etat.SUIVI)
            return r
        elif e == Etat.RETROUVER:
            return self._etat_retrouver(somme, n, d_mm, dt)
        elif e == Etat.DEMI_TOUR:
            return self._etat_demi_tour(n)
        elif e == Etat.RECUL_OBSTACLE:
            return self._etat_recul_obstacle(somme, n, d_mm, dt)
        elif e == Etat.RECUL_LIGNE:
            r = self.recul.calculer(somme, n, dt, self._v_mm_s)
            if self.recul.fin == constantes.Fin.TERMINE:
                self._changer_etat(Etat.SUIVI if self.recul.reprendre_apres_recul else Etat.ARRET)
            elif self.recul.fin == constantes.Fin.LIGNE_PERDUE:
                self._changer_etat(Etat.ARRET)
            return r
        elif e == Etat.BLOQUE:
            if n > 0:
                self._changer_etat(Etat.SUIVI)     # la voiture a été reposée sur la ligne
        # ARRIVEE, ARRET : immobile
        return Vec2(90.0, 0.0)

    def _appliquer(self, angle, cible, somme, n, d_mm, dt):
        """Limites communes à tous les états (arrêt d'urgence, bornes, reprise douce, rampe), puis mise à jour de
        l'odométrie, du sens de marche, de la mémoire de la ligne et du braquage moyen."""
        # Arrêt d'urgence pendant une manœuvre (obstacle imprévu tout près) : on attend, sans faire courir le timeout
        if self._en_manoeuvre() and cible > 0.0 and d_mm >= 0.0 and d_mm < self.contournement.distance_securite_mm:
            cible = 0.0
            self._t_urgence += dt            # compte la durée de l'arrêt ; _verifier_duree_manoeuvre() y met fin au tour suivant
        else:
            self._t_etat += dt
            self._t_urgence = 0.0
        angle = gd.clampf(angle, ReglagesRobot.ANGLE_MIN, ReglagesRobot.ANGLE_MAX)
        if self.reprise.en_cours(self._etat == Etat.SUIVI, dt):
            cible = self.reprise.limiter_vitesse(cible)
            angle = self.reprise.limiter_angle(angle, dt)
        self.reprise.memoriser_angle(angle)
        acceleration = self.robot.deceleration_freinage_mm_s2
        # Changement de sens (cible * _v_mm_s < 0) : décélération de freinage, choix volontaire
        if (self.robot.acceleration_montee_mm_s2 > 0.0 and gd.absf(cible) > gd.absf(self._v_mm_s)
                and cible * self._v_mm_s >= 0.0):
            acceleration = self.robot.acceleration_montee_mm_s2   # accélération (en avant ou en arrière)
        self._v_mm_s = gd.move_toward(self._v_mm_s, cible, acceleration * dt)
        # commande entière réellement envoyée (zone morte comprise) ; même valeur pour l'odométrie
        cmd = self._commande_envoyee()
        angle_servo = int(gd.roundf(angle))
        self.odo.suivre_servo(float(angle_servo), dt)              # toujours : angle réel du servo connu au début d'une manœuvre
        self.odo.integrer_global(self.robot.vitesse_odometrie_mm_s(cmd), dt)    # toujours : position globale (sens de marche)
        self._avance_mm += self.robot.vitesse_odometrie_mm_s(cmd) * dt
        a_contresens = self._surveiller_sens(somme, n, dt)
        if n > 0 and not a_contresens and (self._etat == Etat.SUIVI or self._ligne_de_sortie_visible()):
            self.memoire.memoriser(somme / n)
        if self._en_manoeuvre():
            self.odo.integrer(self.robot.vitesse_odometrie_mm_s(cmd), dt)
        if self._etat == Etat.SUIVI and gd.absf(self._v_mm_s) > self.SEUIL_MARCHE_MM_S:
            self._braquage_moyen = gd.lerpf(self._braquage_moyen, angle_servo - 90.0,
                                            gd.minf(1.0, dt / self.LISSAGE_BRAQUAGE_S))
        return (int(gd.roundf(cmd)), angle_servo)

    def _commande_envoyee(self):
        """Commande que le véhicule reçoit (entière : il fait int(round(cmd))). Modèle réel : arrondie ici, avant
        l'odométrie. Modèle idéal (table linéaire) : laissée flottante pour l'odométrie, afin de garder les résultats
        de référence identiques."""
        c = self.robot.commande_pour_vitesse(self._v_mm_s)
        return c if self.robot.est_lineaire() else gd.roundf(c)

    def _vitesse_odometrie_mm_s(self):
        """Vitesse (mm/s) que l'odométrie croit, pour la commande qui part vers le robot (table du moteur, zone
        morte comprise)."""
        return self.robot.vitesse_odometrie_mm_s(self._commande_envoyee())

    def _surveiller_sens(self, somme, n, dt):
        """En suivi : vrai si la voiture remonte un morceau de ligne déjà parcouru. Au-delà de contresens_max_mm :
        demi-tour, quelle que soit la raison pour laquelle la voiture s'est retournée."""
        if not (n > 0 and n <= 3 and self._etat == Etat.SUIVI and self._v_mm_s > self.SEUIL_MARCHE_MM_S):
            return False
        a_contresens = self.memoire.remonte_le_chemin(somme / n)
        pas_mm = self._vitesse_odometrie_mm_s() * dt
        self._contresens_mm = self._contresens_mm + pas_mm if a_contresens else gd.maxf(0.0, self._contresens_mm - pas_mm)
        if self._contresens_mm >= self.contresens_max_mm and self.memoire.demi_tour_autorise():
            self._contresens_mm = 0.0
            self.memoire.annuler_passage()
            self._demarrer_demi_tour()
        return a_contresens

    # =================================================================================================================
    # États

    def _etat_suivi(self, sondes, d_mm, dt):
        """Suivi de ligne, avec arrêt (ou contournement) devant un obstacle situé sur le chemin."""
        r = self.suivi.calculer(sondes, dt)
        if (self.suivi.ligne == constantes.Ligne.PERDUE and self.suivi.temps_sans_ligne() > self.retrouver.apres_s
                and self.memoire.ligne):
            self._lancer_retrouver()
            return Vec2(90.0, 0.0)
        if self.suivi.arrivee:
            self._changer_etat(Etat.ARRIVEE)
            return Vec2(90.0, 0.0)
        self.rattrapage.observer(dt)
        # Piste Recul : fin de la ligne -> arrêt, puis recul en suivant la ligne (les sondes la retrouvent en reculant)
        if self.recul_fin_ligne_mm > 0.0 and self.rattrapage.ligne_perdue():
            if gd.absf(self._v_mm_s) < self.SEUIL_ARRET_MM_S:
                self.demarrer_recul(self.recul_fin_ligne_mm, 3.0)
                self.recul_fin_ligne_mm = 0.0
            return Vec2(90.0, 0.0)
        reaction = self.rattrapage.reaction()
        if reaction == constantes.Reaction.RATTRAPER:
            self.rattrapage.demarrer(1 if self.suivi.derniere_erreur >= 0.0 else -1)
            self._changer_etat(Etat.RATTRAPAGE)
            return Vec2(90.0, 0.0)
        elif reaction == constantes.Reaction.ARRETER:
            self._changer_etat(Etat.BLOQUE)        # fin de ligne : repart seule si la voiture est reposée sur la ligne
            return Vec2(90.0, 0.0)
        return self._approche_obstacle(r.x, r.y, d_mm, dt)

    def _approche_obstacle(self, angle, cible, d_mm, dt):
        """Suite du suivi : obstacle sur le chemin -> freinage jusqu'à l'arrêt à ReglagesEpreuve.distance_arret_mm,
        puis DECISION (sauf piste « Arrêt devant obstacle », où le contournement est désactivé : la voiture reste
        arrêtée)."""
        contourner = self.contournement.actif
        # Ligne perdue : la voiture braque à fond et la portée utile tomberait à ~14 cm, sous la distance d'arrêt :
        # elle ne verrait plus rien. Sans ligne à protéger, tout écho proche est sur son chemin.
        hors_ligne = (self.suivi.ligne == constantes.Ligne.PERDUE
                      and self.suivi.temps_sans_ligne() >= self.hors_ligne_apres_s)
        portee = self.portee_hors_ligne_mm if hors_ligne else self._portee_utile_mm()
        d_chemin = d_mm if d_mm >= 0.0 and d_mm <= portee else -1.0
        if d_chemin >= 0.0:
            cible = self.arret.limiter(d_chemin, cible, self.epreuve.distance_arret_mm)
        else:
            self.arret.relacher()
        if self.arret.arrete or self.arret.en_recul:
            self.suivi.geler()                     # pas d'accumulation pendant l'arrêt
        if cible < 0.0:
            angle = 90.0                           # recul en ligne droite (arrêt trop près)
        immobile = self.arret.arrete and gd.absf(self._v_mm_s) < self.SEUIL_ARRET_MM_S
        self._t_arret = self._t_arret + dt if immobile else 0.0
        if contourner and immobile:
            self._evitement_hors_ligne = hors_ligne
            self._changer_etat(Etat.DECISION)
        return Vec2(angle, cible)

    def _etat_decision(self, somme, n, d_mm):
        """Immobile devant l'obstacle : après la pause, choisit la manœuvre."""
        if d_mm < 0.0:
            self._evitement_hors_ligne = False
            self._changer_etat(Etat.SUIVI)         # l'obstacle a disparu
        elif self._t_etat >= self.epreuve.pause_avant_recul_s:
            if self._evitement_hors_ligne:
                self._noter_repli("hors_ligne")
                self._demarrer_evitement_hors_ligne(d_mm)   # pas sur la ligne : impossible de reculer en la suivant
            elif (self.memoire.obstacle_deja_contourne(d_mm) and not self.memoire.en_avant_sur_chemin(somme, n)
                    and self.memoire.demi_tour_autorise()):
                self._demarrer_demi_tour()         # on roule à l'envers : demi-tour plutôt que recontourner
            else:
                self._demarrer_recul_obstacle()

    def _etat_recul_obstacle(self, somme, n, d_mm, dt):
        """Recul de distance_recul_mm en suivant la ligne, puis pause, puis contournement depuis l'arrêt.
        Ligne perdue pendant le recul : ReculLigne finit la distance tout droit (voir ReculLigne.finir_sans_ligne)."""
        if not self._recul_fini:
            r = self.recul.calculer(somme, n, dt, self._v_mm_s)
            if self.recul.fin == constantes.Fin.EN_COURS:
                return r
            self._recul_fini = True
            self._t_pause = 0.0
        if gd.absf(self._v_mm_s) < self.SEUIL_ARRET_MM_S:
            self._t_pause += dt
        if self._t_pause >= self.epreuve.pause_avant_contournement_s:
            d_devant = d_mm if d_mm >= 0.0 else (self.epreuve.distance_arret_mm + self.robot.ultrason_avant_mm
                                                  + self.odo.parcours)
            self._demarrer_contournement(d_devant)
        return Vec2(90.0, 0.0)

    def _etat_contournement(self, somme, n, d_mm, distance_cm, dt):
        phase_avant = self.contournement.phase
        d_brut = distance_cm * 10.0 if distance_cm >= 0.0 else -1.0   # non filtré : pour confirmer un écho
        r = self.contournement.calculer(somme, n, self._vitesse_odometrie_mm_s(), d_mm, d_brut, dt)
        if self.contournement.termine:
            self.suivi.derniere_erreur = self.contournement.ecart_ligne_mm()
            if self.contournement.hors_ligne and n == 0 and self.memoire.ligne:
                self._lancer_retrouver()
            else:
                self._changer_etat(Etat.SUIVI)
        elif self.contournement.echec:
            if not self.memoire.ligne:
                self._changer_etat(Etat.BLOQUE)
            else:
                self._lancer_retrouver()           # manœuvre impossible : on continue de chercher la ligne
        elif self.contournement.phase != phase_avant:
            self._t_etat = 0.0                     # le timeout compte pour chaque phase
        elif self._obstacle_pendant_recherche(n, d_mm, d_brut):
            self._evitement_hors_ligne = True
            self._changer_etat(Etat.DECISION)
            return Vec2(90.0, 0.0)
        return r

    def _etat_retrouver(self, somme, n, d_mm, dt):
        if n > 0:
            e = somme / n
            self.suivi.derniere_erreur = e if e != 0.0 else 1.0
            if self.memoire.sur_ligne_a_contresens(e) and self.memoire.demi_tour_autorise():
                self._demarrer_demi_tour()
            else:
                self._changer_etat(Etat.SUIVI)
            return Vec2(90.0, 0.0)
        if self._t_etat > self.retrouver.max_s:
            self._changer_etat(Etat.BLOQUE)
            return Vec2(90.0, 0.0)
        if d_mm >= 0.0 and d_mm <= self._distance_evitement_mm():
            self._evitement_hors_ligne = True
            self._changer_etat(Etat.DECISION)
            return Vec2(90.0, 0.0)
        return self.retrouver.conduire(self.memoire.obstacle_hors_ligne, self._t_etat, dt)

    def _etat_demi_tour(self, n):
        r = self.demi_tour.calculer(n, self.odo.cap_global, self._t_etat)
        if self.demi_tour.termine:
            self.suivi.derniere_erreur = self.demi_tour.ecart_reprise_mm()
            if n == 0 and self.memoire.ligne:
                self._lancer_retrouver()           # demi-tour fini sans ligne sous les sondes : on la cherche
            else:
                self._changer_etat(Etat.SUIVI)
        return r

    # =================================================================================================================
    # Début des manœuvres

    def _demarrer_recul_obstacle(self):
        """Le recul garde la ligne sous les sondes. Il ne cherche pas à réaligner la voiture : arrêtée de travers en
        sortie de virage (17 deg, graine 7920), elle devrait déplacer son essieu de ~60 mm sur le côté, impossible en
        150 mm sans perdre la ligne (essai : recul interrompu à 59 mm). Le contournement part du cap réel par rapport
        à la ligne."""
        self.recul.demarrer(self.epreuve.distance_recul_mm)
        self._recul_fini = False
        self._changer_etat(Etat.RECUL_OBSTACLE)

    def _demarrer_contournement(self, d_mm):
        """Contournement depuis l'arrêt, d_mm devant l'ultrason : fluide si la trajectoire prévue passe sans toucher
        l'obstacle (ContournementFluide.choisir), sinon par phases."""
        pose = self._pose_par_rapport_a_la_ligne()
        if self.contournement_fluide and self.fluide.choisir(d_mm, self._cote_contournement(), pose.x, pose.y):
            self._demarrer_fluide(d_mm)
            return
        if self.contournement_fluide:
            self._noter_repli("place_insuffisante")
        self.memoire.noter_obstacle(self.odo.point_devant_global(
            self.robot.longueur_avant_mm + d_mm + self.epreuve.profondeur_a_contourner_mm() / 2.0))
        self.contournement.demarrer(d_mm, self._braquage_moyen)
        if not self.contournement.choisir_cap(d_mm, self.fluide.marge_obstacle_mm):
            # Aucune trajectoire ne passe (#72, choix de Mehdi du 13 nov.) : arrêt définitif plutôt que contact.
            # ARRET et non BLOQUE : BLOQUE repart dès que la ligne est vue, et elle est sous la voiture.
            self._noter_repli("phases_sans_trajectoire")
            self._changer_etat(Etat.ARRET)
            return
        self.contournement.chemin_entrant = self.memoire.vers_repere_manoeuvre()
        self._changer_etat(Etat.CONTOURNEMENT)

    def _cote_contournement(self):
        """Côté du contournement fluide : celui par défaut, ou l'intérieur du virage si la voiture braquait (ligne qui
        tourne)."""
        if (self.contournement.cote_interieur_virage
                and gd.absf(self._braquage_moyen) >= self.contournement.seuil_virage_deg):
            return 1 if self._braquage_moyen > 0.0 else -1
        return self.contournement.cote_par_defaut

    def _demarrer_fluide(self, d_mm):
        self.memoire.noter_obstacle(self.odo.point_devant_global(
            self.robot.longueur_avant_mm + d_mm + self.epreuve.profondeur_a_contourner_mm() / 2.0))
        self.fluide.demarrer()
        self._changer_etat(Etat.CONTOURNEMENT_FLUIDE)

    def _demarrer_evitement_hors_ligne(self, d_mm):
        """Évite un obstacle alors que la voiture n'est pas sur la ligne : du côté où la ligne doit se trouver, sans
        supposer qu'elle passe sous l'obstacle ; ensuite RETROUVER ramène la voiture vers elle."""
        self._evitement_hors_ligne = False
        self._n_echos_hors_ligne = 0
        self.memoire.obstacle_hors_ligne = self.odo.point_devant_global(
            self.robot.longueur_avant_mm + d_mm + self.epreuve.profondeur_a_contourner_mm() / 2.0)
        c = 1 if self.suivi.derniere_erreur >= 0.0 else -1
        if self.memoire.ligne:
            vers = self.retrouver.cible_ligne(self.memoire) - Vec2(self.odo.x_global, self.odo.y_global)
            c = 1 if gd.wrapf(gd.atan2(vers.x, vers.y) - self.odo.cap_global, -gd.PI, gd.PI) >= 0.0 else -1
        self.contournement.demarrer_hors_ligne(d_mm, c)
        self.contournement.chemin_entrant = self.memoire.vers_repere_manoeuvre()
        self._changer_etat(Etat.CONTOURNEMENT)

    def _demarrer_demi_tour(self):
        self.memoire.noter_demi_tour()
        self._changer_etat(Etat.DEMI_TOUR)
        self.demi_tour.demarrer(self.odo.cap_global, 1 if self.suivi.derniere_erreur >= 0.0 else -1)

    def _lancer_retrouver(self):
        self.retrouver.demarrer(self.memoire, self.suivi.derniere_erreur)
        self._changer_etat(Etat.RETROUVER)

    def demarrer_recul(self, distance_mm, attente_ligne_s=0.0):
        """Recule en suivant la ligne sur distance_mm (limité à ReglagesEpreuve.RECUL_MAX_MM).
        La voiture doit être sur la ligne et à peu près dans son axe : ce cap sert de référence (cap = 0)."""
        self.recul.demarrer(distance_mm, attente_ligne_s)
        self._changer_etat(Etat.RECUL_LIGNE)

    def _changer_etat(self, e):
        if e == Etat.SUIVI and self._etat in self.ETATS_MANOEUVRE:
            self.reprise.demarrer()
        self._etat = e
        self._t_etat = 0.0
        self._t_urgence = 0.0
        if e == Etat.SUIVI:
            self.memoire.nouveau_passage()
            self.rattrapage.nouveau_suivi()
            self.suivi.reinitialiser()
            self.arret.relacher()

    # =================================================================================================================
    # Outils

    def _pose_par_rapport_a_la_ligne(self):
        """Position de l'essieu par rapport à la dernière partie droite de la ligne mémorisée : Vec2(écart latéral en
        mm, > 0 = à droite de la ligne ; cap en rad, > 0 = vers la droite). Calculée avec l'odométrie globale, sans
        les sondes : juste même si la ligne n'est plus sous les sondes (voiture arrêtée de travers en sortie de
        virage, graine 71272)."""
        if len(self.memoire.ligne) < 6:
            return Vec2.ZERO
        d = self.memoire.derniere_droite()
        centre = d[0]
        sens = d[1]
        droite = Vec2(sens.y, -sens.x)
        ecart = (Vec2(self.odo.x_global, self.odo.y_global) - centre).dot(droite)
        return Vec2(ecart, gd.wrapf(self.odo.cap_global - gd.atan2(sens.x, sens.y), -gd.PI, gd.PI))

    def place_avant_obstacle_mm(self):
        """Ligne droite occupée devant un obstacle par la séquence arrêt + recul (mm) : de la face de l'obstacle à
        l'essieu arrière à la fin du recul. Le générateur de pistes en laisse au moins autant (précision du
        professeur : il y aura toujours la place pour la manœuvre)."""
        return self.epreuve.distance_arret_mm + self.robot.longueur_avant_mm + self.epreuve.distance_recul_mm

    def _en_manoeuvre(self):
        """Vrai pendant les états où la position est suivie par odométrie."""
        return (self._etat == Etat.CONTOURNEMENT or self._etat == Etat.RECUL_LIGNE or self._etat == Etat.RATTRAPAGE
                or self._etat == Etat.CONTOURNEMENT_FLUIDE or self._etat == Etat.RECUL_OBSTACLE)

    def _ligne_de_sortie_visible(self):
        """Vrai pendant les phases du contournement où les sondes voient la ligne de sortie (pas la ligne entrante)."""
        return self._etat == Etat.CONTOURNEMENT and self.contournement.phase in [
            constantes.Phase.CROISEMENT, constantes.Phase.ALIGNEMENT, constantes.Phase.SORTIE_ORBITE,
            constantes.Phase.TRAVERS]

    def _portee_utile_mm(self):
        """Distance jusqu'à laquelle un écho peut venir d'un obstacle posé sur la ligne.
        En virage de rayon R (déduit du braquage moyen), la ligne s'écarte de l'axe du capteur d'environ y^2 / (2 R)
        à la distance y ; elle reste dans le cône tant que y^2 / (2 R) < y * tan(demi-angle) + demi-largeur
        d'obstacle."""
        delta = gd.deg_to_rad(gd.absf(self.robot.servo_vers_roues_deg(self._braquage_moyen)))
        if delta < gd.deg_to_rad(1.0):
            return gd.INF                          # ligne droite : toute la portée
        rayon = self.robot.empattement_mm / gd.tan(delta)
        a = rayon * gd.tan(gd.deg_to_rad(self.robot.demi_angle_ultrason_deg))
        return a + gd.sqrt(a * a + rayon * self.epreuve.largeur_a_contourner_mm())

    def _distance_evitement_mm(self):
        """Hors de la ligne : distance (mm, mesurée par l'ultrason) à laquelle la voiture s'arrête pour éviter un
        obstacle."""
        return self.contournement.distance_manoeuvre_cm * 10.0 + self.robot.ultrason_avant_mm + 10.0

    def _obstacle_pendant_recherche(self, n, d_mm, d_brut):
        """Pendant une orbite ou une recherche, sans ligne sous les sondes : vrai si un autre obstacle que celui
        contourné est devant."""
        cherche = self.contournement.phase in [constantes.Phase.ORBITE, constantes.Phase.RECHERCHE]
        if (not cherche or n > 0 or d_mm < 0.0 or d_mm > self._distance_evitement_mm() or d_brut < 0.0
                or self.memoire.obstacle_deja_contourne(d_mm)):
            self._n_echos_hors_ligne = 0
            return False
        self._n_echos_hors_ligne += 1
        return self._n_echos_hors_ligne >= self.echos_hors_ligne

    def _noter_repli(self, raison):
        self.raisons_repli[raison] = int(self.raisons_repli.get(raison, 0)) + 1

    # =================================================================================================================
    # Interface (simulation, HUD, journal)

    def nom_etat(self):
        """Nom de l'état courant, à afficher dans le HUD pour vérifier ce que fait la voiture."""
        if self._etat == Etat.CONTOURNEMENT:
            return "CONTOURNEMENT " + self.contournement.nom_phase()
        if self._etat == Etat.SUIVI or self._etat == Etat.DECISION:
            # le braquage moyen décide du côté de contournement en virage (> 0 = droite) : utile pour comprendre le choix
            return "%s (ligne %s, braquage moyen %+.0f)" % (self._etat.name, self.suivi.nom_ligne(), self._braquage_moyen)
        return self._etat.name

    def etat_court(self):
        """Nom court de l'état (journal)."""
        return self._etat.name

    def braquage_moyen(self):
        """Braquage moyen récent pendant le suivi (servo - 90, > 0 = virage à droite)."""
        return self._braquage_moyen

    def reprendre(self):
        """Repart en suivi de ligne (marche avant) après un arrêt en fin de recul."""
        self._changer_etat(Etat.SUIVI)

    def reinitialiser(self):
        """Remet le contrôleur à zéro (nouveau départ, sortie de BLOQUE)."""
        self._v_mm_s = 0.0
        self._braquage_moyen = 0.0
        self.suivi.derniere_erreur = 0.0
        self.ultrason.reinitialiser()
        self._evitement_hors_ligne = False
        self.memoire.obstacle_hors_ligne = Vec2(gd.INF, gd.INF)
        self._changer_etat(Etat.SUIVI)
