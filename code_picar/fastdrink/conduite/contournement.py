"""Contournement d'un obstacle en quittant la ligne-guide, en boucle fermée sur le cap (odométrie)
et sur l'ultrason : la manœuvre s'adapte à la taille et au décalage de l'obstacle.
(Portage de simulation/contournement.gd.)

Phases :
  ECART      : on prend le cap d'évitement jusqu'à ce que l'ultrason ne voie plus l'obstacle. À cet instant,
               le bord de l'obstacle est forcément en deçà du rayon extrême du cône : on en déduit sa position
               latérale et on s'écarte jusqu'à ce bord + demi-largeur de voiture + marge
               (minimum : obstacle de ReglagesEpreuve centré sur la ligne, à la tolérance de centrage près).
  LONGER     : cap parallèle à la ligne, à ce décalage, jusqu'à avoir dépassé l'obstacle le plus long prévu.
               Si les sondes voient la ligne (près d'un virage, elle revient sous la voiture), le PID reprend la main.
  RETOUR     : cap de retour vers la ligne, jusqu'à ce que les sondes la voient.
  RECHERCHE  : la ligne n'était pas là où prévu (elle a tourné) : virage serré vers elle ; dès qu'elle est vue,
               le PID reprend la main.
  Pendant LONGER et RECHERCHE, la ligne retrouvée a pu tourner : CROISEMENT / ALIGNEMENT, qui supposent une
  ligne parallèle au cap de départ, viseraient à côté. Le PID, lui, suit les sondes quelle que soit sa direction
  (en simulation : +10 points de réussite près et dans les virages, aucun recul en ligne droite).
  CROISEMENT : ligne vue ; tout droit jusqu'au point où le virage d'alignement finira dessus.
               On vérifie d'abord que la ligne est bien parallèle au cap de départ : en avançant, le point où elle
               coupe la barre de sondes glisse lentement du côté du contournement (ligne parallèle, vue sous ~30 deg)
               ou vite de l'autre côté (ligne qui a tourné : virage opposé au contournement). Dans ce cas, ou si
               elle quitte la barre en quelques cm, le PID reprend la main (+12 points près des virages en simulation).
  ALIGNEMENT : virage qui finit sur la ligne, dans l'axe -> terminé.
  ORBITE     : (remplace RECHERCHE) la ligne n'était pas là où prévu. L'obstacle est posé SUR la ligne : la ligne
               part de l'obstacle et s'en éloigne, donc tout cercle centré sur l'obstacle la coupe, quel que soit
               le sens du virage. La voiture tourne autour de l'obstacle (essieu à rayon_orbite_mm du centre) ;
               dès que les sondes voient la ligne (hors de la ligne entrante, derrière l'obstacle) :
  SORTIE_ORBITE : elle tourne vers l'extérieur jusqu'à pointer à l'opposé de l'obstacle, puis le PID suit la ligne
               dans le bon sens (en s'éloignant de l'obstacle).
  TRAVERS    : la ligne est coupée presque de face (au moins n_sondes_travers sondes : plus de ~60 deg avec le cap).
               L'écart lu vaut ~0 : le PID partait tout droit et sortait de la ligne (journaux Godot : ligne revue
               6 à 8 % du temps ensuite). Braquage maximal du côté qui s'éloigne de l'obstacle : les sondes passent
               la ligne, puis y reviennent après ~70 deg de rotation, et le PID reprend.
Un obstacle vu devant pendant LONGER ou RETOUR (obstacle suivant, ou décalé) relance ECART, à deux conditions :
  - l'écho ne vient pas d'un obstacle déjà contourné (mémoire de sa zone : en le longeant, le cône de l'ultrason
    attrape souvent son coin) ;
  - il est confirmé par echos_confirmation pas consécutifs sur la mesure brute (un écho isolé ne suffit pas).
(Journal Godot du 5 novembre : un seul écho à 23 cm sur le coin de l'obstacle longé relançait tout l'ECART.)
En cas d'impossibilité (trop loin sur le côté, ligne introuvable), `echec` passe à True : le contrôleur s'arrête
(BLOQUE) au lieu de partir au hasard.
Le servo met du temps à braquer : les changements de phase sont anticipés (_retard_mm).
Le Controleur choisit ce contournement quand le contournement fluide n'a pas la place (ContournementFluide.choisir :
aucun rayon ne passe sans toucher l'obstacle), et pour éviter un obstacle quand la voiture n'est pas sur la ligne.
VÉRIFICATION (13 nov., #72) : avant de partir, choisir_cap() rejoue la manœuvre prévue (ECART, LONGER, début du
RETOUR) avec la même loi de commande et le modèle bicyclette, et vérifie que la voiture ne touche pas l'obstacle
(GeometrieVoiture). Premier cap de caps_evitement_candidats_deg qui passe ; aucun : la manœuvre est refusée et le
Controleur s'arrête (choix de Mehdi : arrêt sûr plutôt que contact, pas de recul supplémentaire).
Avant : cap de 40 deg sans vérification ; avec arrêt + recul = 150 mm, le coin avant touchait un bloc de 150 de
large (banc D_recul 50, graine 63353 : la loi de cap proportionnelle n'avait décalé la voiture que de 43 mm après
142 mm d'avance).
"""
from . import constantes
from . import godot as gd
from .constantes import Phase
from .geometrie_voiture import GeometrieVoiture
from .godot import Vec2


class Contournement:
    Phase = Phase

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « contournement »), lues au démarrage ; ici :
    ## type et justification seulement.
    REGLAGES = {
        "actif": bool,
        "cote_par_defaut": int,
        "cote_interieur_virage": bool,
        "seuil_virage_deg": float,
        "vitesse_manoeuvre_mm_s": float,
        "distance_manoeuvre_cm": float,
        "distance_securite_mm": float,
        "braquage_manoeuvre": float,
        "cap_evitement_deg": float,
        "caps_evitement_candidats_deg": list,
        "cap_retour_deg": float,
        "gain_cap": float,
        "gain_lateral": float,
        "correction_laterale_max_deg": float,
        "distance_vue_mm": float,
        "distance_obstacle_suivant_mm": float,
        "echos_confirmation": int,
        "tolerance_memoire_mm": float,
        "marge_couloir_mm": float,
        "largeur_passe_mm": float,
        "decalage_max_mm": float,
        "depassement_max_mm": float,
        "recherche_max_deg": float,
        "recherche_en_orbite": bool,
        "rayon_orbite_mm": float,
        "gain_rayon_orbite": float,
        "correction_orbite_max_deg": float,
        "arc_orbite_max_deg": float,
        "rayon_orbite_plus_mm": float,
        "distance_entrante_mm": float,
        "sortie_orbite_tolerance_deg": float,
        "distance_observation_mm": float,
        "pente_ligne_tournee": float,
        "sortie_rapide_mm": float,
        "angle_ligne_tournee_deg": float,
        "croisement_max_mm": float,
        "n_sondes_travers": int,
        "rotation_travers_min_deg": float,
        "rotation_travers_max_deg": float,
        "marge_laterale_mm": float,
        "marge_longitudinale_mm": float,
    }

    def __init__(self, robot, odo, epreuve):
        self.actif = False                 # False = on s'arrête devant l'obstacle (test "Arrêt devant obstacle")
        self.cote_par_defaut = 0           # 1 = par la droite, -1 = par la gauche (en ligne droite)
        self.cote_interieur_virage = False  # en virage, passer par l'intérieur (la ligne revient vers la voiture)
        self.seuil_virage_deg = 0.0        # braquage moyen (servo, par rapport à 90) au-dessus duquel on est en virage
        ## (= ancienne commande 25 à 4 mm/s par commande) Banc d'essai du 12 nov. : à la commande 40, la bille tombait
        ## dans 4 pistes sur 6 (braquage par paliers) ; 25 = valeur d'origine
        self.vitesse_manoeuvre_mm_s = 0.0
        ## Hors de la ligne seulement (évitement) : distance d'arrêt avant d'éviter un obstacle.
        ## Sur la ligne, la séquence arrêt / recul / contournement utilise ReglagesEpreuve.
        self.distance_manoeuvre_cm = 0.0
        self.distance_securite_mm = 0.0    # arrêt d'urgence si un obstacle apparaît tout près pendant la manœuvre
        self.braquage_manoeuvre = 0.0      # degrés de servo par rapport à 90 (max 45)
        self.cap_evitement_deg = 0.0       # cap pris pour s'écarter de la ligne (évitement hors ligne, sans vérification)
        ## Sur la ligne : caps essayés par choisir_cap, du plus doux au plus serré (90 = perpendiculaire à la ligne).
        self.caps_evitement_candidats_deg = []
        self.cap_retour_deg = 0.0          # cap pris pour revenir vers la ligne (45 plutôt que 30 : on reste moins loin de la ligne)
        self.gain_cap = 0.0                # degrés de roue par degré d'erreur de cap
        self.gain_lateral = 0.0            # rad de cap par mm d'écart latéral (phase LONGER)
        self.correction_laterale_max_deg = 0.0
        # Détection pendant la manœuvre
        self.distance_vue_mm = 0.0         # en ECART, un écho plus proche que ça = l'obstacle est encore devant
        self.distance_obstacle_suivant_mm = 0.0  # en LONGER / RETOUR, un écho plus proche = nouvel obstacle à éviter
        self.echos_confirmation = 0        # pas consécutifs (à 50 Hz) avec écho brut : au moins 2 mesures de l'ultrason (20 Hz)
        self.tolerance_memoire_mm = 0.0    # marge autour de la zone d'un obstacle déjà contourné
        self.marge_couloir_mm = 0.0        # en LONGER, une ligne vue plus loin que le couloir prévu + ça est ignorée...
        self.largeur_passe_mm = 0.0        # ... sauf à hauteur de l'obstacle ou après, jusqu'à couloir + ça (ligne en travers)
        self.decalage_max_mm = 0.0         # au-delà, on abandonne (BLOQUE)
        self.depassement_max_mm = 0.0      # en RETOUR, on dépasse la position prévue de la ligne de ça avant de la chercher
        self.recherche_max_deg = 0.0       # rotation maximale pendant la recherche de la ligne (si recherche_en_orbite = False)
        # Recherche en orbite autour de l'obstacle
        self.recherche_en_orbite = False
        self.rayon_orbite_mm = 0.0         # essieu arrière -> centre de l'obstacle
        self.gain_rayon_orbite = 0.0       # rad de cap par mm d'écart au rayon
        self.correction_orbite_max_deg = 0.0
        self.arc_orbite_max_deg = 0.0      # tour presque complet ; ensuite une 2e orbite plus large, puis abandon (BLOQUE)
        self.rayon_orbite_plus_mm = 0.0    # élargissement de la 2e orbite
        ## Ligne entrante : points de la ligne vus par les sondes sur le dernier mètre avant l'arrêt (repère de la
        ## manœuvre), fournis par le contrôleur (liste de Vec2). Pendant l'orbite, une ligne vue à moins de
        ## distance_entrante_mm de ce chemin est celle par laquelle la voiture est arrivée : refusée, même en virage
        ## (journal Godot : la voiture repartait vers le départ).
        self.chemin_entrant = []
        self.distance_entrante_mm = 0.0
        ## SORTIE_ORBITE rend la main au PID dès que le cap est à moins de ça de la direction "opposée à l'obstacle" :
        ## la ligne est déjà sous les sondes, il suffit que le bon sens soit le plus proche des deux (journal Godot :
        ## à 35 deg, la voiture continuait de tourner plus d'une seconde avec la ligne sous les sondes).
        self.sortie_orbite_tolerance_deg = 0.0
        # Direction de la ligne retrouvée (CROISEMENT)
        self.distance_observation_mm = 0.0  # distance parcourue avant de juger la direction de la ligne
        self.pente_ligne_tournee = 0.0     # glissement (mm de sondes par mm parcouru, vers le contournement) en dessous : a tourné
        self.sortie_rapide_mm = 0.0        # ligne perdue après moins que ça : vue sous un angle fort, elle a tourné
        self.angle_ligne_tournee_deg = 0.0  # angle de la ligne balayée (par rapport au cap de départ) au-delà : a tourné
        self.croisement_max_mm = 0.0       # ligne perdue depuis plus que ça en CROISEMENT : le PID reprend la main
        # Ligne coupée de face (TRAVERS)
        self.n_sondes_travers = 0
        self.rotation_travers_min_deg = 0.0  # avant, la ligne encore sous les sondes est celle que l'on vient de couper
        self.rotation_travers_max_deg = 0.0
        # Dimensions (mm). Obstacle : ReglagesEpreuve, plus grande emprise possible (largeur + tolérance de centrage de
        # chaque côté, profondeur) : le bloc peut être posé sur n'importe quelle face.
        self.marge_laterale_mm = 0.0
        self.marge_longitudinale_mm = 0.0

        self.phase = Phase.ECART
        self.cote = 1
        self.termine = False
        self.echec = False
        self.hors_ligne = False            # évitement d'un obstacle qui n'est pas sur la ligne (demarrer_hors_ligne)

        self._robot = robot
        self._odo = odo
        self._epreuve = epreuve
        self._x_requis = 0.0               # décalage latéral visé (mm, positif du côté du contournement)
        self._cap_ecart_deg = 40.0         # cap de la phase ECART en cours (choisir_cap, sinon cap_evitement_deg)
        self._perdu = False                # l'ultrason a perdu l'obstacle pendant ECART
        self._d_vu = 0.0                   # dernière distance à laquelle l'obstacle a été vu (mm)
        self._e_reprise = gd.NAN           # écart lu par les sondes quand le PID reprend directement la main (mm)
        self._e_vu = 0.0                   # dernier écart lu par les sondes (mm)
        self._obs_actif = False            # observation de la direction de la ligne en cours (CROISEMENT)
        self._obs_depart = Vec2.ZERO       # position (odométrie) au début de l'observation
        self._obs_e = 0.0                  # écart lu au début de l'observation
        self._obs_dernier = Vec2.ZERO      # position (odométrie) à la dernière vue de la ligne
        self._zones = []                   # obstacles déjà contournés : (y début, y fin) dans le repère de la manœuvre (mm)
        self._centre = Vec2.ZERO           # centre estimé de l'obstacle (repère de la manœuvre, mm)
        self._phi_debut = 0.0              # angle de départ autour de l'obstacle (orbite)
        self._arc = 0.0                    # angle cumulé parcouru autour de l'obstacle (rad)
        self._phi_prec = 0.0
        self._rayon_orbite = 300.0         # rayon de l'orbite en cours (mm)
        self._n_orbite = 0                 # 0 = première orbite, 1 = orbite élargie
        self._n_echos = 0                  # pas consécutifs avec un écho brut devant
        self._y_fin = 0.0                  # y de l'essieu à atteindre avant de revenir
        self._x_ligne = 0.0                # position x de la ligne, repérée quand les sondes la revoient
        self._vu_cote_alignement = False   # ALIGNEMENT : ligne déjà vue du côté du contournement (c * e > 0)
        self._e_alignement = gd.NAN        # ALIGNEMENT : dernier écart lu par les sondes (mm), NAN = pas vue
        self._cap_debut_recherche = 0.0
        self._cote_travers = 1             # 1 = braquage à droite (phase TRAVERS)
        self._cap_travers = 0.0

    def _largeur_obstacle(self):
        """Largeur à éviter (mm) : obstacle et tolérance de centrage des deux côtés."""
        return self._epreuve.largeur_a_contourner_mm() + 2.0 * self._epreuve.tolerance_centrage_mm

    def _longueur_obstacle(self):
        """Profondeur de l'obstacle dans le sens de la marche (mm)."""
        return self._epreuve.profondeur_a_contourner_mm()

    def demarrer(self, d0_mm, braquage_moyen):
        """d0_mm : distance de l'obstacle à l'arrêt. braquage_moyen : braquage moyen récent (servo - 90), pour le choix
        du côté."""
        self._odo.reinitialiser()
        self.cote = self.cote_par_defaut
        if self.cote_interieur_virage and gd.absf(braquage_moyen) >= self.seuil_virage_deg:
            self.cote = 1 if braquage_moyen > 0.0 else -1
        self.hors_ligne = False
        self._nouvel_obstacle(d0_mm)
        self._x_requis = self._largeur_obstacle() / 2.0 + self._robot.largeur_voiture_mm / 2.0 + self.marge_laterale_mm
        self.termine = False
        self.echec = False
        self._e_reprise = gd.NAN
        self._zones = [Vec2(d0_mm + self._robot.longueur_avant_mm,
                            d0_mm + self._robot.longueur_avant_mm + self._longueur_obstacle())]
        self._centre = Vec2(0.0, d0_mm + self._robot.longueur_avant_mm + self._longueur_obstacle() / 2.0)
        self._n_echos = 0
        self._cap_ecart_deg = self.cap_evitement_deg

    def choisir_cap(self, d0_mm, marge_mm):
        """Sur la ligne, juste après demarrer() : premier cap de caps_evitement_candidats_deg dont la manœuvre prévue
        passe sans toucher l'obstacle (centré sur la ligne, plus grande emprise, tolérance de centrage). False : aucun
        ne passe.
        marge_mm : marge de la vérification géométrique (le Controleur passe ContournementFluide.marge_obstacle_mm : une
        seule valeur)."""
        for cap_deg in self.caps_evitement_candidats_deg:
            if self._manoeuvre_libre(float(cap_deg), d0_mm, marge_mm):
                self._cap_ecart_deg = cap_deg
                return True
        return False

    def _manoeuvre_libre(self, cap_deg, d0_mm, marge_mm):
        """Rejoue ECART, LONGER et le début du RETOUR avec les mêmes lois que _phase_ecart, _phase_longer et
        _phase_retour (côté droit ; le gauche est symétrique). Hypothèses : vitesse de manœuvre dès le départ (prudent :
        plus de chemin pendant que le servo braque), obstacle perdu par l'ultrason dès le début d'ECART, servo à sa
        vitesse nominale."""
        demi = self._largeur_obstacle() / 2.0 + marge_mm
        face = d0_mm + self._robot.longueur_avant_mm
        obs_min = Vec2(-demi, face - marge_mm)
        obs_max = Vec2(demi, face + self._longueur_obstacle() + marge_mm)
        v = self.vitesse_manoeuvre_mm_s
        dt = gd.div_reel(1.0, self._robot.frequence_controle_hz)   # pas de la prévision = période de la boucle du robot (doit être > 0)
        p = Vec2.ZERO
        cap = 0.0
        servo = self._odo.servo_estime
        etape = Phase.ECART
        t = 0.0
        while t < 30.0:
            angle = 90.0
            if etape == Phase.ECART:
                angle = self._angle_cap(gd.deg_to_rad(cap_deg), cap)
                duree = gd.absf(90.0 - self.braquage_manoeuvre - servo) / self._robot.vitesse_servo_deg_s \
                    if self._robot.vitesse_servo_deg_s > 0.0 else 0.0
                anticipation = self._rayon_virage_mm() * (1.0 - gd.cos(cap)) + v * duree / 2.0 * gd.absf(gd.sin(cap))
                if p.x >= self._x_requis - anticipation:
                    etape = Phase.LONGER
                elif p.x > self.decalage_max_mm:
                    return False
            elif etape == Phase.LONGER:
                corr = gd.clampf((self._x_requis - p.x) * self.gain_lateral,
                                 -gd.deg_to_rad(self.correction_laterale_max_deg),
                                 gd.deg_to_rad(self.correction_laterale_max_deg))
                angle = self._angle_cap(corr, cap)
                if p.y >= self._y_fin:
                    etape = Phase.RETOUR
            elif etape == Phase.RETOUR:
                angle = self._angle_cap(-gd.deg_to_rad(self.cap_retour_deg), cap)
                if p.x <= 0.0 or p.y > obs_max.y + self._robot.avant_voiture_mm() + self._robot.arriere_voiture_mm:
                    return True            # la voiture est revenue sur la ligne, ou entièrement après l'obstacle
            servo = angle if self._robot.vitesse_servo_deg_s <= 0.0 \
                else gd.move_toward(servo, angle, self._robot.vitesse_servo_deg_s * dt)
            delta = gd.deg_to_rad(self._robot.servo_vers_roues_deg(servo - 90.0))
            cap += v / self._robot.empattement_mm * gd.tan(delta) * dt
            p = p + Vec2(gd.sin(cap), gd.cos(cap)) * v * dt
            if GeometrieVoiture.touche(p, cap, self._robot.avant_voiture_mm(), self._robot.arriere_voiture_mm,
                                       self._robot.largeur_voiture_mm / 2.0, obs_min, obs_max):
                return False
            t += dt
        return False

    def demarrer_hors_ligne(self, d0_mm, cote_voulu):
        """La voiture n'est pas sur la ligne : on contourne l'obstacle du côté demandé (ECART, LONGER), puis on rend la
        main. Toute ligne vue en LONGER (sauf la ligne déjà parcourue) est reprise ; aucune recherche de ligne derrière
        l'obstacle, elle n'y passe pas forcément."""
        self.demarrer(d0_mm, 0.0)
        self.cote = cote_voulu
        self.hors_ligne = True

    def nom_phase(self):
        return Phase(self.phase).name + (" (droite)" if self.cote > 0 else " (gauche)")

    def ecart_ligne_mm(self):
        """Écart de la ligne à la fin de la manœuvre (mm, > 0 = à droite), pour le suivi si les sondes ne la voient
        pas."""
        if not gd.is_nan(self._e_reprise):
            return self._e_reprise
        return gd.clampf(self._x_ligne - self._odo.x, -constantes.ERREUR_MAX_MM, constantes.ERREUR_MAX_MM)

    def _reprise_par_pid(self, e):
        """La ligne vient d'être vue et a pu tourner : on termine, le PID suit les sondes (e = écart lu, mm)."""
        self._e_reprise = e
        self.termine = True

    def _ligne_dans_couloir(self, erreur_mm):
        """En LONGER : vrai si la ligne vue n'est pas plus loin sur le côté que le couloir prévu. À l'entrée de LONGER,
        la voiture est encore inclinée et ses sondes sont 300 à 500 mm sur le côté : une ligne vue là est le plus
        souvent une autre partie de la piste (journal Godot du 5 novembre). La vraie ligne qui tourne vers la voiture
        traverse le couloir (vers _x_requis)."""
        x_ligne = self.cote * (self._odo.x + self._robot.distance_sondes_mm * gd.sin(self._odo.cap)
                               + erreur_mm * gd.cos(self._odo.cap))
        return x_ligne <= self._x_requis + self.marge_couloir_mm

    def _ligne_en_travers(self, erreur_mm):
        """En LONGER : vrai si la ligne vue hors du couloir est celle qui tourne en travers sous la voiture (obstacle
        dans un virage) : à hauteur de l'obstacle ou après, à moins de largeur_passe_mm du côté, et pas sur la ligne
        entrante. (Pistes du journal Godot du 6 novembre rejouées au banc : rejetée, la voiture revenait vers l'ancienne
        ligne et se perdait ; +4 points d'obstacles franchis sur les 37 pistes.)"""
        q = Vec2(self._odo.x + self._robot.distance_sondes_mm * gd.sin(self._odo.cap) + erreur_mm * gd.cos(self._odo.cap),
                 self._odo.y + self._robot.distance_sondes_mm * gd.cos(self._odo.cap) - erreur_mm * gd.sin(self._odo.cap))
        return q.y >= self._centre.y - self._longueur_obstacle() / 2.0 \
            and self.cote * q.x <= self._x_requis + self.largeur_passe_mm \
            and not self._ligne_entrante(erreur_mm)

    def _ligne_a_tourne(self, somme, n):
        """En CROISEMENT : vrai si la ligne retrouvée n'est pas parallèle au cap de départ (elle a tourné)."""
        if not self._obs_actif:
            self._obs_actif = True
            self._obs_depart = Vec2(self._odo.x, self._odo.y)
            self._obs_e = self._e_vu
            self._obs_dernier = self._obs_depart
        p = Vec2(self._odo.x, self._odo.y)
        parcouru = p.distance_to(self._obs_depart)
        if n > 0:
            self._e_vu = somme / n
            self._obs_dernier = p
            return parcouru >= self.distance_observation_mm \
                and gd.div_reel(self.cote * (self._e_vu - self._obs_e), parcouru) < self.pente_ligne_tournee
        # Une ligne parallèle traverse toute la barre et la quitte du côté du contournement. Si elle la quitte de
        # l'autre côté, elle tourne dans le sens où va la voiture (obstacle dans un virage) : l'alignement braquerait à
        # l'envers (journal Godot du 6 novembre, 19 h 17 : la voiture perdait la ligne juste après l'avoir retrouvée).
        if parcouru < self.sortie_rapide_mm or self.cote * self._e_vu < 0.0:
            return True
        # Angle de la ligne (repère de la manœuvre) d'après son balayage de la barre : cap + c * atan(c * de/ds).
        # Ligne parallèle au cap de départ : ~0. Sinon elle a tourné : l'alignement suppose une ligne parallèle et la
        # voiture filerait tout droit sans jamais la rejoindre (journal Godot du 6 novembre, 19 h 49).
        balaye = self._obs_dernier.distance_to(self._obs_depart)
        if balaye >= self.distance_observation_mm:
            angle_ligne = self._odo.cap + self.cote * gd.atan(gd.div_reel(self.cote * (self._e_vu - self._obs_e), balaye))
            if gd.absf(angle_ligne) > gd.deg_to_rad(self.angle_ligne_tournee_deg):
                return True
        # Filet de sécurité : ligne perdue depuis trop longtemps sans que l'alignement commence
        return p.distance_to(self._obs_dernier) > self.croisement_max_mm

    def calculer(self, somme, n, v_mm_s, d_mm, d_brut_mm=-1.0, _dt=0.02):
        """v_mm_s : vitesse actuelle ; d_mm : distance ultrason filtrée (-1 = rien) ; d_brut_mm : dernière mesure brute.
        Retourne Vec2(angle servo, vitesse cible). Regarder `termine` et `echec` ensuite."""
        c = float(self.cote)
        x = c * self._odo.x               # décalage latéral, positif du côté du contournement
        obstacle_devant = False
        self._detecter_ligne_de_face(somme, n)
        if self.phase == Phase.LONGER or self.phase == Phase.RETOUR:
            obstacle_devant = self._nouvel_obstacle_devant(d_mm, d_brut_mm)
        else:
            self._n_echos = 0
        angle = 90.0
        if self.phase == Phase.ECART:
            angle = self._phase_ecart(c, x, d_mm, v_mm_s)
        elif self.phase == Phase.LONGER:
            angle = self._phase_longer(c, x, somme, n, d_mm, obstacle_devant)
        elif self.phase == Phase.RETOUR:
            angle = self._phase_retour(c, x, somme, n, d_mm, obstacle_devant)
        elif self.phase == Phase.ORBITE:
            angle = self._phase_orbite(c, somme, n)
        elif self.phase == Phase.SORTIE_ORBITE:
            angle = self._phase_sortie_orbite(c, somme, n)
        elif self.phase == Phase.RECHERCHE:
            angle = self._phase_recherche(c, somme, n)
        elif self.phase == Phase.TRAVERS:
            angle = self._phase_travers(somme, n)
        elif self.phase == Phase.CROISEMENT:
            angle = self._phase_croisement(c, somme, n, v_mm_s)
        elif self.phase == Phase.ALIGNEMENT:
            angle = self._phase_alignement(c, somme, n, v_mm_s)
        return Vec2(angle, self.vitesse_manoeuvre_mm_s)

    # ----- Phases (chacune retourne l'angle servo et change `phase`, `termine` ou `echec` si besoin)

    def _phase_ecart(self, c, x, d_mm, v_mm_s):
        angle = self._angle_vers_cap(c * gd.deg_to_rad(self._cap_ecart_deg))
        vu = d_mm >= 0.0 and d_mm < self.distance_vue_mm
        if vu:
            self._perdu = False
            self._d_vu = d_mm
        elif not self._perdu:
            self._perdu = True
            # l'obstacle vient de sortir du cône : son bord est au plus sur le rayon extrême (cap - demi-angle)
            cap_c = c * self._odo.cap
            x_ultrason = x + self._robot.longueur_avant_mm * gd.sin(cap_c)
            bord = x_ultrason + self._d_vu * gd.sin(cap_c - gd.deg_to_rad(self._robot.demi_angle_ultrason_deg))
            self._x_requis = gd.maxf(self._x_requis, bord + self._robot.largeur_voiture_mm / 2.0 + self.marge_laterale_mm)
        # le redressement ajoute encore R * (1 - cos(cap)) de décalage, plus le trajet pendant que le servo braque
        anticipation = self._rayon_virage_mm() * (1.0 - gd.cos(self._odo.cap)) \
            + self._retard_mm(90.0 - c * self.braquage_manoeuvre, v_mm_s) * gd.absf(gd.sin(self._odo.cap))
        if self._perdu and x >= self._x_requis - anticipation:
            self.phase = Phase.LONGER
        elif x > self.decalage_max_mm:
            self.echec = True
        return angle

    def _phase_longer(self, c, x, somme, n, d_mm, obstacle_devant):
        corr = gd.clampf((self._x_requis - x) * self.gain_lateral,
                         -gd.deg_to_rad(self.correction_laterale_max_deg), gd.deg_to_rad(self.correction_laterale_max_deg))
        angle = self._angle_vers_cap(c * corr)
        if self.hors_ligne and n > 0 and not self._ligne_entrante(somme / n):
            self._reprise_par_pid(somme / n)         # pas la ligne déjà parcourue : la voiture repartirait en arrière
        elif self.hors_ligne and self._odo.y >= self._y_fin:
            self._e_reprise = c * constantes.ERREUR_MAX_MM
            self.termine = True
        elif n > 0 and (self._ligne_dans_couloir(somme / n) or self._ligne_en_travers(somme / n)):
            self._reprise_par_pid(somme / n)         # près d'un virage, la ligne revient sous la voiture
        elif obstacle_devant:
            self._relancer_ecart(d_mm)
        elif self._odo.y >= self._y_fin:
            self.phase = Phase.RETOUR
        return angle

    def _phase_retour(self, c, x, somme, n, d_mm, obstacle_devant):
        angle = self._angle_vers_cap(-c * gd.deg_to_rad(self.cap_retour_deg))
        if n > 0:
            self._reperer_ligne(somme / n)
            self.phase = Phase.CROISEMENT
        elif obstacle_devant:
            self._relancer_ecart(d_mm)
        elif x < -self.depassement_max_mm:
            self._cap_debut_recherche = self._odo.cap
            if self.recherche_en_orbite:
                self._phi_debut = self._angle_autour(Vec2(self._odo.x, self._odo.y))
                self._phi_prec = self._phi_debut
                self._arc = 0.0
                self._rayon_orbite = self.rayon_orbite_mm
                self._n_orbite = 0
                self.phase = Phase.ORBITE
            else:
                self.phase = Phase.RECHERCHE
        return angle

    def _phase_orbite(self, c, somme, n):
        p = Vec2(self._odo.x, self._odo.y)
        phi = self._angle_autour(p)
        r = p.distance_to(self._centre)
        # en contournant par la droite, on tourne autour de l'obstacle dans le sens trigonométrique
        cap_tangent = -phi if self.cote > 0 else gd.PI - phi
        corr = gd.clampf(self.gain_rayon_orbite * (r - self._rayon_orbite), -gd.deg_to_rad(self.correction_orbite_max_deg),
                         gd.deg_to_rad(self.correction_orbite_max_deg))
        servo = self._robot.roues_vers_servo(
            gd.rad_to_deg(gd.wrapf(cap_tangent - c * corr - self._odo.cap, -gd.PI, gd.PI)) * self.gain_cap)
        angle = 90.0 + gd.clampf(servo, -self.braquage_manoeuvre, self.braquage_manoeuvre)
        self._arc += gd.absf(gd.wrapf(phi - self._phi_prec, -gd.PI, gd.PI))   # angle cumulé : peut dépasser 180 deg
        self._phi_prec = phi
        if n > 0 and not self._ligne_entrante(somme / n):
            self._e_vu = somme / n
            self.phase = Phase.SORTIE_ORBITE
        elif n == 0 and self._arc > gd.deg_to_rad(self.arc_orbite_max_deg):
            if self._n_orbite == 0:
                self._n_orbite = 1                   # on ne se décourage pas : 2e orbite, plus large
                self._rayon_orbite += self.rayon_orbite_plus_mm
                self._arc = 0.0
            else:
                self.echec = True
        return angle

    def _phase_sortie_orbite(self, c, somme, n):
        """Tourne vers l'extérieur jusqu'à pointer à l'opposé de l'obstacle, puis le PID suit la ligne dans ce sens."""
        p = Vec2(self._odo.x, self._odo.y)
        cap_sortie = gd.atan2(p.x - self._centre.x, p.y - self._centre.y)
        if n > 0:
            self._e_vu = somme / n
        if gd.absf(gd.wrapf(cap_sortie - self._odo.cap, -gd.PI, gd.PI)) < gd.deg_to_rad(self.sortie_orbite_tolerance_deg):
            self._reprise_par_pid(self._e_vu)
            return 90.0
        return 90.0 + c * self.braquage_manoeuvre

    def _phase_recherche(self, c, somme, n):
        """La ligne a tourné : on tourne serré vers le côté d'où l'on vient."""
        if n > 0:
            self._reprise_par_pid(somme / n)
        elif gd.absf(self._odo.cap - self._cap_debut_recherche) > gd.deg_to_rad(self.recherche_max_deg):
            self.echec = True
        return 90.0 - c * self.braquage_manoeuvre

    def _phase_travers(self, somme, n):
        rotation_faite = gd.absf(gd.wrapf(self._odo.cap - self._cap_travers, -gd.PI, gd.PI))
        if (rotation_faite >= gd.deg_to_rad(self.rotation_travers_min_deg) and n > 0 and n <= 2) \
                or rotation_faite >= gd.deg_to_rad(self.rotation_travers_max_deg):
            self._reprise_par_pid(somme / n if n > 0 else self._cote_travers * constantes.ERREUR_MAX_MM)
        return 90.0 + self._cote_travers * self.braquage_manoeuvre

    def _phase_croisement(self, c, somme, n, v_mm_s):
        if self._ligne_a_tourne(somme, n):
            self._reprise_par_pid(self._e_vu)
            return 90.0
        # tout droit : le virage d'alignement décale encore l'essieu de R * (1 - cos(cap)) vers la ligne
        avance_braquage = self._retard_mm(90.0 + c * self.braquage_manoeuvre, v_mm_s) * gd.absf(gd.sin(self._odo.cap))
        if c * (self._odo.x - self._x_ligne) <= self._rayon_virage_mm() * (1.0 - gd.cos(self._odo.cap)) + avance_braquage:
            self.phase = Phase.ALIGNEMENT
            self._vu_cote_alignement = False
            self._e_alignement = gd.NAN
        return 90.0

    def _phase_alignement(self, c, somme, n, v_mm_s):
        """Braquage constant : on finit dans l'axe, l'essieu arrière sur la ligne.
        Ligne parallèle : elle revient sous la barre du côté du contournement (c * e > 0) et s'arrête près du centre.
        Si elle traverse la barre jusqu'à l'autre côté, elle tourne à l'opposé du contournement : le PID reprend dès
        maintenant, la ligne sous les sondes. (Banc D_recul 50, graine 79191, 14 nov. : obstacle 0,65 m avant un virage
        à gauche ; la ligne passait de 00001 à 10000 puis était perdue, le suivi repartait à droite d'après
        l'odométrie : 9 s en cercle, RETROUVER, ligne à contresens, DEMI_TOUR, bille tombée.)"""
        if n > 0:
            e = somme / n
            self._e_alignement = e
            if c * e > 0.0:
                self._vu_cote_alignement = True
            elif c * e < 0.0 and self._vu_cote_alignement:
                self._reprise_par_pid(e)
                return 90.0 + c * self.braquage_manoeuvre
        if c * self._odo.cap + self._inertie_cap(v_mm_s) >= -gd.deg_to_rad(3.0):
            self.termine = True
            if n == 0:
                self._e_reprise = self._ecart_fin_alignement_sans_ligne(c)
        return 90.0 + c * self.braquage_manoeuvre

    def _ecart_fin_alignement_sans_ligne(self, c):
        """Fin d'ALIGNEMENT sans ligne sous les sondes : de quel côté la chercher (mm, > 0 = à droite).
        L'estimation odométrique (_x_ligne - x) suppose une ligne droite ; si elle est plus petite que ERREUR_MAX_MM, la
        ligne devrait être sous les sondes : elle a tourné et le signe de l'estimation n'est que du bruit (graine 63353,
        D_recul 50 : 2,4 mm d'écart entre deux essais, l'un repartait du bon côté, l'autre non et la bille tombait). On
        prend alors le dernier côté où elle a été vue : en ALIGNEMENT s'il y a lieu, sinon le côté du contournement, par
        où elle a quitté la barre en CROISEMENT."""
        if not gd.is_nan(self._e_alignement) and self._e_alignement != 0.0:
            return gd.signf(self._e_alignement) * constantes.ERREUR_MAX_MM
        estime = self._x_ligne - self._odo.x
        if gd.absf(estime) >= constantes.ERREUR_MAX_MM:
            return gd.clampf(estime, -constantes.ERREUR_MAX_MM, constantes.ERREUR_MAX_MM)
        return c * constantes.ERREUR_MAX_MM

    # ----- Outils

    def _detecter_ligne_de_face(self, somme, n):
        """Passe en TRAVERS si la ligne de sortie est coupée de face. Côté du virage : celui qui s'éloigne de
        l'obstacle."""
        if self.hors_ligne or n < self.n_sondes_travers or self.phase not in (
                Phase.LONGER, Phase.RETOUR, Phase.CROISEMENT, Phase.ALIGNEMENT, Phase.ORBITE, Phase.SORTIE_ORBITE):
            return
        e = somme / n
        if self._ligne_entrante(e) or (self.phase == Phase.LONGER and not self._ligne_en_travers(e)):
            return
        droite = Vec2(gd.cos(self._odo.cap), -gd.sin(self._odo.cap))
        q = Vec2(self._odo.x + self._robot.distance_sondes_mm * gd.sin(self._odo.cap),
                 self._odo.y + self._robot.distance_sondes_mm * gd.cos(self._odo.cap)) + droite * e
        self._cote_travers = 1 if (q - self._centre).dot(droite) > 0.0 else -1
        self._cap_travers = self._odo.cap
        self.phase = Phase.TRAVERS

    def _nouvel_obstacle(self, d_mm):
        """Un obstacle est vu à d_mm devant : il faudra le dépasser entièrement avant de revenir."""
        self._y_fin = self._odo.y + d_mm + self._robot.longueur_avant_mm + self._longueur_obstacle() + self.marge_longitudinale_mm
        self._perdu = False
        self._d_vu = d_mm
        self.phase = Phase.ECART

    def _relancer_ecart(self, d_mm):
        """Obstacle devant pendant la manœuvre : on s'écarte encore, à partir d'ici."""
        self._x_requis = self.cote * self._odo.x + self._largeur_obstacle() / 2.0 + self._robot.largeur_voiture_mm / 2.0 \
            + self.marge_laterale_mm
        y0 = self._odo.y + d_mm + self._robot.longueur_avant_mm
        self._zones.append(Vec2(y0, y0 + self._longueur_obstacle()))
        self._nouvel_obstacle(d_mm)

    def _nouvel_obstacle_devant(self, d_mm, d_brut_mm):
        """Vrai si un NOUVEL obstacle est devant (LONGER, RETOUR) : écho proche, hors de la zone d'un obstacle déjà
        contourné, et confirmé sur plusieurs pas de mesure brute."""
        if not (d_mm >= 0.0 and d_mm < self.distance_obstacle_suivant_mm) or self._echo_connu(d_mm):
            self._n_echos = 0
            return False
        self._n_echos = self._n_echos + 1 if d_brut_mm >= 0.0 else 0
        return self._n_echos >= self.echos_confirmation

    def _echo_connu(self, d_mm):
        """Vrai si le point d'écho (à d_mm devant l'ultrason) tombe sur un obstacle déjà contourné."""
        portee = self._robot.longueur_avant_mm + d_mm
        y_echo = self._odo.y + portee * gd.cos(self._odo.cap)
        x_echo = self.cote * (self._odo.x + portee * gd.sin(self._odo.cap))     # positif du côté du contournement
        bord = self._x_requis - self._robot.largeur_voiture_mm / 2.0 - self.marge_laterale_mm
        if x_echo > bord + self.tolerance_memoire_mm:
            return False
        for z in self._zones:
            if y_echo >= z.x - self.tolerance_memoire_mm and y_echo <= z.y + self.tolerance_memoire_mm:
                return True
        return False

    def _angle_autour(self, p):
        """Angle (sens trigonométrique, depuis +x) du point p vu du centre de l'obstacle."""
        return gd.atan2(p.y - self._centre.y, p.x - self._centre.x)

    def _ligne_entrante(self, erreur_mm):
        """Vrai si la ligne vue par les sondes est la ligne entrante (sur le chemin suivi avant l'arrêt) : on ne la
        reprend pas. Sans mémoire (piste Blender démarrée près d'un obstacle...), on se rabat sur "derrière l'obstacle,
        dans l'axe"."""
        q = Vec2(self._odo.x + self._robot.distance_sondes_mm * gd.sin(self._odo.cap) + erreur_mm * gd.cos(self._odo.cap),
                 self._odo.y + self._robot.distance_sondes_mm * gd.cos(self._odo.cap) - erreur_mm * gd.sin(self._odo.cap))
        if len(self.chemin_entrant) == 0:
            return gd.absf(gd.wrapf(self._angle_autour(q) + gd.PI / 2.0, -gd.PI, gd.PI)) < gd.deg_to_rad(35.0)
        for p in self.chemin_entrant:
            if q.distance_squared_to(p) < self.distance_entrante_mm * self.distance_entrante_mm:
                return True
        return False

    def _reperer_ligne(self, erreur_mm):
        """Position x de la ligne (repère odométrie) à partir de l'écart lu par les sondes (mm, > 0 = à droite).
        La barre de sondes est à distance_sondes_mm devant l'essieu, perpendiculaire au cap."""
        self._x_ligne = self._odo.x + self._robot.distance_sondes_mm * gd.sin(self._odo.cap) + erreur_mm * gd.cos(self._odo.cap)
        self._e_vu = erreur_mm
        self._obs_actif = False                    # la direction de la ligne sera observée en CROISEMENT

    def _angle_vers_cap(self, cap_voulu):
        """Angle servo pour rejoindre le cap voulu (rad), braquage limité à braquage_manoeuvre."""
        return self._angle_cap(cap_voulu, self._odo.cap)

    def _angle_cap(self, cap_voulu, cap):
        """Même loi pour un cap donné (prévision de _manoeuvre_libre)."""
        servo = self._robot.roues_vers_servo(gd.rad_to_deg(cap_voulu - cap) * self.gain_cap)
        return 90.0 + gd.clampf(servo, -self.braquage_manoeuvre, self.braquage_manoeuvre)

    def _inertie_cap(self, v_mm_s):
        """Cap (rad, en valeur absolue) que la voiture prendra encore pendant que le servo revient à 90."""
        delta = gd.deg_to_rad(self._robot.servo_vers_roues_deg(self._odo.servo_estime - 90.0))
        return gd.absf(v_mm_s / self._robot.empattement_mm * gd.tan(delta) / 2.0 * self._odo.duree_servo(90.0))

    def _retard_mm(self, cible, v_mm_s):
        """Distance (mm) parcourue avant que le braquage vers `cible` soit établi."""
        return v_mm_s * self._odo.duree_servo(cible) / 2.0

    def _rayon_virage_mm(self):
        """Rayon de braquage (mm) de l'essieu arrière avec braquage_manoeuvre."""
        delta = gd.deg_to_rad(self._robot.servo_vers_roues_deg(self.braquage_manoeuvre))
        return gd.div_reel(self._robot.empattement_mm, gd.tan(delta))
