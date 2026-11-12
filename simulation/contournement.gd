## Contournement d'un obstacle en quittant la ligne-guide, en boucle fermee sur le cap (odometrie)
## et sur l'ultrason : la manoeuvre s'adapte a la taille et au decalage de l'obstacle.
##
## Phases :
##   ECART      : on prend le cap d'evitement jusqu'a ce que l'ultrason ne voie plus l'obstacle. A cet instant,
##                le bord de l'obstacle est forcement en deca du rayon extreme du cone : on en deduit sa position
##                laterale et on s'ecarte jusqu'a ce bord + demi-largeur de voiture + marge
##                (minimum : obstacle de largeur_obstacle_min_mm centre sur la ligne).
##   LONGER     : cap parallele a la ligne, a ce decalage, jusqu'a avoir depasse l'obstacle le plus long prevu.
##                Si les sondes voient la ligne (pres d'un virage, elle revient sous la voiture), le PID reprend la main.
##   RETOUR     : cap de retour vers la ligne, jusqu'a ce que les sondes la voient.
##   RECHERCHE  : la ligne n'etait pas la ou prevu (elle a tourne) : virage serre vers elle ; des qu'elle est vue,
##                le PID reprend la main.
##   Pendant LONGER et RECHERCHE, la ligne retrouvee a pu tourner : CROISEMENT / ALIGNEMENT, qui supposent une
##   ligne parallele au cap de depart, viseraient a cote. Le PID, lui, suit les sondes quelle que soit sa direction
##   (en simulation : +10 points de reussite pres et dans les virages, aucun recul en ligne droite).
##   CROISEMENT : ligne vue ; tout droit jusqu'au point ou le virage d'alignement finira dessus.
##                On verifie d'abord que la ligne est bien parallele au cap de depart : en avancant, le point ou elle
##                coupe la barre de sondes glisse lentement du cote du contournement (ligne parallele, vue sous ~30 deg)
##                ou vite de l'autre cote (ligne qui a tourne : virage oppose au contournement). Dans ce cas, ou si
##                elle quitte la barre en quelques cm, le PID reprend la main (+12 points pres des virages en simulation).
##   ALIGNEMENT : virage qui finit sur la ligne, dans l'axe -> termine.
##   ORBITE     : (remplace RECHERCHE) la ligne n'etait pas la ou prevu. L'obstacle est pose SUR la ligne : la ligne
##                part de l'obstacle et s'en eloigne, donc tout cercle centre sur l'obstacle la coupe, quel que soit
##                le sens du virage. La voiture tourne autour de l'obstacle (essieu a rayon_orbite_mm du centre) ;
##                des que les sondes voient la ligne (hors de la ligne entrante, derriere l'obstacle) :
##   SORTIE_ORBITE : elle tourne vers l'exterieur jusqu'a pointer a l'oppose de l'obstacle, puis le PID suit la ligne
##                dans le bon sens (en s'eloignant de l'obstacle).
##   TRAVERS    : la ligne est coupee presque de face (au moins n_sondes_travers sondes : plus de ~60 deg avec le cap).
##                L'ecart lu vaut ~0 : le PID partait tout droit et sortait de la ligne (journaux Godot : ligne revue
##                6 a 8 % du temps ensuite). Braquage maximal du cote qui s'eloigne de l'obstacle : les sondes passent
##                la ligne, puis y reviennent apres ~70 deg de rotation, et le PID reprend.
## Un obstacle vu devant pendant LONGER ou RETOUR (obstacle suivant, ou decale) relance ECART, a deux conditions :
##   - l'echo ne vient pas d'un obstacle deja contourne (memoire de sa zone : en le longeant, le cone de l'ultrason
##     attrape souvent son coin) ;
##   - il est confirme par echos_confirmation pas consecutifs sur la mesure brute (un echo isole ne suffit pas).
## (Journal Godot du 5 novembre : un seul echo a 23 cm sur le coin de l'obstacle longe relancait tout l'ECART.)
## En cas d'impossibilite (trop loin sur le cote, ligne introuvable), `echec` passe a true : le controleur s'arrete
## (BLOQUE) au lieu de partir au hasard.
## Le servo met du temps a braquer : les changements de phase sont anticipes (_retard_mm).
## Le Controleur choisit ce contournement quand le contournement fluide n'est pas possible (obstacle vu a moins de
## ContournementFluide.distance_min_mm, en virage, ou voiture hors de la ligne).
class_name Contournement
extends RefCounted

enum Phase { ECART, LONGER, RETOUR, RECHERCHE, CROISEMENT, ALIGNEMENT, ORBITE, SORTIE_ORBITE, TRAVERS }

var actif := true                 # false = on s'arrete devant l'obstacle (test "Arret devant obstacle")
var cote_par_defaut := 1          # 1 = par la droite, -1 = par la gauche (en ligne droite)
var cote_interieur_virage := true # en virage, passer par l'interieur (la ligne revient vers la voiture)
var seuil_virage_deg := 8.0       # braquage moyen (servo, par rapport a 90) au-dessus duquel on est en virage
var vitesse_manoeuvre := 25       # Banc d'essai du 12 nov. : a 40, la bille tombait dans 4 pistes sur 6 (braquage par paliers) ; 25 = valeur d'origine
var distance_manoeuvre_cm := 30.0 # distance d'arret avant de contourner : a 20 cm, le coin avant touche les obstacles larges en braquant
var pause_s := 0.3                # immobilisation avant la manoeuvre (valeur d'origine ; 0,15 faisait tomber la bille)
var distance_securite_mm := 40.0  # arret d'urgence si un obstacle apparait tout pres pendant la manoeuvre
var braquage_manoeuvre := 40.0    # degres de servo par rapport a 90 (max 45)
var cap_evitement_deg := 40.0     # cap pris pour s'ecarter de la ligne
var cap_retour_deg := 45.0        # cap pris pour revenir vers la ligne (45 plutot que 30 : on reste moins loin de la ligne)
var gain_cap := 1.0               # degres de roue par degre d'erreur de cap
var gain_lateral := 0.005         # rad de cap par mm d'ecart lateral (phase LONGER)
var correction_laterale_max_deg := 10.0
# Detection pendant la manoeuvre
var distance_vue_mm := 450.0      # en ECART, un echo plus proche que ca = l'obstacle est encore devant
var distance_obstacle_suivant_mm := 300.0  # en LONGER / RETOUR, un echo plus proche = nouvel obstacle a eviter
var echos_confirmation := 4       # pas consecutifs (a 50 Hz) avec echo brut : au moins 2 mesures de l'ultrason (20 Hz)
var tolerance_memoire_mm := 40.0  # marge autour de la zone d'un obstacle deja contourne
var marge_couloir_mm := 60.0      # en LONGER, une ligne vue plus loin que le couloir prevu + ca est ignoree...
var largeur_passe_mm := 300.0     # ... sauf a hauteur de l'obstacle ou apres, jusqu'a couloir + ca (ligne en travers)
var decalage_max_mm := 450.0      # au-dela, on abandonne (BLOQUE)
var depassement_max_mm := 200.0   # en RETOUR, on depasse la position prevue de la ligne de ca avant de la chercher
var recherche_max_deg := 150.0    # rotation maximale pendant la recherche de la ligne (si recherche_en_orbite = false)
# Recherche en orbite autour de l'obstacle
var recherche_en_orbite := true
var rayon_orbite_mm := 300.0      # essieu arriere -> centre de l'obstacle
var gain_rayon_orbite := 0.004    # rad de cap par mm d'ecart au rayon
var correction_orbite_max_deg := 35.0
var arc_orbite_max_deg := 330.0   # tour presque complet ; ensuite une 2e orbite plus large, puis abandon (BLOQUE)
var rayon_orbite_plus_mm := 150.0 # elargissement de la 2e orbite
## Ligne entrante : points de la ligne vus par les sondes sur le dernier metre avant l'arret (repere de la manoeuvre),
## fournis par le controleur. Pendant l'orbite, une ligne vue a moins de distance_entrante_mm de ce chemin est celle
## par laquelle la voiture est arrivee : refusee, meme en virage (journal Godot : la voiture repartait vers le depart).
var chemin_entrant: Array[Vector2] = []
var distance_entrante_mm := 80.0
## SORTIE_ORBITE rend la main au PID des que le cap est a moins de ca de la direction "opposee a l'obstacle" :
## la ligne est deja sous les sondes, il suffit que le bon sens soit le plus proche des deux (journal Godot :
## a 35 deg, la voiture continuait de tourner plus d'une seconde avec la ligne sous les sondes).
var sortie_orbite_tolerance_deg := 70.0
# Direction de la ligne retrouvee (CROISEMENT)
var distance_observation_mm := 40.0  # distance parcourue avant de juger la direction de la ligne
var pente_ligne_tournee := -0.2      # glissement (mm de sondes par mm parcouru, vers le contournement) en dessous : a tourne
var sortie_rapide_mm := 70.0         # ligne perdue apres moins que ca : vue sous un angle fort, elle a tourne
var angle_ligne_tournee_deg := 15.0  # angle de la ligne balayee (par rapport au cap de depart) au-dela : a tourne
var croisement_max_mm := 150.0       # ligne perdue depuis plus que ca en CROISEMENT : le PID reprend la main
# Ligne coupee de face (TRAVERS)
var n_sondes_travers := 3
var rotation_travers_min_deg := 35.0 # avant, la ligne encore sous les sondes est celle que l'on vient de couper
var rotation_travers_max_deg := 150.0
# Dimensions (mm) : A MESURER sur la voiture et sur les obstacles de l'epreuve
var largeur_obstacle_min_mm := 100.0
## Dans le sens de la marche : on longe toujours au moins ca. 150 plutot que 200 : detour plus court, la ligne a moins
## le temps de tourner (simulation : +13 points pres des virages) ; les obstacles de 200 mm passent encore.
var longueur_obstacle_max_mm := 100.0   # 12 nov. : obstacle reel de 60 mm de profondeur (obstacle.blend) + marge ; 150 avant
var largeur_voiture_mm := 141.0   # modele 3D (Vehicule.ENCOMBREMENT)
var marge_laterale_mm := 40.0
var marge_longitudinale_mm := 60.0

var phase := Phase.ECART
var cote := 1
var termine := false
var echec := false
var hors_ligne := false           # evitement d'un obstacle qui n'est pas sur la ligne (demarrer_hors_ligne)

var _robot: ReglagesRobot
var _odo: Odometrie
var _x_requis := 0.0              # decalage lateral vise (mm, positif du cote du contournement)
var _perdu := false               # l'ultrason a perdu l'obstacle pendant ECART
var _d_vu := 0.0                  # derniere distance a laquelle l'obstacle a ete vu (mm)
var _e_reprise := NAN             # ecart lu par les sondes quand le PID reprend directement la main (mm)
var _e_vu := 0.0                  # dernier ecart lu par les sondes (mm)
var _obs_actif := false           # observation de la direction de la ligne en cours (CROISEMENT)
var _obs_depart := Vector2.ZERO   # position (odometrie) au debut de l'observation
var _obs_e := 0.0                 # ecart lu au debut de l'observation
var _obs_dernier := Vector2.ZERO  # position (odometrie) a la derniere vue de la ligne
var _zones: Array[Vector2] = []   # obstacles deja contournes : (y debut, y fin) dans le repere de la manoeuvre (mm)
var _centre := Vector2.ZERO       # centre estime de l'obstacle (repere de la manoeuvre, mm)
var _phi_debut := 0.0             # angle de depart autour de l'obstacle (orbite)
var _arc := 0.0                   # angle cumule parcouru autour de l'obstacle (rad)
var _phi_prec := 0.0
var _rayon_orbite := 300.0        # rayon de l'orbite en cours (mm)
var _n_orbite := 0                # 0 = premiere orbite, 1 = orbite elargie
var _n_echos := 0                 # pas consecutifs avec un echo brut devant
var _y_fin := 0.0                 # y de l'essieu a atteindre avant de revenir
var _x_ligne := 0.0               # position x de la ligne, reperee quand les sondes la revoient
var _cap_debut_recherche := 0.0
var _cote_travers := 1            # 1 = braquage a droite (phase TRAVERS)
var _cap_travers := 0.0

func _init(robot: ReglagesRobot, odo: Odometrie) -> void:
	_robot = robot
	_odo = odo

## d0_mm : distance de l'obstacle a l'arret. braquage_moyen : braquage moyen recent (servo - 90), pour le choix du cote.
func demarrer(d0_mm: float, braquage_moyen: float) -> void:
	_odo.reinitialiser()
	cote = cote_par_defaut
	if cote_interieur_virage and absf(braquage_moyen) >= seuil_virage_deg:
		cote = 1 if braquage_moyen > 0.0 else -1
	hors_ligne = false
	_nouvel_obstacle(d0_mm)
	_x_requis = largeur_obstacle_min_mm / 2.0 + largeur_voiture_mm / 2.0 + marge_laterale_mm
	termine = false
	echec = false
	_e_reprise = NAN
	_zones = [Vector2(d0_mm + _robot.longueur_avant_mm, d0_mm + _robot.longueur_avant_mm + longueur_obstacle_max_mm)]
	_centre = Vector2(0.0, d0_mm + _robot.longueur_avant_mm + longueur_obstacle_max_mm / 2.0)
	_n_echos = 0

## La voiture n'est pas sur la ligne : on contourne l'obstacle du cote demande (ECART, LONGER), puis on rend la main.
## Toute ligne vue en LONGER (sauf la ligne deja parcourue) est reprise ; aucune recherche de ligne derriere l'obstacle, elle n'y passe pas forcement.
func demarrer_hors_ligne(d0_mm: float, cote_voulu: int) -> void:
	demarrer(d0_mm, 0.0)
	cote = cote_voulu
	hors_ligne = true

func nom_phase() -> String:
	return Phase.keys()[phase] + (" (droite)" if cote > 0 else " (gauche)")

## Ecart de la ligne a la fin de la manoeuvre (mm, > 0 = a droite), pour le suivi si les sondes ne la voient pas.
func ecart_ligne_mm() -> float:
	if not is_nan(_e_reprise):
		return _e_reprise
	return clampf(_x_ligne - _odo.x, -SuiviLigne.ERREUR_MAX_MM, SuiviLigne.ERREUR_MAX_MM)

## La ligne vient d'etre vue et a pu tourner : on termine, le PID suit les sondes (e = ecart lu, mm).
func _reprise_par_pid(e: float) -> void:
	_e_reprise = e
	termine = true

## En LONGER : vrai si la ligne vue n'est pas plus loin sur le cote que le couloir prevu. A l'entree de LONGER, la voiture
## est encore inclinee et ses sondes sont 300 a 500 mm sur le cote : une ligne vue la est le plus souvent une autre partie
## de la piste (journal Godot du 5 novembre). La vraie ligne qui tourne vers la voiture traverse le couloir (vers _x_requis).
func _ligne_dans_couloir(erreur_mm: float) -> bool:
	var x_ligne := cote * (_odo.x + _robot.distance_sondes_mm * sin(_odo.cap) + erreur_mm * cos(_odo.cap))
	return x_ligne <= _x_requis + marge_couloir_mm

## En LONGER : vrai si la ligne vue hors du couloir est celle qui tourne en travers sous la voiture (obstacle dans un
## virage) : a hauteur de l'obstacle ou apres, a moins de largeur_passe_mm du cote, et pas sur la ligne entrante.
## (Pistes du journal Godot du 6 novembre rejouees au banc : rejetee, la voiture revenait vers l'ancienne ligne et se
## perdait ; +4 points d'obstacles franchis sur les 37 pistes.)
func _ligne_en_travers(erreur_mm: float) -> bool:
	var q := Vector2(_odo.x + _robot.distance_sondes_mm * sin(_odo.cap) + erreur_mm * cos(_odo.cap),
			_odo.y + _robot.distance_sondes_mm * cos(_odo.cap) - erreur_mm * sin(_odo.cap))
	return q.y >= _centre.y - longueur_obstacle_max_mm / 2.0 and cote * q.x <= _x_requis + largeur_passe_mm \
			and not _ligne_entrante(erreur_mm)

## En CROISEMENT : vrai si la ligne retrouvee n'est pas parallele au cap de depart (elle a tourne).
func _ligne_a_tourne(somme: float, n: int) -> bool:
	if not _obs_actif:
		_obs_actif = true
		_obs_depart = Vector2(_odo.x, _odo.y)
		_obs_e = _e_vu
		_obs_dernier = _obs_depart
	var p := Vector2(_odo.x, _odo.y)
	var parcouru := p.distance_to(_obs_depart)
	if n > 0:
		_e_vu = somme / n
		_obs_dernier = p
		return parcouru >= distance_observation_mm and cote * (_e_vu - _obs_e) / parcouru < pente_ligne_tournee
	# Une ligne parallele traverse toute la barre et la quitte du cote du contournement. Si elle la quitte de l'autre
	# cote, elle tourne dans le sens ou va la voiture (obstacle dans un virage) : l'alignement braquerait a l'envers
	# (journal Godot du 6 novembre, 19 h 17 : la voiture perdait la ligne juste apres l'avoir retrouvee).
	if parcouru < sortie_rapide_mm or cote * _e_vu < 0.0:
		return true
	# Angle de la ligne (repere de la manoeuvre) d'apres son balayage de la barre : cap + c * atan(c * de/ds).
	# Ligne parallele au cap de depart : ~0. Sinon elle a tourne : l'alignement suppose une ligne parallele et la
	# voiture filerait tout droit sans jamais la rejoindre (journal Godot du 6 novembre, 19 h 49).
	var balaye := _obs_dernier.distance_to(_obs_depart)
	if balaye >= distance_observation_mm:
		var angle_ligne := _odo.cap + cote * atan(cote * (_e_vu - _obs_e) / balaye)
		if absf(angle_ligne) > deg_to_rad(angle_ligne_tournee_deg):
			return true
	# Filet de securite : ligne perdue depuis trop longtemps sans que l'alignement commence
	return p.distance_to(_obs_dernier) > croisement_max_mm

## v_mm_s : vitesse actuelle ; d_mm : distance ultrason filtree (-1 = rien) ; d_brut_mm : derniere mesure brute.
## Retourne Vector2(angle servo, vitesse cible). Regarder `termine` et `echec` ensuite.
func calculer(somme: float, n: int, v_mm_s: float, d_mm: float, d_brut_mm := -1.0, _dt := 0.02) -> Vector2:
	var c := float(cote)
	var x := c * _odo.x               # decalage lateral, positif du cote du contournement
	var obstacle_devant := false
	_detecter_ligne_de_face(somme, n)
	if phase == Phase.LONGER or phase == Phase.RETOUR:
		obstacle_devant = _nouvel_obstacle_devant(d_mm, d_brut_mm)
	else:
		_n_echos = 0
	var angle := 90.0
	match phase:
		Phase.ECART:
			angle = _phase_ecart(c, x, d_mm, v_mm_s)
		Phase.LONGER:
			angle = _phase_longer(c, x, somme, n, d_mm, obstacle_devant)
		Phase.RETOUR:
			angle = _phase_retour(c, x, somme, n, d_mm, obstacle_devant)
		Phase.ORBITE:
			angle = _phase_orbite(c, somme, n)
		Phase.SORTIE_ORBITE:
			angle = _phase_sortie_orbite(c, somme, n)
		Phase.RECHERCHE:
			angle = _phase_recherche(c, somme, n)
		Phase.TRAVERS:
			angle = _phase_travers(somme, n)
		Phase.CROISEMENT:
			angle = _phase_croisement(c, somme, n, v_mm_s)
		Phase.ALIGNEMENT:
			angle = _phase_alignement(c, v_mm_s)
	return Vector2(angle, float(vitesse_manoeuvre))


# ----- Phases (chacune retourne l'angle servo et change `phase`, `termine` ou `echec` si besoin)

func _phase_ecart(c: float, x: float, d_mm: float, v_mm_s: float) -> float:
	var angle := _angle_vers_cap(c * deg_to_rad(cap_evitement_deg))
	var vu := d_mm >= 0.0 and d_mm < distance_vue_mm
	if vu:
		_perdu = false
		_d_vu = d_mm
	elif not _perdu:
		_perdu = true
		# l'obstacle vient de sortir du cone : son bord est au plus sur le rayon extreme (cap - demi-angle)
		var cap_c := c * _odo.cap
		var x_ultrason := x + _robot.longueur_avant_mm * sin(cap_c)
		var bord := x_ultrason + _d_vu * sin(cap_c - deg_to_rad(_robot.demi_angle_ultrason_deg))
		_x_requis = maxf(_x_requis, bord + largeur_voiture_mm / 2.0 + marge_laterale_mm)
	# le redressement ajoute encore R * (1 - cos(cap)) de decalage, plus le trajet pendant que le servo braque
	var anticipation := _rayon_virage_mm() * (1.0 - cos(_odo.cap)) \
			+ _retard_mm(90.0 - c * braquage_manoeuvre, v_mm_s) * absf(sin(_odo.cap))
	if _perdu and x >= _x_requis - anticipation:
		phase = Phase.LONGER
	elif x > decalage_max_mm:
		echec = true
	return angle


func _phase_longer(c: float, x: float, somme: float, n: int, d_mm: float, obstacle_devant: bool) -> float:
	var corr := clampf((_x_requis - x) * gain_lateral,
			-deg_to_rad(correction_laterale_max_deg), deg_to_rad(correction_laterale_max_deg))
	var angle := _angle_vers_cap(c * corr)
	if hors_ligne and n > 0 and not _ligne_entrante(somme / n):
		_reprise_par_pid(somme / n)         # pas la ligne deja parcourue : la voiture repartirait en arriere
	elif hors_ligne and _odo.y >= _y_fin:
		_e_reprise = c * SuiviLigne.ERREUR_MAX_MM
		termine = true
	elif n > 0 and (_ligne_dans_couloir(somme / n) or _ligne_en_travers(somme / n)):
		_reprise_par_pid(somme / n)         # pres d'un virage, la ligne revient sous la voiture
	elif obstacle_devant:
		_relancer_ecart(d_mm)
	elif _odo.y >= _y_fin:
		phase = Phase.RETOUR
	return angle


func _phase_retour(c: float, x: float, somme: float, n: int, d_mm: float, obstacle_devant: bool) -> float:
	var angle := _angle_vers_cap(-c * deg_to_rad(cap_retour_deg))
	if n > 0:
		_reperer_ligne(somme / n)
		phase = Phase.CROISEMENT
	elif obstacle_devant:
		_relancer_ecart(d_mm)
	elif x < -depassement_max_mm:
		_cap_debut_recherche = _odo.cap
		if recherche_en_orbite:
			_phi_debut = _angle_autour(Vector2(_odo.x, _odo.y))
			_phi_prec = _phi_debut
			_arc = 0.0
			_rayon_orbite = rayon_orbite_mm
			_n_orbite = 0
			phase = Phase.ORBITE
		else:
			phase = Phase.RECHERCHE
	return angle


func _phase_orbite(c: float, somme: float, n: int) -> float:
	var p := Vector2(_odo.x, _odo.y)
	var phi := _angle_autour(p)
	var r := p.distance_to(_centre)
	# en contournant par la droite, on tourne autour de l'obstacle dans le sens trigonometrique
	var cap_tangent := -phi if cote > 0 else PI - phi
	var corr := clampf(gain_rayon_orbite * (r - _rayon_orbite), -deg_to_rad(correction_orbite_max_deg), deg_to_rad(correction_orbite_max_deg))
	var servo := _robot.roues_vers_servo(rad_to_deg(wrapf(cap_tangent - c * corr - _odo.cap, -PI, PI)) * gain_cap)
	var angle := 90.0 + clampf(servo, -braquage_manoeuvre, braquage_manoeuvre)
	_arc += absf(wrapf(phi - _phi_prec, -PI, PI))   # angle cumule : peut depasser 180 deg
	_phi_prec = phi
	if n > 0 and not _ligne_entrante(somme / n):
		_e_vu = somme / n
		phase = Phase.SORTIE_ORBITE
	elif n == 0 and _arc > deg_to_rad(arc_orbite_max_deg):
		if _n_orbite == 0:
			_n_orbite = 1                   # on ne se decourage pas : 2e orbite, plus large
			_rayon_orbite += rayon_orbite_plus_mm
			_arc = 0.0
		else:
			echec = true
	return angle


## Tourne vers l'exterieur jusqu'a pointer a l'oppose de l'obstacle, puis le PID suit la ligne dans ce sens.
func _phase_sortie_orbite(c: float, somme: float, n: int) -> float:
	var p := Vector2(_odo.x, _odo.y)
	var cap_sortie := atan2(p.x - _centre.x, p.y - _centre.y)
	if n > 0:
		_e_vu = somme / n
	if absf(wrapf(cap_sortie - _odo.cap, -PI, PI)) < deg_to_rad(sortie_orbite_tolerance_deg):
		_reprise_par_pid(_e_vu)
		return 90.0
	return 90.0 + c * braquage_manoeuvre


## La ligne a tourne : on tourne serre vers le cote d'ou l'on vient.
func _phase_recherche(c: float, somme: float, n: int) -> float:
	if n > 0:
		_reprise_par_pid(somme / n)
	elif absf(_odo.cap - _cap_debut_recherche) > deg_to_rad(recherche_max_deg):
		echec = true
	return 90.0 - c * braquage_manoeuvre


func _phase_travers(somme: float, n: int) -> float:
	var rotation_faite := absf(wrapf(_odo.cap - _cap_travers, -PI, PI))
	if (rotation_faite >= deg_to_rad(rotation_travers_min_deg) and n > 0 and n <= 2) \
			or rotation_faite >= deg_to_rad(rotation_travers_max_deg):
		_reprise_par_pid(somme / n if n > 0 else _cote_travers * SuiviLigne.ERREUR_MAX_MM)
	return 90.0 + _cote_travers * braquage_manoeuvre


func _phase_croisement(c: float, somme: float, n: int, v_mm_s: float) -> float:
	if _ligne_a_tourne(somme, n):
		_reprise_par_pid(_e_vu)
		return 90.0
	# tout droit : le virage d'alignement decale encore l'essieu de R * (1 - cos(cap)) vers la ligne
	var avance_braquage := _retard_mm(90.0 + c * braquage_manoeuvre, v_mm_s) * absf(sin(_odo.cap))
	if c * (_odo.x - _x_ligne) <= _rayon_virage_mm() * (1.0 - cos(_odo.cap)) + avance_braquage:
		phase = Phase.ALIGNEMENT
	return 90.0


## Braquage constant : on finit dans l'axe, l'essieu arriere sur la ligne.
func _phase_alignement(c: float, v_mm_s: float) -> float:
	if c * _odo.cap + _inertie_cap(v_mm_s) >= -deg_to_rad(3.0):
		termine = true
	return 90.0 + c * braquage_manoeuvre


# ----- Outils

## Passe en TRAVERS si la ligne de sortie est coupee de face. Cote du virage : celui qui s'eloigne de l'obstacle.
func _detecter_ligne_de_face(somme: float, n: int) -> void:
	if hors_ligne or n < n_sondes_travers or not phase in [Phase.LONGER, Phase.RETOUR, Phase.CROISEMENT, Phase.ALIGNEMENT, Phase.ORBITE, Phase.SORTIE_ORBITE]:
		return
	var e := somme / n
	if _ligne_entrante(e) or (phase == Phase.LONGER and not _ligne_en_travers(e)):
		return
	var droite := Vector2(cos(_odo.cap), -sin(_odo.cap))
	var q := Vector2(_odo.x + _robot.distance_sondes_mm * sin(_odo.cap), _odo.y + _robot.distance_sondes_mm * cos(_odo.cap)) + droite * e
	_cote_travers = 1 if (q - _centre).dot(droite) > 0.0 else -1
	_cap_travers = _odo.cap
	phase = Phase.TRAVERS

## Un obstacle est vu a d_mm devant : il faudra le depasser entierement avant de revenir.
func _nouvel_obstacle(d_mm: float) -> void:
	_y_fin = _odo.y + d_mm + _robot.longueur_avant_mm + longueur_obstacle_max_mm + marge_longitudinale_mm
	_perdu = false
	_d_vu = d_mm
	phase = Phase.ECART

## Obstacle devant pendant la manoeuvre : on s'ecarte encore, a partir d'ici.
func _relancer_ecart(d_mm: float) -> void:
	_x_requis = cote * _odo.x + largeur_obstacle_min_mm / 2.0 + largeur_voiture_mm / 2.0 + marge_laterale_mm
	var y0 := _odo.y + d_mm + _robot.longueur_avant_mm
	_zones.append(Vector2(y0, y0 + longueur_obstacle_max_mm))
	_nouvel_obstacle(d_mm)

## Vrai si un NOUVEL obstacle est devant (LONGER, RETOUR) : echo proche, hors de la zone d'un obstacle deja
## contourne, et confirme sur plusieurs pas de mesure brute.
func _nouvel_obstacle_devant(d_mm: float, d_brut_mm: float) -> bool:
	if not (d_mm >= 0.0 and d_mm < distance_obstacle_suivant_mm) or _echo_connu(d_mm):
		_n_echos = 0
		return false
	_n_echos = _n_echos + 1 if d_brut_mm >= 0.0 else 0
	return _n_echos >= echos_confirmation

## Vrai si le point d'echo (a d_mm devant l'ultrason) tombe sur un obstacle deja contourne.
func _echo_connu(d_mm: float) -> bool:
	var portee := _robot.longueur_avant_mm + d_mm
	var y_echo := _odo.y + portee * cos(_odo.cap)
	var x_echo := cote * (_odo.x + portee * sin(_odo.cap))     # positif du cote du contournement
	var bord := _x_requis - largeur_voiture_mm / 2.0 - marge_laterale_mm
	if x_echo > bord + tolerance_memoire_mm:
		return false
	for z in _zones:
		if y_echo >= z.x - tolerance_memoire_mm and y_echo <= z.y + tolerance_memoire_mm:
			return true
	return false

## Angle (sens trigonometrique, depuis +x) du point p vu du centre de l'obstacle.
func _angle_autour(p: Vector2) -> float:
	return atan2(p.y - _centre.y, p.x - _centre.x)

## Vrai si la ligne vue par les sondes est la ligne entrante (sur le chemin suivi avant l'arret) : on ne la reprend pas.
## Sans memoire (piste Blender demarree pres d'un obstacle...), on se rabat sur "derriere l'obstacle, dans l'axe".
func _ligne_entrante(erreur_mm: float) -> bool:
	var q := Vector2(_odo.x + _robot.distance_sondes_mm * sin(_odo.cap) + erreur_mm * cos(_odo.cap),
			_odo.y + _robot.distance_sondes_mm * cos(_odo.cap) - erreur_mm * sin(_odo.cap))
	if chemin_entrant.is_empty():
		return absf(wrapf(_angle_autour(q) + PI / 2.0, -PI, PI)) < deg_to_rad(35.0)
	for p in chemin_entrant:
		if q.distance_squared_to(p) < distance_entrante_mm * distance_entrante_mm:
			return true
	return false

## Position x de la ligne (repere odometrie) a partir de l'ecart lu par les sondes (mm, > 0 = a droite).
## La barre de sondes est a distance_sondes_mm devant l'essieu, perpendiculaire au cap.
func _reperer_ligne(erreur_mm: float) -> void:
	_x_ligne = _odo.x + _robot.distance_sondes_mm * sin(_odo.cap) + erreur_mm * cos(_odo.cap)
	_e_vu = erreur_mm
	_obs_actif = false                    # la direction de la ligne sera observee en CROISEMENT

## Angle servo pour rejoindre le cap voulu (rad), braquage limite a braquage_manoeuvre.
func _angle_vers_cap(cap_voulu: float) -> float:
	var servo := _robot.roues_vers_servo(rad_to_deg(cap_voulu - _odo.cap) * gain_cap)
	return 90.0 + clampf(servo, -braquage_manoeuvre, braquage_manoeuvre)

## Cap (rad, en valeur absolue) que la voiture prendra encore pendant que le servo revient a 90.
func _inertie_cap(v_mm_s: float) -> float:
	var delta := deg_to_rad(_robot.servo_vers_roues_deg(_odo.servo_estime - 90.0))
	return absf(v_mm_s / _robot.empattement_mm * tan(delta) / 2.0 * _odo.duree_servo(90.0))

## Distance (mm) parcourue avant que le braquage vers `cible` soit etabli.
func _retard_mm(cible: float, v_mm_s: float) -> float:
	return v_mm_s * _odo.duree_servo(cible) / 2.0

## Rayon de braquage (mm) de l'essieu arriere avec braquage_manoeuvre.
func _rayon_virage_mm() -> float:
	var delta := deg_to_rad(_robot.servo_vers_roues_deg(braquage_manoeuvre))
	return _robot.empattement_mm / tan(delta)
