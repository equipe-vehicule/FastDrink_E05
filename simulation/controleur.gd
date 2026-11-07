## ALGORITHMES DU VEHICULE : suivi de ligne PID + arret devant obstacle + contournement.
## Entrees  : sondes (5 valeurs 0/1, de gauche a droite), distance_cm (ultrason, -1 si rien), dt (s)
## Sorties  : {"vitesse": -100..100, "angle": 45..135}
## Ces fichiers ne dependent pas de Godot : la meme logique peut tourner sur le vrai robot.
##
## Exigences :
##   - Arreter a une distance predeterminee d'un obstacle (precision +/- 30 mm).   -> ArretObstacle
##   - Contourner des obstacles en quittant la ligne-guide au sol.                 -> Contournement
##   - Suivre la ligne-guide en reculant sur de courtes distances (< 200 mm).      -> ReculLigne
##   Suivi de ligne : SuiviLigne. Mesures : FiltreUltrason, Odometrie. Robot : ReglagesRobot.
##
## Machine a etats (ce fichier ne fait que choisir qui commande la voiture) :
##   SUIVI         : suit la ligne ; un obstacle sur le chemin fait freiner et s'arreter a distance_manoeuvre_cm.
##   DECISION      : immobile pause_s ; choisit le cote (interieur du virage en cours, sinon cote_par_defaut).
##   CONTOURNEMENT : Contournement commande (ECART, LONGER, RETOUR, RECHERCHE, CROISEMENT, ALIGNEMENT) -> SUIVI.
##   BLOQUE        : arret de securite (manoeuvre impossible, phase trop longue) ; repart si la ligne est revue.
##   DEMI_TOUR     : la voiture roule a l'envers sur la ligne (voir ci-dessous) : elle pivote d'environ 180 deg
##                   (avant braquage maximal, arriere braquage oppose, plusieurs fois) puis reprend le suivi.
##   RECUL_LIGNE / ARRET : recul en suivant la ligne, puis arret.
##
## Sens de marche : apres un contournement pres d'une epingle, la ligne retrouvee peut partir dans l'autre sens et le
## PID la suit a l'envers (journal Godot du 6 novembre : 8 reprises sur 20). La piste ne se croisant jamais, une
## voiture qui roule a l'envers finit toujours devant un obstacle qu'elle a deja contourne, vu de dos. On memorise
## donc la position de chaque obstacle contourne (odometrie globale) : s'arreter devant l'un d'eux = demi-tour.
##
## Detection : en virage, l'ultrason vise tout droit, hors de la piste. Un echo plus loin que la distance ou
## la ligne sort du cone du capteur vient d'un objet qui n'est pas sur le chemin : il est ignore (_portee_utile_mm).
class_name Controleur
extends RefCounted

## ARRIVEE : la voiture a franchi la barre d'arrivee (5 sondes en travers, voir SuiviLigne) : arret definitif.
## RETROUVER : ligne perdue en suivi depuis retrouver_apres_s (virage rate apres un contournement...) : au lieu de
##   s'arreter, la voiture retourne vers le dernier point de ligne vu (odometrie globale). La ligne passe par ce point :
##   en le visant avec la barre de sondes, les sondes la recroisent. Cible derriere : marche arriere, braquage oppose
##   (la voiture pivote vers elle). (Journal Godot du 6 novembre, 19 h 17 : la voiture tournait en rond 8 s puis
##   s'arretait seule.)
enum Etat { SUIVI, DECISION, BLOQUE, ARRET, CONTOURNEMENT, RECUL_LIGNE, DEMI_TOUR, ARRIVEE, RETROUVER }

const TIMEOUT_ETAT_S := 10.0      # securite : une phase de manoeuvre ne dure jamais plus longtemps
const LISSAGE_BRAQUAGE_S := 0.5   # constante de temps du braquage moyen (detection et choix du cote)

# ----- Sens de marche et demi-tour
var rayon_meme_obstacle_mm := 200.0  # echo a moins de ca d'un obstacle deja contourne = c'est lui, vu de dos
var obstacles_memorises := 8         # derniers obstacles contournes gardes en memoire
var demi_tour_avant_s := 1.0         # duree de chaque marche avant (braquage maximal)
var demi_tour_arriere_s := 1.0       # duree de chaque marche arriere (braquage oppose)
var vitesse_demi_tour := 18
var demi_tour_max_s := 20.0

# ----- Ligne perdue en suivi : retour vers le dernier point vu
var retrouver_apres_s := 4.0         # ligne perdue depuis plus longtemps : RETROUVER (doit rester < suivi.abandon_s)
var retrouver_max_s := 30.0          # ensuite BLOQUE
var vitesse_retrouver := 18
var gain_retrouver := 2.0            # degres de roue par degre d'ecart de direction

# ----- Modules (leurs reglages s'ecrivent par ex. controleur.suivi.kp = 2.0)
var robot := ReglagesRobot.new()
var odo := Odometrie.new(robot)
var suivi := SuiviLigne.new()
var ultrason := FiltreUltrason.new()
var arret := ArretObstacle.new(robot)
var contournement := Contournement.new(robot, odo)
var recul := ReculLigne.new(robot, odo, suivi)

var _etat := Etat.SUIVI
var _t_etat := 0.0
var _t_arret := 0.0
var _cmd := 0.0
var _braquage_moyen := 0.0        # servo - 90, moyenne glissante pendant le suivi (> 0 = virage a droite)
var distance_filtree_mm := -1.0   # derniere distance ultrason apres filtrage (journal)
var _obstacles_contournes: Array[Vector2] = []   # centres des obstacles contournes, repere global (mm)
var _memoire_ligne: Array[Vector2] = []          # points de la ligne vus par les sondes, dernier metre (repere global)
const PAS_MEMOIRE_MM := 20.0
const POINTS_MEMOIRE := 50                       # 50 x 20 mm = 1 m de ligne entrante
var _cap_debut_demi_tour := 0.0
var _cote_demi_tour := 1
var _cible_retrouver := Vector2.ZERO


func calculer(sondes: Array, distance_cm: float, dt: float) -> Dictionary:
	dt = maxf(dt, 0.001)
	var ligne := SuiviLigne.lire(sondes)
	var somme := ligne.x
	var n := int(ligne.y)
	var d_mm := ultrason.filtrer(distance_cm, dt)
	distance_filtree_mm = d_mm

	if _etat == Etat.CONTOURNEMENT and contournement.phase in [Contournement.Phase.ORBITE, Contournement.Phase.SORTIE_ORBITE]:
		_t_etat = minf(_t_etat, TIMEOUT_ETAT_S / 2.0)   # un tour d'orbite dure plus longtemps qu'une phase ordinaire
	if _en_manoeuvre() and _t_etat > TIMEOUT_ETAT_S:
		_changer_etat(Etat.BLOQUE)      # odometrie perdue : on s'arrete plutot que de partir au hasard

	var r := Vector2(90.0, 0.0)         # (angle servo, vitesse cible)
	match _etat:
		Etat.SUIVI:
			r = _etat_suivi(sondes, d_mm, dt)

		Etat.DECISION:
			if d_mm < 0.0:
				_changer_etat(Etat.SUIVI)       # l'obstacle a disparu
			elif _t_etat >= contournement.pause_s:
				if _obstacle_deja_contourne(d_mm):
					_demarrer_demi_tour()               # on roule a l'envers : demi-tour plutot que recontourner
				else:
					var centre := odo.point_devant_global(robot.longueur_avant_mm + d_mm + contournement.longueur_obstacle_max_mm / 2.0)
					_obstacles_contournes.append(centre)
					if _obstacles_contournes.size() > obstacles_memorises:
						_obstacles_contournes.pop_front()
					contournement.demarrer(d_mm, _braquage_moyen)
					contournement.chemin_entrant = _memoire_vers_manoeuvre()
					_changer_etat(Etat.CONTOURNEMENT)

		Etat.DEMI_TOUR:
			r = _demi_tour(n)

		Etat.RETROUVER:
			r = _retrouver(somme, n)

		Etat.ARRIVEE:
			pass                                # arrete sur la ligne d'arrivee

		Etat.BLOQUE:
			# arret de securite ; on repart si la voiture est reposee sur la ligne
			if n > 0:
				_changer_etat(Etat.SUIVI)

		Etat.ARRET:
			pass                                # immobile jusqu'a reprendre() ou demarrer_recul()

		Etat.CONTOURNEMENT:
			var phase_avant := contournement.phase
			var d_brut := distance_cm * 10.0 if distance_cm >= 0.0 else -1.0   # non filtre : pour confirmer un echo
			r = contournement.calculer(somme, n, _cmd * robot.mm_s_par_cmd, d_mm, d_brut, dt)
			if contournement.termine:
				suivi.derniere_erreur = contournement.ecart_ligne_mm()
				_changer_etat(Etat.SUIVI)
			elif contournement.echec:
				_changer_etat(Etat.BLOQUE)
			elif contournement.phase != phase_avant:
				_t_etat = 0.0                   # le timeout compte pour chaque phase

		Etat.RECUL_LIGNE:
			r = recul.calculer(somme, n, dt, _cmd)
			if recul.fin == ReculLigne.Fin.TERMINE:
				_changer_etat(Etat.SUIVI if recul.reprendre_apres_recul else Etat.ARRET)
			elif recul.fin == ReculLigne.Fin.LIGNE_PERDUE:
				_changer_etat(Etat.ARRET)

	var angle := r.x
	var cible := r.y

	# Arret d'urgence pendant la manoeuvre (obstacle imprevu tout pres) : on attend, sans faire courir le timeout
	var urgence := _en_manoeuvre() and cible > 0.0 and d_mm >= 0.0 and d_mm < contournement.distance_securite_mm
	if urgence:
		cible = 0.0
	else:
		_t_etat += dt

	# Sorties bornees, acceleration limitee
	angle = clampf(angle, ReglagesRobot.ANGLE_MIN, ReglagesRobot.ANGLE_MAX)
	_cmd = move_toward(_cmd, cible, robot.rampe_cmd_par_s * dt)
	var angle_servo := int(round(angle))
	odo.suivre_servo(float(angle_servo), dt)   # toujours : l'angle reel du servo est connu au debut d'une manoeuvre
	odo.integrer_global(_cmd * robot.mm_s_par_cmd, dt)   # toujours : position globale (sens de marche)
	if n > 0 and (_etat == Etat.SUIVI or _ligne_reprise()):
		_memoriser_ligne(somme / n)
	if _en_manoeuvre():
		odo.integrer(_cmd * robot.mm_s_par_cmd, dt)
	if _etat == Etat.SUIVI and absf(_cmd) > 1.0:
		_braquage_moyen = lerpf(_braquage_moyen, angle_servo - 90.0, minf(1.0, dt / LISSAGE_BRAQUAGE_S))
	return {"vitesse": int(round(_cmd)), "angle": angle_servo}


## Garde le point de la ligne vu par les sondes (repere global), un tous les PAS_MEMOIRE_MM, sur le dernier metre.
func _memoriser_ligne(erreur_mm: float) -> void:
	var d := robot.distance_sondes_mm
	var c := odo.cap_global
	var p := Vector2(odo.x_global + d * sin(c) + erreur_mm * cos(c), odo.y_global + d * cos(c) - erreur_mm * sin(c))
	if _memoire_ligne.is_empty() or p.distance_to(_memoire_ligne[-1]) > PAS_MEMOIRE_MM:
		_memoire_ligne.append(p)
		if _memoire_ligne.size() > POINTS_MEMOIRE:
			_memoire_ligne.pop_front()


## Vrai pendant les phases du contournement ou les sondes voient la ligne de sortie (pas la ligne entrante).
func _ligne_reprise() -> bool:
	return _etat == Etat.CONTOURNEMENT and contournement.phase in [Contournement.Phase.CROISEMENT,
			Contournement.Phase.ALIGNEMENT, Contournement.Phase.SORTIE_ORBITE]


## Vise le dernier point de ligne vu avec la barre de sondes ; des qu'une sonde voit la ligne, le suivi reprend.
func _retrouver(somme: float, n: int) -> Vector2:
	if n > 0:
		var e := somme / n
		_changer_etat(Etat.SUIVI)
		suivi.derniere_erreur = e if e != 0.0 else 1.0
		return Vector2(90.0, 0.0)
	if _t_etat > retrouver_max_s:
		_changer_etat(Etat.BLOQUE)
		return Vector2(90.0, 0.0)
	var vers := _cible_retrouver - Vector2(odo.x_global, odo.y_global)
	var rel := wrapf(atan2(vers.x, vers.y) - odo.cap_global, -PI, PI)
	if absf(rel) > deg_to_rad(100.0):
		# cible derriere : marche arriere braquage oppose, la voiture pivote vers elle
		return Vector2(90.0 - 45.0 * signf(rel), -float(vitesse_retrouver))
	var servo := robot.roues_vers_servo(rad_to_deg(rel) * gain_retrouver)
	return Vector2(90.0 + clampf(servo, -45.0, 45.0), float(vitesse_retrouver))


## La ligne entrante memorisee, exprimee dans le repere de la manoeuvre qui commence (origine : essieu arriere,
## y vers l'avant, x vers la droite).
func _memoire_vers_manoeuvre() -> Array[Vector2]:
	var c := odo.cap_global
	var avant := Vector2(sin(c), cos(c))
	var droite := Vector2(cos(c), -sin(c))
	var origine := Vector2(odo.x_global, odo.y_global)
	var res: Array[Vector2] = []
	for p in _memoire_ligne:
		res.append(Vector2((p - origine).dot(droite), (p - origine).dot(avant)))
	return res


## Vrai si l'echo a d_mm devant tombe sur un obstacle deja contourne : la voiture le revoit de dos, elle roule a l'envers.
func _obstacle_deja_contourne(d_mm: float) -> bool:
	var echo := odo.point_devant_global(robot.longueur_avant_mm + d_mm)
	for o in _obstacles_contournes:
		if echo.distance_to(o) < rayon_meme_obstacle_mm:
			return true
	return false


func _demarrer_demi_tour() -> void:
	_changer_etat(Etat.DEMI_TOUR)
	_cap_debut_demi_tour = odo.cap_global
	_cote_demi_tour = 1 if suivi.derniere_erreur >= 0.0 else -1


## Pivote sur place en plusieurs manoeuvres (avant braquage maximal, arriere braquage oppose : la voiture tourne
## toujours du meme cote) jusqu'a environ 180 deg, puis reprend le suivi des que la ligne est vue.
func _demi_tour(n: int) -> Vector2:
	var tourne := absf(odo.cap_global - _cap_debut_demi_tour)
	if (tourne >= deg_to_rad(150.0) and n > 0) or tourne >= deg_to_rad(210.0) or _t_etat > demi_tour_max_s:
		suivi.derniere_erreur = 38.0 * _cote_demi_tour
		_changer_etat(Etat.SUIVI)
		return Vector2(90.0, 0.0)
	var periode := demi_tour_avant_s + demi_tour_arriere_s
	if fmod(_t_etat, periode) < demi_tour_avant_s:
		return Vector2(90.0 + 45.0 * _cote_demi_tour, float(vitesse_demi_tour))
	return Vector2(90.0 - 45.0 * _cote_demi_tour, -float(vitesse_demi_tour))


## Recule en suivant la ligne sur distance_mm (limite a ReculLigne.RECUL_MAX_MM).
## La voiture doit etre sur la ligne et a peu pres dans son axe : ce cap sert de reference (cap = 0).
func demarrer_recul(distance_mm: float) -> void:
	recul.demarrer(distance_mm)
	_changer_etat(Etat.RECUL_LIGNE)


## Nom de l'etat courant, a afficher dans le HUD pour verifier ce que fait la voiture.
func nom_etat() -> String:
	if _etat == Etat.CONTOURNEMENT:
		return "CONTOURNEMENT " + contournement.nom_phase()
	if _etat == Etat.SUIVI or _etat == Etat.DECISION:
		# le braquage moyen decide du cote de contournement en virage (> 0 = droite) : utile pour comprendre le choix
		return "%s (ligne %s, braquage moyen %+.0f)" % [Etat.keys()[_etat], suivi.nom_ligne(), _braquage_moyen]
	return Etat.keys()[_etat]


## Nom court de l'etat (journal).
func etat_court() -> String:
	return Etat.keys()[_etat]


## Braquage moyen recent pendant le suivi (servo - 90, > 0 = virage a droite).
func braquage_moyen() -> float:
	return _braquage_moyen


## Repart en suivi de ligne (marche avant) apres un arret en fin de recul.
func reprendre() -> void:
	_changer_etat(Etat.SUIVI)


## Remet le controleur a zero (nouveau depart, sortie de BLOQUE).
func reinitialiser() -> void:
	_cmd = 0.0
	_braquage_moyen = 0.0
	suivi.derniere_erreur = 0.0
	ultrason.reinitialiser()
	_changer_etat(Etat.SUIVI)


## Suivi de ligne, avec arret (ou recul) devant un obstacle situe sur le chemin. Retourne Vector2(angle, vitesse cible).
func _etat_suivi(sondes: Array, d_mm: float, dt: float) -> Vector2:
	var r := suivi.calculer(sondes, dt)
	if suivi.ligne == SuiviLigne.Ligne.PERDUE and suivi.temps_sans_ligne() > retrouver_apres_s and not _memoire_ligne.is_empty():
		_cible_retrouver = _memoire_ligne[-1]
		_changer_etat(Etat.RETROUVER)
		return Vector2(90.0, 0.0)
	if suivi.arrivee:
		_changer_etat(Etat.ARRIVEE)
		return Vector2(90.0, 0.0)
	var angle := r.x
	var cible := r.y
	var contourner := contournement.actif and recul.distance_auto_mm <= 0.0
	var d_chemin := d_mm if d_mm >= 0.0 and d_mm <= _portee_utile_mm() else -1.0
	if d_chemin >= 0.0:
		var d_voulue_cm := contournement.distance_manoeuvre_cm if contourner else arret.distance_arret_cm
		cible = arret.limiter(d_chemin, cible, d_voulue_cm * 10.0)
	else:
		arret.relacher()
	if arret.arrete or arret.en_recul:
		suivi.geler()                       # pas d'accumulation pendant l'arret
	if cible < 0.0:
		angle = 90.0                        # recul en ligne droite
	var immobile := arret.arrete and absf(_cmd) < 0.5
	_t_arret = _t_arret + dt if immobile else 0.0
	if recul.distance_auto_mm > 0.0 and _t_arret >= recul.attente_auto_s:
		demarrer_recul(recul.distance_auto_mm)
	elif contourner and immobile:
		_changer_etat(Etat.DECISION)
	return Vector2(angle, cible)


## Distance jusqu'a laquelle un echo peut venir d'un obstacle pose sur la ligne.
## En virage de rayon R (deduit du braquage moyen), la ligne s'ecarte de l'axe du capteur d'environ y^2 / (2 R)
## a la distance y ; elle reste dans le cone tant que y^2 / (2 R) < y * tan(demi-angle) + demi-largeur d'obstacle.
func _portee_utile_mm() -> float:
	var delta := deg_to_rad(absf(robot.servo_vers_roues_deg(_braquage_moyen)))
	if delta < deg_to_rad(1.0):
		return INF                          # ligne droite : toute la portee
	var rayon := robot.empattement_mm / tan(delta)
	var a := rayon * tan(deg_to_rad(robot.demi_angle_ultrason_deg))
	return a + sqrt(a * a + rayon * contournement.largeur_obstacle_min_mm)


## Vrai pendant les etats ou la position est suivie par odometrie.
func _en_manoeuvre() -> bool:
	return _etat == Etat.CONTOURNEMENT or _etat == Etat.RECUL_LIGNE


func _changer_etat(e: Etat) -> void:
	_etat = e
	_t_etat = 0.0
	if e == Etat.SUIVI:
		suivi.reinitialiser()
		arret.relacher()
