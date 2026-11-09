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
## Hors de la ligne (suivi avec ligne perdue, RETROUVER, ORBITE, RECHERCHE) : un obstacle devant est evite lui aussi,
## sans supposer que la ligne passe dessous (Contournement.demarrer_hors_ligne), puis RETROUVER ramene vers la ligne.
##
## En suivi, si la ligne sous les sondes est un morceau deja parcouru (4 derniers metres) remonte en sens inverse sur
## contresens_max_mm : demi-tour, quelle que soit la raison pour laquelle la voiture s'est retournee.
##
## Detection : en virage, l'ultrason vise tout droit, hors de la piste. Un echo plus loin que la distance ou
## la ligne sort du cone du capteur vient d'un objet qui n'est pas sur le chemin : il est ignore (_portee_utile_mm).
class_name Controleur
extends RefCounted

## ARRIVEE : la voiture a franchi la barre d'arrivee (5 sondes en travers, voir SuiviLigne) : arret definitif.
## RETROUVER : ligne perdue en suivi depuis retrouver_apres_s (virage rate apres un contournement...) : au lieu de
##   s'arreter, la voiture va, toujours en avancant, devant le dernier point de ligne vu (dans le sens de marche), puis cherche en
##   spirale jusqu'a croiser la ligne. Si elle la reprend a contresens, demi-tour. (Journal Godot du 6 novembre, 19 h 17 :
##   la voiture tournait en rond 8 s puis s'arretait seule.)
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
var retrouver_apres_s := 9.0         # pas moins : en virage serre, la voiture retrouve la ligne seule en tournant (a 4 s, elle bouclait)
var retrouver_max_s := 90.0          # ensuite BLOQUE
var approche_max_s := 15.0           # ensuite, ou une fois la cible atteinte : recherche en spirale
var avance_cible_mm := 200.0         # la cible est devant le dernier point vu, dans le sens de marche : jamais en arriere
var rayon_cible_mm := 80.0           # cible consideree comme atteinte
var spirale_s := 8.0                 # plus grand = la spirale s'ouvre plus lentement
var rayon_meme_ligne_mm := 100.0     # ligne vue a moins de ca du chemin deja suivi : c'est la ligne entrante
var vitesse_retrouver := 18
var gain_retrouver := 2.0            # degres de roue par degre d'ecart de direction

# ----- Obstacle vu alors que la voiture n'est pas sur la ligne
var hors_ligne_apres_s := 0.5        # ligne perdue depuis ca : la voiture n'est plus sur la ligne
var portee_hors_ligne_mm := 500.0    # hors ligne, tout echo plus proche que ca est sur son chemin
var echos_hors_ligne := 4            # pas consecutifs avec echo brut avant d'interrompre une orbite ou une recherche
var rayon_obstacle_recent_mm := 400.0  # si pres de l'obstacle evite, jamais de marche arriere (pas de capteur derriere)

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
var _evitement_hors_ligne := false   # la prochaine DECISION lance un evitement hors ligne
var _obstacle_recent := Vector2(INF, INF)   # dernier obstacle evite hors ligne (repere global)
var _n_echos_hors_ligne := 0
var _t_spirale := -1.0               # < 0 : la voiture va encore vers la cible
var _cote_spirale := 1
# Surveillance du sens de marche : chemin deja parcouru sur la ligne (repere global), un point tous les PAS_MEMOIRE_MM
const POINTS_CHEMIN := 200                       # 4 m
const POINTS_CHEMIN_RECENTS := 15                # les derniers points (300 mm) ne comptent pas : la voiture vient d'y passer
var rayon_chemin_mm := 40.0                      # ligne vue a moins de ca d'un point deja parcouru
var contresens_max_mm := 70.0                    # distance parcourue a contresens avant le demi-tour
var _chemin: Array[Vector2] = []
var _contresens_mm := 0.0
const DEMI_TOURS_MEMORISES := 6
var rayon_meme_demi_tour_mm := 300.0
var demi_tours_max_au_meme_endroit := 2          # ensuite on continue : tourner sur place ne resoudrait rien
var _demi_tours: Array[Vector2] = []
var _points_passage := 0                         # points ajoutes a _chemin depuis la derniere reprise du suivi


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
		if _etat == Etat.CONTOURNEMENT and not _memoire_ligne.is_empty():
			_lancer_retrouver()         # manoeuvre trop longue : on continue de chercher la ligne
		else:
			_changer_etat(Etat.BLOQUE)  # odometrie perdue : on s'arrete plutot que de partir au hasard

	var r := Vector2(90.0, 0.0)         # (angle servo, vitesse cible)
	match _etat:
		Etat.SUIVI:
			r = _etat_suivi(sondes, d_mm, dt)

		Etat.DECISION:
			if d_mm < 0.0:
				_evitement_hors_ligne = false
				_changer_etat(Etat.SUIVI)       # l'obstacle a disparu
			elif _t_etat >= contournement.pause_s:
				if _evitement_hors_ligne:
					_demarrer_evitement_hors_ligne(d_mm)
				elif _obstacle_deja_contourne(d_mm) and not _en_avant_sur_chemin(somme, n) and _demi_tour_autorise():
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
			r = _retrouver(somme, n, d_mm, dt)

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
				if contournement.hors_ligne and n == 0 and not _memoire_ligne.is_empty():
					_lancer_retrouver()
				else:
					_changer_etat(Etat.SUIVI)
			elif contournement.echec:
				if _memoire_ligne.is_empty():
					_changer_etat(Etat.BLOQUE)
				else:
					_lancer_retrouver()         # manoeuvre impossible : on continue de chercher la ligne
			elif contournement.phase != phase_avant:
				_t_etat = 0.0                   # le timeout compte pour chaque phase
			elif _obstacle_pendant_recherche(n, d_mm, d_brut):
				_evitement_hors_ligne = true
				_changer_etat(Etat.DECISION)
				r = Vector2(90.0, 0.0)

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
	var a_contresens := false
	if n > 0 and n <= 3 and _etat == Etat.SUIVI and _cmd > 1.0:
		a_contresens = _remonte_le_chemin(somme / n)
		_contresens_mm = _contresens_mm + _cmd * robot.mm_s_par_cmd * dt if a_contresens else maxf(0.0, _contresens_mm - _cmd * robot.mm_s_par_cmd * dt)
		if _contresens_mm >= contresens_max_mm and _demi_tour_autorise():
			_contresens_mm = 0.0
			# Ce qui vient d'etre enregistre l'a ete a l'envers : garde, ce passage retournerait la voiture sans fin
			_chemin.resize(maxi(0, _chemin.size() - _points_passage))
			_points_passage = 0
			_demarrer_demi_tour()
	if n > 0 and not a_contresens and (_etat == Etat.SUIVI or _ligne_reprise()):
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
		_chemin.append(p)
		_points_passage += 1
		if _chemin.size() > POINTS_CHEMIN:
			_chemin.pop_front()


## Accord entre le cap et le sens dans lequel ce morceau de ligne a deja ete parcouru (1 = meme sens, -1 = sens inverse).
## NAN si la ligne sous les sondes n'est pas sur le chemin deja parcouru.
func _sens_sur_chemin(e: float) -> float:
	var d := robot.distance_sondes_mm
	var cap := odo.cap_global
	var q := Vector2(odo.x_global + d * sin(cap) + e * cos(cap), odo.y_global + d * cos(cap) - e * sin(cap))
	var avant := Vector2(sin(cap), cos(cap))
	for i in range(_chemin.size() - POINTS_CHEMIN_RECENTS - 1, 2, -1):
		if q.distance_squared_to(_chemin[i]) < rayon_chemin_mm * rayon_chemin_mm:
			var sens := _chemin[i] - _chemin[i - 3]
			return sens.normalized().dot(avant) if sens.length() > 1.0 else NAN
	return NAN


func _remonte_le_chemin(e: float) -> bool:
	var accord := _sens_sur_chemin(e)
	return not is_nan(accord) and accord < -0.5


## Vrai si la voiture roule dans le bon sens sur un morceau deja parcouru. Devant un obstacle deja contourne, elle doit
## alors le recontourner : un demi-tour la renverrait a contresens, puis la surveillance la retournerait, sans fin.
func _en_avant_sur_chemin(somme: float, n: int) -> bool:
	if n == 0:
		return false
	var accord := _sens_sur_chemin(somme / n)
	return not is_nan(accord) and accord > 0.3


## Vrai pendant les phases du contournement ou les sondes voient la ligne de sortie (pas la ligne entrante).
func _ligne_reprise() -> bool:
	return _etat == Etat.CONTOURNEMENT and contournement.phase in [Contournement.Phase.CROISEMENT,
			Contournement.Phase.ALIGNEMENT, Contournement.Phase.SORTIE_ORBITE, Contournement.Phase.TRAVERS]


## Sens de marche sur la ligne (repere global), d'apres les derniers points memorises.
func _sens_ligne() -> Vector2:
	if _memoire_ligne.size() >= 2:
		var sens := _memoire_ligne[-1] - _memoire_ligne[maxi(0, _memoire_ligne.size() - 6)]
		if sens.length() > 1.0:
			return sens.normalized()
	return Vector2(sin(odo.cap_global), cos(odo.cap_global))


## Ou chercher la ligne : devant le dernier point vu (ou devant l'obstacle qui vient d'etre passe), dans le sens de marche.
## Revenir au dernier point vu ramenait la voiture sur le chemin deja parcouru.
func _cible_ligne() -> Vector2:
	var base := _memoire_ligne[-1]
	var proches: Array[Vector2] = [_obstacle_recent]
	if not _obstacles_contournes.is_empty():
		proches.append(_obstacles_contournes[-1])
	for o in proches:
		if o.is_finite() and o.distance_to(_memoire_ligne[-1]) < rayon_obstacle_recent_mm and (o - base).dot(_sens_ligne()) > 0.0:
			base = o
	return base + _sens_ligne() * avance_cible_mm


func _lancer_retrouver() -> void:
	_cible_retrouver = _cible_ligne()
	_t_spirale = -1.0
	_cote_spirale = 1 if suivi.derniere_erreur >= 0.0 else -1
	_changer_etat(Etat.RETROUVER)


## Vrai si la ligne vue est la ligne deja suivie et que la voiture y roule a contresens.
func _sur_ligne_a_contresens(e: float) -> bool:
	var d := robot.distance_sondes_mm
	var cap := odo.cap_global
	var q := Vector2(odo.x_global + d * sin(cap) + e * cos(cap), odo.y_global + d * cos(cap) - e * sin(cap))
	for i in range(_memoire_ligne.size() - 1, 0, -1):
		if q.distance_to(_memoire_ligne[i]) < rayon_meme_ligne_mm:
			var sens := _memoire_ligne[i] - _memoire_ligne[maxi(0, i - 5)]
			return sens.length() > 1.0 and sens.normalized().dot(Vector2(sin(cap), cos(cap))) < -0.3
	return false


## Va vers l'endroit ou la ligne doit se trouver, toujours en avancant, puis la cherche en spirale jusqu'a la croiser.
func _retrouver(somme: float, n: int, d_mm: float, dt: float) -> Vector2:
	if n > 0:
		var e := somme / n
		suivi.derniere_erreur = e if e != 0.0 else 1.0
		if _sur_ligne_a_contresens(e) and _demi_tour_autorise():
			_demarrer_demi_tour()
		else:
			_changer_etat(Etat.SUIVI)
		return Vector2(90.0, 0.0)
	if _t_etat > retrouver_max_s:
		_changer_etat(Etat.BLOQUE)
		return Vector2(90.0, 0.0)
	if d_mm >= 0.0 and d_mm <= _distance_evitement_mm():
		_evitement_hors_ligne = true
		_changer_etat(Etat.DECISION)
		return Vector2(90.0, 0.0)
	var ici := Vector2(odo.x_global, odo.y_global)
	var vers := _cible_retrouver - ici
	var rel := wrapf(atan2(vers.x, vers.y) - odo.cap_global, -PI, PI)
	if ici.distance_to(_obstacle_recent) < rayon_obstacle_recent_mm:
		# Tout pres de l'obstacle evite, l'ultrason ne voit pas ses coins sur le cote : on s'en ecarte d'abord.
		var vers_obstacle := _obstacle_recent - ici
		var rel_obstacle := wrapf(atan2(vers_obstacle.x, vers_obstacle.y) - odo.cap_global, -PI, PI)
		if absf(rel_obstacle) < deg_to_rad(100.0):
			return Vector2(90.0 - 45.0 * signf(rel_obstacle), float(vitesse_retrouver))
	if _t_spirale < 0.0 and (vers.length() < rayon_cible_mm or _t_etat > approche_max_s):
		_t_spirale = 0.0
		_cote_spirale = 1 if rel >= 0.0 else -1
	if _t_spirale >= 0.0:
		_t_spirale += dt
		return Vector2(90.0 + _cote_spirale * 45.0 / (1.0 + _t_spirale / spirale_s), float(vitesse_retrouver))
	var servo := robot.roues_vers_servo(rad_to_deg(rel) * gain_retrouver)
	return Vector2(90.0 + clampf(servo, -45.0, 45.0), float(vitesse_retrouver))


## Distance (mm, mesuree par l'ultrason) a laquelle la voiture s'arrete pour eviter un obstacle.
func _distance_evitement_mm() -> float:
	return contournement.distance_manoeuvre_cm * 10.0 + arret.decalage_capteur_mm + 10.0


## Pendant une orbite ou une recherche, sans ligne sous les sondes : vrai si un autre obstacle que celui contourne est devant.
func _obstacle_pendant_recherche(n: int, d_mm: float, d_brut: float) -> bool:
	var cherche := contournement.phase in [Contournement.Phase.ORBITE, Contournement.Phase.RECHERCHE]
	if not cherche or n > 0 or d_mm < 0.0 or d_mm > _distance_evitement_mm() or d_brut < 0.0 or _obstacle_deja_contourne(d_mm):
		_n_echos_hors_ligne = 0
		return false
	_n_echos_hors_ligne += 1
	return _n_echos_hors_ligne >= echos_hors_ligne


## Evite un obstacle alors que la voiture n'est pas sur la ligne : du cote du dernier point de ligne vu, sans supposer
## que la ligne passe sous l'obstacle ; ensuite RETROUVER ramene la voiture vers ce point.
func _demarrer_evitement_hors_ligne(d_mm: float) -> void:
	_evitement_hors_ligne = false
	_n_echos_hors_ligne = 0
	_obstacle_recent = odo.point_devant_global(robot.longueur_avant_mm + d_mm + contournement.longueur_obstacle_max_mm / 2.0)
	var c := 1 if suivi.derniere_erreur >= 0.0 else -1
	if not _memoire_ligne.is_empty():
		var vers := _cible_ligne() - Vector2(odo.x_global, odo.y_global)
		c = 1 if wrapf(atan2(vers.x, vers.y) - odo.cap_global, -PI, PI) >= 0.0 else -1
	contournement.demarrer_hors_ligne(d_mm, c)
	contournement.chemin_entrant = _memoire_vers_manoeuvre()
	_changer_etat(Etat.CONTOURNEMENT)


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


## Faux apres plusieurs demi-tours au meme endroit : la voiture tournerait sur place indefiniment.
func _demi_tour_autorise() -> bool:
	var ici := Vector2(odo.x_global, odo.y_global)
	var deja := 0
	for p in _demi_tours:
		if ici.distance_to(p) < rayon_meme_demi_tour_mm:
			deja += 1
	return deja < demi_tours_max_au_meme_endroit


func _demarrer_demi_tour() -> void:
	_demi_tours.append(Vector2(odo.x_global, odo.y_global))
	if _demi_tours.size() > DEMI_TOURS_MEMORISES:
		_demi_tours.pop_front()
	_changer_etat(Etat.DEMI_TOUR)
	_cap_debut_demi_tour = odo.cap_global
	_cote_demi_tour = 1 if suivi.derniere_erreur >= 0.0 else -1


## Pivote sur place en plusieurs manoeuvres (avant braquage maximal, arriere braquage oppose : la voiture tourne
## toujours du meme cote) jusqu'a environ 180 deg, puis reprend le suivi des que la ligne est vue.
func _demi_tour(n: int) -> Vector2:
	var tourne := absf(odo.cap_global - _cap_debut_demi_tour)
	if (tourne >= deg_to_rad(150.0) and n > 0) or tourne >= deg_to_rad(210.0) or _t_etat > demi_tour_max_s:
		suivi.derniere_erreur = 38.0 * _cote_demi_tour
		if n == 0 and not _memoire_ligne.is_empty():
			_lancer_retrouver()         # demi-tour fini sans ligne sous les sondes : on la cherche au lieu de tourner en rond
		else:
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
	_evitement_hors_ligne = false
	_obstacle_recent = Vector2(INF, INF)
	_changer_etat(Etat.SUIVI)


## Suivi de ligne, avec arret (ou recul) devant un obstacle situe sur le chemin. Retourne Vector2(angle, vitesse cible).
func _etat_suivi(sondes: Array, d_mm: float, dt: float) -> Vector2:
	var r := suivi.calculer(sondes, dt)
	if suivi.ligne == SuiviLigne.Ligne.PERDUE and suivi.temps_sans_ligne() > retrouver_apres_s and not _memoire_ligne.is_empty():
		_lancer_retrouver()
		return Vector2(90.0, 0.0)
	if suivi.arrivee:
		_changer_etat(Etat.ARRIVEE)
		return Vector2(90.0, 0.0)
	var angle := r.x
	var cible := r.y
	var contourner := contournement.actif and recul.distance_auto_mm <= 0.0
	# Ligne perdue : la voiture braque a fond et la portee utile tomberait a ~14 cm, sous la distance d'arret :
	# elle ne verrait plus rien. Sans ligne a proteger, tout echo proche est sur son chemin.
	var hors_ligne := suivi.ligne == SuiviLigne.Ligne.PERDUE and suivi.temps_sans_ligne() >= hors_ligne_apres_s
	var portee := portee_hors_ligne_mm if hors_ligne else _portee_utile_mm()
	var d_chemin := d_mm if d_mm >= 0.0 and d_mm <= portee else -1.0
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
		_evitement_hors_ligne = hors_ligne
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
		_points_passage = 0
		suivi.reinitialiser()
		arret.relacher()
