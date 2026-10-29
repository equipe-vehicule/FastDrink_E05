## ALGORITHMES DU VEHICULE : suivi de ligne PID + arret devant obstacle + contournement.
## Entrees  : sondes (5 valeurs 0/1, de gauche a droite), distance_cm (ultrason, -1 si rien), dt (s)
## Sorties  : {"vitesse": -100..100, "angle": 45..135}
## Ce fichier ne depend pas de Godot : la meme logique peut tourner sur le vrai robot.
##
## Exigences :
##   - Arreter a une distance predeterminee d'un obstacle (precision +/- 30 mm).
##   - Contourner des obstacles en quittant la ligne-guide au sol.
##   - Suivre la ligne-guide en reculant sur de courtes distances (< 200 mm) : demarrer_recul(distance_mm).
##
## Marche arriere : les sondes sont a l'avant, donc a l'arriere du mouvement. Garder les sondes sur la
## ligne serait instable (l'essieu, qui mene, s'ecarterait de plus en plus). On asservit plutot un point
## virtuel situe derriere l'essieu arriere, estime avec l'ecart lu par les sondes et le cap odometrique.
##
## Deroulement du contournement (obstacle sur la ligne, cote choisi par cote_contournement) :
##   SUIVI -> arret a distance_manoeuvre_cm -> PAUSE -> SORTIE (on quitte la ligne en braquant)
##   -> DECALAGE (tout droit jusqu'au decalage lateral voulu) -> REDRESSEMENT (cap parallele a la ligne)
##   -> PARALLELE (on longe l'obstacle) -> RETOUR_VIRAGE -> RETOUR (on revient vers la ligne)
##   -> CROISEMENT (ligne vue : on continue tout droit jusqu'au bon point de braquage)
##   -> ALIGNEMENT (virage qui finit sur la ligne, dans l'axe) -> SUIVI.
## Les sondes sont devant l'essieu : si l'on braque des qu'elles voient la ligne, l'arriere de la voiture
## est encore loin de la ligne et les sondes la perdent en se redressant. On repere donc la position
## de la ligne au moment ou elle est vue, et l'on braque quand le virage nous amenera dessus.
## Si une phase depasse TIMEOUT_ETAT_S, le vehicule passe en BLOQUE (arret) au lieu de repartir au hasard.
## La position pendant la manoeuvre est estimee par odometrie (vitesse commandee + angle de braquage).
class_name Controleur
extends RefCounted

enum Etat { SUIVI, PAUSE, BLOQUE, ARRET, SORTIE, DECALAGE, REDRESSEMENT, PARALLELE, RETOUR_VIRAGE, RETOUR, CROISEMENT, ALIGNEMENT, RECUL_LIGNE }

const POSITIONS_SONDES_MM := [-38.0, -19.0, 0.0, 19.0, 38.0]
const ANGLE_MIN := 45.0
const ANGLE_MAX := 135.0
const ERREUR_MAX_MM := 38.0       # ecart maximal mesurable avec la ligne visible
const INTEGRALE_MAX := 60.0       # anti-windup (mm*s)
const FILTRE_DERIVEE := 0.3       # 0..1 : plus petit = derivee plus lissee
const ARRET_TOLERANCE_MM := 1.0   # arret declenche a 1 mm de la cible
const HYSTERESIS_MM := 30.0       # l'obstacle doit s'eloigner de plus que ca pour repartir (= tolerance exigee)
const TIMEOUT_ETAT_S := 10.0      # securite : une phase de manoeuvre ne dure jamais plus longtemps
const TAILLE_FILTRE := 3          # mediane sur les 3 dernieres mesures ultrason
const RECUL_MAX_MM := 200.0       # le suivi en marche arriere est limite aux courtes distances

# ----- Gains PID (a regler dans cet ordre : kp, puis kd, puis ki)
var kp := 1.8                     # degres par mm d'ecart
var ki := 0.5                     # degres par (mm*s)
var kd := 0.08                    # degres par (mm/s)

# ----- Vitesses (commande 0..100)
var vitesse_max := 40             # ligne droite, bien centre
var vitesse_virage := 15          # grand ecart ou ligne perdue
var vitesse_manoeuvre := 25       # pendant le contournement

# ----- Arret devant obstacle (ultrason)
var distance_arret_cm := 3.0      # distance d'arret si contourner_obstacles = false (HC-SR04 : fiable a partir de ~3 cm)
var decalage_capteur_mm := 0.0    # distance entre le capteur et l'avant de la voiture (a mesurer)
var vitesse_approche_min := 8     # vitesse minimale d'approche (evite de caler avant la cible)
var mm_s_par_cmd := 4.0           # vitesse reelle par unite de commande (0,40 m/s a 100) : A RECALCULER
var marge_freinage := 0.6         # 0..1 : fraction de la rampe utilisee pour freiner
var recul_autorise := true        # recule si on s'est arrete trop pres (depassement)
var seuil_recul_mm := 15.0        # recule si on est plus pres que cible - seuil (moitie de la tolerance)
var vitesse_recul := 10           # commande de recul (doit vaincre la zone morte du moteur)
# Filtrage ultrason : une mesure -1 isolee ne fait pas repartir la voiture
var portee_max_mm := 1500.0       # au-dela, la mesure est ignoree
var absence_s := 0.15             # duree sans echo avant de considerer qu'il n'y a plus d'obstacle
var zone_proche_mm := 60.0        # sous cette distance, le capteur peut "perdre" un obstacle bien present
var absence_proche_s := 1.0       # duree sans echo exigee quand l'obstacle etait tres proche

# ----- Contournement
var contourner_obstacles := true  # false = on s'arrete devant l'obstacle (test "Arret devant obstacle")
var cote_contournement := 1       # 1 = par la droite, -1 = par la gauche
var distance_manoeuvre_cm := 20.0 # distance d'arret avant de contourner (laisse la place de braquer)
var pause_s := 0.5                # immobilisation avant la manoeuvre
var distance_securite_mm := 40.0  # arret d'urgence si un obstacle apparait devant pendant la manoeuvre
var braquage_manoeuvre := 40.0    # degres de servo par rapport a 90 pendant les virages (max 45)
var cap_evitement_deg := 40.0     # cap pris pour s'ecarter de la ligne
var cap_retour_deg := 30.0        # cap pris pour revenir vers la ligne
var gain_cap := 1.0               # degres de roue par degre d'erreur de cap
var gain_lateral := 0.005         # rad de cap par mm d'ecart lateral (phase PARALLELE)
var correction_laterale_max_deg := 10.0
# Dimensions (mm) : A MESURER sur la piste "Contournement" et sur la voiture
var largeur_obstacle_mm := 100.0
var longueur_obstacle_mm := 100.0 # dans le sens de la marche
var largeur_voiture_mm := 120.0
var marge_laterale_mm := 30.0
var marge_longitudinale_mm := 60.0
# Geometrie du vehicule pour l'odometrie
var empattement_mm := 140.0       # voir Vehicule.EMPATTEMENT (en m) x 1000
var longueur_avant_mm := 160.0    # de l'essieu arriere au capteur ultrason
var distance_sondes_mm := 160.0   # de l'essieu arriere a la barre de sondes de ligne (A MESURER)
var angle_roues_max_deg := 45.0   # angle reel des roues quand le servo est a 45 ou 135 (voir "Roues" au HUD)

# ----- Suivi de ligne en marche arriere
var vitesse_recul_ligne := 15     # commande de recul (valeur absolue)
var point_virtuel_mm := 100.0     # distance du point asservi derriere l'essieu arriere (plus grand = plus doux)
var gain_recul := 1.5             # 1/s : vitesse de convergence du point virtuel vers la ligne
var ligne_perdue_recul_s := 0.3   # arret si la ligne reste invisible plus longtemps
var reprendre_apres_recul := false # true = repart en marche avant a la fin du recul, false = reste arrete
# Test "recul" sans toucher au reste du jeu : apres l'arret devant l'obstacle, on recule de recul_auto_mm.
# 0 = desactive (comportement normal : arret ou contournement selon contourner_obstacles).
var recul_auto_mm := 0.0
var attente_avant_recul_s := 1.0

# ----- Limitation de l'acceleration (bille)
var rampe_cmd_par_s := 125.0

# ----- Etat interne
var _cmd := 0.0
var _derniere_erreur := 0.0
var _erreur_prec := 0.0
var _integrale := 0.0
var _derivee := 0.0
var _ligne_visible_avant := false
var _arrete := false
var _recul := false
var _urgence := false
var _etat := Etat.SUIVI
var _t_etat := 0.0
var _mesures: Array[float] = []
var _d_filtre := -1.0
var _t_absence := 0.0
var _x := 0.0                     # position estimee (mm), repere de l'essieu arriere au debut de la manoeuvre
var _y := 0.0                     # x = vers la droite, y = vers l'avant
var _cap := 0.0                   # cap estime (rad), > 0 = vers la droite
var _y_fin_obstacle := 0.0
var _x_ligne := 0.0               # position x de la ligne, reperee quand les sondes la revoient
var _parcours := 0.0              # distance parcourue en marche arriere (mm)
var _distance_recul_mm := 0.0
var _t_ligne_perdue := 0.0
var _t_arret := 0.0


func calculer(sondes: Array, distance_cm: float, dt: float) -> Dictionary:
	dt = maxf(dt, 0.001)

	# Lecture des sondes
	var somme := 0.0
	var n := 0
	for i in 5:
		if sondes[i] == 1:
			somme += POSITIONS_SONDES_MM[i]
			n += 1
	var d_mm := _filtrer_distance(distance_cm, dt)

	if _etat >= Etat.SORTIE and _t_etat > TIMEOUT_ETAT_S:
		_changer_etat(Etat.BLOQUE)      # odometrie perdue : on s'arrete plutot que de partir au hasard

	var angle := 90.0
	var cible := 0.0
	var cote := float(cote_contournement)
	var x_cible := largeur_obstacle_mm / 2.0 + largeur_voiture_mm / 2.0 + marge_laterale_mm

	match _etat:
		Etat.SUIVI:
			var r := _suivre_ligne(somme, n, dt)
			angle = r.x
			cible = r.y
			var contourner := contourner_obstacles and recul_auto_mm <= 0.0
			if d_mm >= 0.0:
				var d_voulue_cm := distance_manoeuvre_cm if contourner else distance_arret_cm
				cible = _limiter_pour_arret(d_mm, cible, d_voulue_cm * 10.0)
			else:
				_arrete = false
				_recul = false
			if _arrete or _recul:
				_integrale = 0.0                    # pas d'accumulation pendant l'arret
				_derivee = 0.0
			if cible < 0.0:
				angle = 90.0                        # recul en ligne droite
			var immobile := _arrete and absf(_cmd) < 0.5
			_t_arret = _t_arret + dt if immobile else 0.0
			if recul_auto_mm > 0.0 and _t_arret >= attente_avant_recul_s:
				demarrer_recul(recul_auto_mm)
			elif contourner and immobile:
				_changer_etat(Etat.PAUSE)

		Etat.PAUSE:
			if d_mm < 0.0:
				_changer_etat(Etat.SUIVI)       # l'obstacle a disparu
			elif _t_etat >= pause_s:
				_demarrer_odometrie(d_mm)
				_changer_etat(Etat.SORTIE)

		Etat.BLOQUE:
			# arret de securite ; on repart si la voiture est reposee sur la ligne
			if n > 0:
				_changer_etat(Etat.SUIVI)

		Etat.ARRET:
			pass                                # immobile jusqu'a reprendre() ou demarrer_recul()

		Etat.RECUL_LIGNE:
			var restante := _distance_recul_mm - _parcours
			if n > 0:
				_derniere_erreur = somme / n
				_t_ligne_perdue = 0.0
			else:
				_t_ligne_perdue += dt
			if restante <= ARRET_TOLERANCE_MM:
				_changer_etat(Etat.SUIVI if reprendre_apres_recul else Etat.ARRET)
			elif _t_ligne_perdue > ligne_perdue_recul_s:
				_changer_etat(Etat.ARRET)       # ligne perdue : on ne recule pas a l'aveugle
			else:
				angle = _angle_recul(_derniere_erreur)
				# meme profil de freinage que devant un obstacle, pour s'arreter a la distance demandee
				var v_freinage := sqrt(2.0 * rampe_cmd_par_s * marge_freinage * restante / mm_s_par_cmd)
				cible = -minf(float(vitesse_recul_ligne), maxf(v_freinage, float(vitesse_approche_min)))

		Etat.SORTIE:
			angle = 90.0 + cote * braquage_manoeuvre
			cible = float(vitesse_manoeuvre)
			if cote * _cap >= deg_to_rad(cap_evitement_deg):
				_changer_etat(Etat.DECALAGE)

		Etat.DECALAGE:
			angle = _angle_vers_cap(cote * deg_to_rad(cap_evitement_deg))
			cible = float(vitesse_manoeuvre)
			# le redressement ajoute encore R * (1 - cos(cap)) de decalage : on l'anticipe
			var gain_redressement := _rayon_virage_mm() * (1.0 - cos(_cap))
			if cote * _x >= x_cible - gain_redressement:
				_changer_etat(Etat.REDRESSEMENT)

		Etat.REDRESSEMENT:
			angle = 90.0 - cote * braquage_manoeuvre
			cible = float(vitesse_manoeuvre)
			if cote * _cap <= deg_to_rad(3.0):
				_changer_etat(Etat.PARALLELE)

		Etat.PARALLELE:
			# cap parallele a la ligne + petite correction pour rester a x_cible de la ligne
			var corr := clampf((cote * x_cible - _x) * gain_lateral,
					-deg_to_rad(correction_laterale_max_deg), deg_to_rad(correction_laterale_max_deg))
			angle = _angle_vers_cap(corr)
			cible = float(vitesse_manoeuvre)
			if _y >= _y_fin_obstacle:
				_changer_etat(Etat.RETOUR_VIRAGE)

		Etat.RETOUR_VIRAGE:
			angle = 90.0 - cote * braquage_manoeuvre
			cible = float(vitesse_manoeuvre)
			if n > 0:
				_reperer_ligne(somme / n)
				_changer_etat(Etat.CROISEMENT)
			elif cote * _cap <= -deg_to_rad(cap_retour_deg):
				_changer_etat(Etat.RETOUR)

		Etat.RETOUR:
			# garde le cap de retour (-cote * cap_retour_deg) jusqu'a revoir la ligne
			angle = _angle_vers_cap(-cote * deg_to_rad(cap_retour_deg))
			cible = float(vitesse_manoeuvre)
			if n > 0:
				_reperer_ligne(somme / n)
				_changer_etat(Etat.CROISEMENT)

		Etat.CROISEMENT:
			# tout droit : le virage d'alignement decale encore l'essieu de R * (1 - cos(cap)) vers la ligne
			angle = 90.0
			cible = float(vitesse_manoeuvre)
			if cote * (_x - _x_ligne) <= _rayon_virage_mm() * (1.0 - cos(_cap)):
				_changer_etat(Etat.ALIGNEMENT)

		Etat.ALIGNEMENT:
			# braquage constant : on finit dans l'axe, l'essieu arriere sur la ligne
			angle = 90.0 + cote * braquage_manoeuvre
			cible = float(vitesse_manoeuvre)
			if cote * _cap >= -deg_to_rad(3.0):
				# cote ou se trouve la ligne si les sondes ne la voient pas (> 0 = a droite)
				_derniere_erreur = clampf(_x_ligne - _x, -ERREUR_MAX_MM, ERREUR_MAX_MM)
				_changer_etat(Etat.SUIVI)

	# Arret d'urgence pendant la manoeuvre (obstacle imprevu devant) : on attend, sans faire courir le timeout
	_urgence = _etat >= Etat.SORTIE and cible > 0.0 and d_mm >= 0.0 and d_mm < distance_securite_mm
	if _urgence:
		cible = 0.0
	else:
		_t_etat += dt

	# Sorties bornees, acceleration limitee
	angle = clampf(angle, ANGLE_MIN, ANGLE_MAX)
	_cmd = move_toward(_cmd, cible, rampe_cmd_par_s * dt)
	var angle_servo := int(round(angle))
	if _etat >= Etat.SORTIE:
		_integrer_odometrie(_cmd * mm_s_par_cmd, float(angle_servo), dt)
	return {"vitesse": int(round(_cmd)), "angle": angle_servo}


## Recule en suivant la ligne sur distance_mm (limite a RECUL_MAX_MM).
## La voiture doit etre sur la ligne et a peu pres dans son axe : ce cap sert de reference (cap = 0).
func demarrer_recul(distance_mm: float) -> void:
	_distance_recul_mm = clampf(distance_mm, 0.0, RECUL_MAX_MM)
	_x = 0.0
	_y = 0.0
	_cap = 0.0
	_parcours = 0.0
	_t_ligne_perdue = 0.0
	_changer_etat(Etat.RECUL_LIGNE)


## Nom de l'etat courant, a afficher dans le HUD pour verifier ce que fait la voiture.
func nom_etat() -> String:
	return Etat.keys()[_etat]


## Repart en suivi de ligne (marche avant) apres un arret en fin de recul.
func reprendre() -> void:
	_changer_etat(Etat.SUIVI)


## Remet le controleur a zero (nouveau depart, sortie de BLOQUE).
func reinitialiser() -> void:
	_cmd = 0.0
	_derniere_erreur = 0.0
	_mesures.clear()
	_d_filtre = -1.0
	_t_absence = 0.0
	_changer_etat(Etat.SUIVI)


## Suivi de ligne PID. Retourne Vector2(angle, vitesse_cible).
func _suivre_ligne(somme: float, n: int, dt: float) -> Vector2:
	var angle := 90.0
	var cible := float(vitesse_virage)
	if n > 0:
		var erreur: float = somme / n          # mm, > 0 = ligne a droite
		if erreur != 0.0:
			_derniere_erreur = erreur

		# Terme I : remis a zero quand la ligne change de cote (fin de courbe)
		if _ligne_visible_avant and erreur * _erreur_prec < 0.0:
			_integrale = 0.0
		_integrale = clampf(_integrale + erreur * dt, -INTEGRALE_MAX, INTEGRALE_MAX)

		# Terme D (filtre, car les sondes 0/1 donnent des sauts d'erreur)
		if _ligne_visible_avant:
			var brute := (erreur - _erreur_prec) / dt
			_derivee = lerpf(_derivee, brute, FILTRE_DERIVEE)
		else:
			_derivee = 0.0
		_erreur_prec = erreur
		_ligne_visible_avant = true

		angle = 90.0 + kp * erreur + ki * _integrale + kd * _derivee

		# Vitesse adaptative : plus on est decentre, plus on ralentit
		var ratio := clampf(absf(erreur) / ERREUR_MAX_MM, 0.0, 1.0)
		cible = lerpf(float(vitesse_max), float(vitesse_virage), ratio)
	else:
		# Ligne perdue : braquage maximal du dernier cote vu, vitesse reduite
		_integrale = 0.0
		_derivee = 0.0
		_ligne_visible_avant = false
		if _derniere_erreur > 0.0:
			angle = ANGLE_MAX
		elif _derniere_erreur < 0.0:
			angle = ANGLE_MIN
	return Vector2(angle, cible)


## Mediane des dernieres mesures ; un "pas d'echo" n'est accepte qu'apres absence_s
## (absence_proche_s si l'obstacle etait dans la zone ou le capteur devient aveugle).
func _filtrer_distance(distance_cm: float, dt: float) -> float:
	var mm := distance_cm * 10.0
	if distance_cm >= 0.0 and mm <= portee_max_mm:
		_mesures.push_back(mm)
		if _mesures.size() > TAILLE_FILTRE:
			_mesures.pop_front()
		_t_absence = 0.0
		var tri := _mesures.duplicate()
		tri.sort()
		# 1 ou 2 mesures : on garde la plus proche (prudent) ; 3 : la mediane
		_d_filtre = tri[1] if tri.size() >= 3 else tri[0]
	else:
		_t_absence += dt
		var delai := absence_proche_s if _d_filtre >= 0.0 and _d_filtre < zone_proche_mm else absence_s
		if _t_absence >= delai:
			_mesures.clear()
			_d_filtre = -1.0
	return _d_filtre


## Profil de freinage : vitesse permettant de s'arreter a d_voulue_mm de l'obstacle.
## Retourne une vitesse negative si l'on doit reculer (arret trop pres).
func _limiter_pour_arret(d_mm: float, cible: float, d_voulue_mm: float) -> float:
	var restante := d_mm - decalage_capteur_mm - d_voulue_mm      # mm, < 0 = trop pres
	if recul_autorise and restante < -seuil_recul_mm:
		_recul = true
	if _recul:
		if restante >= 0.0:
			_recul = false
			_arrete = true
		else:
			return -float(vitesse_recul)
	if restante <= ARRET_TOLERANCE_MM:
		_arrete = true
	elif restante > HYSTERESIS_MM:
		_arrete = false
	if _arrete:
		return 0.0
	# deceleration constante : d = K * c^2 / (2 * rampe * marge)
	var v_freinage := sqrt(2.0 * rampe_cmd_par_s * marge_freinage * restante / mm_s_par_cmd)
	return minf(cible, maxf(v_freinage, float(vitesse_approche_min)))


func _changer_etat(e: Etat) -> void:
	_etat = e
	_t_etat = 0.0
	if e == Etat.SUIVI:
		_integrale = 0.0
		_derivee = 0.0
		_ligne_visible_avant = false
		_arrete = false
		_recul = false


func _demarrer_odometrie(d0_mm: float) -> void:
	_x = 0.0
	_y = 0.0
	_cap = 0.0
	# l'essieu arriere doit depasser l'arriere de l'obstacle (+ marge) avant de revenir vers la ligne
	_y_fin_obstacle = d0_mm + longueur_avant_mm + longueur_obstacle_mm + marge_longitudinale_mm


## Position x de la ligne (repere odometrie) a partir de l'ecart lu par les sondes (mm, > 0 = a droite).
## La barre de sondes est a distance_sondes_mm devant l'essieu, perpendiculaire au cap.
func _reperer_ligne(erreur_mm: float) -> void:
	_x_ligne = _x + distance_sondes_mm * sin(_cap) + erreur_mm * cos(_cap)


## Braquage en marche arriere. On asservit un point P situe point_virtuel_mm (B) derriere l'essieu :
##   ecart de P a la ligne : e_p = e + (D + B) * sin(cap)        (D = distance_sondes_mm)
##   loi : delta = L / B * (cap + gain_recul * e_p / |v|)  =>  P converge vers la ligne
## et, contrairement aux sondes, garder P sur la ligne est stable en marche arriere.
func _angle_recul(erreur_mm: float) -> float:
	var e_p := erreur_mm + (distance_sondes_mm + point_virtuel_mm) * sin(_cap)
	var v := maxf(absf(_cmd) * mm_s_par_cmd, 20.0)              # evite la division par ~0 au demarrage
	var delta := empattement_mm / point_virtuel_mm * (_cap + gain_recul * e_p / v)
	var servo := rad_to_deg(delta) * 45.0 / angle_roues_max_deg
	return 90.0 + clampf(servo, -45.0, 45.0)


## Angle servo pour rejoindre le cap voulu (rad), braquage limite a braquage_manoeuvre.
func _angle_vers_cap(cap_voulu: float) -> float:
	var roues_deg := rad_to_deg(cap_voulu - _cap) * gain_cap
	var servo := roues_deg * 45.0 / angle_roues_max_deg
	return 90.0 + clampf(servo, -braquage_manoeuvre, braquage_manoeuvre)


## Rayon de braquage (mm) de l'essieu arriere avec braquage_manoeuvre.
func _rayon_virage_mm() -> float:
	var delta := deg_to_rad(braquage_manoeuvre / 45.0 * angle_roues_max_deg)
	return empattement_mm / tan(delta)


## Modele "bicyclette" : met a jour la position et le cap de l'essieu arriere.
func _integrer_odometrie(v_mm_s: float, angle_servo: float, dt: float) -> void:
	var delta := deg_to_rad((angle_servo - 90.0) / 45.0 * angle_roues_max_deg)
	_cap += v_mm_s / empattement_mm * tan(delta) * dt
	_x += v_mm_s * sin(_cap) * dt
	_y += v_mm_s * cos(_cap) * dt
	_parcours -= v_mm_s * dt          # > 0 en marche arriere
