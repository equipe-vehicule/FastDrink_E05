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
## Ce fichier ne fait que choisir quel algorithme commande la voiture (machine a etats) :
##   SUIVI -> (obstacle, voiture immobile) -> PAUSE -> CONTOURNEMENT -> SUIVI
##   SUIVI -> (recul demande) -> RECUL_LIGNE -> ARRET ou SUIVI
## Si une phase de manoeuvre depasse TIMEOUT_ETAT_S, le vehicule passe en BLOQUE (arret) au lieu de repartir au hasard.
class_name Controleur
extends RefCounted

enum Etat { SUIVI, PAUSE, BLOQUE, ARRET, CONTOURNEMENT, RECUL_LIGNE }

const TIMEOUT_ETAT_S := 10.0      # securite : une phase de manoeuvre ne dure jamais plus longtemps

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


func calculer(sondes: Array, distance_cm: float, dt: float) -> Dictionary:
	dt = maxf(dt, 0.001)
	var ligne := SuiviLigne.lire(sondes)
	var somme := ligne.x
	var n := int(ligne.y)
	var d_mm := ultrason.filtrer(distance_cm, dt)

	if _en_manoeuvre() and _t_etat > TIMEOUT_ETAT_S:
		_changer_etat(Etat.BLOQUE)      # odometrie perdue : on s'arrete plutot que de partir au hasard

	var r := Vector2(90.0, 0.0)         # (angle servo, vitesse cible)
	match _etat:
		Etat.SUIVI:
			r = _etat_suivi(sondes, d_mm, dt)

		Etat.PAUSE:
			if d_mm < 0.0:
				_changer_etat(Etat.SUIVI)       # l'obstacle a disparu
			elif _t_etat >= contournement.pause_s:
				contournement.demarrer(d_mm)
				_changer_etat(Etat.CONTOURNEMENT)

		Etat.BLOQUE:
			# arret de securite ; on repart si la voiture est reposee sur la ligne
			if n > 0:
				_changer_etat(Etat.SUIVI)

		Etat.ARRET:
			pass                                # immobile jusqu'a reprendre() ou demarrer_recul()

		Etat.CONTOURNEMENT:
			var phase_avant := contournement.phase
			r = contournement.calculer(somme, n)
			if contournement.termine:
				suivi.derniere_erreur = contournement.ecart_ligne_mm()
				_changer_etat(Etat.SUIVI)
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

	# Arret d'urgence pendant la manoeuvre (obstacle imprevu devant) : on attend, sans faire courir le timeout
	var urgence := _en_manoeuvre() and cible > 0.0 and d_mm >= 0.0 and d_mm < contournement.distance_securite_mm
	if urgence:
		cible = 0.0
	else:
		_t_etat += dt

	# Sorties bornees, acceleration limitee
	angle = clampf(angle, ReglagesRobot.ANGLE_MIN, ReglagesRobot.ANGLE_MAX)
	_cmd = move_toward(_cmd, cible, robot.rampe_cmd_par_s * dt)
	var angle_servo := int(round(angle))
	if _en_manoeuvre():
		odo.integrer(_cmd * robot.mm_s_par_cmd, float(angle_servo), dt)
	return {"vitesse": int(round(_cmd)), "angle": angle_servo}


## Recule en suivant la ligne sur distance_mm (limite a ReculLigne.RECUL_MAX_MM).
## La voiture doit etre sur la ligne et a peu pres dans son axe : ce cap sert de reference (cap = 0).
func demarrer_recul(distance_mm: float) -> void:
	recul.demarrer(distance_mm)
	_changer_etat(Etat.RECUL_LIGNE)


## Nom de l'etat courant, a afficher dans le HUD pour verifier ce que fait la voiture.
func nom_etat() -> String:
	if _etat == Etat.CONTOURNEMENT:
		return contournement.nom_phase()
	if _etat == Etat.SUIVI:
		return "SUIVI (ligne %s)" % suivi.nom_ligne()
	return Etat.keys()[_etat]


## Repart en suivi de ligne (marche avant) apres un arret en fin de recul.
func reprendre() -> void:
	_changer_etat(Etat.SUIVI)


## Remet le controleur a zero (nouveau depart, sortie de BLOQUE).
func reinitialiser() -> void:
	_cmd = 0.0
	suivi.derniere_erreur = 0.0
	ultrason.reinitialiser()
	_changer_etat(Etat.SUIVI)


## Suivi de ligne, avec arret (ou recul) devant un obstacle. Retourne Vector2(angle, vitesse cible).
func _etat_suivi(sondes: Array, d_mm: float, dt: float) -> Vector2:
	var r := suivi.calculer(sondes, dt)
	var angle := r.x
	var cible := r.y
	var contourner := contournement.actif and recul.distance_auto_mm <= 0.0
	if d_mm >= 0.0:
		var d_voulue_cm := contournement.distance_manoeuvre_cm if contourner else arret.distance_arret_cm
		cible = arret.limiter(d_mm, cible, d_voulue_cm * 10.0)
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
		_changer_etat(Etat.PAUSE)
	return Vector2(angle, cible)


## Vrai pendant les etats ou la position est suivie par odometrie.
func _en_manoeuvre() -> bool:
	return _etat == Etat.CONTOURNEMENT or _etat == Etat.RECUL_LIGNE


func _changer_etat(e: Etat) -> void:
	_etat = e
	_t_etat = 0.0
	if e == Etat.SUIVI:
		suivi.reinitialiser()
		arret.relacher()
