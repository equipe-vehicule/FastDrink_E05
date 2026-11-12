## CONTROLEUR DU VEHICULE : a chaque pas, choisit quel algorithme commande la voiture (machine a etats).
## Entrees  : sondes (5 valeurs 0/1, de gauche a droite), distance_cm (ultrason, -1 si rien), dt (s)
## Sorties  : {"vitesse": -100..100, "angle": 45..135}
## Aucune dependance a Godot (RefCounted) : la meme logique se porte telle quelle en Python sur le robot.
##
## Exigences du client et modules qui y repondent :
##   - suivre la ligne-guide (ruban de 18 mm, courbes de 120 mm et plus)  -> SuiviLigne, Rattrapage
##   - s'arreter a une distance predeterminee d'un obstacle (+/- 30 mm)   -> ArretObstacle
##   - contourner des obstacles en quittant la ligne                      -> ContournementFluide, Contournement
##   - suivre la ligne en reculant sur moins de 200 mm                    -> ReculLigne
## Outils : FiltreUltrason, Odometrie, MemoireParcours, RepriseDouce. Dimensions du robot : ReglagesRobot.
##
## Etats :
##   SUIVI                : SuiviLigne commande. Obstacle sur le chemin : contournement fluide s'il est vu assez loin en
##                          ligne droite, sinon arret a Contournement.distance_manoeuvre_cm puis DECISION.
##   DECISION             : immobile Contournement.pause_s, puis contournement par phases, evitement hors ligne, ou
##                          demi-tour (obstacle deja contourne, vu de dos : la voiture roule a l'envers).
##   CONTOURNEMENT        : Contournement (par phases) commande -> SUIVI ou RETROUVER.
##   CONTOURNEMENT_FLUIDE : ContournementFluide commande (trajectoire lisse, sans arret) -> SUIVI.
##   RATTRAPAGE           : ligne perdue dans un virage serre : marche arriere (Rattrapage) -> SUIVI.
##   RETROUVER            : ligne perdue depuis longtemps : va ou elle doit etre, puis spirale (RetrouverLigne).
##   DEMI_TOUR            : pivot d'environ 180 deg (DemiTour) -> SUIVI ou RETROUVER.
##   RECUL_LIGNE / ARRET  : recul en suivant la ligne (ReculLigne), puis arret.
##   BLOQUE               : arret de securite ; repart si la ligne est revue.
##   ARRIVEE              : barre d'arrivee franchie (5 sondes en travers) : arret definitif.
## Apres toute manoeuvre, RepriseDouce limite vitesse et braquage pendant 1 s (sinon la bille tombe).
##
## Sens de marche : apres un contournement pres d'une epingle, la ligne retrouvee peut partir dans l'autre sens et le
## PID la suivre a l'envers (journal Godot du 6 novembre : 8 reprises sur 20). MemoireParcours garde le chemin parcouru
## et les obstacles contournes : remonter le chemin sur contresens_max_mm, ou retrouver devant soi un obstacle deja
## contourne, declenche un demi-tour.
## Detection en virage : l'ultrason vise tout droit, hors de la piste ; un echo au-dela de la distance ou la ligne sort
## du cone vient d'un objet qui n'est pas sur le chemin : il est ignore (_portee_utile_mm).
class_name Controleur
extends RefCounted

enum Etat { SUIVI, DECISION, BLOQUE, ARRET, CONTOURNEMENT, RECUL_LIGNE, DEMI_TOUR, ARRIVEE, RETROUVER, RATTRAPAGE, CONTOURNEMENT_FLUIDE }

## Etats apres lesquels le suivi reprend en douceur (RepriseDouce).
const ETATS_MANOEUVRE := [Etat.CONTOURNEMENT, Etat.CONTOURNEMENT_FLUIDE, Etat.RATTRAPAGE, Etat.DEMI_TOUR, Etat.RETROUVER]
const TIMEOUT_ETAT_S := 10.0         # securite : une phase de manoeuvre ne dure jamais plus longtemps
const LISSAGE_BRAQUAGE_S := 0.5      # constante de temps du braquage moyen (detection en virage, choix du cote)

# ----- Reglages propres au controleur
var contournement_fluide := true     # false : toujours le contournement par phases
var recul_fin_ligne_mm := 0.0        # piste Recul : > 0 = a la fin de la ligne, s'arreter puis reculer de ca
var hors_ligne_apres_s := 0.5        # ligne perdue depuis ca : la voiture n'est plus sur la ligne
var portee_hors_ligne_mm := 500.0    # hors ligne, tout echo plus proche que ca est sur son chemin
var echos_hors_ligne := 4            # pas consecutifs avec echo brut avant d'interrompre une orbite ou une recherche
var contresens_max_mm := 70.0        # distance parcourue a contresens sur la ligne avant le demi-tour

# ----- Modules (leurs reglages s'ecrivent par ex. controleur.suivi.kp = 2.0, controleur.reprise.duree_s = 1.0)
var robot := ReglagesRobot.new()
var odo := Odometrie.new(robot)
var suivi := SuiviLigne.new()
var ultrason := FiltreUltrason.new()
var arret := ArretObstacle.new(robot)
var contournement := Contournement.new(robot, odo)
var fluide := ContournementFluide.new(robot, odo)
var recul := ReculLigne.new(robot, odo, suivi)
var rattrapage := Rattrapage.new(odo, suivi)
var retrouver := RetrouverLigne.new(robot, odo)
var demi_tour := DemiTour.new()
var reprise := RepriseDouce.new()
var memoire := MemoireParcours.new(robot, odo)

# ----- Journal et diagnostic
var distance_filtree_mm := -1.0      # derniere distance ultrason apres filtrage
## Banc d'essai : pourquoi le contournement par phases a ete choisi plutot que le fluide (nombre de fois par raison).
## N'influence pas la conduite.
var raisons_repli := {}

var _etat := Etat.SUIVI
var _t_etat := 0.0
var _t_arret := 0.0
var _cmd := 0.0                      # commande de vitesse actuelle (apres rampe)
var _braquage_moyen := 0.0           # servo - 90, moyenne glissante pendant le suivi (> 0 = virage a droite)
var _evitement_hors_ligne := false   # la prochaine DECISION lance un evitement hors ligne
var _n_echos_hors_ligne := 0
var _n_echos_fluide := 0
var _raison_repli := ""
var _contresens_mm := 0.0


# =====================================================================================================================
# Pas de controle

func calculer(sondes: Array, distance_cm: float, dt: float) -> Dictionary:
	dt = maxf(dt, 0.001)
	var ligne := SuiviLigne.lire(sondes)
	var somme := ligne.x
	var n := int(ligne.y)
	var d_mm := ultrason.filtrer(distance_cm, dt)
	distance_filtree_mm = d_mm
	_verifier_duree_manoeuvre()
	var r := _commande_selon_etat(sondes, somme, n, d_mm, distance_cm, dt)
	return _appliquer(r.x, r.y, somme, n, d_mm, dt)


## Securite : une phase de manoeuvre trop longue (odometrie perdue...) -> recherche de la ligne ou arret.
func _verifier_duree_manoeuvre() -> void:
	if _etat == Etat.CONTOURNEMENT and contournement.phase in [Contournement.Phase.ORBITE, Contournement.Phase.SORTIE_ORBITE]:
		_t_etat = minf(_t_etat, TIMEOUT_ETAT_S / 2.0)   # un tour d'orbite dure plus longtemps qu'une phase ordinaire
	if _en_manoeuvre() and _t_etat > TIMEOUT_ETAT_S:
		if _etat == Etat.CONTOURNEMENT and not memoire.ligne.is_empty():
			_lancer_retrouver()
		else:
			_changer_etat(Etat.BLOQUE)


## Retourne Vector2(angle servo, vitesse cible) demandes par l'algorithme de l'etat courant.
func _commande_selon_etat(sondes: Array, somme: float, n: int, d_mm: float, distance_cm: float, dt: float) -> Vector2:
	match _etat:
		Etat.SUIVI:
			return _etat_suivi(sondes, d_mm, dt)
		Etat.DECISION:
			_etat_decision(somme, n, d_mm)
		Etat.CONTOURNEMENT:
			return _etat_contournement(somme, n, d_mm, distance_cm, dt)
		Etat.CONTOURNEMENT_FLUIDE:
			var r := fluide.calculer(somme, n)
			if fluide.termine:
				suivi.derniere_erreur = fluide.e_reprise
				_changer_etat(Etat.SUIVI)
			return r
		Etat.RATTRAPAGE:
			var r := rattrapage.calculer(sondes, somme, n, _cmd, _t_etat, dt)
			if rattrapage.termine:
				_changer_etat(Etat.SUIVI)
			return r
		Etat.RETROUVER:
			return _etat_retrouver(somme, n, d_mm, dt)
		Etat.DEMI_TOUR:
			return _etat_demi_tour(n)
		Etat.RECUL_LIGNE:
			var r := recul.calculer(somme, n, dt, _cmd)
			if recul.fin == ReculLigne.Fin.TERMINE:
				_changer_etat(Etat.SUIVI if recul.reprendre_apres_recul else Etat.ARRET)
			elif recul.fin == ReculLigne.Fin.LIGNE_PERDUE:
				_changer_etat(Etat.ARRET)
			return r
		Etat.BLOQUE:
			if n > 0:
				_changer_etat(Etat.SUIVI)     # la voiture a ete reposee sur la ligne
		# ARRIVEE, ARRET : immobile
	return Vector2(90.0, 0.0)


## Limites communes a tous les etats (arret d'urgence, bornes, reprise douce, rampe), puis mise a jour de l'odometrie,
## du sens de marche, de la memoire de la ligne et du braquage moyen.
func _appliquer(angle: float, cible: float, somme: float, n: int, d_mm: float, dt: float) -> Dictionary:
	# Arret d'urgence pendant une manoeuvre (obstacle imprevu tout pres) : on attend, sans faire courir le timeout
	if _en_manoeuvre() and cible > 0.0 and d_mm >= 0.0 and d_mm < contournement.distance_securite_mm:
		cible = 0.0
	else:
		_t_etat += dt
	angle = clampf(angle, ReglagesRobot.ANGLE_MIN, ReglagesRobot.ANGLE_MAX)
	if reprise.en_cours(_etat == Etat.SUIVI, dt):
		cible = reprise.limiter_vitesse(cible)
		angle = reprise.limiter_angle(angle, dt)
	reprise.memoriser_angle(angle)
	_cmd = move_toward(_cmd, cible, robot.rampe_cmd_par_s * dt)
	var angle_servo := int(round(angle))
	odo.suivre_servo(float(angle_servo), dt)              # toujours : angle reel du servo connu au debut d'une manoeuvre
	odo.integrer_global(_cmd * robot.mm_s_par_cmd, dt)    # toujours : position globale (sens de marche)
	var a_contresens := _surveiller_sens(somme, n, dt)
	if n > 0 and not a_contresens and (_etat == Etat.SUIVI or _ligne_de_sortie_visible()):
		memoire.memoriser(somme / n)
	if _en_manoeuvre():
		odo.integrer(_cmd * robot.mm_s_par_cmd, dt)
	if _etat == Etat.SUIVI and absf(_cmd) > 1.0:
		_braquage_moyen = lerpf(_braquage_moyen, angle_servo - 90.0, minf(1.0, dt / LISSAGE_BRAQUAGE_S))
	return {"vitesse": int(round(_cmd)), "angle": angle_servo}


## En suivi : vrai si la voiture remonte un morceau de ligne deja parcouru. Au-dela de contresens_max_mm : demi-tour,
## quelle que soit la raison pour laquelle la voiture s'est retournee.
func _surveiller_sens(somme: float, n: int, dt: float) -> bool:
	if not (n > 0 and n <= 3 and _etat == Etat.SUIVI and _cmd > 1.0):
		return false
	var a_contresens := memoire.remonte_le_chemin(somme / n)
	var pas_mm := _cmd * robot.mm_s_par_cmd * dt
	_contresens_mm = _contresens_mm + pas_mm if a_contresens else maxf(0.0, _contresens_mm - pas_mm)
	if _contresens_mm >= contresens_max_mm and memoire.demi_tour_autorise():
		_contresens_mm = 0.0
		memoire.annuler_passage()
		_demarrer_demi_tour()
	return a_contresens


# =====================================================================================================================
# Etats

## Suivi de ligne, avec arret (ou contournement) devant un obstacle situe sur le chemin.
func _etat_suivi(sondes: Array, d_mm: float, dt: float) -> Vector2:
	var r := suivi.calculer(sondes, dt)
	if suivi.ligne == SuiviLigne.Ligne.PERDUE and suivi.temps_sans_ligne() > retrouver.apres_s and not memoire.ligne.is_empty():
		_lancer_retrouver()
		return Vector2(90.0, 0.0)
	if suivi.arrivee:
		_changer_etat(Etat.ARRIVEE)
		return Vector2(90.0, 0.0)
	rattrapage.observer(dt)
	# Piste Recul : fin de la ligne -> arret, puis recul en suivant la ligne (les sondes la retrouvent en reculant)
	if recul_fin_ligne_mm > 0.0 and rattrapage.ligne_perdue():
		if absf(_cmd) < 0.5:
			demarrer_recul(recul_fin_ligne_mm, 3.0)
			recul_fin_ligne_mm = 0.0
		return Vector2(90.0, 0.0)
	match rattrapage.reaction():
		Rattrapage.Reaction.RATTRAPER:
			rattrapage.demarrer(1 if suivi.derniere_erreur >= 0.0 else -1)
			_changer_etat(Etat.RATTRAPAGE)
			return Vector2(90.0, 0.0)
		Rattrapage.Reaction.ARRETER:
			_changer_etat(Etat.BLOQUE)        # fin de ligne : repart seule si la voiture est reposee sur la ligne
			return Vector2(90.0, 0.0)
	return _approche_obstacle(sondes, r.x, r.y, d_mm, dt)


## Suite du suivi : obstacle sur le chemin -> contournement fluide, ou freinage jusqu'a l'arret puis DECISION.
func _approche_obstacle(sondes: Array, angle: float, cible: float, d_mm: float, dt: float) -> Vector2:
	var contourner := contournement.actif and recul.distance_auto_mm <= 0.0
	# Ligne perdue : la voiture braque a fond et la portee utile tomberait a ~14 cm, sous la distance d'arret :
	# elle ne verrait plus rien. Sans ligne a proteger, tout echo proche est sur son chemin.
	var hors_ligne := suivi.ligne == SuiviLigne.Ligne.PERDUE and suivi.temps_sans_ligne() >= hors_ligne_apres_s
	var portee := portee_hors_ligne_mm if hors_ligne else _portee_utile_mm()
	var d_chemin := d_mm if d_mm >= 0.0 and d_mm <= portee else -1.0
	if _essayer_fluide(sondes, d_chemin, hors_ligne, contourner):
		return Vector2(90.0, float(fluide.vitesse))
	if d_chemin >= 0.0:
		var d_voulue_cm := contournement.distance_manoeuvre_cm if contourner else arret.distance_arret_cm
		cible = arret.limiter(d_chemin, cible, d_voulue_cm * 10.0)
	else:
		arret.relacher()
	if arret.arrete or arret.en_recul:
		suivi.geler()                     # pas d'accumulation pendant l'arret
	if cible < 0.0:
		angle = 90.0                      # recul en ligne droite
	var immobile := arret.arrete and absf(_cmd) < 0.5
	_t_arret = _t_arret + dt if immobile else 0.0
	if recul.distance_auto_mm > 0.0 and _t_arret >= recul.attente_auto_s:
		demarrer_recul(recul.distance_auto_mm)
	elif contourner and immobile:
		_evitement_hors_ligne = hors_ligne
		_noter_repli("ligne_perdue" if hors_ligne else (_raison_repli if _raison_repli != "" else "echo_intermittent"))
		_changer_etat(Etat.DECISION)
	return Vector2(angle, cible)


## Lance le contournement fluide si l'obstacle est confirme assez loin, voiture en ligne droite et sur la ligne.
## Sinon (vu trop pres, en virage) : rien, le freinage normal menera au contournement par phases.
func _essayer_fluide(sondes: Array, d_chemin: float, hors_ligne: bool, contourner: bool) -> bool:
	if not (contourner and contournement_fluide and not hors_ligne and d_chemin >= 0.0 and d_chemin <= fluide.distance_declenchement_mm):
		_n_echos_fluide = 0
		_raison_repli = ""
		return false
	_n_echos_fluide += 1
	if _n_echos_fluide < fluide.confirmations:
		return false
	if d_chemin >= fluide.distance_min_mm and absf(_braquage_moyen) < contournement.seuil_virage_deg:
		var lu := SuiviLigne.lire(sondes)
		_demarrer_fluide(d_chemin, lu.x / lu.y if lu.y > 0.0 else 0.0)
		return true
	if _raison_repli == "":
		_raison_repli = "vu_trop_pres" if d_chemin < fluide.distance_min_mm else "en_virage"
	return false


## Immobile devant l'obstacle : apres la pause, choisit la manoeuvre.
func _etat_decision(somme: float, n: int, d_mm: float) -> void:
	if d_mm < 0.0:
		_evitement_hors_ligne = false
		_changer_etat(Etat.SUIVI)         # l'obstacle a disparu
	elif _t_etat >= contournement.pause_s:
		if _evitement_hors_ligne:
			_demarrer_evitement_hors_ligne(d_mm)
		elif memoire.obstacle_deja_contourne(d_mm) and not memoire.en_avant_sur_chemin(somme, n) and memoire.demi_tour_autorise():
			_demarrer_demi_tour()         # on roule a l'envers : demi-tour plutot que recontourner
		else:
			_demarrer_contournement(d_mm)


func _etat_contournement(somme: float, n: int, d_mm: float, distance_cm: float, dt: float) -> Vector2:
	var phase_avant := contournement.phase
	var d_brut := distance_cm * 10.0 if distance_cm >= 0.0 else -1.0   # non filtre : pour confirmer un echo
	var r := contournement.calculer(somme, n, _cmd * robot.mm_s_par_cmd, d_mm, d_brut, dt)
	if contournement.termine:
		suivi.derniere_erreur = contournement.ecart_ligne_mm()
		if contournement.hors_ligne and n == 0 and not memoire.ligne.is_empty():
			_lancer_retrouver()
		else:
			_changer_etat(Etat.SUIVI)
	elif contournement.echec:
		if memoire.ligne.is_empty():
			_changer_etat(Etat.BLOQUE)
		else:
			_lancer_retrouver()           # manoeuvre impossible : on continue de chercher la ligne
	elif contournement.phase != phase_avant:
		_t_etat = 0.0                     # le timeout compte pour chaque phase
	elif _obstacle_pendant_recherche(n, d_mm, d_brut):
		_evitement_hors_ligne = true
		_noter_repli("pendant_manoeuvre")
		_changer_etat(Etat.DECISION)
		return Vector2(90.0, 0.0)
	return r


func _etat_retrouver(somme: float, n: int, d_mm: float, dt: float) -> Vector2:
	if n > 0:
		var e := somme / n
		suivi.derniere_erreur = e if e != 0.0 else 1.0
		if memoire.sur_ligne_a_contresens(e) and memoire.demi_tour_autorise():
			_demarrer_demi_tour()
		else:
			_changer_etat(Etat.SUIVI)
		return Vector2(90.0, 0.0)
	if _t_etat > retrouver.max_s:
		_changer_etat(Etat.BLOQUE)
		return Vector2(90.0, 0.0)
	if d_mm >= 0.0 and d_mm <= _distance_evitement_mm():
		_evitement_hors_ligne = true
		_noter_repli("pendant_recherche_ligne")
		_changer_etat(Etat.DECISION)
		return Vector2(90.0, 0.0)
	return retrouver.conduire(memoire.obstacle_hors_ligne, _t_etat, dt)


func _etat_demi_tour(n: int) -> Vector2:
	var r := demi_tour.calculer(n, odo.cap_global, _t_etat)
	if demi_tour.termine:
		suivi.derniere_erreur = demi_tour.ecart_reprise_mm()
		if n == 0 and not memoire.ligne.is_empty():
			_lancer_retrouver()           # demi-tour fini sans ligne sous les sondes : on la cherche
		else:
			_changer_etat(Etat.SUIVI)
	return r


# =====================================================================================================================
# Debut des manoeuvres

func _demarrer_contournement(d_mm: float) -> void:
	memoire.noter_obstacle(odo.point_devant_global(robot.longueur_avant_mm + d_mm + contournement.longueur_obstacle_max_mm / 2.0))
	contournement.demarrer(d_mm, _braquage_moyen)
	contournement.chemin_entrant = memoire.vers_repere_manoeuvre()
	_changer_etat(Etat.CONTOURNEMENT)


## e_mm : ecart de la ligne lu par les sondes au depart (mm, > 0 = a droite).
func _demarrer_fluide(d_mm: float, e_mm: float) -> void:
	var c := contournement.cote_par_defaut
	if contournement.cote_interieur_virage and absf(_braquage_moyen) >= contournement.seuil_virage_deg:
		c = 1 if _braquage_moyen > 0.0 else -1
	memoire.noter_obstacle(odo.point_devant_global(robot.longueur_avant_mm + d_mm + fluide.profondeur_obstacle_mm / 2.0))
	_n_echos_fluide = 0
	# Cap de la voiture par rapport a la ligne (droite ajustee sur les derniers points memorises) et position laterale
	# de l'essieu : les sondes, a distance_sondes_mm devant, voient la ligne a e_mm a droite du centre de la barre.
	var cap_rel := 0.0
	if memoire.ligne.size() >= 6:
		var sens := memoire.direction_ajustee(fluide.points_direction_ligne)
		cap_rel = wrapf(odo.cap_global - atan2(sens.x, sens.y), -PI, PI)
	var x0 := -e_mm * cos(cap_rel) - robot.distance_sondes_mm * sin(cap_rel)
	fluide.demarrer(d_mm, c, x0, cap_rel)
	_changer_etat(Etat.CONTOURNEMENT_FLUIDE)


## Evite un obstacle alors que la voiture n'est pas sur la ligne : du cote ou la ligne doit se trouver, sans supposer
## qu'elle passe sous l'obstacle ; ensuite RETROUVER ramene la voiture vers elle.
func _demarrer_evitement_hors_ligne(d_mm: float) -> void:
	_evitement_hors_ligne = false
	_n_echos_hors_ligne = 0
	memoire.obstacle_hors_ligne = odo.point_devant_global(robot.longueur_avant_mm + d_mm + contournement.longueur_obstacle_max_mm / 2.0)
	var c := 1 if suivi.derniere_erreur >= 0.0 else -1
	if not memoire.ligne.is_empty():
		var vers := retrouver.cible_ligne(memoire) - Vector2(odo.x_global, odo.y_global)
		c = 1 if wrapf(atan2(vers.x, vers.y) - odo.cap_global, -PI, PI) >= 0.0 else -1
	contournement.demarrer_hors_ligne(d_mm, c)
	contournement.chemin_entrant = memoire.vers_repere_manoeuvre()
	_changer_etat(Etat.CONTOURNEMENT)


func _demarrer_demi_tour() -> void:
	memoire.noter_demi_tour()
	_changer_etat(Etat.DEMI_TOUR)
	demi_tour.demarrer(odo.cap_global, 1 if suivi.derniere_erreur >= 0.0 else -1)


func _lancer_retrouver() -> void:
	retrouver.demarrer(memoire, suivi.derniere_erreur)
	_changer_etat(Etat.RETROUVER)


## Recule en suivant la ligne sur distance_mm (limite a ReculLigne.RECUL_MAX_MM).
## La voiture doit etre sur la ligne et a peu pres dans son axe : ce cap sert de reference (cap = 0).
func demarrer_recul(distance_mm: float, attente_ligne_s := 0.0) -> void:
	recul.demarrer(distance_mm, attente_ligne_s)
	_changer_etat(Etat.RECUL_LIGNE)


func _changer_etat(e: Etat) -> void:
	if e == Etat.SUIVI and _etat in ETATS_MANOEUVRE:
		reprise.demarrer()
	_etat = e
	_t_etat = 0.0
	if e == Etat.SUIVI:
		memoire.nouveau_passage()
		rattrapage.nouveau_suivi()
		suivi.reinitialiser()
		arret.relacher()


# =====================================================================================================================
# Outils

## Vrai pendant les etats ou la position est suivie par odometrie.
func _en_manoeuvre() -> bool:
	return _etat == Etat.CONTOURNEMENT or _etat == Etat.RECUL_LIGNE or _etat == Etat.RATTRAPAGE \
			or _etat == Etat.CONTOURNEMENT_FLUIDE


## Vrai pendant les phases du contournement ou les sondes voient la ligne de sortie (pas la ligne entrante).
func _ligne_de_sortie_visible() -> bool:
	return _etat == Etat.CONTOURNEMENT and contournement.phase in [Contournement.Phase.CROISEMENT,
			Contournement.Phase.ALIGNEMENT, Contournement.Phase.SORTIE_ORBITE, Contournement.Phase.TRAVERS]


## Distance jusqu'a laquelle un echo peut venir d'un obstacle pose sur la ligne.
## En virage de rayon R (deduit du braquage moyen), la ligne s'ecarte de l'axe du capteur d'environ y^2 / (2 R)
## a la distance y ; elle reste dans le cone tant que y^2 / (2 R) < y * tan(demi-angle) + demi-largeur d'obstacle.
func _portee_utile_mm() -> float:
	var delta := deg_to_rad(absf(robot.servo_vers_roues_deg(_braquage_moyen)))
	if delta < deg_to_rad(1.0):
		return INF                        # ligne droite : toute la portee
	var rayon := robot.empattement_mm / tan(delta)
	var a := rayon * tan(deg_to_rad(robot.demi_angle_ultrason_deg))
	return a + sqrt(a * a + rayon * contournement.largeur_obstacle_min_mm)


## Distance (mm, mesuree par l'ultrason) a laquelle la voiture s'arrete pour eviter un obstacle.
func _distance_evitement_mm() -> float:
	return contournement.distance_manoeuvre_cm * 10.0 + arret.decalage_capteur_mm + 10.0


## Pendant une orbite ou une recherche, sans ligne sous les sondes : vrai si un autre obstacle que celui contourne est devant.
func _obstacle_pendant_recherche(n: int, d_mm: float, d_brut: float) -> bool:
	var cherche := contournement.phase in [Contournement.Phase.ORBITE, Contournement.Phase.RECHERCHE]
	if not cherche or n > 0 or d_mm < 0.0 or d_mm > _distance_evitement_mm() or d_brut < 0.0 or memoire.obstacle_deja_contourne(d_mm):
		_n_echos_hors_ligne = 0
		return false
	_n_echos_hors_ligne += 1
	return _n_echos_hors_ligne >= echos_hors_ligne


func _noter_repli(raison: String) -> void:
	raisons_repli[raison] = int(raisons_repli.get(raison, 0)) + 1
	_raison_repli = ""


# =====================================================================================================================
# Interface (simulation, HUD, journal)

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
	memoire.obstacle_hors_ligne = Vector2(INF, INF)
	_changer_etat(Etat.SUIVI)
