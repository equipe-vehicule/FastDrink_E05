## Modele du PiCar-S pour la simulation.
## Recoit les MEMES commandes que le vrai robot (bibliotheque SunFounder) :
##   - commande_vitesse : -100 a 100 (bw.speed, le signe donne le sens)
##   - angle_servo      : 45 a 135 (fw.turn, 90 = tout droit, > 90 = a droite)
## et fournit les MEMES mesures : 5 sondes de ligne (0 ou 1) et la distance ultrason en cm.
##
## Limites du vrai materiel reproduites :
##   - le servo tourne a vitesse finie (vitesse_servo_deg_s) : les roues n'atteignent pas l'angle tout de suite ;
##   - le moteur suit la table de vitesse (zone morte comprise, ReglagesRobot.vitesse_mm_s) et un temps de reponse ;
##   - l'ultrason voit un cone (pas un rayon), mesure a cadence limitee, avec bruit, valeurs aberrantes, biais
##     (gain x distance + biais) et retard pur ; l'aleatoire vient de generateurs propres au vehicule, reensemences
##     a chaque placer() (essais rejouables) ;
##   - une sonde peut donner une lecture fausse (reflet, lumiere ambiante).
class_name Vehicule
extends AnimatableBody3D

# ---------- Valeurs du robot : fichier unique reglages_robot.json (valeur, unite, statut, source, tache).
# Remplies par regler() AVANT add_child (donc avant _ready). Aucune valeur ici : voir le fichier.
var empattement := 0.0              # m, axe avant -> axe arriere
var rayon_roue := 0.0               # m
var encombrement := Vector3.ZERO    # m (largeur, hauteur, longueur) : boite de collision du vehicule
var centre_encombrement := Vector3.ZERO  # m, centre de cette boite par rapport a l'origine du vehicule
var vitesse_max := 0.0              # m/s a la commande 100, modele lineaire (table vide) seulement
var reglages_robot := ReglagesRobot.new()  # table de vitesse (commande -> mm/s) et dimensions du robot
var angle_roues_max_deg := 0.0      # angle reel des roues pour un servo a 45 ou 135
var vitesse_servo_deg_s := 0.0      # deg de servo par seconde (0 = instantane)
var constante_temps_moteur := 0.0   # s, temps de reponse du moteur (0 = instantane)
var bruit_ultrason_cm := 0.0        # ecart-type du bruit de l'ultrason (cm ; cle bruit_ultrason_mm en mm)
var proba_aberrante_ultrason := 0.0 # probabilite qu'une mesure soit aberrante (ecart de signe aleatoire)
var aberrante_min_cm := 0.0         # amplitude minimale de l'ecart aberrant (cm)
var aberrante_max_cm := 0.0         # amplitude maximale (cm), tirage uniforme entre les deux
var gain_ultrason := 1.0            # mesure = gain x distance reelle (depuis la face du capteur) + biais
var biais_ultrason_cm := 0.0        # cm
var retard_ultrason_s := 0.0        # retard pur de la mesure livree (s)
var portee_ultrason := 0.0          # m ; au-dela, la mesure vaut -1 (comme la bibliotheque SunFounder)
var frequence_ultrason_hz := 0.0    # mesures par seconde (0 = a chaque pas)
var demi_angle_ultrason_deg := 0.0  # demi-ouverture du cone de l'ultrason
var rayon_tache_sonde := 0.0        # m, rayon de la zone vue par une sonde
var proba_erreur_sonde := 0.0       # probabilite qu'une sonde donne une lecture fausse a chaque mesure

## Valeurs du robot depuis {cle JSON: valeur} (FichierReglages.lire). Conversion mm -> m par / 1000.0.
func regler(r: Dictionary) -> void:
	empattement = float(r["empattement_mm"]) / 1000.0
	rayon_roue = float(r["rayon_roue_mm"]) / 1000.0
	encombrement = Vector3(float(r["largeur_mm"]), float(r["hauteur_mm"]), float(r["longueur_mm"])) / 1000.0
	centre_encombrement = Vector3(0.0, float(r["centre_encombrement_y_mm"]), float(r["centre_encombrement_z_mm"])) / 1000.0
	reglages_robot.appliquer(r)
	vitesse_max = reglages_robot.vitesse_mm_s(100.0) / 1000.0   # utilise seulement si la table est vide (lineaire)
	angle_roues_max_deg = float(r["angle_roues_max_deg"])
	vitesse_servo_deg_s = float(r["vitesse_servo_deg_s"])
	constante_temps_moteur = float(r["constante_temps_moteur_s"])
	bruit_ultrason_cm = float(r["bruit_ultrason_mm"]) / 10.0
	proba_aberrante_ultrason = float(r["proba_aberrante_ultrason"])
	aberrante_min_cm = float(r["aberrante_ultrason_mm"][0]) / 10.0
	aberrante_max_cm = float(r["aberrante_ultrason_mm"][1]) / 10.0
	gain_ultrason = float(r["gain_ultrason"])
	biais_ultrason_cm = float(r["biais_ultrason_mm"]) / 10.0
	retard_ultrason_s = float(r["retard_ultrason_s"])
	portee_ultrason = float(r["portee_ultrason_m"])
	frequence_ultrason_hz = float(r["frequence_ultrason_hz"])
	demi_angle_ultrason_deg = float(r["demi_angle_ultrason_deg"])
	rayon_tache_sonde = float(r["rayon_tache_sonde_mm"]) / 1000.0
	proba_erreur_sonde = float(r["proba_erreur_sonde"])

# ---------- Etat
var commande_vitesse := 0                # -100 .. 100
var angle_servo := 90                    # 45 .. 135 (commande)
var angle_servo_reel := 90.0             # angle reellement atteint par le servo
var vitesse := 0.0                       # m/s reelle simulee
var acceleration := 0.0                  # m/s^2 instantanee
var distance_parcourue := 0.0            # m
var sondes := [0, 0, 0, 0, 0]
var distance_cm := -1.0                  # derniere mesure livree (cm, -1 = pas d'echo) ; garde entre deux mesures
## Nombre de mesures livrees depuis placer() : le programme du robot compare a la valeur vue au tour precedent pour
## savoir si distance_cm est NOUVELLE (il tourne a 50 Hz, le capteur a ~16 Hz).
var compteur_mesures := 0
## Graine de l'aleatoire de ce cas (bruit, aberrantes, sondes fausses) : fixee par qui lance le cas, utilisee par placer().
var graine_aleatoire := 0
var en_collision := false
var consigne_essai := -1.0               # m/s ; >= 0 : remplace la commande (bancs d'essai uniquement)

var _modele: Node3D
var _rayons_sondes: Array = []           # 5 sondes x 5 rayons (centre + 4 autour)
var _rayons_ultrason: Array = []         # cone : axe + 2 rayons a +/- demi_angle_ultrason_deg
var _t_ultrason := 0.0
var _temps := 0.0                        # s depuis placer() (horloge du retard de l'ultrason)
var _en_transit: Array = []              # mesures prises mais pas encore livrees : [[instant de livraison, cm]]
var _alea_ultrason := RandomNumberGenerator.new()
var _alea_sondes := RandomNumberGenerator.new()
var _requete_collision: PhysicsShapeQueryParameters3D
var _roues := {}
var _rotation_roues := 0.0
var _placement_en_attente := false
var _placement := Transform3D()
## Pose calculee par le modele cinematique. On ne relit pas global_transform : avec sync_to_physics, elle repasse par
## le serveur physique et garde un reste (1e-7) de l'orientation du cas precedent (banc non rejouable, 13 nov.).
var _pose := Transform3D()

const COUCHE_OBSTACLES := 1
const COUCHE_RUBAN := 2
const COUCHE_VEHICULE := 8   # couche 4

func _ready() -> void:
	_pose = global_transform
	sync_to_physics = true
	collision_layer = COUCHE_VEHICULE
	collision_mask = 0
	_modele = (load("res://assets/picar/picar.glb") as PackedScene).instantiate()
	_modele.name = "Modele"
	add_child(_modele)
	var racine := _modele.get_node("PiCar")
	for n in ["Roue_AVG", "Roue_AVD", "Roue_ARG", "Roue_ARD"]:
		_roues[n] = racine.get_node(n)
	# Collision de la plaquette (cuvette) pour la bille
	var plaquette := racine.get_node("Plaquette") as MeshInstance3D
	var forme := CollisionShape3D.new()
	forme.shape = plaquette.mesh.create_trimesh_shape()
	forme.position = plaquette.position
	add_child(forme)
	# 5 sondes de ligne : rayons vers le sol qui ne voient que le ruban (couche 2)
	# Une vraie sonde voit une petite tache, pas un point : 5 rayons par sonde (centre + 4 a rayon_tache_sonde).
	var decalages := [Vector3.ZERO, Vector3(rayon_tache_sonde, 0, 0), Vector3(-rayon_tache_sonde, 0, 0), Vector3(0, 0, rayon_tache_sonde), Vector3(0, 0, -rayon_tache_sonde)]
	for i in range(1, 6):
		var groupe := []
		for dec in decalages:
			var r := RayCast3D.new()
			r.position = (racine.get_node("Sonde_%d" % i) as Node3D).position + dec
			r.target_position = Vector3(0, -0.03, 0)
			r.collision_mask = COUCHE_RUBAN
			add_child(r)
			groupe.append(r)
		_rayons_sondes.append(groupe)
	# Ultrason : cone vers l'avant (-Z) qui voit les obstacles (couche 1), la plus courte distance l'emporte.
	# 5 rayons : l'axe, +/- demi_angle a gauche et a droite, +/- demi_angle en haut et en bas.
	# HYPOTHESE : cone circulaire (meme demi-angle en hauteur ; le manuel p. 34 donne un cone de 30 a 40 deg sans
	# distinguer). Le rayon du bas voit un obstacle couche de 50 mm, plus bas que le capteur (60 mm du sol).
	# Le sol n'est pas sur la couche des obstacles : un echo du sol sur le vrai capteur n'est PAS simule (#53).
	var demi := deg_to_rad(demi_angle_ultrason_deg)
	for axe_angle in [[Vector3.UP, 0.0], [Vector3.UP, demi], [Vector3.UP, -demi], [Vector3.RIGHT, demi], [Vector3.RIGHT, -demi]]:
		var r := RayCast3D.new()
		r.position = (racine.get_node("Capteur_Ultrason") as Node3D).position
		r.target_position = Vector3(0, 0, -portee_ultrason).rotated(axe_angle[0], axe_angle[1])
		r.collision_mask = COUCHE_OBSTACLES
		add_child(r)
		_rayons_ultrason.append(r)
	# Detection de collision avec un obstacle : requete sur la boite d'encombrement du vehicule
	_requete_collision = PhysicsShapeQueryParameters3D.new()
	var b := BoxShape3D.new(); b.size = encombrement
	_requete_collision.shape = b
	_requete_collision.collision_mask = COUCHE_OBSTACLES
	_requete_collision.collide_with_areas = false

## Place le vehicule (applique a la prochaine etape physique, obligatoire avec sync_to_physics).
func placer(t: Transform3D) -> void:
	_placement = t
	_pose = t
	_placement_en_attente = true
	global_transform = t
	vitesse = 0.0
	distance_parcourue = 0.0
	angle_servo_reel = 90.0
	# Comme un vehicule neuf : sinon le cas suivant du banc demarre avec l'acceleration et les sondes du cas
	# precedent (barre d'arrivee 11111) et n'est plus rejouable seul (graine 221733 : bille 0,79 seule, 0,97 en serie)
	acceleration = 0.0
	sondes = [0, 0, 0, 0, 0]
	distance_cm = -1.0
	en_collision = false
	_t_ultrason = 0.0
	_temps = 0.0
	_en_transit.clear()
	compteur_mesures = 0
	_alea_ultrason.seed = graine_aleatoire
	_alea_sondes.seed = graine_aleatoire * 2 + 1

## Applique les commandes (bornees comme sur le vrai robot).
func commander(vitesse_cmd: int, servo: int) -> void:
	commande_vitesse = clampi(vitesse_cmd, -100, 100)
	angle_servo = clampi(servo, 45, 135)

## Angle reel des roues (celui que le servo a atteint, pas celui commande).
func angle_roues_rad() -> float:
	return deg_to_rad((angle_servo_reel - 90.0) * angle_roues_max_deg / 45.0)

## Distances mesurees sur le modele 3D (mm), pour verifier que le controleur utilise les memes.
## Reference : milieu de l'essieu arriere, a empattement / 2 derriere l'origine du vehicule.
func geometrie_mm() -> Dictionary:
	var racine := _modele.get_node("PiCar")
	var z_essieu := empattement / 2.0
	return {
		"empattement_mm": empattement * 1000.0,
		"distance_sondes_mm": (z_essieu - (racine.get_node("Sonde_3") as Node3D).position.z) * 1000.0,
		"longueur_avant_mm": (z_essieu - (racine.get_node("Capteur_Ultrason") as Node3D).position.z) * 1000.0,
		"hauteur_ultrason_mm": (racine.get_node("Capteur_Ultrason") as Node3D).position.y * 1000.0,
	}

func _physics_process(dt: float) -> void:
	if _placement_en_attente:
		global_transform = _placement
		_placement_en_attente = false
		# Pas de lecture des capteurs a ce pas : l'ancienne piste n'est pas encore liberee et la nouvelle pas encore
		# dans l'espace physique (banc : sondes 11111 de l'ancienne barre d'arrivee, 13 nov.). Lecture au pas suivant.
		return
	# Servo : tourne vers la commande a vitesse finie
	if vitesse_servo_deg_s <= 0.0:
		angle_servo_reel = float(angle_servo)
	else:
		angle_servo_reel = move_toward(angle_servo_reel, float(angle_servo), vitesse_servo_deg_s * dt)
	# Vitesse : zone morte, puis premier ordre vers la consigne
	# Vitesse cible : table de vitesse (zone morte comprise) ; table vide = proportionnelle (modele ideal, inchange)
	var cible := commande_vitesse / 100.0 * vitesse_max
	if not reglages_robot.est_lineaire():
		cible = reglages_robot.vitesse_mm_s(float(commande_vitesse)) / 1000.0
	if consigne_essai >= 0.0:
		cible = consigne_essai
	var v_prec := vitesse
	if constante_temps_moteur <= 0.0:
		vitesse = cible
	else:
		vitesse += (cible - vitesse) * (1.0 - exp(-dt / constante_temps_moteur))
	acceleration = (vitesse - v_prec) / dt
	# Modele cinematique « bicyclette », reference = milieu de l'essieu arriere
	var t := _pose
	var avant := -t.basis.z
	var essieu_ar := t.origin + t.basis.z * (empattement / 2.0)
	essieu_ar += avant * vitesse * dt
	var lacet := -vitesse / empattement * tan(angle_roues_rad()) * dt   # angle > 90 : virage a droite
	t.basis = t.basis.rotated(Vector3.UP, lacet).orthonormalized()
	t.origin = essieu_ar - t.basis.z * (empattement / 2.0)
	_pose = t
	global_transform = t
	distance_parcourue += abs(vitesse) * dt
	_animer_roues(dt)
	_lire_capteurs(dt)

func _animer_roues(dt: float) -> void:
	_rotation_roues -= vitesse * dt / rayon_roue
	for n in _roues:
		var roue: Node3D = _roues[n]
		var braquage := -angle_roues_rad() if n.begins_with("Roue_AV") else 0.0
		roue.basis = Basis(Vector3.UP, braquage) * Basis(Vector3.RIGHT, _rotation_roues)

func _lire_capteurs(dt: float) -> void:
	for i in 5:
		var vu := 0
		for r in _rayons_sondes[i]:
			(r as RayCast3D).force_raycast_update()
			if (r as RayCast3D).is_colliding(): vu = 1
		if proba_erreur_sonde > 0.0 and _alea_sondes.randf() < proba_erreur_sonde:
			vu = 1 - vu
		sondes[i] = vu
	_t_ultrason += dt
	_temps += dt
	# La periode reelle est un multiple du pas physique : 62,5 ms a 1/240 s (bancs), 66,7 ms a 1/60 s (vitesse_temps 4)
	if frequence_ultrason_hz <= 0.0 or _t_ultrason >= 1.0 / frequence_ultrason_hz:
		_t_ultrason = 0.0
		_en_transit.push_back([_temps + retard_ultrason_s, _mesurer_ultrason()])
	# Retard pur : la mesure livree a l'instant t est celle prise a t - retard (sans retard, livree aussitot)
	while not _en_transit.is_empty() and _en_transit[0][0] <= _temps:
		distance_cm = _en_transit.pop_front()[1]
		compteur_mesures += 1
	_requete_collision.transform = global_transform * Transform3D(Basis(), centre_encombrement)
	en_collision = not get_world_3d().direct_space_state.intersect_shape(_requete_collision, 1).is_empty()

# Prend une mesure (cm, -1 = pas d'echo) : distance reelle depuis la face du capteur, puis biais (gain x d + biais),
# bruit gaussien, valeur aberrante rare. Livree par _lire_capteurs apres retard_ultrason_s ; entre deux mesures,
# distance_cm garde la valeur precedente (comme une lecture du vrai capteur).
func _mesurer_ultrason() -> float:
	var d := -1.0
	for r in _rayons_ultrason:
		(r as RayCast3D).force_raycast_update()
		if (r as RayCast3D).is_colliding():
			var di := (r as RayCast3D).global_position.distance_to((r as RayCast3D).get_collision_point()) * 100.0
			if d < 0.0 or di < d: d = di
	if d >= 0.0:
		d = gain_ultrason * d + biais_ultrason_cm
		if bruit_ultrason_cm > 0.0:
			d += _alea_ultrason.randfn(0.0, bruit_ultrason_cm)
		if proba_aberrante_ultrason > 0.0 and _alea_ultrason.randf() < proba_aberrante_ultrason:
			var ecart := _alea_ultrason.randf_range(aberrante_min_cm, aberrante_max_cm)
			d += ecart if _alea_ultrason.randf() < 0.5 else -ecart
		d = maxf(d, 0.0)   # une distance mesuree n'est jamais negative (-1 = pas d'echo)
	return d

## Point ou poser la bille : centre du fond de la cuvette (repere monde).
func position_bille(rayon_bille: float) -> Vector3:
	var p := _modele.get_node("PiCar/Plaquette") as Node3D
	return global_transform * (p.position + Vector3(0, 0.0025 + rayon_bille, 0))

func centre_plaquette() -> Vector3:
	var p := _modele.get_node("PiCar/Plaquette") as Node3D
	return global_transform * (p.position + Vector3(0, 0.0025, 0))
