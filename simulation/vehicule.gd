## Modele du PiCar-S pour la simulation.
## Recoit les MEMES commandes que le vrai robot (bibliotheque SunFounder) :
##   - commande_vitesse : -100 a 100 (bw.speed, le signe donne le sens)
##   - angle_servo      : 45 a 135 (fw.turn, 90 = tout droit, > 90 = a droite)
## et fournit les MEMES mesures : 5 sondes de ligne (0 ou 1) et la distance ultrason en cm.
class_name Vehicule
extends AnimatableBody3D

# ---------- Geometrie (mesuree, voir 07_Modelisation-Blender/LISEZMOI.md)
const EMPATTEMENT := 0.138          # m, axe avant -> axe arriere
const RAYON_ROUE := 0.0325          # m (1 tour de roue = 205 mm)

# ---------- Parametres A MESURER sur le vrai robot (valeurs provisoires)
@export var vitesse_max := 0.40          ## m/s atteinte a la commande 100. PROVISOIRE : a mesurer (test de vitesse).
@export var angle_roues_max_deg := 45.0  ## angle reel des roues pour un servo a 45 ou 135. PROVISOIRE : a mesurer (test du cercle).
@export var constante_temps_moteur := 0.0  ## s, temps de reponse du moteur (0 = instantane). PROVISOIRE.
@export var bruit_ultrason_cm := 0.0     ## ecart-type du bruit de l'ultrason. A fixer avec le rapport « Test ta confiance ».
@export var portee_ultrason := 3.0       ## m ; au-dela, la mesure vaut -1 (comme la bibliotheque SunFounder)
@export var rayon_tache_sonde := 0.003   ## m, rayon de la zone vue par une sonde. PROVISOIRE : a ajuster sur le vrai robot.

# ---------- Etat
var commande_vitesse := 0                # -100 .. 100
var angle_servo := 90                    # 45 .. 135
var vitesse := 0.0                       # m/s reelle simulee
var acceleration := 0.0                  # m/s^2 instantanee
var distance_parcourue := 0.0            # m
var sondes := [0, 0, 0, 0, 0]
var distance_cm := -1.0
var en_collision := false
var consigne_essai := -1.0               # m/s ; >= 0 : remplace la commande (bancs d'essai uniquement)

var _modele: Node3D
var _rayons_sondes: Array = []           # 5 sondes x 5 rayons (centre + 4 autour)
var _rayon_ultrason: RayCast3D
var _requete_collision: PhysicsShapeQueryParameters3D
const ENCOMBREMENT := Vector3(0.141, 0.080, 0.285)          # m (largeur, hauteur, longueur) du modele
const CENTRE_ENCOMBREMENT := Vector3(0, 0.047, -0.0175)
var _roues := {}
var _rotation_roues := 0.0
var _placement_en_attente := false
var _placement := Transform3D()

const COUCHE_OBSTACLES := 1
const COUCHE_RUBAN := 2
const COUCHE_VEHICULE := 8   # couche 4

func _ready() -> void:
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
	# Ultrason : rayon vers l'avant (-Z) qui voit les obstacles (couche 1)
	_rayon_ultrason = RayCast3D.new()
	_rayon_ultrason.position = (racine.get_node("Capteur_Ultrason") as Node3D).position
	_rayon_ultrason.target_position = Vector3(0, 0, -portee_ultrason)
	_rayon_ultrason.collision_mask = COUCHE_OBSTACLES
	add_child(_rayon_ultrason)
	# Detection de collision avec un obstacle : requete sur la boite d'encombrement du vehicule
	_requete_collision = PhysicsShapeQueryParameters3D.new()
	var b := BoxShape3D.new(); b.size = ENCOMBREMENT
	_requete_collision.shape = b
	_requete_collision.collision_mask = COUCHE_OBSTACLES
	_requete_collision.collide_with_areas = false

## Place le vehicule (applique a la prochaine etape physique, obligatoire avec sync_to_physics).
func placer(t: Transform3D) -> void:
	_placement = t
	_placement_en_attente = true
	global_transform = t
	vitesse = 0.0
	distance_parcourue = 0.0

## Applique les commandes (bornees comme sur le vrai robot).
func commander(vitesse_cmd: int, servo: int) -> void:
	commande_vitesse = clampi(vitesse_cmd, -100, 100)
	angle_servo = clampi(servo, 45, 135)

func angle_roues_rad() -> float:
	return deg_to_rad((angle_servo - 90) * angle_roues_max_deg / 45.0)

func _physics_process(dt: float) -> void:
	if _placement_en_attente:
		global_transform = _placement
		_placement_en_attente = false
		_lire_capteurs()
		return
	# Vitesse : premier ordre vers la consigne
	var cible := commande_vitesse / 100.0 * vitesse_max
	if consigne_essai >= 0.0:
		cible = consigne_essai
	var v_prec := vitesse
	if constante_temps_moteur <= 0.0:
		vitesse = cible
	else:
		vitesse += (cible - vitesse) * (1.0 - exp(-dt / constante_temps_moteur))
	acceleration = (vitesse - v_prec) / dt
	# Modele cinematique « bicyclette », reference = milieu de l'essieu arriere
	var t := global_transform
	var avant := -t.basis.z
	var essieu_ar := t.origin + t.basis.z * (EMPATTEMENT / 2.0)
	essieu_ar += avant * vitesse * dt
	var lacet := -vitesse / EMPATTEMENT * tan(angle_roues_rad()) * dt   # angle > 90 : virage a droite
	t.basis = t.basis.rotated(Vector3.UP, lacet).orthonormalized()
	t.origin = essieu_ar - t.basis.z * (EMPATTEMENT / 2.0)
	global_transform = t
	distance_parcourue += abs(vitesse) * dt
	_animer_roues(dt)
	_lire_capteurs()

func _animer_roues(dt: float) -> void:
	_rotation_roues -= vitesse * dt / RAYON_ROUE
	for n in _roues:
		var roue: Node3D = _roues[n]
		var braquage := -angle_roues_rad() if n.begins_with("Roue_AV") else 0.0
		roue.basis = Basis(Vector3.UP, braquage) * Basis(Vector3.RIGHT, _rotation_roues)

func _lire_capteurs() -> void:
	for i in 5:
		var vu := 0
		for r in _rayons_sondes[i]:
			(r as RayCast3D).force_raycast_update()
			if (r as RayCast3D).is_colliding(): vu = 1
		sondes[i] = vu
	_rayon_ultrason.force_raycast_update()
	if _rayon_ultrason.is_colliding():
		var d := _rayon_ultrason.global_position.distance_to(_rayon_ultrason.get_collision_point()) * 100.0
		if bruit_ultrason_cm > 0.0:
			d += randfn(0.0, bruit_ultrason_cm)
		distance_cm = d
	else:
		distance_cm = -1.0
	_requete_collision.transform = global_transform * Transform3D(Basis(), CENTRE_ENCOMBREMENT)
	en_collision = not get_world_3d().direct_space_state.intersect_shape(_requete_collision, 1).is_empty()

## Point ou poser la bille : centre du fond de la cuvette (repere monde).
func position_bille(rayon_bille: float) -> Vector3:
	var p := _modele.get_node("PiCar/Plaquette") as Node3D
	return global_transform * (p.position + Vector3(0, 0.0025 + rayon_bille, 0))

func centre_plaquette() -> Vector3:
	var p := _modele.get_node("PiCar/Plaquette") as Node3D
	return global_transform * (p.position + Vector3(0, 0.0025, 0))
