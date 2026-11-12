## Bille posee dans la cuvette du support (plan support_bille_v2).
## Tant qu'elle est dans la cuvette, sa position est calculee par un modele dans le repere de la voiture ;
## une fois tombee, la physique de Godot prend le relais.
class_name BilleCuvette
extends RigidBody3D

signal chute

const RAYON := 0.0075
# Cuvette : rayon de courbure 140 mm, bord a 20 mm du centre, 1,5 mm de profondeur
const R_CUVETTE := 0.140
const R_BORD := 0.02
const G := 9.81

var vehicule: Vehicule
var masse := 0.010         ## kg
var amortissement := 1.0   ## 1/s, pertes par roulement

var prete := false
var tombee := false
var _q := Vector2.ZERO        # position du centre de la bille dans la cuvette (m) : x = droite, y = avant (repere voiture)
var _dq := Vector2.ZERO       # vitesse relative (m/s)
var _p1 := Vector3.ZERO; var _v1 := Vector3.ZERO; var _n_mesures := 0

func _init() -> void:
	name = "Bille"

func _ready() -> void:
	mass = masse
	continuous_cd = true
	freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	freeze = true
	collision_layer = Couches.BILLE; collision_mask = Couches.SOL | Couches.VEHICULE
	var pm := PhysicsMaterial.new(); pm.friction = 0.4; pm.bounce = 0.0
	physics_material_override = pm
	var cs := CollisionShape3D.new(); var sp := SphereShape3D.new(); sp.radius = RAYON
	cs.shape = sp; add_child(cs)
	var modele := (load("res://assets/picar/bille.glb") as PackedScene).instantiate()
	for m in modele.find_children("*", "MeshInstance3D"):
		(m as Node3D).position = Vector3.ZERO      # la sphere est centree sur le corps physique
	add_child(modele)

## Vrai quand la bille est posee et que le modele a mesure la plaquette a l'arret (quelques pas).
func est_prete() -> bool:
	return prete and _n_mesures >= 3

func replacer() -> void:
	prete = false; tombee = false
	freeze = true
	_q = Vector2.ZERO; _dq = Vector2.ZERO; _n_mesures = 0
	await get_tree().physics_frame
	await get_tree().physics_frame
	linear_velocity = Vector3.ZERO; angular_velocity = Vector3.ZERO
	global_transform = Transform3D(Basis(), vehicule.position_bille(RAYON))
	prete = true

## Bille qui roule sans glisser dans une cuvette spherique, dans le repere de la voiture.
## Equation (sphere pleine) : q'' = 5/7 * ( -g * q / (R - r) - a_cuvette ) - amortissement * q'
## a_cuvette = acceleration reelle du fond de la cuvette (acceleration de la voiture + effet des virages).
## La bille tombe quand son point de contact depasse le bord de la cuvette.
func mettre_a_jour(dt: float) -> void:
	if not prete or tombee: return
	var t := vehicule.global_transform
	var a_loc := _acceleration_locale(t, dt)
	var rc := R_CUVETTE - RAYON
	var acc := (5.0 / 7.0) * (-G * _q / rc - a_loc) - amortissement * _dq
	_dq += acc * dt
	_q += _dq * dt
	var hauteur := RAYON + _q.length_squared() / (2.0 * rc)
	global_transform = Transform3D(Basis(), vehicule.centre_plaquette() + t.basis * Vector3(_q.x, hauteur, -_q.y))
	if _q.length() * R_CUVETTE / rc >= R_BORD:
		_tomber(t)

# Acceleration de la plaquette (differences finies), exprimee en (droite, avant)
func _acceleration_locale(t: Transform3D, dt: float) -> Vector2:
	var p := vehicule.centre_plaquette()
	var a_monde := Vector3.ZERO
	if _n_mesures >= 2:
		var v := (p - _p1) / dt
		a_monde = (v - _v1) / dt
		_v1 = v
	elif _n_mesures == 1:
		_v1 = (p - _p1) / dt
	_p1 = p; _n_mesures += 1
	return Vector2(t.basis.x.dot(a_monde), -t.basis.z.dot(a_monde))

## Position de la bille rapportee au bord de la cuvette : 0 = au centre, 1 = elle tombe.
func taux_bord() -> float:
	if tombee:
		return 1.0
	return _q.length() * R_CUVETTE / (R_CUVETTE - RAYON) / R_BORD

func _tomber(t: Transform3D) -> void:
	tombee = true
	freeze = false
	linear_velocity = _v1 + t.basis * Vector3(_dq.x, 0, -_dq.y)
	chute.emit()
