## Bille posee dans la cuvette du support (plan support_bille_v2).
## Tant qu'elle est dans la cuvette, sa position est calculee par un modele dans le repere de la voiture ;
## une fois tombee, la physique de Godot prend le relais.
class_name BilleCuvette
extends RigidBody3D

signal chute

const G := 9.81

# Valeurs de la bille et de la cuvette : fichier unique reglages_robot.json, remplies par regler() avant add_child.
var rayon := 0.0           # m, rayon de la bille
var r_cuvette := 0.0       # m, rayon de courbure de la cuvette
var r_bord := 0.0          # m, bord de la cuvette (distance au centre)

var vehicule: Vehicule
var masse := 0.0           ## kg
var amortissement := 0.0   ## 1/s, pertes par roulement

## Valeurs depuis {cle JSON: valeur} (FichierReglages.lire). Conversion mm -> m par / 1000.0.
func regler(r: Dictionary) -> void:
	rayon = float(r["rayon_bille_mm"]) / 1000.0
	r_cuvette = float(r["rayon_cuvette_mm"]) / 1000.0
	r_bord = float(r["rayon_bord_cuvette_mm"]) / 1000.0
	masse = float(r["masse_bille_kg"])
	amortissement = float(r["amortissement_bille_1_s"])

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
	var cs := CollisionShape3D.new(); var sp := SphereShape3D.new(); sp.radius = rayon
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
	global_transform = Transform3D(Basis(), vehicule.position_bille(rayon))
	prete = true

## Bille qui roule sans glisser dans une cuvette spherique, dans le repere de la voiture.
## Equation (sphere pleine) : q'' = 5/7 * ( -g * q / (R - r) - a_cuvette ) - amortissement * q'
## a_cuvette = acceleration reelle du fond de la cuvette (acceleration de la voiture + effet des virages).
## La bille tombe quand son point de contact depasse le bord de la cuvette.
func mettre_a_jour(dt: float) -> void:
	if not prete or tombee: return
	var t := vehicule.global_transform
	var a_loc := _acceleration_locale(t, dt)
	var rc := r_cuvette - rayon
	var acc := (5.0 / 7.0) * (-G * _q / rc - a_loc) - amortissement * _dq
	_dq += acc * dt
	_q += _dq * dt
	var hauteur := rayon + _q.length_squared() / (2.0 * rc)
	global_transform = Transform3D(Basis(), vehicule.centre_plaquette() + t.basis * Vector3(_q.x, hauteur, -_q.y))
	if _q.length() * r_cuvette / rc >= r_bord:
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

## Frequence propre des petites oscillations (Hz), tiree de l'equation de mettre_a_jour() sans amortissement :
## f0 = (1 / 2 pi) * sqrt(5/7 * g / (R - r)).
func frequence_propre_hz() -> float:
	return sqrt(5.0 / 7.0 * G / (r_cuvette - rayon)) / TAU

## Position de la bille rapportee au bord de la cuvette : 0 = au centre, 1 = elle tombe.
func taux_bord() -> float:
	if tombee:
		return 1.0
	return _q.length() * r_cuvette / (r_cuvette - rayon) / r_bord

func _tomber(t: Transform3D) -> void:
	tombee = true
	freeze = false
	linear_velocity = _v1 + t.basis * Vector3(_dq.x, 0, -_dq.y)
	chute.emit()
