## Chargement des pistes de test et reglage de leurs collisions.
## Pour ajouter une piste : une ligne dans LISTE, rien d'autre a modifier.
class_name Pistes
extends Node3D

const LISTE := [
	["Ligne droite (18 mm)", "res://assets/pistes/test_02_ligne_droite.glb"],
	["Courbes de 120 mm", "res://assets/pistes/test_03_courbes_r120.glb"],
	["Arret devant obstacle", "res://assets/pistes/test_01_arret.glb"],
	["Contournement", "res://assets/pistes/test_04_contournement.glb"],
	["Recul", "res://assets/pistes/test_05_recul.glb"],
	["Parcours fourni", "res://assets/pistes/parcours.glb"],
]

var index := 0
var actuelle: Node3D

func charger(i: int) -> void:
	index = clampi(i, 0, LISTE.size() - 1)
	if actuelle: actuelle.queue_free()
	actuelle = (load(LISTE[index][1]) as PackedScene).instantiate()
	add_child(actuelle)
	_regler_couches(actuelle)

func nom() -> String:
	return LISTE[index][0]

## Position de depart : le repere "Depart" de la piste, sinon celui du parcours fourni.
func transform_depart() -> Transform3D:
	var depart := actuelle.find_child("Depart", true, false) as Node3D
	return _depart_parcours() if depart == null else Transform3D(Basis(), depart.global_position)

# Depart du parcours fourni (pas de repere Depart dans le fichier) : debut du ruban, cap +X
func _depart_parcours() -> Transform3D:
	return Transform3D(Basis(Vector3.UP, -PI / 2), Vector3(-1.482, 0, 0.912))

## Ruban -> couche RUBAN (detecte par les sondes), obstacles -> couche OBSTACLE (detectes par l'ultrason).
## Les obstacles recoivent une forme convexe pleine (l'import cree une surface creuse).
func _regler_couches(n: Node) -> void:
	if n is StaticBody3D:
		var est_ruban: bool = n.get_parent().name.begins_with("Ruban")
		n.collision_layer = Couches.RUBAN if est_ruban else Couches.OBSTACLE
		n.collision_mask = 0
		if not est_ruban and n.get_parent() is MeshInstance3D:
			for c in n.get_children():
				if c is CollisionShape3D:
					c.shape = (n.get_parent() as MeshInstance3D).mesh.create_convex_shape(true, false)
	for c in n.get_children(): _regler_couches(c)
