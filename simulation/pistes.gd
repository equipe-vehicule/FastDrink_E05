## Chargement des pistes de test et reglage de leurs collisions.
## Pour ajouter une piste Blender : une ligne dans LISTE, rien d'autre a modifier.
## Un chemin vide ("") designe la piste generee aleatoirement (GenerateurPiste + ConstructionPiste).
class_name Pistes
extends Node3D

const LISTE := [
	["Ligne droite (18 mm)", "res://assets/pistes/test_02_ligne_droite.glb"],
	["Courbes de 120 mm", "res://assets/pistes/test_03_courbes_r120.glb"],
	["Arret devant obstacle", "res://assets/pistes/test_01_arret.glb"],
	["Contournement", "res://assets/pistes/test_04_contournement.glb"],
	["Recul", "res://assets/pistes/test_05_recul.glb"],
	["Parcours fourni", "res://assets/pistes/parcours.glb"],
	["Piste aleatoire", ""],
]
const INDEX_ALEATOIRE := 6

var parametres := ParametresPiste.new()
var generateur: GenerateurPiste   # derniere piste aleatoire (matrice, chemin, graine)
var index := 0
var actuelle: Node3D

## graine : seulement pour la piste aleatoire. 0 = celle des parametres (ou une nouvelle si elle vaut 0 aussi).
func charger(i: int, graine := 0) -> void:
	index = clampi(i, 0, LISTE.size() - 1)
	if actuelle: actuelle.queue_free()
	if LISTE[index][1] == "":
		actuelle = _generer(graine)
	else:
		actuelle = (load(LISTE[index][1]) as PackedScene).instantiate()
	add_child(actuelle)
	_regler_couches(actuelle)

func _generer(graine: int) -> Node3D:
	if graine == 0:
		graine = parametres.graine if parametres.graine != 0 else randi_range(1, 999999)
	generateur = GenerateurPiste.new(parametres)
	generateur.generer(graine)
	print(generateur.en_texte())
	return ConstructionPiste.construire(generateur)

func nom() -> String:
	if LISTE[index][1] == "" and generateur:
		return "%s (graine %d)" % [LISTE[index][0], generateur.graine]
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
