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
var place_avant_obstacle := 0.0   # m, transmis au generateur (voir GenerateurPiste.place_avant_obstacle)
var generateur: GenerateurPiste   # derniere piste aleatoire (matrice, chemin, graine)
var index := 0
var actuelle: Node3D
var scenario := ""                # nom du scenario charge ("" = aucun ; voir charger_scenario)

## graine : seulement pour la piste aleatoire. 0 = celle des parametres (ou une nouvelle si elle vaut 0 aussi).
func charger(i: int, graine := 0) -> void:
	index = clampi(i, 0, LISTE.size() - 1)
	scenario = ""
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
	if parametres.generateur == "libre":
		generateur = GenerateurLibre.new(parametres)
	else:
		generateur = GenerateurPiste.new(parametres)
	generateur.place_avant_obstacle = place_avant_obstacle
	generateur.generer(graine)
	print(generateur.en_texte())
	return ConstructionPiste.construire(generateur)

## Piste d'un scenario (simulation/scenarios.gd). Retourne false si le scenario est invalide (piste non changee).
func charger_scenario(desc: Dictionary) -> bool:
	var g: GenerateurPiste
	if desc.has("trace"):
		g = GenerateurLibre.new(parametres)
	else:
		g = GenerateurPiste.new(parametres)
	g.place_avant_obstacle = place_avant_obstacle
	if not g.depuis_scenario(desc):
		return false
	generateur = g
	index = INDEX_ALEATOIRE
	scenario = desc["nom"]
	if actuelle: actuelle.queue_free()
	print("Scenario %s" % scenario)
	print(generateur.en_texte())
	actuelle = ConstructionPiste.construire(generateur)
	add_child(actuelle)
	_regler_couches(actuelle)
	return true

func nom() -> String:
	if scenario != "":
		return "Scenario %s" % scenario
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

## Dernier point de la ligne (x, z en m, repere monde) : verification « arret en fin de ligne » du banc d'essai.
## Piste generee : dernier point de la metadonnee « ligne ». Piste Blender : la ligne n'est pas exposee ; APPROXIMATION :
## sommet du ruban (noeuds « Ruban* ») le plus eloigne du depart en suivant les aretes du maillage (distance le long du
## ruban ; la distance a vol d'oiseau se trompe quand le ruban fait un coude). Valable pour un ruban qui ne se recoupe pas.
## Retourne (NAN, NAN) si inconnu.
func fin_de_ligne() -> Vector2:
	if actuelle == null:
		return Vector2(NAN, NAN)
	if actuelle.has_meta("ligne"):
		var ligne: PackedVector2Array = actuelle.get_meta("ligne")
		return ligne[ligne.size() - 1] if ligne.size() > 0 else Vector2(NAN, NAN)
	var points: Array[Vector3] = []     # sommets uniques (monde)
	var index := {}                     # sommet arrondi au 0,1 mm -> numero
	var voisins: Array = []             # numero -> [[numero voisin, longueur], ...]
	for n in actuelle.find_children("Ruban*", "MeshInstance3D", true, false):
		var m := n as MeshInstance3D
		var faces := m.mesh.get_faces()
		for t in range(0, faces.size() - 2, 3):
			var num: Array[int] = []
			for k in 3:
				var q := m.global_transform * faces[t + k]
				var cle := Vector3i(roundi(q.x * 10000.0), roundi(q.y * 10000.0), roundi(q.z * 10000.0))
				if not index.has(cle):
					index[cle] = points.size()
					points.append(q)
					voisins.append([])
				num.append(index[cle])
			for k in 3:
				var a := num[k]
				var b := num[(k + 1) % 3]
				if a != b:
					var l := points[a].distance_to(points[b])
					voisins[a].append([b, l])
					voisins[b].append([a, l])
	if points.is_empty():
		return Vector2(NAN, NAN)
	var depart := transform_depart().origin
	var source := 0
	for i in points.size():
		if points[i].distance_squared_to(depart) < points[source].distance_squared_to(depart):
			source = i
	var dist := PackedFloat64Array()
	dist.resize(points.size())
	dist.fill(INF)
	dist[source] = 0.0
	var file: Array[int] = [source]     # plus courts chemins (relaxations successives)
	var tete := 0
	while tete < file.size():
		var a := file[tete]
		tete += 1
		for v in voisins[a]:
			var d := dist[a] + float(v[1])
			if d < dist[v[0]]:
				dist[v[0]] = d
				file.append(v[0])
	var fin := source
	for i in points.size():
		if not is_inf(dist[i]) and dist[i] > dist[fin]:
			fin = i
	return Vector2(points[fin].x, points[fin].z)
