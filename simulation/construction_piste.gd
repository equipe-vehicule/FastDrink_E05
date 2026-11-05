## Transforme une piste generee (matrice + chemin) en noeuds 3D organises comme les pistes Blender :
##   "Ruban" (MeshInstance3D + StaticBody3D), "Obstacle_n" (MeshInstance3D + StaticBody3D) et un repere "Depart".
## Pistes._regler_couches les range ensuite sur les bonnes couches, comme pour un .glb.
##
## Dans chaque case, le ruban va du milieu du bord d'entree au milieu du bord de sortie :
##   - meme direction  -> segment droit ;
##   - virage          -> quart de cercle de rayon = demi-case (= rayon_virage), centre sur le coin interieur.
class_name ConstructionPiste

const POINTS_PAR_ARC := 12
const HAUTEUR_RUBAN := 0.0005   # juste au-dessus du sol, evite le scintillement

static func construire(g: GenerateurPiste) -> Node3D:
	var p := g.p
	var t := p.taille_case()
	var origine := -Vector2(p.colonnes, p.lignes) * t / 2.0   # grille centree sur le sol (8 x 8 m)
	if maxf(p.colonnes, p.lignes) * t > 8.0:
		push_warning("Piste aleatoire : la grille depasse du sol de 8 m.")
	var racine := Node3D.new()
	racine.name = "PisteAleatoire"
	racine.add_child(_ruban(_ligne_centrale(g, origine, t), p.largeur_ruban))
	for n in g.obstacles.size():
		var i := g.obstacles[n]
		var le_long_de_x := (g.chemin[i] - g.chemin[i - 1]).x != 0
		racine.add_child(_obstacle(_centre(g.chemin[i], origine, t), p.taille_obstacle, le_long_de_x, n))
	var depart := Node3D.new()
	depart.name = "Depart"
	var c := _centre(g.chemin[0], origine, t)
	depart.position = Vector3(c.x, 0, c.y)
	racine.add_child(depart)
	return racine

# Centre d'une case, en (x, z) monde
static func _centre(case: Vector2i, origine: Vector2, t: float) -> Vector2:
	return origine + (Vector2(case) + Vector2(0.5, 0.5)) * t

## Points (x, z) de l'axe du ruban, du debut a la fin du chemin.
static func _ligne_centrale(g: GenerateurPiste, origine: Vector2, t: float) -> PackedVector2Array:
	var h := t / 2.0
	var n := g.chemin.size()
	var points := PackedVector2Array()
	for i in n:
		var c := _centre(g.chemin[i], origine, t)
		var d_in := Vector2(g.chemin[i] - g.chemin[i - 1]) if i > 0 else Vector2(Vector2i.UP)
		var d_out := Vector2(g.chemin[i + 1] - g.chemin[i]) if i < n - 1 else d_in
		if i == 0:
			points.append(c - d_in * h)
		if d_in == d_out:
			points.append(c + d_out * h)
		else:
			var coin := c - d_in * h + d_out * h
			var a0 := (-d_out).angle()
			var da := wrapf(d_in.angle() - a0, -PI, PI)   # +-90 deg
			for k in range(1, POINTS_PAR_ARC + 1):
				points.append(coin + Vector2.from_angle(a0 + da * k / POINTS_PAR_ARC) * h)
	return points

## Bande plate de largeur constante le long des points, avec sa collision (vue par les sondes).
static func _ruban(points: PackedVector2Array, largeur: float) -> MeshInstance3D:
	var gauche := PackedVector3Array()
	var droite := PackedVector3Array()
	var n := points.size()
	for i in n:
		var tangente := (points[mini(i + 1, n - 1)] - points[maxi(i - 1, 0)]).normalized()
		var cote := Vector2(-tangente.y, tangente.x) * largeur / 2.0
		gauche.append(Vector3(points[i].x + cote.x, HAUTEUR_RUBAN, points[i].y + cote.y))
		droite.append(Vector3(points[i].x - cote.x, HAUTEUR_RUBAN, points[i].y - cote.y))
	var faces := PackedVector3Array()
	for i in n - 1:
		for v in [gauche[i], droite[i], droite[i + 1], gauche[i], droite[i + 1], gauche[i + 1]]:
			faces.append(v)

	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for v in faces:
		st.set_normal(Vector3.UP)
		st.add_vertex(v)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.05, 0.05, 0.05)
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	st.set_material(mat)

	var mi := MeshInstance3D.new()
	mi.name = "Ruban"
	mi.mesh = st.commit()
	var corps := StaticBody3D.new()
	var cs := CollisionShape3D.new()
	var forme := ConcavePolygonShape3D.new()
	forme.set_faces(faces)
	forme.backface_collision = true   # detecte quel que soit le sens des triangles
	cs.shape = forme
	corps.add_child(cs)
	mi.add_child(corps)
	return mi

## Bloc pose sur le ruban, sa largeur en travers du chemin.
static func _obstacle(c: Vector2, taille: Vector3, le_long_de_x: bool, n: int) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.name = "Obstacle_%d" % n
	var bm := BoxMesh.new()
	bm.size = taille
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.75, 0.25, 0.2)
	bm.material = mat
	mi.mesh = bm
	mi.position = Vector3(c.x, taille.y / 2.0, c.y)
	if le_long_de_x:
		mi.rotation.y = PI / 2
	var corps := StaticBody3D.new()
	var cs := CollisionShape3D.new()
	var bx := BoxShape3D.new()
	bx.size = taille
	cs.shape = bx
	corps.add_child(cs)
	mi.add_child(corps)
	return mi
