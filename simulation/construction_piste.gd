## Transforme une piste generee (matrice + chemin) en noeuds 3D organises comme les pistes Blender :
##   "Ruban" (MeshInstance3D + StaticBody3D), "Obstacle_n" (MeshInstance3D + StaticBody3D) et un repere "Depart".
## Pistes._regler_couches les range ensuite sur les bonnes couches, comme pour un .glb.
##
## Dans chaque case, le ruban va du milieu du bord d'entree au milieu du bord de sortie :
##   - meme direction  -> segment droit ;
##   - virage          -> quart de cercle de rayon = demi-case (= rayon de la piste), centre sur le coin interieur.
## Les obstacles sont poses sur une partie droite du ruban, a leur position le long de la ligne, et orientes dans le sens
## de la marche.
class_name ConstructionPiste

const POINTS_PAR_ARC := 12
const HAUTEUR_RUBAN := 0.0005   # juste au-dessus du sol, evite le scintillement

static func construire(g: GenerateurPiste) -> Node3D:
	if g is GenerateurLibre:
		var libre := g as GenerateurLibre
		var obstacles_libres := []   # [centre, taille, tangente]
		for n in libre.obstacles_monde.size():
			obstacles_libres.append([libre.obstacles_monde[n][0], libre.tailles_obstacles[n], libre.obstacles_monde[n][1]])
		return construire_depuis_ligne(libre.ligne, obstacles_libres, libre.depart, libre.p.largeur_ruban, libre.rayon, libre.graine)
	var p := g.p
	var t := g.taille_case()
	var origine := -Vector2(g.colonnes, g.lignes) * t / 2.0   # grille centree sur le sol (8 x 8 m)
	var ligne := _ligne_centrale(g, origine, t)
	var obstacles := []   # [centre, taille, tangente]
	for n in g.obstacles.size():
		var sur_ruban := _point_sur_ruban(g, g.obstacles[n], origine, t)
		var pos: Vector2 = sur_ruban[0]
		var tangente: Vector2 = sur_ruban[1]
		pos += tangente * g.positions_obstacles[n]                         # position le long de la ligne droite
		pos += Vector2(-tangente.y, tangente.x) * g.decalages_obstacles[n]   # decalage lateral (> 0 = a droite)
		obstacles.append([pos, g.tailles_obstacles[n], tangente])
	var c := _centre(g.chemin[0], origine, t)
	return construire_depuis_ligne(ligne, obstacles, c, p.largeur_ruban, g.rayon, g.graine)

## Piste a partir de l'axe du ruban (points (x, z) en m), des obstacles [centre: Vector2, taille: Vector3,
## tangente: Vector2] et de la position de depart. Commun a la grille et au generateur libre.
static func construire_depuis_ligne(ligne: PackedVector2Array, obstacles: Array, depart_pos: Vector2, largeur_ruban: float,
		rayon: float, graine: int) -> Node3D:
	var racine := Node3D.new()
	racine.name = "PisteAleatoire"
	racine.add_child(_ruban(ligne, largeur_ruban))
	racine.add_child(_barre_arrivee(ligne))
	var description_obstacles := []   # [x, z, largeur, profondeur, angle] pour le journal
	for n in obstacles.size():
		var pos: Vector2 = obstacles[n][0]
		var taille: Vector3 = obstacles[n][1]
		var tangente: Vector2 = obstacles[n][2]
		racine.add_child(_obstacle(pos, taille, tangente, n))
		description_obstacles.append([pos.x, pos.y, taille.x, taille.z, atan2(tangente.x, tangente.y)])
	# Geometrie gardee pour le journal CSV (JournalCSV enregistre la piste a cote de l'essai)
	racine.set_meta("ligne", ligne)
	racine.set_meta("obstacles", description_obstacles)
	racine.add_child(DecorPiste.construire(ligne, description_obstacles, rayon, graine))   # purement visuel
	var depart := Node3D.new()
	depart.name = "Depart"
	depart.position = Vector3(depart_pos.x, 0, depart_pos.y)
	racine.add_child(depart)
	return racine

# Centre d'une case, en (x, z) monde
static func _centre(case: Vector2i, origine: Vector2, t: float) -> Vector2:
	return origine + (Vector2(case) + Vector2(0.5, 0.5)) * t

## Milieu du ruban dans la case i (centre de la case en ligne droite, milieu de l'arc en virage)
## et direction de la marche a cet endroit. Retourne [position (x, z), tangente unitaire].
static func _point_sur_ruban(g: GenerateurPiste, i: int, origine: Vector2, t: float) -> Array:
	var h := t / 2.0
	var c := _centre(g.chemin[i], origine, t)
	var d_in := Vector2(g.chemin[i] - g.chemin[i - 1]) if i > 0 else Vector2(Vector2i.UP)
	var d_out := Vector2(g.chemin[i + 1] - g.chemin[i]) if i < g.chemin.size() - 1 else d_in
	if d_in == d_out:
		return [c, d_in]
	var coin := c - d_in * h + d_out * h
	var a0 := (-d_out).angle()
	var da := wrapf(d_in.angle() - a0, -PI, PI)
	var a := a0 + da / 2.0
	return [coin + Vector2.from_angle(a) * h, Vector2.from_angle(a + signf(da) * PI / 2.0)]

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

## Barre d'arrivee : ruban noir en travers de la piste, juste avant le bout de la ligne. Plus large que la barre
## de sondes (140 mm contre 76 mm) : les 5 sondes la voient en meme temps, ce qui arrete la voiture (SuiviLigne).
const LARGEUR_ARRIVEE := 0.14     # m, en travers de la piste
const EPAISSEUR_ARRIVEE := 0.05   # m, dans le sens de la marche
static func _barre_arrivee(ligne: PackedVector2Array) -> MeshInstance3D:
	var fin := ligne[ligne.size() - 1]
	var tangente := (fin - ligne[ligne.size() - 2]).normalized()
	var centre := fin - tangente * (EPAISSEUR_ARRIVEE / 2.0 + 0.02)
	var cote := Vector2(-tangente.y, tangente.x) * LARGEUR_ARRIVEE / 2.0
	var barre := _ruban(PackedVector2Array([centre - cote, centre + cote]), EPAISSEUR_ARRIVEE)
	barre.name = "RubanArrivee"            # commence par "Ruban" : vue par les sondes comme la ligne
	return barre

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

## Bloc pose en c, sa largeur en travers du chemin et sa profondeur dans le sens de la marche (tangente).
static func _obstacle(c: Vector2, taille: Vector3, tangente: Vector2, n: int) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.name = "Obstacle_%d" % n
	var bm := BoxMesh.new()
	bm.size = taille
	bm.material = _materiau_caisse()
	mi.mesh = bm
	mi.position = Vector3(c.x, taille.y / 2.0, c.y)
	mi.rotation.y = atan2(tangente.x, tangente.y)   # axe local +Z (profondeur) le long de la tangente (x, z)
	var corps := StaticBody3D.new()
	var cs := CollisionShape3D.new()
	var bx := BoxShape3D.new()
	bx.size = taille
	cs.shape = bx
	corps.add_child(cs)
	mi.add_child(corps)
	return mi

## Caisse en bois (texture de bruit etiree dans le sens des planches). Le materiau ne change rien a la detection.
static func _materiau_caisse() -> StandardMaterial3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = DecorPiste.texture_bruit(Color(0.45, 0.28, 0.13), Color(0.72, 0.5, 0.27), 0.03, 3)
	mat.uv1_scale = Vector3(3, 0.6, 1)
	mat.roughness = 0.8
	return mat
