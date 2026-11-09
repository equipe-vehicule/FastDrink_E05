## Decor "jeu video" autour d'une piste generee : route en asphalte sous le ruban, bordures rouges et blanches,
## damier de depart, arche d'arrivee, arbres, buissons et rochers dans les zones libres.
##
## PUREMENT VISUEL : aucun StaticBody3D, donc rien n'est vu par les sondes (couche RUBAN), par l'ultrason
## (couche OBSTACLE) ni par la physique. Pistes._regler_couches range tout StaticBody3D en obstacle : il ne faut
## jamais en ajouter ici. La vegetation reste a distance_vegetation de la ligne : la voiture qui contourne un
## obstacle (jusqu'a ~45 cm de la ligne) ne traverse pas les arbres.
class_name DecorPiste

const Y_ROUTE := 0.0001          # ruban a 0.0005 (ConstructionPiste.HAUTEUR_RUBAN), herbe a -0.002 (Monde)
const Y_BORDURE := 0.0003
const LARGEUR_BORDURE := 0.02
const DISTANCE_VEGETATION := 0.6 # m de la ligne et des obstacles
const COULEUR_ASPHALTE := Color(0.36, 0.36, 0.38)

## ligne : axe du ruban (x, z). obstacles : [x, z, ...] (meta "obstacles" de la piste). rayon : rayon des virages.
static func construire(ligne: PackedVector2Array, obstacles: Array, rayon: float, graine: int) -> Node3D:
	var racine := Node3D.new()
	racine.name = "Decor"
	# route plus etroite que 2 x rayon : sinon le bord interieur se replie sur lui-meme dans les virages
	var largeur := clampf(1.6 * rayon, 0.12, 0.30)
	var dense := densifier(ligne, 0.02)
	racine.add_child(_bande(dense, largeur, Y_ROUTE, _materiau_asphalte()))
	racine.add_child(_bordures(dense, largeur))
	racine.add_child(_damier_depart(dense, largeur))
	racine.add_child(_arche_arrivee(dense, largeur))
	# vegetation dans les zones libres du sol (8 x 8 m)
	var interdits := densifier(ligne, 0.1)
	for o in obstacles:
		interdits.append(Vector2(o[0], o[1]))
	var rng := RandomNumberGenerator.new()
	rng.seed = graine
	var places := positions_libres(rng, Vector2(-3.9, -3.9), Vector2(3.9, 3.9), 70, interdits, DISTANCE_VEGETATION)
	racine.add_child(vegetation(places, rng))
	return racine

## Points de la ligne espaces d'au plus `pas` (m).
static func densifier(ligne: PackedVector2Array, pas: float) -> PackedVector2Array:
	var res := PackedVector2Array()
	for i in ligne.size() - 1:
		var a := ligne[i]
		var b := ligne[i + 1]
		var n := maxi(1, int(ceil(a.distance_to(b) / pas)))
		for k in n:
			res.append(a.lerp(b, float(k) / n))
	res.append(ligne[ligne.size() - 1])
	return res

## Jusqu'a `nombre` positions tirees au hasard dans [mini, maxi], a plus de `marge` de tous les points interdits.
static func positions_libres(rng: RandomNumberGenerator, mini: Vector2, maxi: Vector2, nombre: int,
		interdits: PackedVector2Array, marge: float) -> Array[Vector2]:
	var grille := {}                                  # case de cote `marge` -> points interdits qu'elle contient
	for p in interdits:
		var k := Vector2i((p / marge).floor())
		if not grille.has(k): grille[k] = []
		(grille[k] as Array).append(p)
	var res: Array[Vector2] = []
	for _essai in nombre * 6:
		if res.size() >= nombre: break
		var c := Vector2(rng.randf_range(mini.x, maxi.x), rng.randf_range(mini.y, maxi.y))
		var k := Vector2i((c / marge).floor())
		var libre := true
		for dx in range(-1, 2):
			for dy in range(-1, 2):
				for p in grille.get(k + Vector2i(dx, dy), []):
					if (p as Vector2).distance_squared_to(c) < marge * marge:
						libre = false
		for q in res:                                  # pas deux elements l'un dans l'autre
			if q.distance_squared_to(c) < 0.04:
				libre = false
		if libre:
			res.append(c)
	return res

## Arbres (sapins et feuillus), buissons et rochers, en MultiMesh (un appel de dessin par type).
## ombres = false pour la foret lointaine : les ombres redessinent tous les arbres et ralentissaient la simulation.
static func vegetation(places: Array[Vector2], rng: RandomNumberGenerator, echelle := 1.0, ombres := true) -> Node3D:
	var racine := Node3D.new()
	racine.name = "Vegetation"
	var troncs: Array[Transform3D] = []
	var sapins: Array[Transform3D] = []
	var sapins_haut: Array[Transform3D] = []
	var feuillus: Array[Transform3D] = []
	var buissons: Array[Transform3D] = []
	var rochers: Array[Transform3D] = []
	var c_sapins := []
	var c_feuillus := []
	var c_buissons := []
	var c_rochers := []
	for p in places:
		var s := rng.randf_range(0.7, 1.5) * echelle
		var base := Transform3D(Basis(Vector3.UP, rng.randf() * TAU).scaled(Vector3.ONE * s), Vector3(p.x, 0, p.y))
		var tirage := rng.randf()
		if tirage < 0.4:
			troncs.append(base)
			sapins.append(base.translated_local(Vector3(0, 0.22, 0)))
			sapins_haut.append(base.translated_local(Vector3(0, 0.36, 0)))
			c_sapins.append(Color(0.12, 0.38, 0.2).lerp(Color(0.2, 0.5, 0.25), rng.randf()))
		elif tirage < 0.7:
			troncs.append(base)
			feuillus.append(base.translated_local(Vector3(0, 0.24, 0)))
			c_feuillus.append(Color(0.3, 0.6, 0.2).lerp(Color(0.55, 0.7, 0.2), rng.randf()))
		elif tirage < 0.9:
			buissons.append(base.translated_local(Vector3(0, 0.03, 0)).scaled_local(Vector3(1, 0.7, 1)))
			c_buissons.append(Color(0.25, 0.5, 0.18).lerp(Color(0.4, 0.62, 0.22), rng.randf()))
		else:
			var r := base.scaled_local(Vector3(rng.randf_range(0.8, 1.4), rng.randf_range(0.5, 0.9), 1.0))
			rochers.append(r.translated_local(Vector3(0, 0.01, 0)))
			c_rochers.append(Color(0.5, 0.5, 0.5).lerp(Color(0.62, 0.6, 0.55), rng.randf()))
	var tronc := CylinderMesh.new()
	tronc.top_radius = 0.012; tronc.bottom_radius = 0.018; tronc.height = 0.16; tronc.radial_segments = 6
	var cone := CylinderMesh.new()
	cone.top_radius = 0.0; cone.bottom_radius = 0.11; cone.height = 0.24; cone.radial_segments = 7
	var cone_haut := CylinderMesh.new()
	cone_haut.top_radius = 0.0; cone_haut.bottom_radius = 0.075; cone_haut.height = 0.18; cone_haut.radial_segments = 7
	var boule := SphereMesh.new()
	boule.radius = 0.1; boule.height = 0.18; boule.radial_segments = 8; boule.rings = 4
	var buisson := SphereMesh.new()
	buisson.radius = 0.06; buisson.height = 0.1; buisson.radial_segments = 7; buisson.rings = 3
	var rocher := SphereMesh.new()
	rocher.radius = 0.05; rocher.height = 0.07; rocher.radial_segments = 5; rocher.rings = 3
	racine.add_child(_multi(tronc, troncs, [], Color(0.4, 0.27, 0.15), ombres))
	racine.add_child(_multi(cone, sapins, c_sapins, Color.WHITE, ombres))
	racine.add_child(_multi(cone_haut, sapins_haut, c_sapins, Color.WHITE, ombres))
	racine.add_child(_multi(boule, feuillus, c_feuillus, Color.WHITE, ombres))
	racine.add_child(_multi(buisson, buissons, c_buissons, Color.WHITE, ombres))
	racine.add_child(_multi(rocher, rochers, c_rochers, Color.WHITE, ombres))
	return racine

static func _multi(mesh: Mesh, xf: Array[Transform3D], couleurs: Array, couleur := Color.WHITE, ombres := true) -> MultiMeshInstance3D:
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = true
	mm.mesh = mesh
	mm.instance_count = xf.size()
	for i in xf.size():
		mm.set_instance_transform(i, xf[i])
		mm.set_instance_color(i, couleurs[i] if i < couleurs.size() else couleur)
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.roughness = 0.9
	var mi := MultiMeshInstance3D.new()
	mi.multimesh = mm
	mi.material_override = mat
	if not ombres:
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return mi

## Bande plate sans collision le long des points (route).
static func _bande(points: PackedVector2Array, largeur: float, y: float, mat: Material) -> MeshInstance3D:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var longueur := 0.0
	var n := points.size()
	for i in n - 1:
		var a := _bords(points, i, largeur / 2.0)
		var b := _bords(points, i + 1, largeur / 2.0)
		var l2 := longueur + points[i].distance_to(points[i + 1])
		_quad(st, a[0], a[1], b[1], b[0], y, Color.WHITE, longueur, l2, largeur)
		longueur = l2
	st.set_material(mat)
	var mi := MeshInstance3D.new()
	mi.name = "Route"
	mi.mesh = st.commit()
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return mi

## Bordures rouges et blanches (5 cm par couleur) des deux cotes de la route.
static func _bordures(points: PackedVector2Array, largeur: float) -> MeshInstance3D:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for cote in [-1.0, 1.0]:
		var longueur := 0.0
		for i in points.size() - 1:
			var ext_a := _bords(points, i, largeur / 2.0)
			var int_a := _bords(points, i, largeur / 2.0 - LARGEUR_BORDURE)
			var ext_b := _bords(points, i + 1, largeur / 2.0)
			var int_b := _bords(points, i + 1, largeur / 2.0 - LARGEUR_BORDURE)
			var j := 0 if cote < 0.0 else 1
			var couleur := Color(0.85, 0.1, 0.1) if int(longueur / 0.05) % 2 == 0 else Color(0.95, 0.95, 0.95)
			if j == 1:
				_quad(st, int_a[j], ext_a[j], ext_b[j], int_b[j], Y_BORDURE, couleur, 0, 0, 1)
			else:                                       # meme sens de rotation des deux cotes (face vers le haut)
				_quad(st, ext_a[j], int_a[j], int_b[j], ext_b[j], Y_BORDURE, couleur, 0, 0, 1)
			longueur += points[i].distance_to(points[i + 1])
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat.roughness = 0.6
	st.set_material(mat)
	var mi := MeshInstance3D.new()
	mi.name = "Bordures"
	mi.mesh = st.commit()
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return mi

## [bord gauche, bord droit] (x, z) du point i, a la distance d de l'axe.
static func _bords(points: PackedVector2Array, i: int, d: float) -> Array:
	var n := points.size()
	var t := (points[mini(i + 1, n - 1)] - points[maxi(i - 1, 0)]).normalized()
	var cote := Vector2(-t.y, t.x) * d
	return [points[i] + cote, points[i] - cote]

static func _quad(st: SurfaceTool, a: Vector2, b: Vector2, c: Vector2, d: Vector2, y: float, couleur: Color,
		v0: float, v1: float, u_max: float) -> void:
	var sommets := [[a, Vector2(0, v0)], [b, Vector2(u_max, v0)], [c, Vector2(u_max, v1)], [d, Vector2(0, v1)]]
	for k in [0, 1, 2, 0, 2, 3]:
		st.set_color(couleur)
		st.set_normal(Vector3.UP)
		st.set_uv(sommets[k][1] * 4.0)                # asphalte : un motif tous les 25 cm
		st.add_vertex(Vector3(sommets[k][0].x, y, sommets[k][0].y))

static func _materiau_asphalte() -> StandardMaterial3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = texture_bruit(COULEUR_ASPHALTE.darkened(0.15), COULEUR_ASPHALTE.lightened(0.1), 0.2, 7)
	mat.roughness = 0.95
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	return mat

## Texture de bruit entre deux couleurs (herbe, asphalte, bois...).
static func texture_bruit(c1: Color, c2: Color, frequence: float, graine: int) -> NoiseTexture2D:
	var bruit := FastNoiseLite.new()
	bruit.seed = graine
	bruit.frequency = frequence
	bruit.fractal_octaves = 4
	var degrade := Gradient.new()
	degrade.set_color(0, c1)
	degrade.set_color(1, c2)
	var tex := NoiseTexture2D.new()
	tex.width = 256
	tex.height = 256
	tex.seamless = true
	tex.noise = bruit
	tex.color_ramp = degrade
	return tex

## Damier noir et blanc (colonnes x lignes cases).
static func texture_damier(colonnes: int, lignes: int) -> ImageTexture:
	var img := Image.create(colonnes, lignes, false, Image.FORMAT_RGB8)
	for x in colonnes:
		for y in lignes:
			img.set_pixel(x, y, Color.WHITE if (x + y) % 2 == 0 else Color(0.08, 0.08, 0.08))
	return ImageTexture.create_from_image(img)

static func _materiau_damier(colonnes: int, lignes: int) -> StandardMaterial3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = texture_damier(colonnes, lignes)
	mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	return mat

## Damier peint en travers de la route, derriere la voiture au depart (visuel : les sondes ne le voient pas).
static func _damier_depart(points: PackedVector2Array, largeur: float) -> MeshInstance3D:
	var t := (points[1] - points[0]).normalized()
	var q := QuadMesh.new()
	q.size = Vector2(largeur, 0.04)
	q.material = _materiau_damier(8, 2)
	var mi := MeshInstance3D.new()
	mi.name = "DamierDepart"
	mi.mesh = q
	var p := points[0] + t * 0.03
	mi.transform = Transform3D(Basis(Vector3.UP, atan2(t.x, t.y)) * Basis(Vector3.RIGHT, -PI / 2), Vector3(p.x, Y_BORDURE, p.y))
	return mi

## Arche au-dessus de la barre d'arrivee : deux poteaux et une banderole a damier (assez haute pour la voiture).
static func _arche_arrivee(points: PackedVector2Array, largeur: float) -> Node3D:
	var n := points.size()
	var t := (points[n - 1] - points[n - 2]).normalized()
	var fin := points[n - 1] - t * 0.045
	var arche := Node3D.new()
	arche.name = "ArcheArrivee"
	arche.transform = Transform3D(Basis(Vector3.UP, atan2(t.x, t.y)), Vector3(fin.x, 0, fin.y))
	var ecart := largeur / 2.0 + 0.05
	var poteau := CylinderMesh.new()
	poteau.top_radius = 0.01; poteau.bottom_radius = 0.012; poteau.height = 0.34
	var mat_poteau := StandardMaterial3D.new()
	mat_poteau.albedo_color = Color(0.85, 0.85, 0.88); mat_poteau.metallic = 0.6; mat_poteau.roughness = 0.3
	poteau.material = mat_poteau
	for s in [-1.0, 1.0]:
		var mi := MeshInstance3D.new()
		mi.mesh = poteau
		mi.position = Vector3(s * ecart, 0.17, 0)
		arche.add_child(mi)
	var banderole := QuadMesh.new()
	banderole.size = Vector2(2.0 * ecart + 0.03, 0.07)
	banderole.material = _materiau_damier(12, 2)
	var b := MeshInstance3D.new()
	b.mesh = banderole
	b.position = Vector3(0, 0.30, 0)
	arche.add_child(b)
	return arche
