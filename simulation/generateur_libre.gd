## Piste aleatoire LIBRE (sans grille) : droites et virages d'angle quelconque (angle_min_deg a angle_max_deg).
## Pure logique (aucun noeud 3D) : ConstructionPiste.construire_depuis_ligne transforme le resultat en piste.
##
## Le guide (section 1) n'impose que le rayon de courbure minimal (120 mm), aucun angle (#73). Le trace est de type
## « tortue » : une droite, puis des virages (arcs de cercle de rayon `rayon`) et des droites, jusqu'a la longueur visee.
## Repere : celui de ConstructionPiste (sol centre sur l'origine, points (x, z) en m). La voiture part vers -Z :
## direction (0, -1), sa droite est vers +x. Un virage a droite (angle > 0) tourne (0, -1) vers (+1, 0).
## Herite de GenerateurPiste pour garder l'interface (Pistes.generateur, banc d'essai, Simulation) ; les champs de
## grille (matrice, chemin, colonnes, lignes) restent vides.
class_name GenerateurLibre
extends GenerateurPiste

const PAS_POINTS := 0.01          # m, espacement des points sur les arcs (et pour les verifications)
const ESSAIS_SEGMENT := 50        # tirages d'un segment avant d'abandonner (specification)
const FENETRE_VOISINS := 0.6      # m de chemin juste avant un point : voisins immediats, exclus du test d'ecart
const TOLERANCE_ECART := 1e-4     # m, tolerance numerique sur l'ecart minimal (deux lignes a 2 x rayon exactement)
const RECOMMENCER_MAX := ESSAIS_MAX / 100   # traces complets recommences au plus (garde-fou, cf. ESSAIS_MAX)
const HAUTEUR_REPLI := 3.0        # m, longueur de la droite de repli si aucun trace n'aboutit

## Axe du ruban : un point tous les ~10 mm sur les arcs, aux extremites sur les droites.
var ligne := PackedVector2Array()
## Une entree par segment droit : [p0: Vector2, direction: Vector2 unitaire, longueur_m: float]
var droites: Array = []
## Angles des virages en degres, signes (> 0 = a droite), dans l'ordre du trace.
var angles_virages: Array[float] = []
## Une entree par obstacle (meme ordre que tailles_obstacles) : [centre: Vector2, tangente: Vector2], centre decale.
var obstacles_monde: Array = []
var depart := Vector2.ZERO        # position de depart de la voiture (m)

# Etat du trace en cours
var _pos := Vector2.ZERO
var _dir := Vector2(0, -1)
var _s := 0.0                                  # longueur de chemin posee (m)
var _dpts := PackedVector2Array()              # points denses deja poses (verification d'ecart)
var _dcum := PackedFloat64Array()              # leur abscisse curviligne
var _grille := {}                              # cellule (Vector2i) -> PackedInt32Array d'indices de _dpts
var _dernier_segment_droit := false

func generer(graine_choisie: int) -> void:
	graine = graine_choisie
	_rng.seed = graine
	rayon = _rng.randf_range(p.rayon_virage, maxf(p.rayon_virage, p.rayon_virage_max))
	colonnes = 0
	lignes = 0
	matrice.clear()
	chemin.clear()
	_tracer()
	_placer_obstacles()

## Matrice remplacee par une description : le trace n'a pas de grille.
func en_texte() -> String:
	var angles := PackedStringArray()
	for a in angles_virages:
		angles.append("%+d" % roundi(a))
	return "Piste libre, graine %d, rayon %d mm, %d virages (angles : %s), longueur %.1f m, %d obstacle(s)" % [
		graine, roundi(rayon * 1000.0), angles_virages.size(), ", ".join(angles), _s, obstacles.size()]

# ---------------------------------------------------------------- trace
func _tracer() -> void:
	var t := taille_case()
	var cible := float(_rng.randi_range(p.longueur_min, maxi(p.longueur_min, p.longueur_max))) * t
	var minimum := float(p.longueur_min) * t
	var court: Dictionary = {}     # meilleur trace valide mais plus court que le minimum
	for essai in RECOMMENCER_MAX:
		var etat := _essai(cible, minimum)
		if etat == 1:
			return
		if etat == 0 and (court.is_empty() or _s > court["s"]):
			court = _instantane()
	if not court.is_empty():
		push_warning("Piste libre : longueur minimale (%d cases) non atteinte, trace de %.1f m garde." % [p.longueur_min, court["s"]])
		_restaurer(court)
		return
	push_error("Piste libre : aucun trace valide, repli sur une droite.")
	_reinitialiser(Vector2(0.0, 3.4))
	_ajouter_droite(HAUTEUR_REPLI)

## Un trace complet. Retourne 1 = termine et assez long, 0 = termine mais trop court, -1 = echec (a recommencer).
func _essai(cible: float, minimum: float) -> int:
	var t := taille_case()
	_reinitialiser(Vector2(_rng.randf_range(-1.5, 1.5), 3.4))
	var depart_min := float(maxi(p.cases_libres_depart, 2)) * t
	var l0 := maxf(depart_min, _rng.randf_range(p.droite_libre_min, maxf(p.droite_libre_min, p.droite_libre_max)))
	if not _ajouter_droite(l0):
		return -1
	var apres_droite := true
	while true:
		if apres_droite:
			if _s >= cible:
				break
			if not _essayer_virage():
				return _terminer(minimum)
			apres_droite = false
		else:
			var forcee := _s >= cible
			if forcee or _rng.randf() < p.proba_tout_droit:
				if not _essayer_droite(cible):
					return _terminer(minimum)
				apres_droite = true
			elif not _essayer_virage():
				return _terminer(minimum)
	return 1 if _s >= minimum else 0

## Fin du trace apres un echec : la piste doit finir par une droite d'au moins 2 cases (barre d'arrivee).
func _terminer(minimum: float) -> int:
	var t := taille_case()
	var finie := false
	if _dernier_segment_droit:
		finie = droites[-1][2] >= 2.0 * t - 1e-9
	else:
		finie = _essayer_droite_finale(2.0 * t)
	if not finie:
		return -1
	return 1 if _s >= minimum else 0

## Tire un virage (angle et cote au hasard) jusqu'a ESSAIS_SEGMENT fois. Vrai si l'un est valide (et pose).
func _essayer_virage() -> bool:
	for i in ESSAIS_SEGMENT:
		var angle := _rng.randf_range(p.angle_min_deg, maxf(p.angle_min_deg, p.angle_max_deg))
		if _rng.randf() < 0.5:
			angle = -angle
		if _ajouter_virage(angle):
			return true
	return false

## Tire une droite (longueur entre droite_libre_min et droite_libre_max). Si elle atteint la cible elle doit faire au
## moins 2 cases (barre d'arrivee).
func _essayer_droite(cible: float) -> bool:
	var t := taille_case()
	for i in ESSAIS_SEGMENT:
		var longueur := _rng.randf_range(p.droite_libre_min, maxf(p.droite_libre_min, p.droite_libre_max))
		if _s + longueur >= cible:
			longueur = maxf(longueur, 2.0 * t)
		if _ajouter_droite(longueur):
			return true
	return false

func _essayer_droite_finale(minimum: float) -> bool:
	for i in ESSAIS_SEGMENT:
		var longueur := minimum + _rng.randf_range(0.0, maxf(0.0, p.droite_libre_max))
		if _ajouter_droite(longueur):
			return true
	return false

func _reinitialiser(debut: Vector2) -> void:
	_pos = debut
	_dir = Vector2(0, -1)
	_s = 0.0
	_dpts = PackedVector2Array([debut])
	_dcum = PackedFloat64Array([0.0])
	_grille = {}
	_indexer(0)
	ligne = PackedVector2Array([debut])
	droites = []
	angles_virages = []
	depart = debut + _dir * rayon
	_dernier_segment_droit = false

## Pose une droite de la longueur donnee si elle est valide.
func _ajouter_droite(longueur: float) -> bool:
	var n := maxi(1, ceili(longueur / PAS_POINTS))
	var pas := longueur / n
	var pts := PackedVector2Array()
	for k in range(1, n + 1):
		pts.append(_pos + _dir * (pas * k))
	if not _valide(pts, pas):
		return false
	droites.append([_pos, _dir, longueur])
	_poser(pts, pas)
	ligne.append(pts[pts.size() - 1])
	_dernier_segment_droit = true
	return true

## Pose un virage (angle en degres, > 0 = a droite) de rayon `rayon` si valide.
func _ajouter_virage(angle_deg: float) -> bool:
	var da := deg_to_rad(angle_deg)
	var cote := signf(da)
	var centre := _pos + _dir.rotated(cote * PI / 2.0) * rayon
	var arc := rayon * absf(da)
	var n := maxi(1, ceili(arc / PAS_POINTS))
	var pts := PackedVector2Array()
	for k in range(1, n + 1):
		pts.append(centre + (_pos - centre).rotated(da * k / n))
	if not _valide(pts, arc / n):
		return false
	var nouvelle_dir := _dir.rotated(da)
	_poser(pts, arc / n)
	ligne.append_array(pts)
	_dir = nouvelle_dir
	angles_virages.append(angle_deg)
	_dernier_segment_droit = false
	return true

## Points dans le carre utile et a au moins ecart_lignes_min de tous les points deja poses, sauf les voisins immediats
## (moins de FENETRE_VOISINS de chemin avant le point).
func _valide(pts: PackedVector2Array, pas: float) -> bool:
	var demi := TAILLE_SOL_UTILE / 2.0
	var ecart := p.ecart_lignes_min - TOLERANCE_ECART
	var ecart2 := ecart * ecart
	for k in pts.size():
		var q := pts[k]
		if absf(q.x) > demi or absf(q.y) > demi:
			return false
		var s := _s + pas * (k + 1) - FENETRE_VOISINS
		if s <= 0.0:
			continue
		var cx := floori(q.x / p.ecart_lignes_min)
		var cy := floori(q.y / p.ecart_lignes_min)
		for dx in range(-1, 2):
			for dy in range(-1, 2):
				var liste = _grille.get(Vector2i(cx + dx, cy + dy))
				if liste == null:
					continue
				for j in liste:
					if _dcum[j] < s and _dpts[j].distance_squared_to(q) < ecart2:
						return false
	return true

func _poser(pts: PackedVector2Array, pas: float) -> void:
	for k in pts.size():
		_s += pas
		_dpts.append(pts[k])
		_dcum.append(_s)
		_indexer(_dpts.size() - 1)
	_pos = pts[pts.size() - 1]

func _indexer(j: int) -> void:
	var c := Vector2i(floori(_dpts[j].x / p.ecart_lignes_min), floori(_dpts[j].y / p.ecart_lignes_min))
	if not _grille.has(c):
		_grille[c] = PackedInt32Array()
	_grille[c].append(j)

func _instantane() -> Dictionary:
	return {"s": _s, "ligne": ligne.duplicate(), "droites": droites.duplicate(true), "angles": angles_virages.duplicate(),
			"depart": depart, "dir": _dir}

func _restaurer(d: Dictionary) -> void:
	_s = d["s"]
	ligne = d["ligne"]
	droites = d["droites"]
	angles_virages = d["angles"]
	depart = d["depart"]
	_dir = d["dir"]

# ---------------------------------------------------------------- obstacles
## Longueur utile de la droite k (m) : la derniere perd une case (barre d'arrivee), comme sur la grille.
func _longueur_utile(k: int) -> float:
	var longueur: float = droites[k][2]
	return longueur - taille_case() if k == droites.size() - 1 else longueur

## Debut permis de la face sur la droite k : la premiere droite garde les cases du depart libres.
func _debut_permis(k: int) -> float:
	var avant := place_avant_obstacle + p.marge_droite_avant
	if k == 0:
		return float(maxi(p.cases_libres_depart, 2)) * taille_case() + avant
	return avant

## Obstacles seulement sur les segments droits (meme regle de place que la grille, voir GenerateurPiste._placer_obstacles).
func _placer_obstacles() -> void:
	_vider_obstacles()
	var t := taille_case()
	for k in droites.size():
		var longueur := _longueur_utile(k)
		var avant := place_avant_obstacle + p.marge_droite_avant
		var s_libre := _debut_permis(k)
		for i in ceili(longueur / t):
			if obstacles.size() >= p.nb_obstacles: return
			if _rng.randf() >= p.proba_obstacle: continue
			var taille := _tirer_taille()
			var s_min := maxf(s_libre, i * t)
			var s_max := minf((i + 1) * t, longueur - p.droite_apres_min) - taille.z
			if s_max < s_min: continue
			var s_face := _rng.randf_range(s_min, s_max)
			_ajouter_obstacle_libre(k, s_face, taille, _rng.randf_range(-p.decalage_obstacle_max, p.decalage_obstacle_max))
			s_libre = s_face + taille.z + p.droite_apres_min + avant

func _vider_obstacles() -> void:
	obstacles.clear()
	tailles_obstacles.clear()
	decalages_obstacles.clear()
	positions_obstacles.clear()
	obstacles_monde.clear()

## Obstacle dont la face avant est a s_face (m) du debut de la droite k.
func _ajouter_obstacle_libre(k: int, s_face: float, taille: Vector3, decalage: float) -> void:
	var p0: Vector2 = droites[k][0]
	var dir: Vector2 = droites[k][1]
	var longueur: float = droites[k][2]
	var s_centre := s_face + taille.z / 2.0
	var centre := p0 + dir * s_centre + Vector2(-dir.y, dir.x) * decalage   # decalage > 0 = a droite
	obstacles.append(k)
	tailles_obstacles.append(taille)
	decalages_obstacles.append(decalage)
	positions_obstacles.append(s_centre - longueur / 2.0)
	obstacles_monde.append([centre, dir])

# ---------------------------------------------------------------- scenario
## Piste decrite par desc["trace"] : [["T", longueur_mm], ["G", angle_deg], ["D", angle_deg], ...], desc["rayon_mm"].
## Pas de hasard. Retourne false si le trace sort du sol ou se croise.
func depuis_scenario(desc: Dictionary) -> bool:
	graine = 0
	rayon = float(desc.get("rayon_mm", 120.0)) / 1000.0
	colonnes = 0
	lignes = 0
	matrice.clear()
	chemin.clear()
	_reinitialiser(Vector2.ZERO)
	# Trace depuis l'origine, puis translation pour centrer sa boite d'encombrement sur le sol
	for etape in desc["trace"]:
		var lettre := String(etape[0])
		var valeur := float(etape[1])
		match lettre:
			"T":
				_poser_sans_verifier_droite(valeur / 1000.0)
			"G": _poser_sans_verifier_virage(-valeur)
			"D": _poser_sans_verifier_virage(valeur)
			_:
				push_error("Scenario %s : lettre « %s » inconnue (T, G ou D)." % [desc["nom"], lettre])
				return false
	# Centrage
	var mini_p := ligne[0]
	var maxi_p := ligne[0]
	for q in _dpts:
		mini_p = Vector2(minf(mini_p.x, q.x), minf(mini_p.y, q.y))
		maxi_p = Vector2(maxf(maxi_p.x, q.x), maxf(maxi_p.y, q.y))
	var decalage_trace := -(mini_p + maxi_p) / 2.0
	if maxf(maxi_p.x - mini_p.x, maxi_p.y - mini_p.y) > TAILLE_SOL_UTILE:
		push_error("Scenario %s : piste plus grande que le sol." % desc["nom"])
		return false
	_translater(decalage_trace)
	# Croisement : meme test d'ecart que le trace aleatoire, sur le trace complet
	if not _sans_croisement():
		push_error("Scenario %s : le chemin se croise (ou deux lignes a moins de %d mm)." % [desc["nom"],
				roundi(p.ecart_lignes_min * 1000.0)])
		return false
	return _placer_obstacles_trace(desc)

func _poser_sans_verifier_droite(longueur: float) -> void:
	var n := maxi(1, ceili(longueur / PAS_POINTS))
	var pas := longueur / n
	var pts := PackedVector2Array()
	for k in range(1, n + 1):
		pts.append(_pos + _dir * (pas * k))
	droites.append([_pos, _dir, longueur])
	_poser(pts, pas)
	ligne.append(pts[pts.size() - 1])

func _poser_sans_verifier_virage(angle_deg: float) -> void:
	var da := deg_to_rad(angle_deg)
	var centre := _pos + _dir.rotated(signf(da) * PI / 2.0) * rayon
	var arc := rayon * absf(da)
	var n := maxi(1, ceili(arc / PAS_POINTS))
	var pts := PackedVector2Array()
	for k in range(1, n + 1):
		pts.append(centre + (_pos - centre).rotated(da * k / n))
	_poser(pts, arc / n)
	ligne.append_array(pts)
	_dir = _dir.rotated(da)
	angles_virages.append(angle_deg)

func _translater(d: Vector2) -> void:
	for i in ligne.size():
		ligne[i] += d
	for i in droites.size():
		droites[i][0] = droites[i][0] + d
	for i in _dpts.size():
		_dpts[i] += d
	_pos += d
	depart = ligne[0] + Vector2(0, -1) * rayon

func _sans_croisement() -> bool:
	var demi := TAILLE_SOL_UTILE / 2.0
	var ecart := p.ecart_lignes_min - TOLERANCE_ECART
	var ecart2 := ecart * ecart
	var g := {}
	for j in _dpts.size():
		var q := _dpts[j]
		if absf(q.x) > demi or absf(q.y) > demi:
			return false
		var s := _dcum[j] - FENETRE_VOISINS
		var cx := floori(q.x / p.ecart_lignes_min)
		var cy := floori(q.y / p.ecart_lignes_min)
		if s > 0.0:
			for dx in range(-1, 2):
				for dy in range(-1, 2):
					var liste = g.get(Vector2i(cx + dx, cy + dy))
					if liste == null:
						continue
					for i in liste:
						if _dcum[i] < s and _dpts[i].distance_squared_to(q) < ecart2:
							return false
		var c := Vector2i(cx, cy)
		if not g.has(c):
			g[c] = PackedInt32Array()
		g[c].append(j)
	return true

## Meme logique que GenerateurPiste._placer_obstacles_scenario, en indices de segments droits.
func _placer_obstacles_trace(desc: Dictionary) -> bool:
	_vider_obstacles()
	var avant := place_avant_obstacle + p.marge_droite_avant
	var fin_prec := 0.0
	var troncon_prec := -1
	for o in desc.get("obstacles", []):
		var k: int = o.get("troncon", 0)
		if k >= droites.size():
			push_error("Scenario %s : pas de troncon droit n° %d (il y en a %d)." % [desc["nom"], k, droites.size()])
			return false
		var longueur := _longueur_utile(k)
		var debut := _debut_permis(k)
		var mm: Array = o.get("taille_mm", [100, 150, 50])
		var taille := Vector3(mm[0], mm[1], mm[2]) / 1000.0
		var face = o.get("face", "min")
		var s_face := 0.0
		if face is String:
			match face:
				"min": s_face = debut
				"max": s_face = longueur - p.droite_apres_min - taille.z
				"apres_precedent":
					if troncon_prec != k:
						push_error("Scenario %s : « apres_precedent » sans obstacle precedent sur ce troncon." % desc["nom"])
						return false
					s_face = fin_prec + p.droite_apres_min + avant
				_:
					push_error("Scenario %s : face « %s » inconnue." % [desc["nom"], face])
					return false
		else:
			s_face = float(face) / 1000.0
		if s_face < debut - 1e-6 or s_face + taille.z > longueur - p.droite_apres_min + 1e-6:
			push_warning("Scenario %s : obstacle %d hors de la place exigee (face a %d mm, permis %d a %d mm)." % [
				desc["nom"], obstacles.size(), roundi(s_face * 1000.0), roundi(debut * 1000.0),
				roundi((longueur - p.droite_apres_min - taille.z) * 1000.0)])
		_ajouter_obstacle_libre(k, s_face, taille, float(o.get("decalage_mm", 0.0)) / 1000.0)
		fin_prec = s_face + taille.z
		troncon_prec = k
	return true

# ---------------------------------------------------------------- outils
## Vrai si le point (x, z du monde, en m) se projette sur un segment droit, a moins de `rayon` de distance laterale.
func sur_case_droite(x: float, z: float) -> bool:
	var q := Vector2(x, z)
	for d in droites:
		var v: Vector2 = q - d[0]
		var dir: Vector2 = d[1]
		var s := v.dot(dir)
		if s >= 0.0 and s <= d[2] and absf(v.x * -dir.y + v.y * dir.x) < rayon:
			return true
	return false
