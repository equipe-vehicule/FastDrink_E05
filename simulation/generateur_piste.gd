## Matrice de la piste aleatoire : un chemin sans croisement, puis des obstacles sur ce chemin.
## Pure logique (aucun noeud 3D) : ConstructionPiste transforme le resultat en piste.
##
## Le chemin part de la derniere ligne et monte (Vector2i.UP = cap -Z dans le monde, cap du vehicule au depart).
## Contraintes gardees (cahier des charges et physique) :
##   - ruban de largeur_ruban, virages de rayon >= rayon_virage (120 mm), pas de croisement de lignes ;
##   - les 2 premieres cases sont droites et sans obstacle (la voiture est posee dessus au depart).
## Reglable dans ParametresPiste :
##   - rayon des virages tire au hasard pour chaque piste entre rayon_virage et rayon_virage_max ;
##   - cases voisines autorisees : demi-tours serres, lignes paralleles a une case d'ecart ;
##   - obstacles (precisions du professeur) : seulement sur des lignes droites, centres sur la ligne, n'importe ou le
##     long de la droite, avec la place exigee avant et apres (voir _placer_obstacles).
## Les virages sont des quarts de cercle (90 deg) : le guide ne donne que le rayon minimal, pas d'angle.
class_name GenerateurPiste
extends RefCounted

enum { VIDE, RUBAN, OBSTACLE }
const DIRECTIONS := [Vector2i.UP, Vector2i.DOWN, Vector2i.LEFT, Vector2i.RIGHT]
const ESSAIS_MAX := 20000
const TAILLE_SOL_UTILE := 7.6     # m : la grille doit tenir sur le sol de 8 m

var p: ParametresPiste
## Ligne droite que la sequence arret + recul occupe devant un obstacle (m). Fournie par qui cree le generateur
## (Controleur.place_avant_obstacle_mm) : suit les distances d'arret et de recul de ReglagesEpreuve.
var place_avant_obstacle := 0.0
var graine := 0
var rayon := 0.12                 # rayon des virages de cette piste (m)
var colonnes := 0                 # taille reelle de la grille (reduite si les cases sont grandes)
var lignes := 0
var matrice: Array = []           # matrice[y][x]
var chemin: Array[Vector2i] = []  # cases dans l'ordre de parcours
var obstacles: Array[int] = []    # indices dans chemin
var tailles_obstacles: Array[Vector3] = []   # meme ordre que obstacles
var decalages_obstacles: Array[float] = []   # m, decalage lateral par rapport a la ligne (> 0 = a droite)
var positions_obstacles: Array[float] = []   # m, position du centre le long de la ligne, depuis le centre de la case

var _rng := RandomNumberGenerator.new()

func _init(parametres: ParametresPiste) -> void:
	p = parametres

func generer(graine_choisie: int) -> void:
	graine = graine_choisie
	_rng.seed = graine
	rayon = _rng.randf_range(p.rayon_virage, maxf(p.rayon_virage, p.rayon_virage_max))
	var max_cases := int(TAILLE_SOL_UTILE / taille_case())
	colonnes = mini(p.colonnes, max_cases)
	lignes = mini(p.lignes, max_cases)
	_vider_matrice()
	_tracer_chemin()
	_placer_obstacles()

func taille_case() -> float:
	return 2.0 * rayon

## Matrice lisible dans la console : D depart, # ruban, X obstacle, . vide
func en_texte() -> String:
	var texte := PackedStringArray(["Piste aleatoire, graine %d, rayon %d mm, %d cases, %d obstacle(s)" % [
		graine, roundi(rayon * 1000.0), chemin.size(), obstacles.size()]])
	for y in lignes:
		var ligne := ""
		for x in colonnes:
			if chemin.size() > 0 and Vector2i(x, y) == chemin[0]: ligne += "D "
			else: ligne += [". ", "# ", "X "][matrice[y][x]]
		texte.append(ligne)
	return "\n".join(texte)

# ---------------------------------------------------------------- chemin
## Parcours en profondeur avec retour en arriere : on avance au hasard,
## et quand on est coince on recule d'une case pour essayer une autre direction.
func _tracer_chemin() -> void:
	var cible := _rng.randi_range(p.longueur_min, maxi(p.longueur_min, p.longueur_max))
	var depart := Vector2i(_rng.randi_range(1, colonnes - 2), lignes - 1)
	chemin = [depart]
	_marquer(depart, RUBAN)
	var choix: Array = [_candidats(depart, Vector2i.UP)]   # choix[i] = cases encore a essayer depuis chemin[i]
	var meilleur := chemin.duplicate()
	var essais := 0
	while chemin.size() < cible and essais < ESSAIS_MAX:
		essais += 1
		var options: Array = choix[-1]
		if options.is_empty():
			if chemin.size() == 1: break
			choix.pop_back()
			_marquer(chemin.pop_back(), VIDE)
			continue
		var suivante: Vector2i = options.pop_front()
		chemin.append(suivante)
		_marquer(suivante, RUBAN)
		choix.append(_candidats(suivante, suivante - chemin[-2]))
		if chemin.size() > meilleur.size(): meilleur = chemin.duplicate()
	# Cible inatteignable (grille trop petite) : on garde le plus long chemin trouve
	if chemin.size() < meilleur.size():
		chemin = meilleur
		_vider_matrice()
		for c in chemin: _marquer(c, RUBAN)
	if chemin.size() < p.longueur_min:
		push_warning("Piste aleatoire : seulement %d cases (minimum demande %d). Agrandir la grille ?" % [chemin.size(), p.longueur_min])

## Cases possibles apres `case` quand on y arrive dans la direction `dir`, deja melangees.
func _candidats(case: Vector2i, dir: Vector2i) -> Array:
	var tout_droit := case + dir
	if chemin.size() <= 2:
		return [tout_droit] if _libre(tout_droit, case) else []
	var options := []
	for d in DIRECTIONS:
		if d != -dir and _libre(case + d, case):
			options.append(case + d)
	_melanger(options)
	if tout_droit in options and _rng.randf() < p.proba_tout_droit:
		options.erase(tout_droit)
		options.push_front(tout_droit)
	# Ligne droite minimale : tant que la ligne droite en cours est trop courte, on continue tout droit si possible.
	if tout_droit in options and _longueur_droite(dir) < p.longueur_droite_min:
		options.erase(tout_droit)
		options.push_front(tout_droit)
	return options

## Nombre de deplacements consecutifs dans la direction dir a la fin du chemin.
func _longueur_droite(dir: Vector2i) -> int:
	var n := 0
	while n + 1 < chemin.size() and chemin[-1 - n] - chemin[-2 - n] == dir:
		n += 1
	return n

## Vrai si `v` est dans la grille et vide. Si les cases voisines sont interdites,
## `v` ne doit en plus toucher aucune case du chemin a part `depuis`.
func _libre(v: Vector2i, depuis: Vector2i) -> bool:
	if not _dans_grille(v) or matrice[v.y][v.x] != VIDE: return false
	if p.autoriser_cases_voisines: return true
	for d in DIRECTIONS:
		var voisin: Vector2i = v + d
		if voisin != depuis and _dans_grille(voisin) and matrice[voisin.y][voisin.x] != VIDE:
			return false
	return true

# ---------------------------------------------------------------- obstacles
## Obstacles seulement sur des troncons de lignes droites, a une position tiree au hasard le long de la droite, avec au
## moins place_avant_obstacle + marge_droite_avant de ligne droite avant la face et droite_apres_min apres l'arriere (place pour l'arret, le
## recul et le contournement). Un obstacle peut donc etre juste avant ou juste apres un virage, et plusieurs obstacles
## peuvent se suivre sur une meme droite. Chaque case droite recoit un obstacle avec la probabilite proba_obstacle.
func _placer_obstacles() -> void:
	obstacles.clear()
	tailles_obstacles.clear()
	decalages_obstacles.clear()
	positions_obstacles.clear()
	var t := taille_case()
	for troncon in _troncons_droits():
		var debut: int = troncon[0]
		var longueur: float = (troncon[1] - debut + 1) * t
		var avant := place_avant_obstacle + p.marge_droite_avant
		var s_libre := avant                   # premiere position possible de la face (m depuis le debut du troncon)
		for i in range(debut, troncon[1] + 1):
			if obstacles.size() >= p.nb_obstacles: return
			if _rng.randf() >= p.proba_obstacle: continue
			var taille := _tirer_taille()
			# Face dans la case i : l'obstacle entier tient dans la case, avec la place exigee avant et apres
			var s_min := maxf(s_libre, (i - debut) * t)
			var s_max := minf((i - debut + 1) * t, longueur - p.droite_apres_min) - taille.z
			if s_max < s_min: continue
			var s_face := _rng.randf_range(s_min, s_max)
			_ajouter_obstacle(i, debut, s_face, taille, _rng.randf_range(-p.decalage_obstacle_max, p.decalage_obstacle_max))
			s_libre = s_face + taille.z + p.droite_apres_min + avant

## Obstacle dont la face avant est a s_face (m) du debut du troncon qui commence a la case `debut`, rattache a la case i.
func _ajouter_obstacle(i: int, debut: int, s_face: float, taille: Vector3, decalage: float) -> void:
	var t := taille_case()
	obstacles.append(i)
	tailles_obstacles.append(taille)
	positions_obstacles.append(s_face + taille.z / 2.0 - ((i - debut) * t + t / 2.0))
	decalages_obstacles.append(decalage)
	_marquer(chemin[i], OBSTACLE)

# ---------------------------------------------------------------- scenario
## Piste decrite par un scenario (simulation/scenarios.gd) : meme resultat que generer(), sans hasard.
## La grille entoure le chemin d'une case vide de chaque cote. Retourne false si le scenario est invalide.
func depuis_scenario(desc: Dictionary) -> bool:
	graine = 0
	rayon = float(desc.get("rayon_mm", 120.0)) / 1000.0
	var cases: Array[Vector2i] = [Vector2i.ZERO]
	var dir := Vector2i.UP
	for lettre in String(desc["chemin"]):
		match lettre:
			"T": pass
			"G": dir = Vector2i(dir.y, -dir.x)
			"D": dir = Vector2i(-dir.y, dir.x)
			_:
				push_error("Scenario %s : lettre « %s » inconnue (T, G ou D)." % [desc["nom"], lettre])
				return false
		var suivante := cases[-1] + dir
		if suivante in cases:
			push_error("Scenario %s : le chemin se croise." % desc["nom"])
			return false
		cases.append(suivante)
	var mini_c := cases[0]
	var maxi_c := cases[0]
	for c in cases:
		mini_c = Vector2i(mini(mini_c.x, c.x), mini(mini_c.y, c.y))
		maxi_c = Vector2i(maxi(maxi_c.x, c.x), maxi(maxi_c.y, c.y))
	colonnes = maxi_c.x - mini_c.x + 3
	lignes = maxi_c.y - mini_c.y + 3
	if maxi(colonnes, lignes) * taille_case() > TAILLE_SOL_UTILE:
		push_error("Scenario %s : piste plus grande que le sol." % desc["nom"])
		return false
	_vider_matrice()
	chemin.clear()
	for c in cases:
		chemin.append(c - mini_c + Vector2i.ONE)
		_marquer(chemin[-1], RUBAN)
	return _placer_obstacles_scenario(desc)

func _placer_obstacles_scenario(desc: Dictionary) -> bool:
	obstacles.clear()
	tailles_obstacles.clear()
	decalages_obstacles.clear()
	positions_obstacles.clear()
	var t := taille_case()
	var troncons := _troncons_droits()
	var avant := place_avant_obstacle + p.marge_droite_avant
	var fin_prec := 0.0      # arriere de l'obstacle precedent (m depuis le debut de son troncon)
	var troncon_prec := -1
	for o in desc.get("obstacles", []):
		var k: int = o.get("troncon", 0)
		if k >= troncons.size():
			push_error("Scenario %s : pas de troncon droit n° %d (il y en a %d)." % [desc["nom"], k, troncons.size()])
			return false
		var debut: int = troncons[k][0]
		var longueur: float = (troncons[k][1] - debut + 1) * t
		var mm: Array = o.get("taille_mm", [100, 150, 50])
		var taille := Vector3(mm[0], mm[1], mm[2]) / 1000.0
		var face = o.get("face", "min")
		var s_face := 0.0
		if face is String:
			match face:
				"min": s_face = avant
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
		if s_face < avant - 1e-6 or s_face + taille.z > longueur - p.droite_apres_min + 1e-6:
			push_warning("Scenario %s : obstacle %d hors de la place exigee (face a %d mm, permis %d a %d mm)." % [
				desc["nom"], obstacles.size(), roundi(s_face * 1000.0), roundi(avant * 1000.0),
				roundi((longueur - p.droite_apres_min - taille.z) * 1000.0)])
		var i := debut + clampi(int((s_face + taille.z / 2.0) / t), 0, troncons[k][1] - debut)
		_ajouter_obstacle(i, debut, s_face, taille, float(o.get("decalage_mm", 0.0)) / 1000.0)
		fin_prec = s_face + taille.z
		troncon_prec = k
	return true

## Taille (largeur, hauteur, profondeur), puis pose si faces_aleatoires (tiree APRES la taille : avec faces_aleatoires
## faux, la suite des tirages, donc la piste, est la meme qu'avant le 13 nov.).
func _tirer_taille() -> Vector3:
	var t_min := p.taille_obstacle
	var t_max := p.taille_obstacle_max
	var t := Vector3(_rng.randf_range(t_min.x, maxf(t_min.x, t_max.x)), _rng.randf_range(t_min.y, maxf(t_min.y, t_max.y)),
			_rng.randf_range(t_min.z, maxf(t_min.z, t_max.z)))
	if p.faces_aleatoires:
		t = poser(t, _rng.randi_range(0, 5))
	return t

## Pose n (0 a 5) du bloc : permutation de ses dimensions (largeur, hauteur, profondeur). 0 = tel quel (debout).
static func poser(t: Vector3, n: int) -> Vector3:
	return [Vector3(t.x, t.y, t.z), Vector3(t.z, t.y, t.x), Vector3(t.y, t.x, t.z), Vector3(t.z, t.x, t.y),
			Vector3(t.x, t.z, t.y), Vector3(t.y, t.z, t.x)][n]

## Troncons de cases droites consecutives [premiere, derniere], hors des cases du depart et de la case d'arrivee.
func _troncons_droits() -> Array:
	var troncons := []
	var debut := -1
	for i in range(maxi(p.cases_libres_depart, 2), chemin.size() - 1):
		if _case_droite(i):
			if debut < 0: debut = i
		elif debut >= 0:
			troncons.append([debut, i - 1])
			debut = -1
	if debut >= 0:
		troncons.append([debut, chemin.size() - 2])
	return troncons

## Case droite (la precedente, elle et la suivante sont alignees), avec une case de chaque cote dans la grille
## (le contournement sort de la case).
func _case_droite(i: int) -> bool:
	var dir := chemin[i] - chemin[i - 1]
	if chemin[i + 1] - chemin[i] != dir: return false
	var cote := Vector2i(dir.y, dir.x)
	return _dans_grille(chemin[i] + cote) and _dans_grille(chemin[i] - cote)

## Vrai si le point (x, z du monde, en m) est sur une case droite du chemin (meme direction en entree et en sortie).
## Meme repere que ConstructionPiste : grille centree sur l'origine du monde.
func sur_case_droite(x: float, z: float) -> bool:
	var t := taille_case()
	var c := Vector2i(((Vector2(x, z) + Vector2(colonnes, lignes) * t / 2.0) / t).floor())
	var i := chemin.find(c)
	if i < 0:
		return false
	var d_in := chemin[i] - chemin[i - 1] if i > 0 else Vector2i.UP
	var d_out := chemin[i + 1] - chemin[i] if i < chemin.size() - 1 else d_in
	return d_in == d_out

# ---------------------------------------------------------------- outils
func _vider_matrice() -> void:
	matrice.clear()
	for y in lignes:
		var ligne := []
		ligne.resize(colonnes)
		ligne.fill(VIDE)
		matrice.append(ligne)

func _marquer(c: Vector2i, valeur: int) -> void:
	matrice[c.y][c.x] = valeur

func _dans_grille(c: Vector2i) -> bool:
	return c.x >= 0 and c.x < colonnes and c.y >= 0 and c.y < lignes

# Melange avec le generateur de la graine (shuffle() utiliserait le hasard global)
func _melanger(a: Array) -> void:
	for i in range(a.size() - 1, 0, -1):
		var j := _rng.randi_range(0, i)
		var tmp = a[i]; a[i] = a[j]; a[j] = tmp
