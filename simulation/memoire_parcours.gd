## Ce dont la voiture se souvient de son trajet, dans le repere global de l'odometrie (mm, voir Odometrie) :
##   - la ligne vue par les sondes sur le dernier metre (direction de la ligne, ligne entrante d'une manoeuvre) ;
##   - le chemin deja parcouru sur la ligne (4 derniers metres) : detection du contresens ;
##   - les obstacles deja contournes et les endroits des derniers demi-tours.
## Sens de marche : la piste ne se croisant jamais, une voiture qui roule a l'envers finit devant un obstacle qu'elle a
## deja contourne, vu de dos, ou remonte un morceau de ligne deja parcouru (journal Godot du 6 novembre).
class_name MemoireParcours
extends RefCounted

const PAS_MM := 20.0                 # un point memorise tous les PAS_MM
const POINTS_LIGNE := 50             # 50 x 20 mm = 1 m de ligne entrante
const POINTS_CHEMIN := 200           # 4 m de chemin parcouru
const POINTS_CHEMIN_RECENTS := 15    # les derniers points (300 mm) ne comptent pas : la voiture vient d'y passer
const DEMI_TOURS_MEMORISES := 6

var rayon_chemin_mm := 40.0          # ligne vue a moins de ca d'un point deja parcouru : c'est ce morceau
var rayon_meme_ligne_mm := 100.0     # ligne vue a moins de ca de la ligne memorisee : c'est la ligne entrante
var rayon_meme_obstacle_mm := 200.0  # echo a moins de ca d'un obstacle deja contourne : c'est lui, vu de dos
var obstacles_memorises := 8
var rayon_meme_demi_tour_mm := 300.0
var demi_tours_max_au_meme_endroit := 2  # ensuite on continue : tourner sur place ne resoudrait rien

var ligne: Array[Vector2] = []       # points de la ligne vus par les sondes (dernier metre)
var obstacles: Array[Vector2] = []   # centres des obstacles contournes
var obstacle_hors_ligne := Vector2(INF, INF)  # dernier obstacle evite alors que la voiture n'etait pas sur la ligne

var _chemin: Array[Vector2] = []
var _points_passage := 0             # points ajoutes a _chemin depuis la derniere reprise du suivi
var _demi_tours: Array[Vector2] = []
var _robot: ReglagesRobot
var _odo: Odometrie

func _init(robot: ReglagesRobot, odo: Odometrie) -> void:
	_robot = robot
	_odo = odo


# ----- Ligne et chemin parcouru

## Point de la ligne sous les sondes (repere global) : la barre est a distance_sondes_mm devant l'essieu, e_mm a droite.
func point_sous_sondes(e_mm: float) -> Vector2:
	var d := _robot.distance_sondes_mm
	var c := _odo.cap_global
	return Vector2(_odo.x_global + d * sin(c) + e_mm * cos(c), _odo.y_global + d * cos(c) - e_mm * sin(c))

## Garde le point de la ligne vu par les sondes, un tous les PAS_MM.
func memoriser(e_mm: float) -> void:
	var p := point_sous_sondes(e_mm)
	if ligne.is_empty() or p.distance_to(ligne[-1]) > PAS_MM:
		ligne.append(p)
		if ligne.size() > POINTS_LIGNE:
			ligne.pop_front()
		_chemin.append(p)
		_points_passage += 1
		if _chemin.size() > POINTS_CHEMIN:
			_chemin.pop_front()

## Nouveau passage en suivi : les points suivants pourront etre annules (annuler_passage).
func nouveau_passage() -> void:
	_points_passage = 0

## Oublie le passage en cours : il a ete enregistre a contresens et retournerait la voiture sans fin.
func annuler_passage() -> void:
	_chemin.resize(maxi(0, _chemin.size() - _points_passage))
	_points_passage = 0

## Accord entre le cap et le sens dans lequel ce morceau de ligne a deja ete parcouru (1 = meme sens, -1 = inverse).
## NAN si la ligne sous les sondes n'est pas sur le chemin deja parcouru.
func sens_sur_chemin(e_mm: float) -> float:
	var q := point_sous_sondes(e_mm)
	var avant := Vector2(sin(_odo.cap_global), cos(_odo.cap_global))
	for i in range(_chemin.size() - POINTS_CHEMIN_RECENTS - 1, 2, -1):
		if q.distance_squared_to(_chemin[i]) < rayon_chemin_mm * rayon_chemin_mm:
			var sens := _chemin[i] - _chemin[i - 3]
			return sens.normalized().dot(avant) if sens.length() > 1.0 else NAN
	return NAN

## Vrai si la voiture remonte un morceau de ligne deja parcouru, en sens inverse.
func remonte_le_chemin(e_mm: float) -> bool:
	var accord := sens_sur_chemin(e_mm)
	return not is_nan(accord) and accord < -0.5

## Vrai si la voiture roule dans le bon sens sur un morceau deja parcouru. Devant un obstacle deja contourne, elle doit
## alors le recontourner : un demi-tour la renverrait a contresens, puis la surveillance la retournerait, sans fin.
func en_avant_sur_chemin(somme: float, n: int) -> bool:
	if n == 0:
		return false
	var accord := sens_sur_chemin(somme / n)
	return not is_nan(accord) and accord > 0.3

## Vrai si la ligne vue est la ligne memorisee et que la voiture y roule a contresens.
func sur_ligne_a_contresens(e_mm: float) -> bool:
	var q := point_sous_sondes(e_mm)
	var cap := _odo.cap_global
	for i in range(ligne.size() - 1, 0, -1):
		if q.distance_to(ligne[i]) < rayon_meme_ligne_mm:
			var sens := ligne[i] - ligne[maxi(0, i - 5)]
			return sens.length() > 1.0 and sens.normalized().dot(Vector2(sin(cap), cos(cap))) < -0.3
	return false

## Sens de marche sur la ligne, d'apres les derniers points (environ 100 mm). Pour viser la ligne, pas pour la mesurer.
func sens_ligne() -> Vector2:
	if ligne.size() >= 2:
		var sens := ligne[-1] - ligne[maxi(0, ligne.size() - 6)]
		if sens.length() > 1.0:
			return sens.normalized()
	return Vector2(sin(_odo.cap_global), cos(_odo.cap_global))

## Direction de la derniere partie DROITE de la ligne, orientee dans le sens de marche : moindres carres (axe principal)
## sur les derniers points, en remontant tant que tous restent a moins de tolerance_droite_mm de la droite ajustee
## (au-dela, la ligne tournait : ces points sont dans le virage precedent).
## sens_ligne() (2 points a 100 mm, sondes a +/- 9,5 mm pres) se trompe jusqu'a ~10 deg. Obstacle juste apres un
## virage (banc du 12 nov., graine 7920) : un ajustement sur 400 mm melangeait virage et droite.
var points_direction_max := 20         # 20 x 20 mm = 400 mm au plus
var tolerance_droite_mm := 12.0        # dispersion des points (sondes espacees de 19 mm : +/- 9,5 mm) + marge
## Retourne [centre, direction] : un point de la droite (centre des points retenus) et sa direction unitaire.
func derniere_droite() -> Array:
	var retenus: Array[Vector2] = ligne.slice(maxi(0, ligne.size() - 2))
	var meilleure := sens_ligne()
	for k in range(5, mini(points_direction_max, ligne.size()) + 1):
		var pts := ligne.slice(ligne.size() - k)
		var d := _axe_principal(pts)
		if _ecart_max(pts, d) > tolerance_droite_mm:
			break
		meilleure = d
		retenus = pts
	return [_centre(retenus), meilleure]

static func _centre(pts: Array[Vector2]) -> Vector2:
	var m := Vector2.ZERO
	for p in pts:
		m += p
	return m / float(maxi(1, pts.size()))

## Axe principal (moindres carres) des points, oriente du premier vers le dernier.
static func _axe_principal(pts: Array[Vector2]) -> Vector2:
	var m := _centre(pts)
	var sxx := 0.0
	var syy := 0.0
	var sxy := 0.0
	for p in pts:
		var q: Vector2 = p - m
		sxx += q.x * q.x
		syy += q.y * q.y
		sxy += q.x * q.y
	var theta := 0.5 * atan2(2.0 * sxy, sxx - syy)
	var dir := Vector2(cos(theta), sin(theta))
	return -dir if dir.dot(pts[-1] - pts[0]) < 0.0 else dir

## Plus grande distance d'un point a la droite de direction d passant par le centre des points (mm).
static func _ecart_max(pts: Array[Vector2], d: Vector2) -> float:
	var m := _centre(pts)
	var normale := Vector2(-d.y, d.x)
	var e := 0.0
	for p in pts:
		e = maxf(e, absf((p - m).dot(normale)))
	return e

## La ligne memorisee, exprimee dans le repere de la manoeuvre qui commence (origine : essieu arriere, y vers l'avant,
## x vers la droite) : c'est la « ligne entrante » du contournement.
func vers_repere_manoeuvre() -> Array[Vector2]:
	var c := _odo.cap_global
	var avant := Vector2(sin(c), cos(c))
	var droite := Vector2(cos(c), -sin(c))
	var origine := Vector2(_odo.x_global, _odo.y_global)
	var res: Array[Vector2] = []
	for p in ligne:
		res.append(Vector2((p - origine).dot(droite), (p - origine).dot(avant)))
	return res


# ----- Obstacles contournes et demi-tours

func noter_obstacle(centre: Vector2) -> void:
	obstacles.append(centre)
	if obstacles.size() > obstacles_memorises:
		obstacles.pop_front()

## Vrai si l'echo a d_mm devant tombe sur un obstacle deja contourne : la voiture le revoit de dos, elle roule a l'envers.
func obstacle_deja_contourne(d_mm: float) -> bool:
	var echo := _odo.point_devant_global(_robot.longueur_avant_mm + d_mm)
	for o in obstacles:
		if echo.distance_to(o) < rayon_meme_obstacle_mm:
			return true
	return false

func noter_demi_tour() -> void:
	_demi_tours.append(Vector2(_odo.x_global, _odo.y_global))
	if _demi_tours.size() > DEMI_TOURS_MEMORISES:
		_demi_tours.pop_front()

## Faux apres plusieurs demi-tours au meme endroit : la voiture tournerait sur place indefiniment.
func demi_tour_autorise() -> bool:
	var ici := Vector2(_odo.x_global, _odo.y_global)
	var deja := 0
	for p in _demi_tours:
		if ici.distance_to(p) < rayon_meme_demi_tour_mm:
			deja += 1
	return deja < demi_tours_max_au_meme_endroit
