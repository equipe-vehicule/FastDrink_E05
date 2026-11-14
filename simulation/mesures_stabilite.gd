## Mesures de stabilite d'un essai : activite du braquage, oscillation du braquage comparee a la frequence propre de la
## bille, marge de la bille. Pure logique (aucun noeud) : portable en Python.
##
## Ce sont des CHOIX DE METHODE, pas des exigences (le guide ne chiffre pas la stabilite) :
##   - activite = somme des |variations de l'angle reel des roues| / distance parcourue (deg/m), sur tout l'essai ;
##   - vitesse de braquage RMS (deg/s), sur tout l'essai ;
##   - spectre d'amplitude de l'angle des roues pendant le suivi de ligne (manoeuvres exclues) : chaque segment d'au
##     moins SEGMENT_MIN_S est analyse a part (on ne recolle pas les segments), apres retrait de la moyenne glissante
##     sur FENETRE_TENDANCE_S et fenetre de Hann. Deux mesures distinctes :
##       * OSCILLATION DU SUIVI : segments de suivi en ligne droite seulement (« en_droite », donne par qui appelle,
##         depuis la geometrie de la piste). Un virage est une impulsion de braquage d'environ 1 s : son spectre
##         tombe pres de la frequence de la bille et aucun filtre ne le separe d'une oscillation (essai du 13 nov.).
##         Frequence dominante = pic le plus fort (bande F_MIN..F_MAX) ;
##       * EXCITATION DE LA BILLE : tout le suivi, virages compris (voulu : un virage excite vraiment la bille) ;
##         plus forte amplitude dans la bande f_bille +/- DEMI_BANDE_BILLE_HZ.
##   - Le retrait de la moyenne glissante attenue un peu les basses frequences : environ -13 % a 1,2 Hz (sinusoide
##     de 3 deg mesuree a 2,6 deg), moins de 1 % au-dessus de 4 Hz.
##   - Sans segment assez long, la mesure vaut NAN (« non mesure »), pas 0.
extends RefCounted

const FREQ_ECHANTILLON_HZ := 50.0   # le braquage est re-echantillonne a la frequence de la boucle du robot
const FENETRE_TENDANCE_S := 2.0
const SEGMENT_MIN_S := 4.0          # 2 fenetres de tendance : en dessous, le retrait de la tendance n'a pas de sens
const F_MIN := 0.25
const F_MAX := 10.0
const PAS_F := 0.05
const DEMI_BANDE_BILLE_HZ := 0.3
const AMPLITUDE_MIN_DEG := 0.05     # en dessous : aucune oscillation (frequence dominante non definie)

var f_bille_hz := 0.0               # frequence propre de la bille (BilleCuvette.frequence_propre_hz)

var _angle_prec := NAN
var _somme_variations := 0.0        # deg
var _distance := 0.0                # m (valeur absolue : avant et arriere)
var _distance_prec := NAN
var _somme_vitesse2 := 0.0          # (deg/s)^2 x s
var _duree := 0.0                   # s
var _somme_marge := 0.0
var _n_marge := 0
var _suivi := _Segments.new()      # tout le suivi : excitation de la bille
var _droite := _Segments.new()     # suivi en ligne droite : oscillation du suivi

## Segments d'un signal re-echantillonne a FREQ_ECHANTILLON_HZ.
class _Segments:
	var actuel: PackedFloat32Array = []
	var t_prochain := 0.0
	var finis: Array[PackedFloat32Array] = []

	func ajouter(t: float, valeur: float, actif: bool) -> void:
		if not actif:
			fermer()
			return
		if actuel.is_empty():
			t_prochain = t
		if t >= t_prochain - 1e-9:
			actuel.append(valeur)
			t_prochain += 1.0 / FREQ_ECHANTILLON_HZ

	func fermer() -> void:
		if actuel.size() >= int(SEGMENT_MIN_S * FREQ_ECHANTILLON_HZ):
			finis.append(actuel)
		actuel = PackedFloat32Array()

func reinitialiser() -> void:
	_angle_prec = NAN
	_somme_variations = 0.0
	_distance = 0.0
	_distance_prec = NAN
	_somme_vitesse2 = 0.0
	_duree = 0.0
	_somme_marge = 0.0
	_n_marge = 0
	_suivi = _Segments.new()
	_droite = _Segments.new()

## A appeler a chaque pas physique.
##   angle_deg : angle reel des roues ; distance_m : distance parcourue cumulee ; en_suivi : etat SUIVI du controleur ;
##   en_droite : la voiture est entierement sur une ligne droite (sondes et essieu arriere).
func ajouter(t: float, dt: float, angle_deg: float, distance_m: float, marge: float, en_suivi: bool,
		en_droite: bool) -> void:
	if not is_nan(_angle_prec) and dt > 0.0:
		var d := absf(angle_deg - _angle_prec)
		_somme_variations += d
		_somme_vitesse2 += (d / dt) * (d / dt) * dt
		_duree += dt
	_angle_prec = angle_deg
	if not is_nan(_distance_prec):
		_distance += absf(distance_m - _distance_prec)
	_distance_prec = distance_m
	_somme_marge += marge
	_n_marge += 1
	_suivi.ajouter(t, angle_deg, en_suivi)
	_droite.ajouter(t, angle_deg, en_suivi and en_droite)

## Resultats de l'essai (cles = colonnes du banc d'essai).
func resultat() -> Dictionary:
	_suivi.fermer()
	_droite.fermer()
	var f_dom := NAN
	var amp_dom := NAN
	for s in _droite.finis:
		var spectre := _spectre(_sans_tendance(s))
		for k in spectre.size():
			if is_nan(amp_dom) or spectre[k] > amp_dom:
				amp_dom = spectre[k]
				f_dom = F_MIN + k * PAS_F
	if amp_dom < AMPLITUDE_MIN_DEG:
		f_dom = NAN   # braquage immobile en ligne droite : pas de frequence dominante (amplitude gardee)
	var amp_bille := NAN
	for s in _suivi.finis:
		var spectre := _spectre(_sans_tendance(s))
		for k in spectre.size():
			if absf(F_MIN + k * PAS_F - f_bille_hz) <= DEMI_BANDE_BILLE_HZ + 1e-6:
				amp_bille = spectre[k] if is_nan(amp_bille) else maxf(amp_bille, spectre[k])
	return {
		"braquage_activite_deg_m": _somme_variations / _distance if _distance > 0.01 else 0.0,
		"braquage_vitesse_rms_deg_s": sqrt(_somme_vitesse2 / _duree) if _duree > 0.0 else 0.0,
		"braquage_f_dom_hz": f_dom,
		"braquage_amp_f_dom_deg": amp_dom,
		"f_bille_hz": f_bille_hz,
		"braquage_amp_f_bille_deg": amp_bille,
		"bille_marge_moy": _somme_marge / _n_marge if _n_marge > 0 else 0.0,
		"segments_droits": _droite.finis.size(),
		"segments_suivi": _suivi.finis.size(),
	}

## Signal moins sa moyenne glissante centree (FENETRE_TENDANCE_S ; fenetre raccourcie aux bords).
static func _sans_tendance(x: PackedFloat32Array) -> PackedFloat32Array:
	var n := x.size()
	var demi := int(FENETRE_TENDANCE_S * FREQ_ECHANTILLON_HZ / 2.0)
	var cumul := PackedFloat64Array()
	cumul.resize(n + 1)
	cumul[0] = 0.0
	for i in n:
		cumul[i + 1] = cumul[i] + x[i]
	var y := PackedFloat32Array()
	y.resize(n)
	for i in n:
		var a := maxi(0, i - demi)
		var b := mini(n, i + demi + 1)
		y[i] = x[i] - (cumul[b] - cumul[a]) / (b - a)
	return y

## Spectre d'amplitude (deg) aux frequences F_MIN, F_MIN + PAS_F, ..., F_MAX, fenetre de Hann.
## Amplitude d'une sinusoide pure = sa valeur crete.
static func _spectre(x: PackedFloat32Array) -> PackedFloat32Array:
	var n := x.size()
	var w := PackedFloat32Array()
	w.resize(n)
	var somme_w := 0.0
	for i in n:
		w[i] = 0.5 - 0.5 * cos(TAU * i / (n - 1))
		somme_w += w[i]
	var nb_f := int(round((F_MAX - F_MIN) / PAS_F)) + 1
	var a := PackedFloat32Array()
	a.resize(nb_f)
	for k in nb_f:
		var omega := TAU * (F_MIN + k * PAS_F) / FREQ_ECHANTILLON_HZ
		var re := 0.0
		var im := 0.0
		for i in n:
			var v := x[i] * w[i]
			re += v * cos(omega * i)
			im -= v * sin(omega * i)
		a[k] = 2.0 * sqrt(re * re + im * im) / somme_w
	return a
