## Banc d'essai automatique : enchaine des pistes aleatoires (graines successives) et ecrit une ligne de resultat par
## piste dans journaux/banc_<date>.csv. Sert a verifier le comportement sur beaucoup de situations, pas seulement sur
## les pistes de la demonstration. Active par le reglage `banc_essai_pistes` du noeud Simulation.
## Avec une liste de scenarios (simulation/scenarios.gd, option --scenarios=) : un cas par scenario, resultats dans
## journaux/scenarios_<date>.csv (colonne « cas » en tete) et une ligne de resume par cas dans la console.
## Colonnes ajoutees a la fin (les 21 premieres ne changent pas) : mesures de stabilite (MesuresStabilite), verdict.
class_name BancEssai
extends RefCounted

const MesuresStabilite := preload("res://simulation/mesures_stabilite.gd")

const DOSSIER := "res://journaux"
const ENTETE := ["graine", "obstacles", "resultat", "temps_s", "bille_marge_max", "collision", "arrets",
		"erreur_arret_max_mm", "recul_min_mm", "recul_max_mm", "contournements_fluides", "contournements_phases",
		"rattrapages", "demi_tours", "retrouver", "bloque", "raisons_repli", "pertes_ligne_apres_contournement",
		"perte_ligne_apres_max_s", "perte_ligne_ailleurs_max_s", "fluide_fin_sans_ligne"]
const COLONNES_STABILITE := ["braquage_activite_deg_m", "braquage_vitesse_rms_deg_s", "braquage_f_dom_hz",
		"braquage_amp_f_dom_deg", "f_bille_hz", "braquage_amp_f_bille_deg", "bille_marge_moy", "segments_droits",
		"segments_suivi"]
## Colonnes de l'etape 4 (obstacles poses sur differentes faces), apres celles de la stabilite.
const COLONNES_OBSTACLES := ["marge_obstacle_min_mm", "poses"]
## Colonnes de l'etape 5 (stabilite du suivi, #74).
const COLONNES_SUIVI := ["pertes_ligne_ailleurs"]
const ETATS_CONTOURNEMENT := ["CONTOURNEMENT_FLUIDE", "CONTOURNEMENT"]
## Ligne perdue « apres un contournement » : l'episode commence moins de ca apres la fin du contournement.
const FENETRE_APRES_CONTOURNEMENT_S := 3.0
const PERTE_MIN_S := 0.1          # plus court : ligne entre deux sondes, pas une perte

var nombre_pistes := 0
var graine_depart := 1
var duree_max_s := 200.0
## Scenarios a enchainer (dictionnaires de simulation/scenarios.gd) ; vide = pistes aleatoires.
var scenarios: Array = []
## Obstacles de la piste en cours : [x, z, largeur, profondeur, angle] (m, rad), metadonnee « obstacles » de
## ConstructionPiste. Vide pour une piste Blender. A donner apres chaque chargement de piste.
var obstacles: Array = []
## Dernier point de la ligne de la piste en cours (x, z en m ; NAN = inconnu), Pistes.fin_de_ligne(). A donner apres chaque
## chargement de piste. Sert au controle « fin de ligne atteinte » des scenarios dont fin_attendue vaut BLOQUE.
var fin_ligne := Vector2(NAN, NAN)
## HYPOTHESE (non issue d'une exigence) : fin_attendue = BLOQUE seulement si la barre de sondes est a moins de ca du
## dernier point de la ligne au moment de l'arret ; sinon la voiture s'est bloquee en route (ECHEC « bloque_loin_fin_ligne »).
var distance_fin_ligne_max_mm := 150.0

var termine := false
var _i := 0
var _fichier: FileAccess
var _graine := 0
var _scenario := ""
var _stabilite := MesuresStabilite.new()
var _ok := 0
var _marge_max := 0.0
var _collision := false
var _marge_obstacle := INF        # m, plus petite distance vue de dessus entre la voiture et un obstacle
var _compteurs := {}
var _etat_prec := ""
var _erreur_arret_max := 0.0      # mm, ecart maximal (en valeur absolue) entre l'arret mesure et distance_arret_mm
var _reculs: Array[float] = []    # mm, distance reellement reculee devant chaque obstacle (position dans le monde)
var _debut_recul := Vector3.ZERO
# Ligne perdue pendant le suivi (aucune sonde ne la voit)
var _fin_contournement := -INF    # instant de la fin du dernier contournement (s)
var _debut_perte := -1.0          # debut de l'episode en cours (s), < 0 = ligne vue
var _pertes_apres := 0
var _perte_apres_max := 0.0
var _perte_ailleurs_max := 0.0
var _pertes_ailleurs := 0
var _fin_attendue := "ARRIVEE"    # resultat attendu (scenario « fin_attendue », ARRIVEE par defaut)
var _distance_fin_ligne_mm := NAN  # mm, barre de sondes -> fin de la ligne a l'arret (NAN = non controlee)
var _fins_sans_ligne := 0

func demarrer() -> void:
	DirAccess.make_dir_recursive_absolute(DOSSIER)
	if not FileAccess.file_exists(DOSSIER + "/.gdignore"):
		FileAccess.open(DOSSIER + "/.gdignore", FileAccess.WRITE).close()
	if not scenarios.is_empty():
		nombre_pistes = scenarios.size()
	var prefixe := "banc" if scenarios.is_empty() else "scenarios"
	# Numero du processus dans le nom : deux essais lances dans la meme seconde n'ecrivent pas dans le meme fichier
	var nom := "%s/%s_%s_%d.csv" % [DOSSIER, prefixe, Time.get_datetime_string_from_system().replace(":", "-"),
			OS.get_process_id()]
	_fichier = FileAccess.open(nom, FileAccess.WRITE)
	# Ligne lue par les scripts d'essai pour retrouver le fichier de resultats (voulue, pas du debogage)
	print("FICHIER %s" % ProjectSettings.globalize_path(nom))
	var entete := ([] if scenarios.is_empty() else ["cas"]) + ENTETE + COLONNES_STABILITE + ["verdict", "raisons_echec"] \
			+ COLONNES_OBSTACLES + COLONNES_SUIVI
	_fichier.store_csv_line(PackedStringArray(entete))
	_i = 0
	_ok = 0
	termine = false

## Prochain cas : graine de la piste aleatoire (suite fixe : les memes essais peuvent etre rejoues) ou scenario.
func prochain_cas() -> Variant:
	_reinitialiser()
	if scenarios.is_empty():
		_graine = graine_depart + _i * 7919
		_scenario = ""
		return _graine
	_graine = 0
	_scenario = scenarios[_i]["nom"]
	_fin_attendue = scenarios[_i].get("fin_attendue", "ARRIVEE")
	return scenarios[_i]

func _reinitialiser() -> void:
	_marge_max = 0.0
	_collision = false
	_marge_obstacle = INF
	_compteurs = {"DECISION": 0, "CONTOURNEMENT_FLUIDE": 0, "CONTOURNEMENT": 0, "RATTRAPAGE": 0, "DEMI_TOUR": 0,
			"RETROUVER": 0, "BLOQUE": 0}
	_etat_prec = ""
	_erreur_arret_max = 0.0
	_distance_fin_ligne_mm = NAN
	_reculs.clear()
	_fin_contournement = -INF
	_debut_perte = -1.0
	_pertes_apres = 0
	_perte_apres_max = 0.0
	_perte_ailleurs_max = 0.0
	_pertes_ailleurs = 0
	_fins_sans_ligne = 0
	_stabilite.reinitialiser()

## Arret (entree en DECISION) : distance reelle avant de la voiture -> obstacle, lue par l'ultrason simule (exact,
## sans bruit), comparee a la distance voulue. Recul (etat RECUL_OBSTACLE) : distance parcourue dans le monde.
func _mesurer_sequence(etat: String, v: Vehicule, c: Controleur) -> void:
	if etat == "DECISION" and v.distance_cm >= 0.0:
		var avant_obstacle := v.distance_cm * 10.0 - c.robot.ultrason_avant_mm
		_erreur_arret_max = maxf(_erreur_arret_max, absf(avant_obstacle - c.epreuve.distance_arret_mm))
	if etat == "RECUL_OBSTACLE":
		_debut_recul = v.global_position
	elif _etat_prec == "RECUL_OBSTACLE":
		_reculs.append(v.global_position.distance_to(_debut_recul) * 1000.0)

## Episodes de ligne perdue pendant le suivi : classes « apres un contournement » (commences moins de
## FENETRE_APRES_CONTOURNEMENT_S apres sa fin) ou « ailleurs » (virages, approche d'un obstacle...).
func _mesurer_ligne(temps: float, etat: String, v: Vehicule, c: Controleur) -> void:
	if _etat_prec in ETATS_CONTOURNEMENT and etat not in ETATS_CONTOURNEMENT:
		_fin_contournement = temps
		if _etat_prec == "CONTOURNEMENT_FLUIDE" and c.fluide.fin_sans_ligne:
			_fins_sans_ligne += 1
	var perdue: bool = etat == "SUIVI" and not v.sondes.any(func(s): return s > 0)
	if perdue and _debut_perte < 0.0:
		_debut_perte = temps
	if _debut_perte >= 0.0 and (not perdue or etat != "SUIVI"):
		_fermer_perte(temps)

func _fermer_perte(temps: float) -> void:
	var duree := temps - _debut_perte
	if duree >= PERTE_MIN_S:
		if _debut_perte - _fin_contournement <= FENETRE_APRES_CONTOURNEMENT_S:
			_pertes_apres += 1
			_perte_apres_max = maxf(_perte_apres_max, duree)
		else:
			_perte_ailleurs_max = maxf(_perte_ailleurs_max, duree)
			_pertes_ailleurs += 1
	_debut_perte = -1.0

static func _texte_recul(valeur) -> String:
	return "" if valeur == null else str(snapped(float(valeur), 0.1))

## Tailles des obstacles en mm, largeur x hauteur x profondeur, ex. « 150x100x50;100x50x150 ».
static func _texte_poses(g: GenerateurPiste) -> String:
	var t: PackedStringArray = []
	if g != null:
		for taille in g.tailles_obstacles:
			t.append("%dx%dx%d" % [roundi(taille.x * 1000.0), roundi(taille.y * 1000.0), roundi(taille.z * 1000.0)])
	return ";".join(t)

static func _texte_raisons(d: Dictionary) -> String:
	var t: PackedStringArray = []
	for k in d:
		t.append("%s:%d" % [k, d[k]])
	return ";".join(t)

## Fin de ligne atteinte : distance (mm) de la barre de sondes au dernier point de la ligne, mesuree a l'arret d'un
## scenario dont fin_attendue vaut BLOQUE. NAN = non controle (autre fin attendue, autre resultat, ligne inconnue).
func _mesurer_fin_ligne(v: Vehicule, c: Controleur) -> float:
	if _fin_attendue != "BLOQUE" or is_nan(fin_ligne.x):
		return NAN
	var t := v.global_transform
	var essieu := t.origin + t.basis.z * (v.empattement / 2.0)
	var sondes := essieu - t.basis.z * (c.robot.distance_sondes_mm / 1000.0)
	return Vector2(sondes.x, sondes.z).distance_to(fin_ligne) * 1000.0

func _texte_fin_ligne() -> String:
	return "" if is_nan(_distance_fin_ligne_mm) else " (fin de ligne a %.0f mm)" % _distance_fin_ligne_mm

## Verdict selon le guide (section 1) et les precisions du 12 nov. : arrivee, sans collision ni chute, arret a
## +/- TOLERANCE_ARRET_MM, recul de moins de RECUL_MAX_MM, et un arret et un recul devant CHAQUE obstacle.
func _raisons_echec(resultat: String, nb_obstacles: int) -> PackedStringArray:
	var r: PackedStringArray = []
	if resultat != _fin_attendue:
		r.append(resultat)
	if _collision:
		r.append("collision")
	if _erreur_arret_max > ReglagesEpreuve.TOLERANCE_ARRET_MM:
		r.append("arret a %.0f mm de la cible" % _erreur_arret_max)
	if not _reculs.is_empty() and _reculs.max() >= ReglagesEpreuve.RECUL_MAX_MM:
		r.append("recul de %.0f mm" % _reculs.max())
	if not is_nan(_distance_fin_ligne_mm) and _distance_fin_ligne_mm > distance_fin_ligne_max_mm:
		r.append("bloque_loin_fin_ligne")
	if _compteurs["DECISION"] != nb_obstacles:
		r.append("%d arret(s) pour %d obstacle(s)" % [_compteurs["DECISION"], nb_obstacles])
	if _reculs.size() != nb_obstacles:
		r.append("%d recul(s) pour %d obstacle(s)" % [_reculs.size(), nb_obstacles])
	return r

## Ligne de resume dans la console (une par cas).
func _afficher(resultat: String, temps: float, raisons: PackedStringArray, s: Dictionary) -> void:
	var cas := _scenario if _scenario != "" else "graine %d" % _graine
	var oscillation := "%.2f Hz %.2f deg" % [s["braquage_f_dom_hz"], s["braquage_amp_f_dom_deg"]]
	if is_nan(s["braquage_amp_f_dom_deg"]):
		oscillation = "non mesuree"
	elif is_nan(s["braquage_f_dom_hz"]):
		oscillation = "aucune"
	var excitation := "non mesuree" if is_nan(s["braquage_amp_f_bille_deg"]) else "%.2f deg" % s["braquage_amp_f_bille_deg"]
	var marge := "-" if is_inf(_marge_obstacle) else "%.0f mm" % (_marge_obstacle * 1000.0)
	print("CAS %-22s | %-5s | %6.1f s | bille max %.2f | collision %d | marge obstacle %s | err arret %4.1f mm | recul %s mm | activite %5.1f deg/m | oscillation en droite %s | braquage a f bille (%.2f Hz) %s | %s" % [
		cas, "OK" if raisons.is_empty() else "ECHEC", temps, _marge_max, int(_collision), marge, _erreur_arret_max,
		_texte_recul(_reculs.max()), s["braquage_activite_deg_m"], oscillation, s["f_bille_hz"], excitation,
		"; ".join(raisons) if not raisons.is_empty() else resultat + _texte_fin_ligne()])

## Scenario invalide (piste impossible a construire) : une ligne ECHEC, puis cas suivant.
func noter_invalide() -> void:
	var ligne := PackedStringArray([_scenario, "0", "0", "SCENARIO_INVALIDE"])
	_fichier.store_csv_line(ligne)
	_fichier.flush()
	print("CAS %-22s | ECHEC | scenario invalide (voir l'erreur ci-dessus)" % _scenario)
	_avancer()

func _avancer() -> void:
	_i += 1
	if _i >= nombre_pistes:
		termine = true
		_fichier.close()
		print("BILAN : %d cas OK sur %d" % [_ok, nombre_pistes])

## Voiture entierement sur une ligne droite : barre de sondes ET essieu arriere sur une case droite (sinon l'entree
## dans un virage, vue d'abord par les sondes, compterait comme une oscillation). Sans piste generee : faux.
static func _en_droite(v: Vehicule, c: Controleur, g: GenerateurPiste) -> bool:
	if g == null:
		return false
	var t := v.global_transform
	var essieu := t.origin + t.basis.z * (v.empattement / 2.0)
	var sondes := essieu - t.basis.z * (c.robot.distance_sondes_mm / 1000.0)
	return g.sur_case_droite(essieu.x, essieu.z) and g.sur_case_droite(sondes.x, sondes.z)

## Marge laterale : plus petite distance, vue de dessus, entre la boite d'encombrement de la voiture et chaque
## obstacle (0 = contact). Mesure de la simulation (verite terrain), pas du robot.
func _mesurer_marge_obstacle(v: Vehicule) -> void:
	if obstacles.is_empty():
		return
	var t := v.global_transform * Transform3D(Basis(), v.centre_encombrement)
	var voiture := _rectangle(Vector2(t.origin.x, t.origin.z), Vector2(t.basis.x.x, t.basis.x.z),
			Vector2(t.basis.z.x, t.basis.z.z), v.encombrement.x / 2.0, v.encombrement.z / 2.0)
	for o in obstacles:
		var b := Basis(Vector3.UP, o[4])
		var r := _rectangle(Vector2(o[0], o[1]), Vector2(b.x.x, b.x.z), Vector2(b.z.x, b.z.z), o[2] / 2.0, o[3] / 2.0)
		_marge_obstacle = minf(_marge_obstacle, distance_rectangles(voiture, r))

## Coins d'un rectangle (plan x, z) : centre, axes unitaires et demi-cotes le long de ces axes.
static func _rectangle(centre: Vector2, axe_u: Vector2, axe_v: Vector2, demi_u: float, demi_v: float) -> PackedVector2Array:
	var u := axe_u.normalized() * demi_u
	var w := axe_v.normalized() * demi_v
	return PackedVector2Array([centre + u + w, centre - u + w, centre - u - w, centre + u - w])

## Distance entre deux rectangles (coins dans l'ordre) : 0 s'ils se chevauchent (axes separateurs), sinon la plus
## petite distance d'un coin de l'un aux cotes de l'autre (exact pour deux polygones convexes disjoints).
static func distance_rectangles(a: PackedVector2Array, b: PackedVector2Array) -> float:
	var separes := false
	for poly in [a, b]:
		for i in 2:
			var axe: Vector2 = (poly[i + 1] - poly[i]).orthogonal()
			var min_a := INF; var max_a := -INF; var min_b := INF; var max_b := -INF
			for p in a:
				min_a = minf(min_a, axe.dot(p)); max_a = maxf(max_a, axe.dot(p))
			for p in b:
				min_b = minf(min_b, axe.dot(p)); max_b = maxf(max_b, axe.dot(p))
			if max_a < min_b or max_b < min_a:
				separes = true
	if not separes:
		return 0.0
	var d := INF
	for paire in [[a, b], [b, a]]:
		var coins: PackedVector2Array = paire[0]
		var poly: PackedVector2Array = paire[1]
		for p in coins:
			for i in 4:
				d = minf(d, p.distance_to(Geometry2D.get_closest_point_to_segment(p, poly[i], poly[(i + 1) % 4])))
	return d

## A appeler a chaque pas. Retourne true quand la piste en cours est finie (resultat ecrit).
## g : piste generee (aleatoire ou scenario), null pour une piste Blender.
func suivre(temps: float, dt: float, v: Vehicule, bille: BilleCuvette, c: Controleur, g: GenerateurPiste) -> bool:
	var nb_obstacles := g.obstacles.size() if g != null else 0
	if termine:
		return false
	_marge_max = maxf(_marge_max, bille.taux_bord())
	_mesurer_marge_obstacle(v)
	if temps > 0.1 and v.en_collision:
		_collision = true
	var etat := c.etat_court()
	if etat != _etat_prec:
		if _compteurs.has(etat):
			_compteurs[etat] += 1
		_mesurer_sequence(etat, v, c)
	_mesurer_ligne(temps, etat, v, c)
	_etat_prec = etat
	_stabilite.f_bille_hz = bille.frequence_propre_hz()
	_stabilite.f_echantillon_hz = c.robot.frequence_controle_hz
	_stabilite.ajouter(temps, dt, rad_to_deg(v.angle_roues_rad()), v.distance_parcourue, bille.taux_bord(),
			etat == "SUIVI", _en_droite(v, c, g))
	var resultat := ""
	if bille.tombee:
		resultat = "BILLE_TOMBEE"
	elif etat == "ARRIVEE":
		resultat = "ARRIVEE"
	elif etat == "BLOQUE" or etat == "ARRET":
		resultat = etat
	elif temps > duree_max_s:
		resultat = "TEMPS_DEPASSE"
	if resultat == "":
		return false
	if _debut_perte >= 0.0:
		_fermer_perte(temps)
	var s := _stabilite.resultat()
	_distance_fin_ligne_mm = _mesurer_fin_ligne(v, c)
	var raisons := _raisons_echec(resultat, nb_obstacles)
	if raisons.is_empty():
		_ok += 1
	var ligne := PackedStringArray([str(_graine), str(nb_obstacles), resultat, str(snapped(temps, 0.01)),
			str(snapped(_marge_max, 0.01)), str(int(_collision)), str(_compteurs["DECISION"]),
			str(snapped(_erreur_arret_max, 0.1)), _texte_recul(_reculs.min()), _texte_recul(_reculs.max()),
			str(_compteurs["CONTOURNEMENT_FLUIDE"]), str(_compteurs["CONTOURNEMENT"]), str(_compteurs["RATTRAPAGE"]),
			str(_compteurs["DEMI_TOUR"]), str(_compteurs["RETROUVER"]), str(_compteurs["BLOQUE"]),
			_texte_raisons(c.raisons_repli), str(_pertes_apres), str(snapped(_perte_apres_max, 0.01)),
			str(snapped(_perte_ailleurs_max, 0.01)), str(_fins_sans_ligne)])
	for col in COLONNES_STABILITE:
		ligne.append("" if is_nan(float(s[col])) else str(snapped(float(s[col]), 0.001)))   # vide = non mesure
	ligne.append("OK" if raisons.is_empty() else "ECHEC")
	ligne.append("; ".join(raisons))
	ligne.append("" if is_inf(_marge_obstacle) else str(snapped(_marge_obstacle * 1000.0, 0.1)))
	ligne.append(_texte_poses(g))
	ligne.append(str(_pertes_ailleurs))
	if _scenario != "":
		ligne.insert(0, _scenario)
	_fichier.store_csv_line(ligne)
	_fichier.flush()
	_afficher(resultat, temps, raisons, s)
	_avancer()
	return true
