## Banc d'essai automatique : enchaine des pistes aleatoires (graines successives) et ecrit une ligne de resultat par
## piste dans journaux/banc_<date>.csv. Sert a verifier le comportement sur beaucoup de situations, pas seulement sur
## les pistes de la demonstration. Active par le reglage `banc_essai_pistes` du noeud Simulation.
class_name BancEssai
extends RefCounted

const DOSSIER := "res://journaux"
const ENTETE := ["graine", "obstacles", "resultat", "temps_s", "bille_marge_max", "collision", "arrets",
		"erreur_arret_max_mm", "recul_min_mm", "recul_max_mm", "contournements_fluides", "contournements_phases",
		"rattrapages", "demi_tours", "retrouver", "bloque", "raisons_repli", "pertes_ligne_apres_contournement",
		"perte_ligne_apres_max_s", "perte_ligne_ailleurs_max_s", "fluide_fin_sans_ligne"]
const ETATS_CONTOURNEMENT := ["CONTOURNEMENT_FLUIDE", "CONTOURNEMENT"]
## Ligne perdue « apres un contournement » : l'episode commence moins de ca apres la fin du contournement.
const FENETRE_APRES_CONTOURNEMENT_S := 3.0
const PERTE_MIN_S := 0.1          # plus court : ligne entre deux sondes, pas une perte

var nombre_pistes := 0
var graine_depart := 1
var duree_max_s := 200.0

var termine := false
var _i := 0
var _fichier: FileAccess
var _graine := 0
var _marge_max := 0.0
var _collision := false
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
var _fins_sans_ligne := 0

func demarrer() -> void:
	DirAccess.make_dir_recursive_absolute(DOSSIER)
	if not FileAccess.file_exists(DOSSIER + "/.gdignore"):
		FileAccess.open(DOSSIER + "/.gdignore", FileAccess.WRITE).close()
	var nom := "%s/banc_%s.csv" % [DOSSIER, Time.get_datetime_string_from_system().replace(":", "-")]
	_fichier = FileAccess.open(nom, FileAccess.WRITE)
	_fichier.store_csv_line(PackedStringArray(ENTETE))
	_i = 0
	termine = false

## Graine de la prochaine piste (suite fixe : les memes essais peuvent etre rejoues).
func prochaine_graine() -> int:
	_graine = graine_depart + _i * 7919
	_marge_max = 0.0
	_collision = false
	_compteurs = {"DECISION": 0, "CONTOURNEMENT_FLUIDE": 0, "CONTOURNEMENT": 0, "RATTRAPAGE": 0, "DEMI_TOUR": 0,
			"RETROUVER": 0, "BLOQUE": 0}
	_etat_prec = ""
	_erreur_arret_max = 0.0
	_reculs.clear()
	_fin_contournement = -INF
	_debut_perte = -1.0
	_pertes_apres = 0
	_perte_apres_max = 0.0
	_perte_ailleurs_max = 0.0
	_fins_sans_ligne = 0
	return _graine

## Arret (entree en DECISION) : distance reelle avant de la voiture -> obstacle, lue par l'ultrason simule (exact,
## sans bruit), comparee a la distance voulue. Recul (etat RECUL_OBSTACLE) : distance parcourue dans le monde.
func _mesurer_sequence(etat: String, v: Vehicule, c: Controleur) -> void:
	if etat == "DECISION" and v.distance_cm >= 0.0:
		var avant_obstacle := v.distance_cm * 10.0 - c.arret.decalage_capteur_mm
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
	_debut_perte = -1.0

static func _texte_recul(valeur) -> String:
	return "" if valeur == null else str(snapped(float(valeur), 0.1))

static func _texte_raisons(d: Dictionary) -> String:
	var t: PackedStringArray = []
	for k in d:
		t.append("%s:%d" % [k, d[k]])
	return ";".join(t)

## A appeler a chaque pas. Retourne true quand la piste en cours est finie (resultat ecrit).
func suivre(temps: float, v: Vehicule, bille: BilleCuvette, c: Controleur, nb_obstacles: int) -> bool:
	if termine:
		return false
	_marge_max = maxf(_marge_max, bille.taux_bord())
	if temps > 0.1 and v.en_collision:
		_collision = true
	var etat := c.etat_court()
	if etat != _etat_prec:
		if _compteurs.has(etat):
			_compteurs[etat] += 1
		_mesurer_sequence(etat, v, c)
	_mesurer_ligne(temps, etat, v, c)
	_etat_prec = etat
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
	_fichier.store_csv_line(PackedStringArray([str(_graine), str(nb_obstacles), resultat, str(snapped(temps, 0.01)),
			str(snapped(_marge_max, 0.01)), str(int(_collision)), str(_compteurs["DECISION"]),
			str(snapped(_erreur_arret_max, 0.1)), _texte_recul(_reculs.min()), _texte_recul(_reculs.max()),
			str(_compteurs["CONTOURNEMENT_FLUIDE"]), str(_compteurs["CONTOURNEMENT"]), str(_compteurs["RATTRAPAGE"]),
			str(_compteurs["DEMI_TOUR"]), str(_compteurs["RETROUVER"]), str(_compteurs["BLOQUE"]),
			_texte_raisons(c.raisons_repli), str(_pertes_apres), str(snapped(_perte_apres_max, 0.01)),
			str(snapped(_perte_ailleurs_max, 0.01)), str(_fins_sans_ligne)]))
	_fichier.flush()
	_i += 1
	if _i >= nombre_pistes:
		termine = true
		_fichier.close()
	return true
