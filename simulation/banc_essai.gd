## Banc d'essai automatique : enchaine des pistes aleatoires (graines successives) et ecrit une ligne de resultat par
## piste dans journaux/banc_<date>.csv. Sert a verifier le comportement sur beaucoup de situations, pas seulement sur
## les pistes de la demonstration. Active par le reglage `banc_essai_pistes` du noeud Simulation.
class_name BancEssai
extends RefCounted

const DOSSIER := "res://journaux"
const ENTETE := ["graine", "obstacles", "resultat", "temps_s", "bille_marge_max", "collision",
		"contournements_fluides", "contournements_phases", "rattrapages", "demi_tours", "retrouver", "bloque", "raisons_repli"]

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
	_compteurs = {"CONTOURNEMENT_FLUIDE": 0, "DECISION": 0, "RATTRAPAGE": 0, "DEMI_TOUR": 0, "RETROUVER": 0, "BLOQUE": 0}
	_etat_prec = ""
	return _graine

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
	if etat != _etat_prec and _compteurs.has(etat):
		_compteurs[etat] += 1
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
	_fichier.store_csv_line(PackedStringArray([str(_graine), str(nb_obstacles), resultat, str(snapped(temps, 0.01)),
			str(snapped(_marge_max, 0.01)), str(int(_collision)), str(_compteurs["CONTOURNEMENT_FLUIDE"]),
			str(_compteurs["DECISION"]), str(_compteurs["RATTRAPAGE"]), str(_compteurs["DEMI_TOUR"]),
			str(_compteurs["RETROUVER"]), str(_compteurs["BLOQUE"]), _texte_raisons(c.raisons_repli)]))
	_fichier.flush()
	_i += 1
	if _i >= nombre_pistes:
		termine = true
		_fichier.close()
	return true
