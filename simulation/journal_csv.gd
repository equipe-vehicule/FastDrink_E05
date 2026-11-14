## Journal CSV d'un essai (un fichier horodate par activation).
## En plus des mesures du vehicule, il enregistre ce que pense le controleur (etat, phase du contournement, cote,
## position estimee par odometrie) et la position reelle dans la scene : de quoi comprendre un echec apres coup
## avec outils/analyser_journal.py. Pour une piste aleatoire, la ligne et les obstacles sont enregistres a cote,
## un fichier par piste chargee pendant le journal (piste_<date>.csv, puis piste_<date>_2.csv, ...) ; la colonne
## "piste" indique laquelle est en cours.
class_name JournalCSV
extends RefCounted

const DOSSIER := "res://journaux"
const ENTETE := ["t_s", "commande_vitesse", "servo", "vitesse_m_s", "acceleration_m_s2", "sondes", "ultrason_cm", "bille_tombee",
		"etat", "phase", "cote", "braquage_moyen", "ultrason_filtre_mm", "odo_x_mm", "odo_y_mm", "odo_cap_deg",
		"monde_x_m", "monde_z_m", "monde_cap_deg", "piste", "collision", "bille_bord"]

var _fichier: FileAccess
var _horodatage := ""
var _n_piste := 0

func actif() -> bool:
	return _fichier != null

## piste : noeud de la piste en cours ; si elle a ete generee (metadonnees "ligne" et "obstacles"), on l'enregistre aussi.
func basculer(piste: Node3D = null) -> void:
	if _fichier:
		_fichier.close(); _fichier = null; return
	DirAccess.make_dir_recursive_absolute(DOSSIER)
	# Sans ce fichier, Godot importe chaque CSV du projet comme fichier de traduction (.translation, .csv.import)
	if not FileAccess.file_exists(DOSSIER + "/.gdignore"):
		FileAccess.open(DOSSIER + "/.gdignore", FileAccess.WRITE).close()
	# PID : deux essais lances dans la meme seconde n'ecrivent pas dans le meme fichier (comme le banc)
	_horodatage = "%s_%d" % [Time.get_datetime_string_from_system().replace(":", "-"), OS.get_process_id()]
	_fichier = FileAccess.open("%s/essai_%s.csv" % [DOSSIER, _horodatage], FileAccess.WRITE)
	_fichier.store_csv_line(PackedStringArray(ENTETE))
	_n_piste = 0
	nouvelle_piste(piste)

## A appeler a chaque piste chargee (touches 1-7, G) : elle est enregistree si elle a ete generee.
func nouvelle_piste(piste: Node3D) -> void:
	if _fichier == null: return
	_n_piste += 1
	if piste and piste.has_meta("ligne"):
		var suffixe := "" if _n_piste == 1 else "_%d" % _n_piste
		_enregistrer_piste("%s/piste_%s%s.csv" % [DOSSIER, _horodatage, suffixe], piste)

## bille_bord : position de la bille rapportee au bord de la cuvette (0 = centre, 1 = chute).
func ecrire(temps: float, v: Vehicule, bille_tombee: bool, c: Controleur, bille_bord := 0.0) -> void:
	if _fichier == null: return
	var avant := -v.global_transform.basis.z
	_fichier.store_csv_line(PackedStringArray([
		str(snapped(temps, 0.001)),
		str(v.commande_vitesse),
		str(v.angle_servo),
		str(snapped(v.vitesse, 0.0001)),
		str(snapped(v.acceleration, 0.001)),
		"".join(v.sondes.map(func(s): return str(s))),
		str(snapped(v.distance_cm, 0.1)),
		str(int(bille_tombee)),
		c.etat_court(),
		c.contournement.nom_phase().get_slice(" ", 0) if c.etat_court() == "CONTOURNEMENT" else "",
		str(c.contournement.cote) if c.etat_court() == "CONTOURNEMENT" else "",
		str(snapped(c.braquage_moyen(), 0.1)),
		str(snapped(c.distance_filtree_mm, 0.1)),
		str(snapped(c.odo.x, 0.1)),
		str(snapped(c.odo.y, 0.1)),
		str(snapped(rad_to_deg(c.odo.cap), 0.1)),
		str(snapped(v.global_position.x, 0.0001)),
		str(snapped(v.global_position.z, 0.0001)),
		str(snapped(rad_to_deg(atan2(avant.x, -avant.z)), 0.1)),   # 0 = vers -Z, > 0 = vers la droite (+X)
		str(_n_piste),
		str(int(v.en_collision)),
		str(snapped(bille_bord, 0.001)),
	]))

# Lignes "ligne;x;z" (axe du ruban, m) puis "obstacle;x;z;largeur;profondeur;angle" (m, rad).
func _enregistrer_piste(nom: String, piste: Node3D) -> void:
	var f := FileAccess.open(nom, FileAccess.WRITE)
	f.store_csv_line(PackedStringArray(["type", "x", "z", "largeur", "profondeur", "angle"]))
	for p in piste.get_meta("ligne"):
		f.store_csv_line(PackedStringArray(["ligne", str(snapped(p.x, 0.0001)), str(snapped(p.y, 0.0001)), "", "", ""]))
	for o in piste.get_meta("obstacles"):
		f.store_csv_line(PackedStringArray(["obstacle", str(snapped(o[0], 0.0001)), str(snapped(o[1], 0.0001)),
				str(snapped(o[2], 0.0001)), str(snapped(o[3], 0.0001)), str(snapped(o[4], 0.0001))]))
	f.close()
