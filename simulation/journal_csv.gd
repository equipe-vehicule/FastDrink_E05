## Journal CSV d'un essai (un fichier horodate par activation).
class_name JournalCSV
extends RefCounted

const DOSSIER := "res://journaux"
const ENTETE := ["t_s", "commande_vitesse", "servo", "vitesse_m_s", "acceleration_m_s2", "sondes", "ultrason_cm", "bille_tombee"]

var _fichier: FileAccess

func actif() -> bool:
	return _fichier != null

func basculer() -> void:
	if _fichier:
		_fichier.close(); _fichier = null; return
	DirAccess.make_dir_recursive_absolute(DOSSIER)
	var nom := "%s/essai_%s.csv" % [DOSSIER, Time.get_datetime_string_from_system().replace(":", "-")]
	_fichier = FileAccess.open(nom, FileAccess.WRITE)
	_fichier.store_csv_line(PackedStringArray(ENTETE))

func ecrire(temps: float, v: Vehicule, bille_tombee: bool) -> void:
	if _fichier == null: return
	_fichier.store_csv_line(PackedStringArray([
		str(snapped(temps, 0.001)),
		str(v.commande_vitesse),
		str(v.angle_servo),
		str(snapped(v.vitesse, 0.0001)),
		str(snapped(v.acceleration, 0.001)),
		"".join(v.sondes.map(func(s): return str(s))),
		str(snapped(v.distance_cm, 0.1)),
		str(int(bille_tombee)),
	]))
