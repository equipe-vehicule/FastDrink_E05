## Trace du controleur (etape 11, portage Python) : a CHAQUE tour du programme du robot (pas a chaque pas physique),
## les entrees exactes de Controleur.calculer() et ses sorties, en pleine precision, pour rejouer le meme essai avec le
## controleur Python (code_picar/tests/test_rejeu.py) et trouver le premier tour ou ils different.
## Lecture seule : n'appelle que des fonctions sans effet du controleur ; ne change aucun resultat (verifie : banc et
## journal identiques avec et sans trace, voir LISEZMOI_SIMULATION.md).
## Active par --trace (ligne de commande) ou trace_controleur (noeud Simulation). Un fichier par cas :
## journaux/trace_<date>_<pid>_<cas>.jsonl ; la console imprime « TRACE <chemin> ».
##
## Format (JSON Lines, nombres en pleine precision : JSON.stringify full_precision, 17 chiffres ; INF -> 1e99999,
## NAN -> null ; le signe de -0.0 est perdu) :
##   1re ligne : {"type": "entete", "format", "cas", "modele", "frequence_controle_hz", "colonnes",
##               "reglages": {module: {reglage: valeur}} (TOUS les reglages @export apres reglages_algorithmes.json,
##                            reglages_essai, reglages du scenario et mode de la piste : ce que le controleur a vraiment),
##               "robot": {champ de ReglagesRobot: valeur} (valeurs physiques lues par le controleur)}
##   puis une ligne par tour : tableau dans l'ordre de "colonnes" (voir COLONNES) ;
##   derniere ligne : {"type": "fin", "t", "etat"} (fin du cas : resultat du banc).
extends RefCounted

const ReglagesAlgorithmes := preload("res://simulation/reglages_algorithmes.gd")

const FORMAT := 1
const DOSSIER := "res://journaux"
## Une ligne par tour. Entrees de calculer() : s (5 sondes), d_cm (distance brute, -1 = pas d'echo, AVANT
## corriger_ultrason_cm), dt (s, AVANT le maxf(dt, 0.001) de calculer), nouvelle (mesure_nouvelle). Sorties :
## vitesse, angle (entiers envoyes au vehicule). Apres le tour (pour localiser une divergence) : etat, phase
## (contournement par phases, sinon ""), cmd (commande flottante, _commande_envoyee()), v_mm_s (consigne apres rampe),
## d_filtre_mm, braquage_moyen, odometrie de manoeuvre (x, y, cap, parcours), globale (x, y, cap), servo_estime.
const COLONNES := ["t", "s", "d_cm", "dt", "nouvelle", "vitesse", "angle", "etat", "phase", "cmd", "v_mm_s",
		"d_filtre_mm", "braquage_moyen", "odo_x", "odo_y", "odo_cap", "odo_parcours", "glob_x", "glob_y", "glob_cap",
		"servo_estime"]
## Champs de ReglagesRobot qui ne sont pas des @export (valeurs physiques, ReglagesRobot.appliquer()).
const CHAMPS_ROBOT := ["table_vitesse_mm_s", "mm_s_par_cmd", "filtre_ultrason"]

var _fichier: FileAccess


func actif() -> bool:
	return _fichier != null


## Nouveau cas : ferme la trace precedente, ouvre la suivante et ecrit l'en-tete (controleur deja regle).
func commencer(cas: String, c: Controleur, modele_ideal: bool, frequence_controle_hz: float) -> void:
	fermer()
	DirAccess.make_dir_recursive_absolute(DOSSIER)
	if not FileAccess.file_exists(DOSSIER + "/.gdignore"):
		FileAccess.open(DOSSIER + "/.gdignore", FileAccess.WRITE).close()
	var nom := "%s/trace_%s_%d_%s.jsonl" % [DOSSIER, Time.get_datetime_string_from_system().replace(":", "-"),
			OS.get_process_id(), cas.validate_filename()]
	_fichier = FileAccess.open(nom, FileAccess.WRITE)
	print("TRACE %s" % ProjectSettings.globalize_path(nom))
	var reglages := {}
	var modules := ReglagesAlgorithmes._modules(c)
	for m in modules:
		var valeurs := {}
		for cle in ReglagesAlgorithmes._reglages_de(modules[m]):
			valeurs[cle] = modules[m].get(cle)
		reglages[m] = valeurs
	var robot := {}
	for champ in ReglagesRobot.CLES_JSON:
		robot[champ] = c.robot.get(champ)
	for champ in CHAMPS_ROBOT:
		robot[champ] = c.robot.get(champ)
	_ecrire({"type": "entete", "format": FORMAT, "cas": cas, "modele": "ideal" if modele_ideal else "realiste",
			"frequence_controle_hz": frequence_controle_hz, "colonnes": COLONNES, "reglages": reglages, "robot": robot})


## Un tour : entrees passees a calculer() et sortie rendue, puis ce que le controleur pense apres le tour.
func tour(t: float, sondes: Array, distance_cm: float, dt: float, nouvelle: bool, sortie: Dictionary, c: Controleur) -> void:
	if _fichier == null:
		return
	var en_phases := c.etat_court() == "CONTOURNEMENT"
	_ecrire([t, sondes, distance_cm, dt, nouvelle, int(sortie.get("vitesse", 0)), int(sortie.get("angle", 90)),
			c.etat_court(), Contournement.Phase.keys()[c.contournement.phase] if en_phases else "",
			c._commande_envoyee(), c._v_mm_s, c.distance_filtree_mm, c.braquage_moyen(), c.odo.x, c.odo.y, c.odo.cap,
			c.odo.parcours, c.odo.x_global, c.odo.y_global, c.odo.cap_global, c.odo.servo_estime])


## Fin du cas (resultat du banc) : derniere ligne, puis fermeture.
func finir(t: float, c: Controleur) -> void:
	if _fichier == null:
		return
	_ecrire({"type": "fin", "t": t, "etat": c.etat_court()})
	fermer()


func fermer() -> void:
	if _fichier != null:
		_fichier.close()
		_fichier = null


func _ecrire(donnees: Variant) -> void:
	_fichier.store_line(JSON.stringify(donnees, "", false, true))
