## Lecture du fichier unique des valeurs du robot (reglages_robot.json, racine du depot).
## Seul code lie a Godot pour les reglages : le code Python du robot fera la meme chose avec json.load.
## Chaque entree du fichier : {"valeur", "unite", "statut", "source", "tache"} ; on ne garde que la valeur.
## Charge par preload (pas de class_name) : utilisable sans editeur, sans scan du systeme de fichiers.
extends RefCounted

const CHEMIN := "res://reglages_robot.json"
const CHAMPS := ["valeur", "unite", "statut", "source", "tache"]
const STATUTS := ["provisoire", "mesuré", "hypothèse"]

## MODELE IDEAL (option --modele=ideal, propriete modele_ideal du noeud Simulation) : valeurs appliquees PAR-DESSUS
## reglages_robot.json (qui porte le modele realiste). Seul endroit ou le modele ideal est defini : mesures sans bruit,
## sans valeur aberrante, sans biais, sans retard ; vitesse lineaire de 4 mm/s par commande (pas de table, donc pas de
## zone morte) ; moteur instantane ; filtre de l'ultrason = mediane de 3 ; odometrie exacte. C'est l'etat de la
## simulation avant l'etape 9 : le banc doit redonner exactement les CSV de reference (essais/).
const VALEURS_IDEALES := {
	"frequence_ultrason_hz": 20.0,
	"bruit_ultrason_mm": 0.0,
	"proba_aberrante_ultrason": 0.0,
	"gain_ultrason": 1.0,
	"biais_ultrason_mm": 0.0,
	"retard_ultrason_s": 0.0,
	"filtre_ultrason": "mediane3",
	"table_vitesse_mm_s": [],
	"vitesse_mm_s_par_cmd": 4.0,
	"constante_temps_moteur_s": 0.0,
	"erreur_vitesse_odometrie": 0.0,
}

## Rend {nom: valeur} (avec VALEURS_IDEALES par-dessus si ideal) ; dictionnaire vide si le fichier est absent ou illisible (erreur signalee).
static func lire(chemin := CHEMIN, ideal := false) -> Dictionary:
	var texte := FileAccess.get_file_as_string(chemin)
	if texte.is_empty():
		push_error("Reglages du robot : fichier introuvable ou vide (%s)." % chemin)
		return {}
	var donnees: Variant = JSON.parse_string(texte)
	if not donnees is Dictionary:
		push_error("Reglages du robot : JSON invalide (%s)." % chemin)
		return {}
	var valeurs := {}
	for nom in donnees:
		if String(nom).begins_with("_"):
			continue   # _info : explications, pas une valeur
		var entree: Variant = donnees[nom]
		if not entree is Dictionary:
			push_error("Reglages du robot : %s n'est pas une entree {valeur, unite, statut, source, tache}." % nom)
			continue
		for champ in CHAMPS:
			if not entree.has(champ):
				push_error("Reglages du robot : champ « %s » manquant pour %s." % [champ, nom])
		if entree.has("statut") and not entree["statut"] in STATUTS:
			push_error("Reglages du robot : statut « %s » inconnu pour %s (provisoire, mesuré ou hypothèse)." % [entree["statut"], nom])
		if entree.has("valeur"):
			valeurs[nom] = entree["valeur"]
	if ideal:
		valeurs.merge(VALEURS_IDEALES, true)
	return valeurs
