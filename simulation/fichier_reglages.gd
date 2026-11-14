## Lecture du fichier unique des valeurs du robot (reglages_robot.json, racine du depot).
## Seul code lie a Godot pour les reglages : le code Python du robot fera la meme chose avec json.load.
## Chaque entree du fichier : {"valeur", "unite", "statut", "source", "tache"} ; on ne garde que la valeur.
## Charge par preload (pas de class_name) : utilisable sans editeur, sans scan du systeme de fichiers.
extends RefCounted

const CHEMIN := "res://reglages_robot.json"
const CHAMPS := ["valeur", "unite", "statut", "source", "tache"]
const STATUTS := ["provisoire", "mesuré"]

## Rend {nom: valeur} ; dictionnaire vide si le fichier est absent ou illisible (erreur signalee).
static func lire(chemin := CHEMIN) -> Dictionary:
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
			push_error("Reglages du robot : statut « %s » inconnu pour %s (provisoire ou mesuré)." % [entree["statut"], nom])
		if entree.has("valeur"):
			valeurs[nom] = entree["valeur"]
	return valeurs
