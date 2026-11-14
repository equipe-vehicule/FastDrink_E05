## Scenarios courts et rejouables : cas difficiles choisis a la main (le banc aleatoire ne les vise pas).
## Donnees seulement. Lancement sans editeur :
##   godot --headless --path . --fixed-fps 240 res://simulation/simulation.tscn -- --scenarios=tous --quitter
##   (--scenarios=nom1,nom2 pour quelques cas). Resultats : journaux/scenarios_<date>.csv et une ligne par cas.
##
## Format d'un scenario :
##   "nom", "but" (ce qu'il verifie et sa source) ;
##   "rayon_mm" : rayon des virages (une case = 2 x rayon) ;
##   "chemin" : une lettre par deplacement depuis la case de depart (cap de depart : vers le haut de la grille) :
##       T tout droit, G gauche, D droite. Les 2 premieres cases doivent rester droites (la voiture y est posee) ;
##   "obstacles" : liste de {"troncon": k, "face": ..., "taille_mm": [largeur, hauteur, profondeur]} :
##       troncon = k-ieme troncon de ligne droite (0 = le premier, voir GenerateurPiste._troncons_droits) ;
##       face (position de la face avant, depuis le debut du troncon) :
##         "min" : au plus tot permis (place pour l'arret et le recul + marge_droite_avant, comme le generateur) ;
##         "max" : au plus tard permis (droite_apres_min apres l'arriere, avant la fin du troncon) ;
##         "apres_precedent" : espacement minimal derriere l'obstacle precedent du meme troncon ;
##         un nombre : distance en mm ;
##       "decalage_mm" (facultatif) : decalage lateral, > 0 = a droite ;
##   "trace" (au lieu de chemin) : trace libre, GenerateurLibre : liste [["T", longueur_mm], ["G", angle_deg],
##       ["D", angle_deg], ...] depuis le cap de depart ; virages de rayon rayon_mm et d'angle quelconque. Dans les
##       obstacles, "troncon" compte alors les segments droits (le 0 est le premier, depart compris) ;
##   "piste" (au lieu de chemin et obstacles) : index d'une piste Blender de Pistes.LISTE ;
##   "fin_attendue" (facultatif, ARRIVEE par defaut) : resultat attendu en fin d'essai ;
##   "reglages_essai" (facultatif) : reglages du controleur pour ce cas seulement (meme format que le noeud
##       Simulation). Les cles "robot:" sont interdites ici (lues au lancement seulement) : les passer au lancement.
##
## Obstacle (precisions du 12 et du 13 nov.) : 100 x 150 x 50 mm, pose sur l'une de ses 3 faces, dans l'un des
## 2 sens : 6 tailles (largeur x hauteur x profondeur) dans les cas « face_* ».
extends RefCounted

const OBSTACLE_DEBOUT := [100, 150, 50]

const LISTE := [
	{"nom": "reference_droite", "but": "Cas temoin : ligne droite, 1 obstacle debout au plus tot",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": OBSTACLE_DEBOUT}]},
	{"nom": "obstacle_apres_virage", "but": "Obstacle au plus tot apres un virage de 120 mm (precision du 12 nov.)",
	 "rayon_mm": 120, "chemin": "TTTDTTTTTTT",
	 "obstacles": [{"troncon": 1, "face": "min", "taille_mm": OBSTACLE_DEBOUT}]},
	{"nom": "obstacle_avant_virage", "but": "Obstacle au plus tard avant un virage de 120 mm (precision du 12 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTDTTT",
	 "obstacles": [{"troncon": 0, "face": "max", "taille_mm": OBSTACLE_DEBOUT}]},
	{"nom": "deux_obstacles", "but": "2 obstacles a l'espacement minimal sur la meme droite (precision du 12 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": OBSTACLE_DEBOUT},
			{"troncon": 0, "face": "apres_precedent", "taille_mm": OBSTACLE_DEBOUT}]},
	{"nom": "face_debout", "but": "Obstacle debout, 100 de large (precision du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [100, 150, 50]}]},
	{"nom": "face_debout_tourne", "but": "Obstacle debout tourne, 50 de large, 100 de profondeur (precision du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [50, 150, 100]}]},
	{"nom": "face_flanc", "but": "Obstacle sur le flanc, 150 de large, 100 de haut (precision du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [150, 100, 50]}]},
	{"nom": "face_flanc_tourne", "but": "Obstacle sur le flanc tourne, 50 de large, 150 de profondeur (precision du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [50, 100, 150]}]},
	{"nom": "face_couche", "but": "Obstacle couche, 50 de haut, 150 de profondeur (precision du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [100, 50, 150]}]},
	{"nom": "face_couche_tourne", "but": "Obstacle couche tourne, 150 de large, 50 de haut (precision du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [150, 50, 100]}]},
	{"nom": "pire_cas_avant_virage", "but": "Obstacle couche de 150 de profondeur, au plus tard avant un virage : le retour du contournement finit pres du virage (precisions du 12 et du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTDTTT",
	 "obstacles": [{"troncon": 0, "face": "max", "taille_mm": [100, 50, 150]}]},
	{"nom": "pire_cas_deux_obstacles", "but": "2 obstacles sur le flanc (150 de large) a l'espacement minimal (precisions du 12 et du 13 nov.)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTTTTTT",
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [150, 100, 50]},
			{"troncon": 0, "face": "apres_precedent", "taille_mm": [150, 100, 50]}]},
	{"nom": "virage_serre", "but": "Un virage de 90 deg au rayon minimal de 120 mm (guide, section 1)",
	 "rayon_mm": 120, "chemin": "TTTTDTTTT", "obstacles": []},
	{"nom": "s_serre", "but": "Gauche puis droite colles, rayon 120 mm (guide : rayon ; enchainement non exclu)",
	 "rayon_mm": 120, "chemin": "TTTTGDTTTT", "obstacles": []},
	{"nom": "courbes_120", "but": "Piste Blender « Courbes de 120 mm » : bille a 0,94 le 12 nov. (#67)", "piste": 1,
	 "fin_attendue": "BLOQUE"},   # pas de barre d'arrivee : arret en fin de ligne (SUIVI_SIMULATION.md, section 2)
	{"nom": "parcours_fourni", "but": "Parcours fourni par le cours : pertes de ligne dans les virages serres (#74)", "piste": 5},
	{"nom": "demi_tour", "but": "Deux virages du meme cote colles : demi-tour de 240 mm de large (idem)",
	 "rayon_mm": 120, "chemin": "TTTTDDTTTT", "obstacles": []},
	# Recul court (#72) : D_arret 100 + D_recul 50 = 150 mm devant l'obstacle, le contournement fluide n'a pas la place
	{"nom": "recul_court_debout", "but": "D_recul 50 : contournement par phases verifie (#72), obstacle debout",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT", "reglages_essai": {"epreuve:distance_recul_mm": 50.0},
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": OBSTACLE_DEBOUT}]},
	{"nom": "recul_court_flanc", "but": "D_recul 50, bloc de 150 de large decale de 10 mm du cote du contournement (#72, graine 63353)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTT", "reglages_essai": {"epreuve:distance_recul_mm": 50.0},
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [150, 100, 50], "decalage_mm": 10}]},
	{"nom": "recul_court_deux_obstacles", "but": "D_recul 50, deux blocs de 150 de large a l'espacement minimal (#72)",
	 "rayon_mm": 120, "chemin": "TTTTTTTTTTTTTT", "reglages_essai": {"epreuve:distance_recul_mm": 50.0},
	 "obstacles": [{"troncon": 0, "face": "min", "taille_mm": [150, 100, 50]},
			{"troncon": 0, "face": "apres_precedent", "taille_mm": [150, 100, 50]}]},
	# Trace libre (GenerateurLibre) : virages d'angle quelconque, le guide n'impose que le rayon (#73)
	{"nom": "virage_45", "but": "Virage de 45 deg au rayon minimal (guide : aucun angle impose, #73)",
	 "rayon_mm": 120, "trace": [["T", 1200], ["D", 45], ["T", 1500]], "obstacles": []},
	{"nom": "virage_135", "but": "Virage de 135 deg au rayon minimal (guide : aucun angle impose, #73)",
	 "rayon_mm": 120, "trace": [["T", 1200], ["D", 135], ["T", 1500]], "obstacles": []},
	{"nom": "s_60", "but": "Gauche puis droite de 60 deg colles, rayon 120 mm (guide : rayon seulement, #73)",
	 "rayon_mm": 120, "trace": [["T", 1200], ["G", 60], ["D", 60], ["T", 1500]], "obstacles": []},
	{"nom": "epingle_180", "but": "Epingle de 180 deg au rayon minimal (guide : aucun angle impose, #73)",
	 "rayon_mm": 120, "trace": [["T", 1200], ["D", 180], ["T", 1500]], "obstacles": []},
	{"nom": "obstacle_apres_virage_30", "but": "Obstacle au plus tot apres un virage de 30 deg (precision du 12 nov., #73)",
	 "rayon_mm": 120, "trace": [["T", 1000], ["G", 30], ["T", 2000]],
	 "obstacles": [{"troncon": 1, "face": "min", "taille_mm": OBSTACLE_DEBOUT}]},
]

## Scenarios demandes : "tous" ou "nom1,nom2". Un nom inconnu est signale.
static func choisir(demande: String) -> Array:
	if demande == "tous":
		return LISTE.duplicate()
	var choisis := []
	for nom in demande.split(",", false):
		var trouve := false
		for s in LISTE:
			if s["nom"] == nom.strip_edges():
				choisis.append(s)
				trouve = true
		if not trouve:
			push_error("Scenario inconnu : « %s »." % nom)
	return choisis
