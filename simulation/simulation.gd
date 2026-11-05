## Scene de simulation du PiCar-S (equipe 05).
## Touches : 1-6 choix de piste | R remettre au depart | M manuel/automatique
##           Fleches : conduite manuelle | Espace : arret | C : vue | L : journal CSV on/off
extends Node3D

@export var masse_bille := 0.010      ## kg. PROVISOIRE : a peser (n'influence pas la chute, utile pour la suite).
@export var amortissement_bille := 1.0 ## 1/s, pertes par roulement. PROVISOIRE : a comparer avec le vrai robot.
@export var piste_initiale := 1        ## index dans Pistes.LISTE (1 = courbes)
@export var mode_auto := true

var vehicule: Vehicule
var controleur := Controleur.new()
var pistes: Pistes
var bille: BilleCuvette
var camera: CameraSuivi
var hud: Hud
var entrees := Entrees.new()
var mesure := MesureAcceleration.new()
var journal := JournalCSV.new()
var _temps := 0.0

func _ready() -> void:
	add_child(Monde.new())
	pistes = Pistes.new(); add_child(pistes)
	vehicule = Vehicule.new(); vehicule.name = "Vehicule"; add_child(vehicule)
	bille = BilleCuvette.new()
	bille.vehicule = vehicule; bille.masse = masse_bille; bille.amortissement = amortissement_bille
	add_child(bille)
	camera = CameraSuivi.new(); camera.cible = vehicule; add_child(camera)
	hud = Hud.new(); add_child(hud)
	_brancher_signaux()
	charger_piste(piste_initiale)

func _brancher_signaux() -> void:
	entrees.piste_demandee.connect(charger_piste)
	entrees.remise_demandee.connect(remettre_au_depart)
	entrees.mode_bascule.connect(func(): mode_auto = not mode_auto)
	entrees.vue_bascule.connect(camera.basculer_vue)
	entrees.journal_bascule.connect(journal.basculer)
	bille.chute.connect(func(): print("BILLE TOMBEE a t = %.2f s (acceleration max atteinte %.2f m/s2)" % [_temps, mesure.maximum]))

func charger_piste(i: int) -> void:
	pistes.charger(i)
	remettre_au_depart()

func remettre_au_depart() -> void:
	vehicule.placer(pistes.transform_depart())
	vehicule.commander(0, 90)
	entrees.reinitialiser()
	controleur = Controleur.new()
	_temps = 0.0
	mesure.reinitialiser()
	bille.replacer()

func _physics_process(dt: float) -> void:
	_temps += dt
	entrees.lire(Pistes.LISTE.size())
	_conduire(dt)
	mesure.ajouter(vehicule.vitesse, dt)
	bille.mettre_a_jour(dt)   # appele ici (et non dans la bille) pour garder l'ordre : commande -> mesure -> bille
	journal.ecrire(_temps, vehicule, bille.tombee)

## Qui conduit : personne tant que la bille n'est pas prete, sinon le controleur ou le clavier.
func _conduire(dt: float) -> void:
	if not bille.est_prete():
		vehicule.commander(0, 90)
	elif mode_auto:
		var sortie: Dictionary = controleur.calculer(vehicule.sondes.duplicate(), vehicule.distance_cm, dt)
		vehicule.commander(int(sortie.get("vitesse", 0)), int(sortie.get("angle", 90)))
	else:
		vehicule.commander(entrees.cmd_vitesse, entrees.cmd_angle)

func _process(_dt: float) -> void:
	hud.afficher(pistes.nom(), mode_auto, vehicule, mesure, bille.tombee, journal.actif())
