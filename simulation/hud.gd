## Panneau d'informations en haut a gauche de l'ecran.
class_name Hud
extends CanvasLayer

const AIDE := "[1-7] piste  [G] nouvelle piste aleatoire  [R] depart  [M] mode  [fleches] conduite  [Espace] arret  [C] vue  [L] journal"

var _label: Label

func _ready() -> void:
	var fond := PanelContainer.new(); add_child(fond)
	_label = Label.new(); fond.add_child(_label)

func afficher(nom_piste: String, mode_auto: bool, v: Vehicule, mesure: MesureAcceleration, bille_tombee: bool, journal_actif: bool,
		etat_controleur := "") -> void:
	var s := v.sondes
	var accel_laterale := v.vitesse * v.vitesse * tan(v.angle_roues_rad()) / v.empattement
	var ultrason := ("%.1f cm" % v.distance_cm) if v.distance_cm >= 0 else "rien"
	_label.text = "\n".join([
		"Piste : %s   |   Mode : %s" % [nom_piste, "AUTOMATIQUE" if mode_auto else "MANUEL"],
		"Controleur : %s" % etat_controleur,
		"Commande vitesse : %d   Servo : %d   Roues : %.1f deg" % [v.commande_vitesse, v.angle_servo, rad_to_deg(v.angle_roues_rad())],
		"Vitesse : %.3f m/s   Acceleration avant : %.2f m/s2 (max %.2f)   laterale : %.2f m/s2   Distance : %.2f m" % [
			v.vitesse, mesure.moyenne, mesure.maximum, accel_laterale, v.distance_parcourue],
		"Sondes : %d %d %d %d %d   Ultrason : %s" % [s[0], s[1], s[2], s[3], s[4], ultrason],
		"Bille : %s   %s" % ["TOMBEE" if bille_tombee else "en place", "COLLISION !" if v.en_collision else ""],
		AIDE + ("  (journal en cours)" if journal_actif else ""),
	])
