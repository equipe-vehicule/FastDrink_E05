"""Les fonctions de fastdrink.conduite.godot rendent EXACTEMENT ce que rend Godot 4.7.

Valeurs de référence : tests/donnees/fonctions_godot.json, calculées par Godot (tests/outils/sonde_fonctions_godot.gd).
Pour les régénérer (après un changement de version de Godot) :
    G --headless --script <chemin absolu>/code_picar/tests/outils/sonde_fonctions_godot.gd -- <chemin absolu du .json>
"""
import json
import math
import os

import pytest

from fastdrink.conduite import godot as gd

DONNEES = os.path.join(os.path.dirname(__file__), "donnees", "fonctions_godot.json")

with open(DONNEES, encoding="utf-8") as f:
    REF = json.load(f)


def identiques(a, b):
    """Égalité au bit près ; None (NaN dans le JSON de Godot) = NaN. Le signe du zéro n'est pas comparé :
    JSON.stringify de Godot écrit -0.0 comme 0.0 (vérifié), alors que Godot calcule bien avec -0.0."""
    if b is None:
        return a != a
    if isinstance(b, bool) or isinstance(a, bool):
        return a == b
    a = float(a)
    b = float(b)
    if a != a or b != b:
        return a != a and b != b
    return a == b


def test_vector2_est_en_float32():
    assert REF["double"] is False   # sinon Vec2 doit calculer en double : regler_precision_godot(False)


SCALAIRES = [r for r in REF["scalaires"] if not r.get("special")]


@pytest.mark.parametrize("nom,fonction", [
    ("roundf", lambda r: gd.roundf(r["a"])),
    ("round", lambda r: gd.roundf(r["a"])),
    ("floorf", lambda r: gd.floorf(r["a"])),
    ("ceilf", lambda r: gd.ceilf(r["a"])),
    ("signf", lambda r: gd.signf(r["a"])),
    ("absf", lambda r: gd.absf(r["a"])),
    ("minf", lambda r: gd.minf(r["a"], r["b"])),
    ("maxf", lambda r: gd.maxf(r["a"], r["b"])),
    ("move_toward", lambda r: gd.move_toward(r["a"], r["b"], abs(r["c"]))),
    ("lerpf", lambda r: gd.lerpf(r["a"], r["b"], r["w"])),
    ("clampf", lambda r: gd.clampf(r["a"], gd.minf(r["b"], r["c"]), gd.maxf(r["b"], r["c"]))),
    ("wrapf_pi", lambda r: gd.wrapf(r["a"], -gd.PI, gd.PI)),
    ("wrapf", lambda r: gd.wrapf(r["a"], gd.minf(r["b"], r["c"]), gd.maxf(r["b"], r["c"]))),
    ("fposmod", lambda r: gd.fposmod(r["a"], r["b"]) if r["b"] != 0.0 else 0.0),
    ("fmod", lambda r: gd.fmod(r["a"], r["b"]) if r["b"] != 0.0 else 0.0),
    ("deg_to_rad", lambda r: gd.deg_to_rad(r["a"])),
    ("rad_to_deg", lambda r: gd.rad_to_deg(r["a"])),
    ("sqrt", lambda r: gd.sqrt(abs(r["a"]))),
    ("is_inf", lambda r: gd.is_inf(r["a"])),
    ("is_finite", lambda r: gd.is_finite(r["a"])),
])
def test_fonction_scalaire(nom, fonction):
    ecarts = [(r["a"], r.get("b"), fonction(r), r[nom]) for r in SCALAIRES if not identiques(fonction(r), r[nom])]
    assert not ecarts, "%s : %d écarts, ex. %s" % (nom, len(ecarts), ecarts[:3])


def test_int_tronque_comme_godot():
    for r in SCALAIRES:
        if abs(r["a"]) < 1e9:
            assert int(r["a"]) == r["int"]


@pytest.mark.parametrize("nom,fonction", [
    ("sin", lambda r: gd.sin(r["a"])),
    ("cos", lambda r: gd.cos(r["a"])),
    ("tan", lambda r: gd.tan(r["a"])),
    ("atan", lambda r: gd.atan(r["a"])),
    ("atan2", lambda r: gd.atan2(r["a"], r["b"])),
    ("pow3", lambda r: gd.pow_(r["a"], 3.0)),
    ("pow4", lambda r: gd.pow_(r["a"], 4.0)),
])
def test_libm_proche_de_godot(nom, fonction):
    """Bibliothèque mathématique de CPython contre celle de Godot (Windows), pour des angles réalistes (|a| < 1e5).
    atan, atan2, pow entier : au bit près. sin, cos, tan : 1 ulp d'écart possible (mesuré, voir godot.py) ; sur le Pi
    (glibc), autre bibliothèque, mêmes ordres de grandeur. D'où la tolérance du rejeu (PORTAGE.md)."""
    au_bit_pres = nom not in ("sin", "cos", "tan")
    ecarts = []
    for r in SCALAIRES:
        if abs(r["a"]) >= 1e5:
            continue
        obtenu, attendu = fonction(r), r[nom]
        if au_bit_pres:
            ok = identiques(obtenu, attendu)
        else:
            ok = abs(obtenu - attendu) <= 2.3e-16 * max(1.0, abs(attendu))
        if not ok:
            ecarts.append((r["a"], r["b"], obtenu, attendu))
    assert not ecarts, "%s : %d écarts sur %d, ex. %s" % (nom, len(ecarts), len(SCALAIRES), ecarts[:3])


def test_min_max_cas_particuliers():
    for r in REF["scalaires"]:
        if r.get("special"):
            a = gd.INF if r["a"] > 1e300 else r["a"]
            assert identiques(gd.minf(a, r["b"]), r["minf"])
            assert identiques(gd.maxf(a, r["b"]), r["maxf"])


def test_entiers_comme_gdscript():
    for r in REF["entiers"]:
        a, b = r["a"], r["b"]
        assert gd.div_entier(a, b) == r["div"]
        assert gd.mod_entier(a, b) == r["mod"]
        assert gd.posmod(a, b) == r["posmod"]
        assert gd.maxi(a, b) == r["maxi"] and gd.mini(a, b) == r["mini"]
        assert float(a) / float(b) == r["float_div"]


VECTEURS = [r for r in REF["vecteurs"] if "special" not in r]


def _paire(v):
    return [v.x, v.y]


@pytest.mark.parametrize("nom,fonction", [
    ("a", lambda a, b, s: _paire(a)),
    ("plus", lambda a, b, s: _paire(a + b)),
    ("moins", lambda a, b, s: _paire(a - b)),
    ("neg", lambda a, b, s: _paire(-a)),
    ("fois", lambda a, b, s: _paire(a * s)),
    ("fois_g", lambda a, b, s: _paire(s * a)),
    ("div", lambda a, b, s: _paire(a / s)),
    ("plus_egal", lambda a, b, s: _paire(a + b)),
    ("fois_v", lambda a, b, s: _paire(a * b)),
    ("dot", lambda a, b, s: a.dot(b)),
    ("length", lambda a, b, s: a.length()),
    ("length_squared", lambda a, b, s: a.length_squared()),
    ("normalized", lambda a, b, s: _paire(a.normalized())),
    ("distance_to", lambda a, b, s: a.distance_to(b)),
    ("distance_squared_to", lambda a, b, s: a.distance_squared_to(b)),
    ("is_finite", lambda a, b, s: a.is_finite()),
    ("egal", lambda a, b, s: a == gd.Vec2(a.x, a.y)),
])
def test_vec2_float32(nom, fonction):
    ecarts = []
    for r in VECTEURS:
        a = gd.Vec2(r["ax"], r["ay"])
        b = gd.Vec2(r["bx"], r["by"])
        obtenu = fonction(a, b, r["s"])
        attendu = r[nom]
        if isinstance(attendu, list):
            ok = identiques(obtenu[0], attendu[0]) and identiques(obtenu[1], attendu[1])
        else:
            ok = identiques(obtenu, attendu)
        if not ok:
            ecarts.append((r["ax"], r["ay"], obtenu, attendu))
    assert not ecarts, "%s : %d écarts, ex. %s" % (nom, len(ecarts), ecarts[:2])


def test_vec2_infini():
    r = [x for x in REF["vecteurs"] if x.get("special") == "inf"][0]
    v = gd.Vec2(gd.INF, gd.INF)
    assert v.is_finite() == r["is_finite"]
    assert v.x == gd.INF and r["inf_x"] > 1e300
    assert v.distance_to(gd.Vec2(1.0, 2.0)) == gd.INF


def test_double_precision_optionnelle():
    gd.regler_precision_godot(False)
    try:
        assert gd.Vec2(0.1, 0.0).x == 0.1
    finally:
        gd.regler_precision_godot(True)
    assert gd.Vec2(0.1, 0.0).x != 0.1


def test_division_par_zero_comme_c():
    assert gd.div_reel(1.0, 0.0) == gd.INF
    assert gd.div_reel(-1.0, 0.0) == -gd.INF
    assert gd.is_nan(gd.div_reel(0.0, 0.0))
    assert gd.is_nan(gd.sqrt(-1.0))
