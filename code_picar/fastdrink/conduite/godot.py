"""Fonctions de GDScript (Godot 4.7) reproduites à l'identique, pour porter le contrôleur sans changer un seul résultat.

Chaque fonction suit la définition du moteur Godot (core/math/math_funcs.h), vérifiée contre des valeurs calculées
par Godot lui-même : tests/donnees/fonctions_godot.json, produit par tests/outils/sonde_fonctions_godot.gd.

Pièges couverts (voir PORTAGE.md) :
  - roundf / round de GDScript arrondissent .5 en s'éloignant de zéro ; le round() de Python arrondit au pair ;
  - minf, maxf, clampf, signf : définis par des comparaisons (macros MIN, MAX, CLAMP, SIGN), pas comme min() de Python ;
  - math.sqrt, math.pow, math.tan... lèvent une exception là où Godot rend NaN ou ±INF : enveloppes ci-dessous ;
  - division et % entiers : GDScript tronque vers zéro (-7 / 2 = -3, -7 % 3 = -1) ; Python arrondit vers -inf ;
  - Vector2 : en simple précision (float32) dans Godot 4.7 standard (OS.has_feature("double") = false) : voir Vec2.

Aucune dépendance (pas de numpy) ; Python 3.9+.
"""
import math
import struct

PI = math.pi
TAU = math.tau
INF = math.inf
NAN = math.nan
CMP_EPSILON = 0.00001

# ----- Simple précision (float32) -----------------------------------------------------------------------------------

## Vrai : Vec2 arrondit en float32 comme Godot (rejeu exact des traces). Faux : calcul en double (plus rapide sur le
## Pi, mais les résultats s'écartent de la simulation au dernier chiffre ; à n'utiliser qu'après mesure du temps de
## calcul). Se change par regler_precision_godot().
PRECISION_GODOT = True

_F32 = struct.Struct("<f")


def _f32_exact(x):
    """Arrondi au float32 le plus proche (comme la conversion double -> float de C++), rendu en float Python."""
    try:
        return _F32.unpack(_F32.pack(x))[0]
    except OverflowError:
        # au-delà du plus grand float32 : C++ donne ±INF
        return math.copysign(INF, x)


def _identite(x):
    return x


f32 = _f32_exact


def regler_precision_godot(exacte):
    """Choisit l'arrondi float32 des Vec2 (vrai, défaut) ou le calcul en double (faux)."""
    global PRECISION_GODOT, f32
    PRECISION_GODOT = bool(exacte)
    f32 = _f32_exact if exacte else _identite


# ----- Fonctions scalaires (GDScript float = double) ----------------------------------------------------------------

def roundf(x):
    """round()/roundf() de GDScript : .5 s'éloigne de zéro ; garde le signe (-0.0).

    Mesuré sur Godot 4.7 (Windows) : roundf(0.49999999999999994) = 1.0, donc floor(|x| + 0.5) et non le round de C
    (qui donnerait 0.0). Seul ce cas limite les distingue."""
    if not math.isfinite(x):
        return x
    return math.copysign(math.floor(abs(x) + 0.5), x)


def floorf(x):
    return float(math.floor(x)) if math.isfinite(x) else x


def ceilf(x):
    return float(math.ceil(x)) if math.isfinite(x) else x


def absf(x):
    return abs(x)


def signf(x):
    """SIGN de Godot : 1.0, -1.0 ou 0.0 (0.0 aussi pour -0.0 et NaN)."""
    return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


def minf(a, b):
    """MIN(a, b) = a < b ? a : b (minf(0.0, -0.0) = -0.0)."""
    return a if a < b else b


def maxf(a, b):
    """MAX(a, b) = a > b ? a : b."""
    return a if a > b else b


def mini(a, b):
    return a if a < b else b


def maxi(a, b):
    return a if a > b else b


def clampf(v, mini_, maxi_):
    """CLAMP : v < min ? min : (v > max ? max : v)."""
    return mini_ if v < mini_ else (maxi_ if v > maxi_ else v)


def move_toward(depart, cible, pas):
    return cible if abs(cible - depart) <= pas else depart + signf(cible - depart) * pas


def lerpf(depart, arrivee, poids):
    return depart + (arrivee - depart) * poids


def is_zero_approx(x):
    return abs(x) < CMP_EPSILON


def is_equal_approx(a, b):
    if a == b:
        return True
    tolerance = CMP_EPSILON * abs(a)
    if tolerance < CMP_EPSILON:
        tolerance = CMP_EPSILON
    return abs(a - b) < tolerance


def fmod(a, b):
    if b == 0.0 or not math.isfinite(a):
        return NAN
    return math.fmod(a, b)


def fposmod(x, y):
    """Reste toujours du signe de y (Godot : fmod puis + y si les signes diffèrent)."""
    v = fmod(x, y)
    if (v < 0.0 and y > 0.0) or (v > 0.0 and y < 0.0):
        v += y
    return v + 0.0


def wrapf(valeur, mini_, maxi_):
    """wrapf de Godot : ramène valeur dans [min, max[ ; rend min si le résultat est « presque » max."""
    etendue = maxi_ - mini_
    if is_zero_approx(etendue):
        return mini_
    resultat = valeur - (etendue * math.floor((valeur - mini_) / etendue))
    if is_equal_approx(resultat, maxi_):
        return mini_
    return resultat


def deg_to_rad(x):
    return x * (PI / 180.0)


def rad_to_deg(x):
    return x * (180.0 / PI)


def sqrt(x):
    """sqrt de Godot : NaN pour x < 0 (math.sqrt lèverait une exception)."""
    if x < 0.0 or x != x:
        return NAN
    return math.sqrt(x)


def pow_(x, y):
    """pow de Godot. Exposant entier (|y| <= 64) : multiplications successives par élévation au carré, comme le
    pow de Godot sous Windows (mesuré : pow(x, 3) = x * (x * x), pow(x, 4) = (x * x) * (x * x), au bit près ; le
    math.pow de CPython diffère de 1 à 2 ulp). Sinon : math.pow, avec NaN/INF au lieu d'une exception."""
    if y == math.floor(y) and abs(y) <= 64 and math.isfinite(x):
        n = int(abs(y))
        resultat = 1.0
        base = x
        while n:
            if n & 1:
                resultat *= base
            n >>= 1
            if n:
                base *= base
        return div_reel(1.0, resultat) if y < 0 else resultat
    try:
        return math.pow(x, y)
    except (ValueError, OverflowError):
        try:
            return float(x) ** float(y) if x >= 0.0 else NAN
        except OverflowError:
            return INF


# sin, cos, tan : Godot sous Windows ne rend pas toujours le même dernier bit que CPython (mesuré : 1 ulp d'écart pour
# ~5 % des valeurs ; sin(PI) = 1.2246063538223773e-16 au lieu de 1.2246467991473532e-16, signature de l'instruction
# fsin du x87). Non reproductible simplement en Python : le rejeu compare donc les flottants avec une tolérance
# (tests/test_rejeu.py). atan, atan2, sqrt : identiques au bit près (mesuré).
def tan(x):
    return math.tan(x) if math.isfinite(x) else NAN


def sin(x):
    return math.sin(x) if math.isfinite(x) else NAN


def cos(x):
    return math.cos(x) if math.isfinite(x) else NAN


atan = math.atan
atan2 = math.atan2


def is_nan(x):
    return x != x


def is_inf(x):
    return math.isinf(x)


def is_finite(x):
    return math.isfinite(x)


def div_reel(a, b):
    """a / b en flottants comme C : ±INF ou NaN si b = 0 (Python lèverait ZeroDivisionError)."""
    if b == 0.0:
        if a == 0.0 or a != a:
            return NAN
        return math.copysign(INF, a) * math.copysign(1.0, b)
    return a / b


def div_entier(a, b):
    """a / b entre int de GDScript : tronqué vers zéro (-7 / 2 = -3)."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def mod_entier(a, b):
    """a % b entre int de GDScript : du signe de a (-7 % 3 = -1)."""
    return a - b * div_entier(a, b)


def posmod(a, b):
    v = mod_entier(a, b)
    if (v < 0 and b > 0) or (v > 0 and b < 0):
        v += b
    return v


# ----- Vector2 (float32) ---------------------------------------------------------------------------------------------

class Vec2:
    """Vector2 de Godot 4.7 standard : composantes en float32, chaque opération arrondie en float32.

    Règles reproduites (vérifiées par tests/test_godot.py) :
      - construction depuis des doubles : arrondi float32 ;
      - v * s, s * v, v / s : s est d'abord converti en float32 (real_t) ;
      - dot, length, length_squared, distance_to... : calculés en float32 (chaque produit et chaque somme arrondis) ;
      - .x et .y rendent la valeur float32 (exacte en double) ; les scalaires de GDScript, eux, restent en double.
    Objet immuable : `v.x = ...` du GDScript s'écrit `v = Vec2(..., v.y)`.
    """

    __slots__ = ("x", "y")

    def __init__(self, x=0.0, y=0.0):
        object.__setattr__(self, "x", f32(float(x)))
        object.__setattr__(self, "y", f32(float(y)))

    def __setattr__(self, nom, valeur):
        raise AttributeError("Vec2 est immuable : créer un nouveau Vec2")

    def __repr__(self):
        return "Vec2(%r, %r)" % (self.x, self.y)

    def __eq__(self, autre):
        return isinstance(autre, Vec2) and self.x == autre.x and self.y == autre.y

    def __ne__(self, autre):
        return not self.__eq__(autre)

    def __hash__(self):
        return hash((self.x, self.y))

    def __iter__(self):
        yield self.x
        yield self.y

    def __add__(self, o):
        return Vec2(self.x + o.x, self.y + o.y)

    def __sub__(self, o):
        return Vec2(self.x - o.x, self.y - o.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __mul__(self, o):
        if isinstance(o, Vec2):
            return Vec2(self.x * o.x, self.y * o.y)
        s = f32(float(o))
        return Vec2(self.x * s, self.y * s)

    __rmul__ = __mul__

    def __truediv__(self, o):
        if isinstance(o, Vec2):
            return Vec2(div_reel(self.x, o.x), div_reel(self.y, o.y))
        s = f32(float(o))
        return Vec2(div_reel(self.x, s), div_reel(self.y, s))

    def dot(self, o):
        return f32(f32(self.x * o.x) + f32(self.y * o.y))

    def length_squared(self):
        return f32(f32(self.x * self.x) + f32(self.y * self.y))

    def length(self):
        return f32(math.sqrt(self.length_squared()))

    def distance_squared_to(self, o):
        dx = f32(self.x - o.x)
        dy = f32(self.y - o.y)
        return f32(f32(dx * dx) + f32(dy * dy))

    def distance_to(self, o):
        return f32(math.sqrt(self.distance_squared_to(o)))

    def normalized(self):
        l2 = self.length_squared()
        if l2 == 0.0:
            return Vec2(self.x, self.y)
        l = f32(math.sqrt(l2))
        return Vec2(self.x / l, self.y / l)

    def is_finite(self):
        return math.isfinite(self.x) and math.isfinite(self.y)


Vec2.ZERO = Vec2(0.0, 0.0)
