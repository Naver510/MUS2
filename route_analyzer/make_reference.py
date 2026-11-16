"""Generuje projekt JSON z trasą referencyjną z listy odcinków (osobne narzędzie, nie część app).

Uruchom: python make_reference.py   ->  reference_route.json
W aplikacji: Plik -> Wczytaj projekt, potem F5 / Wczytaj trasę robota.
Układ: start (0,0), początkowy kierunek jazdy = +X, skręt w lewo = +90°, w prawo = -90°.
Wymiary w mm — edytuj listę SEGMENTS i uruchom ponownie.
"""
import json
import math

# ("prosto", długość_mm) lub ("obrót", kąt_stopnie; + = lewo)
L, R = 90, -90
SEGMENTS = [
    ("prosto", 835), ("obrót", L),
    ("prosto", 420), ("obrót", L),
    ("prosto", 348), ("obrót", L),
    ("prosto", 200), ("obrót", R),
    ("prosto", 247), ("obrót", R),
    ("prosto", 205), ("obrót", L),
    ("prosto", 315), ("obrót", L),
    ("prosto", 435), ("obrót", L),   # ostatni obrót: kierunek powrotu na start
]
LINE_WIDTH_MM = 15
CLOSE_LOOP = True   # dodaj punkt startowy na końcu (1 okrążenie)

x = y = 0.0
heading = 0.0
pts_mm = [(x, y)]
for kind, val in SEGMENTS:
    if kind == "prosto":
        x += val * math.cos(math.radians(heading))
        y += val * math.sin(math.radians(heading))
        pts_mm.append((round(x, 3), round(y, 3)))
    else:
        heading += val
print("Koniec ostatniego odcinka: (%.0f, %.0f) mm; luka do startu: %.0f mm"
      % (x, y, math.hypot(x, y)))
if CLOSE_LOOP:
    pts_mm.append((0.0, 0.0))

project = {
    "robot_route": [],
    "reference_route": [[px / 1000.0, py / 1000.0] for px, py in pts_mm],  # metry
    "transform": {"offset_x": 0.0, "offset_y": 0.0, "scale": 1.0, "rotation_deg": 0.0},
}
with open("reference_route.json", "w", encoding="utf-8") as f:
    json.dump(project, f, indent=2)
print("Zapisano reference_route.json, linia %d mm" % LINE_WIDTH_MM)
