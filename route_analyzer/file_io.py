"""Parsowanie pliku trasy robota oraz zapis/odczyt projektu JSON."""
import json
import re


def parse_robot_file(path):
    """Zwraca (points, speeds): punkty (x, y) w metrach oraz prędkości [m/s] (lub []).

    Format pełny (>=7 pól): x_ri y_ri x_pc y_pc speed error_p error_d -> bierzemy [2],[3].
    Format uproszczony (2-3 pola): x_pc y_pc [speed] -> bierzemy [0],[1].
    Separator: tab lub spacje. Puste linie i nagłówki (zaczynające się od litery) pomijane.
    Założenie: linie zaczynające się od '#' też traktuję jako komentarz.
    """
    points = []
    speeds = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip().lstrip("\ufeff")
            if not line or line[0].isalpha() or line[0] == "#":
                continue
            parts = re.split(r"\s+", line)
            # POPRAWKA: przeglądarkowy podgląd robota (web_ui.h drawMap) rysuje kolumny [0],[1]
            # (x_ri, y_ri = środek robota). Ślad czujnika x_pc,y_pc ([2],[3]) leży 12.5 cm przed
            # robotem, więc przy każdym drgnięciu kursu "macha" i daje poszarpany ślad.
            # Zmień na ix, iy = 2, 3 jeśli kiedyś potrzebny będzie ślad czujnika.
            if len(parts) >= 7 or len(parts) in (2, 3):
                ix, iy = 0, 1
            else:
                continue  # nietypowa liczba pól — pomijamy
            try:
                p = (float(parts[ix]), float(parts[iy]))
                # prędkość: kol. [4] (format 7-kol.) lub [2] (format 3-kol.)
                if len(parts) >= 7:
                    sp = float(parts[4])
                else:
                    sp = float(parts[2]) if len(parts) == 3 else None
            except ValueError:
                continue
            points.append(p)
            speeds.append(sp)
    if not points:
        raise ValueError("Brak poprawnych punktów w pliku: %s" % path)
    if any(s is None for s in speeds):
        speeds = []  # brak prędkości w pliku -> bez heatmapy
    return points, speeds


def save_project(path, robot_route, reference_route, transform, robot_speeds=()):
    data = {
        "robot_route": [[x, y] for x, y in robot_route],
        "robot_speeds": list(robot_speeds),  # opcjonalne (dla heatmapy), dodatek do formatu
        "reference_route": [[x, y] for x, y in reference_route],
        "transform": {
            "offset_x": transform["offset_x"],
            "offset_y": transform["offset_y"],
            "scale": transform["scale"],
            "rotation_deg": transform["rotation_deg"],
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def load_project(path):
    """Zwraca (robot_route, robot_speeds, reference_route, transform_dict)."""
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    robot = [(float(p[0]), float(p[1])) for p in d.get("robot_route", [])]
    ref = [(float(p[0]), float(p[1])) for p in d.get("reference_route", [])]
    t = d.get("transform", {})
    transform = {
        "offset_x": float(t.get("offset_x", 0.0)),
        "offset_y": float(t.get("offset_y", 0.0)),
        "scale": float(t.get("scale", 1.0)),
        "rotation_deg": float(t.get("rotation_deg", 0.0)),
    }
    speeds = [float(s) for s in d.get("robot_speeds", [])]
    if len(speeds) != len(robot):
        speeds = []
    return robot, speeds, ref, transform
