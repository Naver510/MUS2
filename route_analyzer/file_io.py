"""Parsowanie pliku trasy robota oraz zapis/odczyt projektu JSON."""
import json
import math
import re

SENSOR_OFFSET_M = 0.128  # odsunięcie listwy czujników przed środek obrotu (128 mm)
SENSOR_PITCH_M = 0.012   # skok czujników na listwie (12 mm / jednostkę error_p)


def _smooth_minor_jitter(points):
    """Delikatne 3-punktowe wygładzenie drobnych oscylacji na prostych, bez ścinania zakrętów."""
    if len(points) < 3:
        return list(points)
    out = list(points)
    for i in range(1, len(points) - 1):
        v1x, v1y = points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]
        v2x, v2y = points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]
        n1, n2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
        if n1 > 1e-6 and n2 > 1e-6 and ((v1x * v2x + v1y * v2y) / (n1 * n2)) > 0.90:
            out[i] = (
                0.25 * points[i - 1][0] + 0.5 * points[i][0] + 0.25 * points[i + 1][0],
                0.25 * points[i - 1][1] + 0.5 * points[i][1] + 0.25 * points[i + 1][1],
            )
    return out


def parse_robot_file_full(path):
    """Zwraca (line_points, axle_points, speeds).

    Format pełny (>=7 pól): x_ri y_ri x_pc y_pc speed error_p error_d
      - axle_points: środek obrotu robota (x_ri, y_ri)
      - line_points: zrekonstruowany ślad linii pod listwą (odsunięcie 128 mm + poprawka error_p)
    Format uproszczony (2-3 pola): x_pc y_pc [speed] -> obie trasy równe [0],[1].
    Separator: tab lub spacje. Puste linie, komentarze '#' i nagłówki pomijane.
    """
    line_points = []
    axle_points = []
    speeds = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip().lstrip("\ufeff")
            if not line or line[0].isalpha() or line[0] == "#":
                continue
            parts = re.split(r"\s+", line)
            if len(parts) >= 7:
                try:
                    x_ri, y_ri = float(parts[0]), float(parts[1])
                    x_pc, y_pc = float(parts[2]), float(parts[3])
                    sp = float(parts[4])
                    error_p = float(parts[5])
                except ValueError:
                    continue
                ori = math.atan2(y_pc - y_ri, x_pc - x_ri)
                x_line = x_ri + SENSOR_OFFSET_M * math.cos(ori) - error_p * SENSOR_PITCH_M * math.sin(ori)
                y_line = y_ri + SENSOR_OFFSET_M * math.sin(ori) + error_p * SENSOR_PITCH_M * math.cos(ori)
                axle_points.append((x_ri, y_ri))
                line_points.append((x_line, y_line))
                speeds.append(sp)
            elif len(parts) in (2, 3):
                try:
                    p = (float(parts[0]), float(parts[1]))
                    sp = float(parts[2]) if len(parts) == 3 else None
                except ValueError:
                    continue
                axle_points.append(p)
                line_points.append(p)
                speeds.append(sp)
            else:
                continue  # nietypowa liczba pól — pomijamy
    if not line_points:
        raise ValueError("Brak poprawnych punktów w pliku: %s" % path)
    if any(s is None for s in speeds):
        speeds = []  # brak prędkości w pliku -> bez heatmapy
    if axle_points != line_points:
        line_points = _smooth_minor_jitter(line_points)
    return line_points, axle_points, speeds


def parse_robot_file(path):
    """Zwraca (points, speeds): zrekonstruowany ślad linii [m] oraz prędkości [m/s] (lub [])."""
    line_points, _axle_points, speeds = parse_robot_file_full(path)
    return line_points, speeds


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
