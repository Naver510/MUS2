"""Czysta logika transformacji nakładki (bez tkintera)."""
import math


def route_center(points):
    """Środek trasy = środek jej bounding-boxa (założenie: stabilniejszy niż średnia)."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


def apply_transform(points, offset, scale, rotation_deg, center=None):
    """Skaluj -> obróć -> przesuń. Skala i obrót wokół środka trasy.

    offset = (offset_x, offset_y) w metrach, rotation_deg dodatni = przeciwnie do
    ruchu wskazówek zegara (oś Y w górę, jak w matematyce).
    center: opcjonalny środek (np. środek trasy robota, gdy transformujemy trasy kół
    tą samą transformacją); domyślnie środek podanych punktów.
    """
    if not points:
        return []
    cx, cy = center if center is not None else route_center(points)
    a = math.radians(rotation_deg)
    ca, sa = math.cos(a), math.sin(a)
    ox, oy = offset
    out = []
    for x, y in points:
        dx, dy = (x - cx) * scale, (y - cy) * scale   # skalowanie
        rx = dx * ca - dy * sa                        # obrót
        ry = dx * sa + dy * ca
        out.append((cx + rx + ox, cy + ry + oy))      # przesunięcie
    return out
