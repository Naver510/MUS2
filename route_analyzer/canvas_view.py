"""Widok Canvas: kamera (world<->screen), rysowanie tras, edycja myszą."""
import math
import tkinter as tk

from transform import apply_transform, route_center

GREEN = "#00ff9d"
WHEEL_TRACK_M = 0.14     # rozstaw kół: 140 mm
REF_LINE_WIDTH_M = 0.015  # szerokość linii trasy rzeczywistej: 15 mm
SMOOTH_K = 4             # półszerokość okna wygładzania trasy dla torów kół (punkty)
WHEEL_L = "#1e90ff"      # lewe koło
WHEEL_R = "#ff8c00"      # prawe koło
MARGIN = 30          # margines auto-fitu w pikselach
HIT_PX = 10          # tolerancja trafienia w trasę robota (piksele)


class CanvasView:
    def __init__(self, parent, on_transform_changed, on_reference_finished):
        self.canvas = tk.Canvas(parent, bg="white", highlightthickness=0)
        self.on_transform_changed = on_transform_changed
        self.on_reference_finished = on_reference_finished

        self.robot = []        # surowe punkty robota (świat, m)
        self.speeds = []       # prędkości punktów robota [m/s] (puste = brak heatmapy)
        self.heatmap = True
        self.show_wheels = False
        self.reference = []    # punkty trasy referencyjnej (świat, m)
        self.t = {"offset_x": 0.0, "offset_y": 0.0, "scale": 1.0, "rotation_deg": 0.0}

        # Kamera: środek widoku w świecie + pikseli na metr
        self.cam_x, self.cam_y, self.cam_scale = 0.0, 0.0, 50.0
        self.need_fit = True
        self.drawing = False
        self.dragging = False
        self._last = (0, 0)

        c = self.canvas
        c.bind("<Configure>", lambda e: self.redraw())
        c.bind("<Button-1>", self._on_press)
        c.bind("<B1-Motion>", self._on_drag)
        c.bind("<ButtonRelease-1>", self._on_release)
        c.bind("<Double-Button-1>", self._on_double)
        c.bind("<MouseWheel>", self._on_wheel)                       # Windows / macOS
        c.bind("<Button-4>", lambda e: self._zoom(1.05))             # Linux
        c.bind("<Button-5>", lambda e: self._zoom(1 / 1.05))

    # ---------- transformacje world <-> screen ----------
    def _center_px(self):
        w = max(self.canvas.winfo_width(), 2)
        h = max(self.canvas.winfo_height(), 2)
        return w / 2.0, h / 2.0

    def world_to_screen(self, x, y):
        cx, cy = self._center_px()
        # oś Y świata w górę, na ekranie w dół -> odwrócenie znaku
        return cx + (x - self.cam_x) * self.cam_scale, cy - (y - self.cam_y) * self.cam_scale

    def screen_to_world(self, px, py):
        cx, cy = self._center_px()
        return self.cam_x + (px - cx) / self.cam_scale, self.cam_y - (py - cy) / self.cam_scale

    # ---------- dane ----------
    def transformed_robot(self):
        t = self.t
        return apply_transform(self.robot, (t["offset_x"], t["offset_y"]),
                               t["scale"], t["rotation_deg"])

    def wheel_tracks(self):
        """Zwraca (lewe, prawe) trasy kół po transformacji nakładki.

        Założenie: plik nie ma kursu, więc kurs liczę ze stycznej do trasy robota
        (różnica punktów i-2..i+2); gdy robot stoi, zostaje poprzedni kurs.
        Koła leżą po obu stronach środka robota, prostopadle do kursu, ±WHEEL_TRACK_M/2.
        Trasy kół przechodzą przez tę samą transformację co trasa robota (środek obrotu
        = środek trasy robota), więc skalują się razem z nią.
        """
        n = len(self.robot)
        if n < 2:
            return [], []
        half = WHEEL_TRACK_M / 2.0
        left, right = [], []
        heading = None
        # Wygładzenie trasy średnią kroczącą (±SMOOTH_K punktów): surowa odometria jest
        # zaszumiona, a przesunięcie o 70 mm w bok wzmacnia każdy szum w "losowe linie".
        k = SMOOTH_K
        src = []
        for i in range(n):
            idx = [min(max(i + d, 0), n - 1) for d in range(-k, k + 1)]
            src.append((sum(self.robot[j][0] for j in idx) / len(idx),
                        sum(self.robot[j][1] for j in idx) / len(idx)))
        for i, (x, y) in enumerate(src):
            a, b = src[max(i - 1, 0)], src[min(i + 1, n - 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            if math.hypot(dx, dy) > 1e-5:
                heading = math.atan2(dy, dx)
            if heading is None:
                # początek bez ruchu: weź pierwszy kierunek w przód
                heading = next((math.atan2(q[1] - y, q[0] - x) for q in src
                                if math.hypot(q[0] - x, q[1] - y) > 1e-5), 0.0)
            nx, ny = -math.sin(heading), math.cos(heading)   # lewo od kierunku jazdy
            left.append((x + nx * half, y + ny * half))
            right.append((x - nx * half, y - ny * half))
        t = self.t
        c = route_center(self.robot)
        args = ((t["offset_x"], t["offset_y"]), t["scale"], t["rotation_deg"], c)
        return apply_transform(left, *args), apply_transform(right, *args)

    def fit(self):
        """Auto-fit kamery do bounding-boxa OBU tras (robot po transformacji + referencyjna).

        Wywoływane przy wczytaniu danych, nie przy każdej zmianie transformacji
        (inaczej widok uciekałby podczas przeciągania).
        """
        pts = self.transformed_robot() + list(self.reference)
        if self.show_wheels:
            pts += self.wheel_tracks()[0] + self.wheel_tracks()[1]
        if not pts:
            return
        w = max(self.canvas.winfo_width(), 2)
        h = max(self.canvas.winfo_height(), 2)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        # jeden współczynnik dla X i Y -> zachowanie proporcji
        sx = (w - 2 * MARGIN) / dx if dx > 1e-9 else float("inf")
        sy = (h - 2 * MARGIN) / dy if dy > 1e-9 else float("inf")
        s = min(sx, sy)
        if not math.isfinite(s) or s <= 0:
            s = 50.0
        self.cam_scale = s
        self.cam_x = (min(xs) + max(xs)) / 2.0
        self.cam_y = (min(ys) + max(ys)) / 2.0
        self.need_fit = False

    def request_fit(self):
        self.need_fit = True
        self.redraw()

    # ---------- rysowanie ----------
    def redraw(self):
        c = self.canvas
        if self.need_fit and c.winfo_width() > 1:
            self.fit()
        c.delete("all")
        if len(self.reference) >= 1:
            sp = [v for p in self.reference for v in self.world_to_screen(*p)]
            if len(sp) >= 4:
                # grubość linii referencyjnej w skali świata (15 mm), min. 2 px
                c.create_line(*sp, fill="black", width=max(2, REF_LINE_WIDTH_M * self.cam_scale))
            for i in range(0, len(sp), 2):
                c.create_oval(sp[i] - 2, sp[i + 1] - 2, sp[i] + 2, sp[i + 1] + 2, fill="black")
        if self.robot:  # robot NA WIERZCHU referencyjnej (rysowany później)
            pts = self.transformed_robot()
            if self.show_wheels:  # tory kół pod linią robota
                for track, col in zip(self.wheel_tracks(), (WHEEL_L, WHEEL_R)):
                    wsp = [v for p in track for v in self.world_to_screen(*p)]
                    if len(wsp) >= 4:
                        c.create_line(*wsp, fill=col, width=1)
            sp = [v for p in pts for v in self.world_to_screen(*p)]
            if self.heatmap and len(self.speeds) == len(pts) and len(pts) >= 2:
                self._draw_heat(sp)
            elif len(sp) >= 4:
                c.create_line(*sp, fill=GREEN, width=2)
            r = 5
            c.create_oval(sp[0] - r, sp[1] - r, sp[0] + r, sp[1] + r, fill="red", outline="red")
        if self.drawing:
            c.create_text(8, 8, anchor="nw", fill="#cc0000",
                          text="Rysowanie trasy referencyjnej: LPM = punkt, dwuklik/Enter/Esc = koniec")

    # ---------- heatmapa prędkości ----------
    @staticmethod
    def _heat_color(t):
        """t w [0,1]: niebieski (wolno) -> cyjan -> zielony -> żółty -> czerwony (szybko)."""
        stops = [(0, 0, 255), (0, 255, 255), (0, 255, 0), (255, 255, 0), (255, 0, 0)]
        t = max(0.0, min(1.0, t)) * (len(stops) - 1)
        i = min(int(t), len(stops) - 2)
        f = t - i
        a, b = stops[i], stops[i + 1]
        return "#%02x%02x%02x" % tuple(int(a[k] + (b[k] - a[k]) * f) for k in range(3))

    def _draw_heat(self, sp):
        c = self.canvas
        vmin, vmax = min(self.speeds), max(self.speeds)
        span = (vmax - vmin) or 1.0
        for i in range(len(self.speeds) - 1):
            v = (self.speeds[i] + self.speeds[i + 1]) / 2.0   # kolor odcinka = średnia z końców
            c.create_line(sp[2 * i], sp[2 * i + 1], sp[2 * i + 2], sp[2 * i + 3],
                          fill=self._heat_color((v - vmin) / span), width=3)
        # legenda (pasek gradientu w prawym górnym rogu)
        w = max(c.winfo_width(), 2)
        x0, y0, n, h = w - 30, 30, 50, 120
        for k in range(n):
            c.create_rectangle(x0, y0 + k * h / n, x0 + 14, y0 + (k + 1) * h / n,
                               fill=self._heat_color(1 - k / (n - 1)), outline="")
        c.create_text(x0 - 4, y0, anchor="ne", text="%.2f m/s" % vmax)
        c.create_text(x0 - 4, y0 + h, anchor="se", text="%.2f m/s" % vmin)

    # ---------- tryb rysowania ----------
    def start_drawing(self):
        self.reference = []
        self.drawing = True
        self.canvas.focus_set()
        self.redraw()

    def finish_drawing(self, _event=None):
        if not self.drawing:
            return
        self.drawing = False
        self.redraw()
        self.on_reference_finished()

    # ---------- mysz ----------
    def _hit_robot(self, px, py):
        pts = [self.world_to_screen(*p) for p in self.transformed_robot()]
        if len(pts) == 1:
            return math.hypot(px - pts[0][0], py - pts[0][1]) <= HIT_PX
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            vx, vy = x2 - x1, y2 - y1
            L2 = vx * vx + vy * vy
            u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / L2))
            if math.hypot(px - (x1 + u * vx), py - (y1 + u * vy)) <= HIT_PX:
                return True
        return False

    def _on_press(self, e):
        self.canvas.focus_set()
        if self.drawing:
            self.reference.append(self.screen_to_world(e.x, e.y))
            self.redraw()
        elif self.robot and self._hit_robot(e.x, e.y):
            self.dragging = True
            self._last = (e.x, e.y)

    def _on_double(self, e):
        if self.drawing:
            # pierwszy klik dwuklika dodał punkt, drugi dodał duplikat -> usuń duplikat
            if len(self.reference) >= 2 and self.reference[-1] == self.reference[-2]:
                self.reference.pop()
            self.finish_drawing()

    def _on_drag(self, e):
        if not self.dragging:
            return
        dx, dy = e.x - self._last[0], e.y - self._last[1]
        self._last = (e.x, e.y)
        # różnica pikseli -> metry; znak Y odwrócony
        self.t["offset_x"] += dx / self.cam_scale
        self.t["offset_y"] -= dy / self.cam_scale
        self.redraw()
        self.on_transform_changed()

    def _on_release(self, _e):
        self.dragging = False

    def _on_wheel(self, e):
        self._zoom(1.05 if e.delta > 0 else 1 / 1.05)

    def _zoom(self, factor):
        if not self.robot:
            return
        self.t["scale"] *= factor
        self.redraw()
        self.on_transform_changed()

    def rotate(self, deg):
        self.t["rotation_deg"] += deg
        self.redraw()
        self.on_transform_changed()
