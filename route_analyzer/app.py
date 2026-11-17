"""Analizator tras robota liniowego. Uruchomienie: python app.py [plik_trasy]"""
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox

import file_io
from analysis import analyze_route
from canvas_view import CanvasView


class App:
    def __init__(self, root):
        self.root = root
        root.title("Analiza trasy robota")
        root.geometry("1100x700")
        self.last_dir = os.getcwd()   # ostatnio użyty katalog (tylko w pamięci procesu)
        self.last_robot_path = None   # do odświeżania F5
        self._updating = False        # blokada pętli pole <-> widok

        side = tk.Frame(root, padx=8, pady=8)
        side.pack(side="right", fill="y")
        self.view = CanvasView(root, self.sync_entries, self.on_ref_done)
        self.view.canvas.pack(side="left", fill="both", expand=True)

        self.vars = {}
        for key, label in (("offset_x", "offset_x [m]"), ("offset_y", "offset_y [m]"),
                           ("scale", "scale"), ("rotation_deg", "rotation_deg [°]")):
            tk.Label(side, text=label).pack(anchor="w")
            var = tk.StringVar()
            e = tk.Entry(side, textvariable=var, width=14)
            e.pack(anchor="w", pady=(0, 6))
            e.bind("<Return>", self.read_entries)
            e.bind("<FocusOut>", self.read_entries)
            var.trace_add("write", lambda *a: self.read_entries())  # zmiana w polu odświeża widok
            self.vars[key] = var
        tk.Button(side, text="Reset transformacji", command=self.reset_transform).pack(fill="x", pady=4)
        tk.Button(side, text="Odśwież trasę robota (F5)", command=self.refresh).pack(fill="x", pady=4)
        tk.Button(side, text="Pokaż mapę / wyśrodkuj (Home)",
                  command=self.view.request_fit).pack(fill="x", pady=4)
        self.heat_var = tk.BooleanVar(value=True)
        tk.Checkbutton(side, text="Heatmapa prędkości", variable=self.heat_var,
                       command=self.toggle_heat).pack(anchor="w", pady=4)
        self.wheel_var = tk.BooleanVar(value=False)
        tk.Checkbutton(side, text="Tory kół (rozstaw 140 mm)", variable=self.wheel_var,
                       command=self.toggle_wheels).pack(anchor="w", pady=4)
        self.comp_var = tk.BooleanVar(value=True)
        tk.Checkbutton(side, text="Kompensacja listwy (128 mm)", variable=self.comp_var,
                       command=self.toggle_compensate).pack(anchor="w", pady=4)
        tk.Button(side, text="Analizuj (AI)", command=self.analyze).pack(fill="x", pady=4)
        tk.Label(side, text="Mysz: przeciągnij zieloną trasę,\nkółko = skala, [ ] = obrót",
                 justify="left", fg="#555").pack(anchor="w", pady=10)

        menu = tk.Menu(root)
        fm = tk.Menu(menu, tearoff=0)
        fm.add_command(label="Wczytaj trasę robota", command=self.load_robot_dialog)
        fm.add_command(label="Odśwież trasę robota", accelerator="F5", command=self.refresh)
        fm.add_command(label="Narysuj trasę referencyjną", command=self.view.start_drawing)
        fm.add_separator()
        fm.add_command(label="Zapisz projekt", command=self.save_project)
        fm.add_command(label="Wczytaj projekt", command=self.load_project)
        menu.add_cascade(label="Plik", menu=fm)
        root.config(menu=menu)

        root.bind("<F5>", lambda e: self.refresh())
        root.bind("<Home>", lambda e: self.view.request_fit())
        root.bind("<bracketleft>", lambda e: self.view.rotate(-2))
        root.bind("<bracketright>", lambda e: self.view.rotate(2))
        root.bind("<Return>", self.view.finish_drawing)
        root.bind("<Escape>", self.view.finish_drawing)
        self.sync_entries()

    # ---------- synchronizacja pól ----------
    def sync_entries(self):
        self._updating = True
        for k, v in self.view.t.items():
            self.vars[k].set("%.4g" % v if k == "scale" else "%.3f" % v)
        self._updating = False

    def read_entries(self, _e=None):
        if self._updating:
            return
        try:
            new = {k: float(v.get().replace(",", ".")) for k, v in self.vars.items()}
        except ValueError:
            return  # niedokończony wpis — ignoruj
        if new["scale"] <= 0:
            return
        if new != self.view.t:
            self.view.t.update(new)
            self.view.redraw()

    def reset_transform(self):
        self.view.t.update(offset_x=0.0, offset_y=0.0, scale=1.0, rotation_deg=0.0)
        self.sync_entries()
        self.view.redraw()

    def toggle_heat(self):
        self.view.heatmap = self.heat_var.get()
        self.view.redraw()

    def toggle_wheels(self):
        self.view.show_wheels = self.wheel_var.get()
        self.view.redraw()

    def toggle_compensate(self):
        self.view.set_compensate(self.comp_var.get())

    # ---------- pliki ----------
    def load_robot_path(self, path):
        try:
            line_pts, axle_pts, speeds = file_io.parse_robot_file_full(path)
        except Exception as ex:
            messagebox.showerror("Błąd", str(ex))
            return
        self.last_robot_path = path
        self.last_dir = os.path.dirname(os.path.abspath(path))
        first = not self.view.robot
        self.view.set_robot_data(line_pts, axle_pts, speeds)
        # przy odświeżeniu zachowujemy transformację i kamerę; fit tylko za pierwszym razem
        if first:
            self.view.request_fit()
        else:
            self.view.redraw()

    def load_robot_dialog(self):
        path = filedialog.askopenfilename(initialdir=self.last_dir, title="Wczytaj trasę robota",
                                          filetypes=[("Pliki tras", "*.txt *.tsv *.csv"), ("Wszystkie", "*.*")])
        if path:
            self.load_robot_path(path)

    def refresh(self):
        if self.last_robot_path:
            self.load_robot_path(self.last_robot_path)  # ponownie z dysku, bez dialogu
        else:
            self.load_robot_dialog()

    def on_ref_done(self):
        self.view.request_fit()

    def save_project(self):
        path = filedialog.asksaveasfilename(initialdir=self.last_dir, defaultextension=".json",
                                            filetypes=[("Projekt JSON", "*.json")])
        if not path:
            return
        self.last_dir = os.path.dirname(os.path.abspath(path))
        try:
            file_io.save_project(path, self.view.robot, self.view.reference, self.view.t,
                                 self.view.speeds)
        except Exception as ex:
            messagebox.showerror("Błąd", str(ex))

    def load_project(self):
        path = filedialog.askopenfilename(initialdir=self.last_dir,
                                          filetypes=[("Projekt JSON", "*.json"), ("Wszystkie", "*.*")])
        if not path:
            return
        self.last_dir = os.path.dirname(os.path.abspath(path))
        try:
            robot, speeds, ref, t = file_io.load_project(path)
        except Exception as ex:
            messagebox.showerror("Błąd", str(ex))
            return
        self.view.set_robot_data(robot, [], speeds)
        self.view.reference = ref
        self.view.t.update(t)
        self.sync_entries()
        self.view.request_fit()

    def analyze(self):
        analyze_route(self.view.reference, self.view.transformed_robot())
        messagebox.showinfo("Analiza", "Funkcja analizy AI zostanie dodana później.")


def main():
    root = tk.Tk()
    app = App(root)
    if len(sys.argv) > 1:
        app.load_robot_path(sys.argv[1])
    root.mainloop()


if __name__ == "__main__":
    main()
