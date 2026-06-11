from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class WaypointScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "4. Waypoint Recorder", T.WP)
        self.waypoints = []  # list of dicts {posj: [...], posx: [...]}

        self.add_intro(
            "Teach-and-playback. Drive the robot to a pose using any other "
            "mode, then come here and click '+ Save Current' — the current "
            "joint angles are recorded. Build a sequence, reorder with ↑↓, "
            "then '▶ Play All' executes them in order via movej. Save the "
            "list to a JSON file to reuse later.")

        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))
        big_button(top, "+ Save Current", self._save_current, bg=T.WP).pack(side="left", padx=2)
        big_button(top, "↑ Move Up", self._move_up, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        big_button(top, "↓ Move Down", self._move_down, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        big_button(top, "✕ Delete", self._delete, bg=T.BAD, fg=T.BG,
                   height=1).pack(side="left", padx=2)
        big_button(top, "→ Go to Selected", self._goto, bg=T.PANEL_HI, fg=T.TITLE
                   ).pack(side="left", padx=10)
        big_button(top, "▶ Play All", self._play_all, bg=T.WP).pack(side="left", padx=2)

        files = tk.Frame(self.body, bg=T.BG)
        files.pack(fill="x", pady=4)
        big_button(files, "Save to file...", self._save_file, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        big_button(files, "Load from file...", self._load_file, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        tk.Label(files, text="(default location: /tmp/m1013_waypoints.json)",
                 bg=T.BG, fg=T.DIM, font=tkfont.Font(family="DejaVu Sans", size=8)
                 ).pack(side="left", padx=10)

        # listbox
        lstwrap = tk.Frame(self.body, bg=T.PANEL)
        lstwrap.pack(fill="both", expand=True, pady=8)
        scroll = tk.Scrollbar(lstwrap, bg=T.PANEL)
        scroll.pack(side="right", fill="y")
        self.listbox = tk.Listbox(lstwrap, bg=T.PANEL, fg=T.VAL,
                                  selectbackground=T.WP, selectforeground=T.BG,
                                  font=tkfont.Font(family="DejaVu Sans Mono", size=10),
                                  yscrollcommand=scroll.set, relief="flat",
                                  highlightthickness=0, activestyle="none")
        self.listbox.pack(fill="both", expand=True, padx=4, pady=4)
        scroll.config(command=self.listbox.yview)

        info = tk.Label(self.body, bg=T.BG, fg=T.DIM, anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=9),
                        text=("Each waypoint stores BOTH joint angles (posj) and TCP pose (posx).\n"
                              "Playback uses movej (joint-space). Use 'Go to Selected' for single jumps."))
        info.pack(fill="x")

    def on_show(self):
        # auto-load if file exists
        if not self.waypoints and WAYPOINTS_FILE.exists():
            try:
                with open(WAYPOINTS_FILE) as f:
                    data = json.load(f)
                self.waypoints = data.get("waypoints", [])
                self._refresh_list()
            except Exception:
                pass

    def _refresh_list(self):
        self.listbox.delete(0, tk.END)
        for i, wp in enumerate(self.waypoints):
            posj = [f"{x:+6.1f}" for x in wp["posj"]]
            self.listbox.insert(tk.END, f"WP{i+1:02d}  posj=[{', '.join(posj)}]")

    def _selected_index(self):
        sel = self.listbox.curselection()
        return sel[0] if sel else None

    def _save_current(self):
        def fetch():
            return self.robot.get_current_posj(), self.robot.get_current_posx(0)
        def update(result):
            posj, posx = result
            if not posj or not posx:
                self.set_status("could not capture pose", T.BAD)
                return
            self.waypoints.append({"posj": posj, "posx": posx})
            self._refresh_list()
            self.set_status(f"saved WP{len(self.waypoints)}", T.OK)
        self.run_async(fetch, on_done=update)

    def _move_up(self):
        i = self._selected_index()
        if i is None or i == 0: return
        self.waypoints[i-1], self.waypoints[i] = self.waypoints[i], self.waypoints[i-1]
        self._refresh_list()
        self.listbox.selection_set(i-1)

    def _move_down(self):
        i = self._selected_index()
        if i is None or i >= len(self.waypoints)-1: return
        self.waypoints[i], self.waypoints[i+1] = self.waypoints[i+1], self.waypoints[i]
        self._refresh_list()
        self.listbox.selection_set(i+1)

    def _delete(self):
        i = self._selected_index()
        if i is None: return
        del self.waypoints[i]
        self._refresh_list()
        self.set_status(f"deleted WP at index {i+1}", T.LABEL)

    def _goto(self):
        i = self._selected_index()
        if i is None:
            self.set_status("select a waypoint first", T.WARN)
            return
        wp = self.waypoints[i]
        self.set_status(f"movej → WP{i+1}", T.LABEL)
        self.run_async(
            lambda: self.robot.movej(wp["posj"], vel=30.0, acc=30.0,
                                     confirm_real_callback=self.app.confirm_real_modal),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def _play_all(self):
        if not self.waypoints:
            self.set_status("no waypoints to play", T.WARN); return
        wps = list(self.waypoints)
        self.set_status(f"playing {len(wps)} waypoints...", T.LABEL)
        def worker():
            for idx, wp in enumerate(wps):
                self.after(0, lambda i=idx: self.set_status(
                    f"playing WP{i+1}/{len(wps)}", T.LABEL))
                ok = self.robot.movej(wp["posj"], vel=30.0, acc=30.0, sync=0,
                                      confirm_real_callback=self.app.confirm_real_modal)
                if not ok:
                    self.after(0, lambda: self.set_status("FAILED mid-sequence", T.BAD))
                    return
            self.after(0, lambda: self.set_status("playback complete", T.OK))
        threading.Thread(target=worker, daemon=True).start()

    def _save_file(self):
        try:
            with open(WAYPOINTS_FILE, "w") as f:
                json.dump({"waypoints": self.waypoints}, f, indent=2)
            self.set_status(f"saved to {WAYPOINTS_FILE}", T.OK)
        except Exception as e:
            self.set_status(f"save error: {e}", T.BAD)

    def _load_file(self):
        try:
            with open(WAYPOINTS_FILE) as f:
                data = json.load(f)
            self.waypoints = data.get("waypoints", [])
            self._refresh_list()
            self.set_status(f"loaded {len(self.waypoints)} waypoints", T.OK)
        except Exception as e:
            self.set_status(f"load error: {e}", T.BAD)


# ──────── 5. Speed Control ─────────────────────────────────
