from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class MoveItScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "6. MoveIt2 Planning", T.MOVEIT)

        self.add_intro(
            "MoveIt2 is the ROS-standard motion-planning framework: drag an "
            "interactive marker on the TCP in RViz, click 'Plan & Execute', "
            "and MoveIt finds a collision-free joint trajectory automatically. "
            "Strong for cluttered workspaces and goal-based programming. "
            "Heavier than the other modes (separate window, takes time to "
            "launch). The dsr_moveit2 package needs a one-time colcon build.")

        big = tk.Label(self.body,
                       text="MoveIt2: motion planning with collision avoidance.",
                       bg=T.BG, fg=T.TITLE,
                       font=tkfont.Font(family="DejaVu Sans", size=12, weight="bold"))
        big.pack(anchor="w", pady=(0, 6))

        desc = tk.Label(self.body, bg=T.BG, fg=T.LABEL, justify="left", anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=10),
                        text=(
                            "MoveIt2 launches a separate RViz window with motion-planning panels.\n"
                            "Drag the orange interactive marker on the end-effector to set a goal,\n"
                            "then click 'Plan & Execute' in the panel. Path planning, collision\n"
                            "checking, and trajectory execution all happen automatically.\n"
                            "\n"
                            "Note: dsr_moveit2 is part of the upstream doosan-robot2 source but is\n"
                            "not built by default. The button below builds and launches it."))
        desc.pack(fill="x", pady=4)

        self.build_status = tk.Label(self.body, text="checking build status...",
                                     bg=T.BG, fg=T.DIM, anchor="w",
                                     font=tkfont.Font(family="DejaVu Sans", size=10))
        self.build_status.pack(fill="x", pady=(12, 4))

        btns = tk.Frame(self.body, bg=T.BG)
        btns.pack(pady=10)
        big_button(btns, "Build dsr_moveit2", self._build,
                   bg=T.PANEL_HI, fg=T.TITLE).pack(side="left", padx=4)
        big_button(btns, "Launch MoveIt2", self._launch, bg=T.MOVEIT).pack(side="left", padx=4)

        self.log = tk.Text(self.body, bg=T.PANEL, fg=T.VAL, height=12,
                           font=tkfont.Font(family="DejaVu Sans Mono", size=9),
                           relief="flat", insertbackground=T.VAL)
        self.log.pack(fill="both", expand=True, pady=8)

    def on_show(self):
        self._check_build()

    def _check_build(self):
        def fetch():
            r = subprocess.run(
                ["docker", "exec", "doosan_kos", "bash", "-c",
                 "[ -d /ros2_ws/install/dsr_moveit2 ] && echo built || echo missing"],
                capture_output=True, text=True, timeout=5)
            return r.stdout.strip()
        def update(s):
            if s == "built":
                self.build_status.config(text="✓ dsr_moveit2 already built", fg=T.OK)
            else:
                self.build_status.config(text="✗ dsr_moveit2 not built — click 'Build' first",
                                         fg=T.WARN)
        self.run_async(fetch, on_done=update)

    def _append_log(self, s):
        self.log.insert(tk.END, s)
        self.log.see(tk.END)

    def _build(self):
        self._append_log("\n[build] starting (5-15min) ...\n")
        self.set_status("building dsr_moveit2 ...", T.LABEL)

        def worker():
            proc = subprocess.Popen(
                ["docker", "exec", "doosan_kos", "bash", "-lc",
                 "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && "
                 "colcon build --packages-select dsr_moveit2 --cmake-args -DCMAKE_BUILD_TYPE=Release"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self.after(0, lambda l=line: self._append_log(l))
            rc = proc.wait()
            self.after(0, lambda: self.set_status(
                "build OK" if rc == 0 else "build FAILED",
                T.OK if rc == 0 else T.BAD))
            self.after(0, self._check_build)
        threading.Thread(target=worker, daemon=True).start()

    def _launch(self):
        self._append_log("\n[launch] starting MoveIt2 ...\n")
        self.set_status("launching MoveIt2 ...", T.LABEL)
        # check it's built first
        def fetch():
            r = subprocess.run(
                ["docker", "exec", "doosan_kos", "bash", "-c",
                 "ls /ros2_ws/install/dsr_moveit2/share/dsr_moveit2/launch/ 2>/dev/null"],
                capture_output=True, text=True, timeout=5)
            return r.stdout.strip().splitlines()
        def update(launch_files):
            if not launch_files:
                self._append_log("[launch] dsr_moveit2 not built. Click 'Build' first.\n")
                self.set_status("not built", T.BAD)
                return
            chosen = launch_files[0]
            self._append_log(f"[launch] using {chosen}\n")
            subprocess.Popen(
                ["docker", "exec", "-d", "doosan_kos", "bash", "-lc",
                 f"source /opt/ros/jazzy/setup.bash && "
                 f"source /ros2_ws/install/setup.bash && "
                 f"ros2 launch dsr_moveit2 {chosen} > /tmp/moveit.log 2>&1"])
            self._append_log("[launch] background process started. Check separate RViz window.\n")
            self._append_log("[launch] log: docker exec doosan_kos tail -f /tmp/moveit.log\n")
            self.set_status("launched (separate window)", T.OK)
        self.run_async(fetch, on_done=update)


# ──────── Gripper footer (Home + 모든 모드 화면 공통) ─────
