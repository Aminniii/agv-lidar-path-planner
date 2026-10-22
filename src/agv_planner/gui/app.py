import itertools
import math
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageTk
from scipy.ndimage import distance_transform_edt

from agv_planner.config.settings import load_settings
from agv_planner.core import (
    adaptive_reduce_path,
    angle_difference,
    approach,
    astar,
    clear_segment,
    compact_path,
    heuristic,
    inflate_free_grid,
    mecanum_body_velocity,
    mecanum_wheel_rates,
    mega_bridge_mix,
    optimized_search,
    search_path,
    simplify_path,
    smooth_corners,
    virtual_mega_wheel_targets,
)
from agv_planner.planning.proximity import (
    nearest_obstacle_distance,
    obstacle_distance_field,
    swept_footprint_is_free,
)
from agv_planner.planning.safety import (
    UnsafePathError,
    validate_footprint_clearance,
    validate_route,
)
from agv_planner.ros_support import load_ros_dependencies

ROS_AVAILABLE, ROS_ERROR, ros = load_ros_dependencies()
if ROS_AVAILABLE:
    rclpy, Node, ActionClient, FollowPath, GoalStatus, Buffer, TransformListener, TransformException, RosPath, PoseStamped, QoSProfile, DurabilityPolicy, ReliabilityPolicy = ros
else:
    rclpy = None
    TransformException = Exception


# ============================================================
# SETTINGS
# ============================================================

DEFAULT_MAP_YAML = (
    Path.home()
    / "lidar_slam_ws"
    / "maps"
    / "real_map.yaml"
)

# Clearance around obstacles.
# Later set this from the actual robot dimensions.
SETTINGS = load_settings()
ROBOT_RADIUS_M = SETTINGS.default_clearance_m
ROBOT_LENGTH_M = SETTINGS.footprint_length_m  # EDIT to measured full footprint
ROBOT_WIDTH_M = SETTINGS.footprint_width_m   # EDIT to measured full footprint
POSE_STALE_SECONDS = SETTINGS.pose_stale_seconds
TRAIL_MAX_POINTS = 3000
TRAIL_MIN_DISTANCE_M = 0.02


# ============================================================
# A*
# ============================================================





# ============================================================
# COLLISION-CHECKED PATH SIMPLIFICATION
# ============================================================




# ============================================================
# OBSTACLE INFLATION
# ============================================================

# ============================================================
# GUI
# ============================================================













# V7.2: Exact command mixing used by the uploaded Mega bridge.
# Units are Arduino command units, NOT calibrated RPM or wheel rad/s.















class PathPlannerGUI:

    def __init__(self, root):

        self.root = root

        self.root.title(
            "AGV LiDAR Path Planner · v9.2-rc1"
        )

        self.root.geometry("1400x900")
        self.root.option_add("*Font", "TkDefaultFont 10")
        style = ttk.Style(self.root)
        style.configure("TNotebook.Tab", padding=(13, 7))
        style.configure("TButton", padding=(7, 4))
        self.root.minsize(900, 580)

        # Map data
        self.yaml_path = None
        self.image_path = None

        self.original_map = None
        self.map_array = None

        self.map_width = 0
        self.map_height = 0

        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = 0.0

        self.negate = 0
        self.occupied_thresh = 0.65
        self.free_thresh = 0.25

        self.free_grid = None
        self.inflated_grid = None

        # Planner state
        self.start = None
        self.goal = None
        self.stops = []  # Up to five intermediate destinations.
        self.visit_order = []
        self.path = None

        self.mode = "start"
        self.auto_start = False
        self.robot_pose = None  # (world_x, world_y, yaw)
        self.last_tf_time = None
        self.trail = deque(maxlen=TRAIL_MAX_POINTS)
        self.show_trail = tk.BooleanVar(value=True)
        self.show_footprint = tk.BooleanVar(value=True)
        self.fullscreen = False
        self.mouse_position = tk.StringVar(value="Cursor: —")
        self.local_status = tk.StringVar(value="Localization: waiting for map → base_link")
        self._drag_origin = None
        self._dragged = False
        self._last_redraw_time = 0.0
        self.ros_node = None
        self.tf_buffer = None
        self.tf_listener = None
        self.nav2_path_publisher = None
        self.nav2_client = None
        self.nav2_goal_handle = None
        self.nav2_pending = False
        self.nav2_segments = []
        self.nav2_segment_index = 0
        self.nav2_dwell_job = None
        self.nav2_cancel_requested = False
        self.nav2_run_id = 0
        self.nav2_execution_enabled = tk.BooleanVar(value=False)
        self.nav2_status = tk.StringVar(value="Nav2: preview only — no motor commands")

        self.scale = 2.0
        self.map_offset_x = 0
        self.map_offset_y = 0
        self.clearance_var = tk.DoubleVar(value=ROBOT_RADIUS_M)
        self.algorithm_var = tk.StringVar(value="A*")
        self.route_mode = tk.StringVar(value="Shortest")
        self.obstacle_radius_var = tk.DoubleVar(value=0.20)
        self.dynamic_obstacles = []
        self.static_inflated_grid = None
        self.distance_field = None
        self.route_stats = tk.StringVar(value="Route: not planned")
        self._sim_replanning = False
        self.sim_speed_var = tk.DoubleVar(value=0.30)
        self.sim_accel_var = tk.DoubleVar(value=0.35)
        self.sim_yaw_rate_var = tk.DoubleVar(value=90.0)
        self.sim_telemetry = tk.StringVar(value="Speed: 0.00 m/s | Distance: 0.00 m | Time: 0.0 s")
        self.sim_velocity = 0.0
        self.wheel_rates = [0.0]*4
        self.sim_distance = 0.0
        self.sim_elapsed = 0.0
        self.sim_dwell_remaining = 0.0
        self.sim_pause_time = None
        self.sim_state = "stopped"
        self.sim_route = []
        self.sim_index = 0
        self.sim_progress = 0.0
        self.sim_pose = None
        self.sim_wait_until = 0.0
        self.sim_last_time = None
        self.sim_status = tk.StringVar(value="Simulation: stopped")
        self.sim_after_id = None
        # V6 controls; all wheel speeds are simulated, never sent to /cmd_vel.
        self.smooth_enabled = tk.BooleanVar(value=True)
        self.smooth_radius_var = tk.DoubleVar(value=0.12)
        self.detection_radius_var = tk.DoubleVar(value=0.45)
        self.show_detection = tk.BooleanVar(value=True)
        self.wheel_radius_var = tk.DoubleVar(value=SETTINGS.wheel_radius_m)  # configure in robot_config.yaml
        self.wheel_base_var = tk.DoubleVar(value=SETTINGS.wheelbase_m)    # front-to-back wheel-centre distance
        self.track_width_var = tk.DoubleVar(value=SETTINGS.track_width_m)  # left-to-right wheel-centre distance
        self.wheel_accel_var = tk.DoubleVar(value=20.0)  # rad/s²
        self.wheel_rates = [0.0]*4
        self.wheel_display = tk.StringVar(value="FL 0.0 | FR 0.0 | RL 0.0 | RR 0.0 rad/s")
        self.mega_mode = tk.BooleanVar(value=True)
        self.mega_commands = (0, 0, 0, 0)
        self.mega_diagnostics = tk.StringVar(value="Mega virtual: no commands (FL RL FR RR Arduino packet order)")
        self.proximity_display = tk.StringVar(value="Detection zone: inactive")
        self.raw_distance_m = None

        # v8.3: a fixed-width, scrollable sidebar and full-height map workspace.
        # All callbacks and state variables remain those of the original planner.
        self.root.configure(bg="#101827")
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("App.TFrame", background="#101827")
        style.configure("Side.TFrame", background="#172338")
        style.configure("Side.TLabel", background="#172338", foreground="#e5eefb", font=("DejaVu Sans", 10))
        style.configure("Heading.TLabel", background="#172338", foreground="#ffffff", font=("DejaVu Sans", 17, "bold"))
        style.configure("Hint.TLabel", background="#172338", foreground="#8fa6c4", font=("DejaVu Sans", 9))
        style.configure("Accent.TButton", padding=(9, 9), background="#2563eb", foreground="#ffffff", font=("DejaVu Sans", 10, "bold"))
        style.map("Accent.TButton", background=[("active", "#3979ff")])
        style.configure("Tool.TButton", padding=(7, 6))
        style.configure("Side.TCheckbutton", background="#172338", foreground="#e5eefb")
        style.configure("Side.TLabelframe", background="#172338", foreground="#e5eefb")
        style.configure("Side.TLabelframe.Label", background="#172338", foreground="#dbeafe", font=("DejaVu Sans", 10, "bold"))
        style.configure("TNotebook", background="#172338", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(7, 7), font=("DejaVu Sans", 9))

        workspace = ttk.Frame(root, style="App.TFrame")
        workspace.pack(fill=tk.BOTH, expand=True)
        sidebar = tk.Frame(workspace, bg="#172338", width=335)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)
        sidebar.pack_propagate(False)
        # Fixed sidebar: no independent vertical scrollbar.
        side_content = ttk.Frame(sidebar, style="Side.TFrame")
        side_content.pack(fill=tk.BOTH, expand=True)
        ttk.Label(side_content, text="AGV  /  PATH PLANNER", style="Heading.TLabel").pack(anchor="w", padx=16, pady=(20, 4))
        ttk.Label(side_content, text="v9.2-rc1  •  SLAM · PLANNING · SIMULATION", style="Hint.TLabel").pack(anchor="w", padx=16, pady=(0, 15))
        notebook = ttk.Notebook(side_content)
        notebook.pack(fill=tk.X, padx=9)
        tabs = {}
        for name, short in (("Map & Route", "Route"), ("Planning", "Plan"), ("Simulation", "Sim"), ("Robot & Wheels", "Wheels"), ("ROS 2 / Nav2", "ROS")):
            tab = ttk.Frame(notebook, style="Side.TFrame")
            notebook.add(tab, text=short)
            tabs[name] = tab
        controls_root, planning_tab, simulation_tab, robot_tab, ros_tab = (tabs[n] for n in ("Map & Route", "Planning", "Simulation", "Robot & Wheels", "ROS 2 / Nav2"))

        def heading(parent, title):
            ttk.Label(parent, text=title, style="Side.TLabel", font=("DejaVu Sans", 12, "bold")).pack(anchor="w", padx=9, pady=(14, 7))
        def btn(parent, text, callback, accent=False):
            return ttk.Button(parent, text=text, command=callback, style="Accent.TButton" if accent else "Tool.TButton")
        def pair(parent):
            frame = ttk.Frame(parent, style="Side.TFrame")
            frame.pack(fill=tk.X, padx=9, pady=3)
            frame.columnconfigure(0, weight=1)
            frame.columnconfigure(1, weight=1)
            return frame
        def setting(parent, label, variable, start, end, step):
            frame = ttk.Frame(parent, style="Side.TFrame")
            frame.pack(fill=tk.X, padx=9, pady=4)
            ttk.Label(frame, text=label, style="Side.TLabel").pack(side=tk.LEFT)
            ttk.Spinbox(frame, from_=start, to=end, increment=step, textvariable=variable, width=7).pack(side=tk.RIGHT)

        heading(controls_root, "MAP & ROUTE")
        btn(controls_root, "＋  Load occupancy map", self.choose_map).pack(fill=tk.X, padx=9, pady=4)
        r=pair(controls_root)
        btn(r,"Set start",self.set_start_mode).grid(row=0,column=0,sticky="ew",padx=(0,3))
        btn(r,"Set goal",self.set_goal_mode).grid(row=0,column=1,sticky="ew",padx=(3,0))
        btn(controls_root,"PLAN PATH  →",self.plan_path,True).pack(fill=tk.X,padx=9,pady=(8,5))
        r=pair(controls_root)
        btn(r,"Add stop",self.set_stop_mode).grid(row=0,column=0,sticky="ew",padx=(0,3))
        btn(r,"Undo stop",self.undo_stop).grid(row=0,column=1,sticky="ew",padx=(3,0))
        r=pair(controls_root)
        btn(r,"Clear route",self.clear).grid(row=0,column=0,sticky="ew",padx=(0,3))
        btn(r,"Export CSV",self.export_path).grid(row=0,column=1,sticky="ew",padx=(3,0))
        self.auto_button=btn(controls_root,"AUTO START: OFF",self.toggle_auto_start)
        self.auto_button.pack(fill=tk.X,padx=9,pady=4)
        ttk.Checkbutton(controls_root,text="Show robot footprint",variable=self.show_footprint,command=self.redraw,style="Side.TCheckbutton").pack(anchor="w",padx=12,pady=3)
        ttk.Checkbutton(controls_root,text="Show travelled trail",variable=self.show_trail,command=self.redraw,style="Side.TCheckbutton").pack(anchor="w",padx=12,pady=3)
        ttk.Label(controls_root,textvariable=self.local_status,style="Hint.TLabel",wraplength=285).pack(anchor="w",padx=12,pady=10)

        heading(planning_tab,"PATH PLANNING")
        ttk.Label(planning_tab,text="Algorithm",style="Side.TLabel").pack(anchor="w",padx=9)
        ttk.Combobox(planning_tab,textvariable=self.algorithm_var,values=("A*","Dijkstra","Theta*"),state="readonly").pack(fill=tk.X,padx=9,pady=4)
        ttk.Label(planning_tab,text="Route preference",style="Side.TLabel").pack(anchor="w",padx=9)
        ttk.Combobox(planning_tab,textvariable=self.route_mode,values=("Shortest","Balanced","Safest"),state="readonly").pack(fill=tk.X,padx=9,pady=4)
        setting(planning_tab,"Obstacle clearance (m)",self.clearance_var,0,1.5,.05)
        btn(planning_tab,"Apply clearance",self.apply_clearance).pack(fill=tk.X,padx=9,pady=4)
        heading(planning_tab,"VIRTUAL OBSTACLES")
        setting(planning_tab,"Obstacle radius (m)",self.obstacle_radius_var,.05,1,.05)
        btn(planning_tab,"Add obstacle on map",self.set_obstacle_mode).pack(fill=tk.X,padx=9,pady=4)
        r=pair(planning_tab)
        btn(r,"Undo",self.undo_obstacle).grid(row=0,column=0,sticky="ew",padx=(0,3))
        btn(r,"Clear all",self.clear_obstacles).grid(row=0,column=1,sticky="ew",padx=(3,0))
        ttk.Label(planning_tab,textvariable=self.route_stats,style="Hint.TLabel",wraplength=285).pack(anchor="w",padx=9,pady=10)

        heading(simulation_tab,"SIMULATION")
        r=pair(simulation_tab)
        btn(r,"▶ Start",self.start_simulation,True).grid(row=0,column=0,sticky="ew",padx=(0,3))
        btn(r,"Ⅱ Pause",self.pause_simulation).grid(row=0,column=1,sticky="ew",padx=(3,0))
        btn(simulation_tab,"■ Stop simulation",self.stop_simulation).pack(fill=tk.X,padx=9,pady=4)
        for args in (("Speed (m/s)",self.sim_speed_var,.05,2,.05),("Acceleration (m/s²)",self.sim_accel_var,.05,2,.05),("Turn rate (°/s)",self.sim_yaw_rate_var,15,360,15),("Smoothing radius (m)",self.smooth_radius_var,0,.5,.02),("Detection radius (m)",self.detection_radius_var,.1,2,.05)):
            setting(simulation_tab,*args)
        for label,var in (("Smooth corners",self.smooth_enabled),("Show detection zone",self.show_detection)):
            ttk.Checkbutton(simulation_tab,text=label,variable=var,command=self.redraw,style="Side.TCheckbutton").pack(anchor="w",padx=12,pady=3)
        for var in (self.sim_status,self.sim_telemetry,self.proximity_display):
            ttk.Label(simulation_tab,textvariable=var,style="Hint.TLabel",wraplength=285).pack(anchor="w",padx=9,pady=5)

        heading(robot_tab,"ROBOT GEOMETRY")
        for args in (("Wheel radius (m)",self.wheel_radius_var,.01,.3,.005),("Wheelbase (m)",self.wheel_base_var,.1,1.5,.01),("Track width (m)",self.track_width_var,.1,1.5,.01),("Wheel accel (rad/s²)",self.wheel_accel_var,1,100,1)):
            setting(robot_tab,*args)
        ttk.Checkbutton(robot_tab,text="Virtual Mega bridge mixing",variable=self.mega_mode,style="Side.TCheckbutton").pack(anchor="w",padx=9,pady=6)
        heading(robot_tab,"LIVE VIRTUAL WHEEL SPEEDS")
        self.wheel_indicators={}
        self.wheel_bars={}
        for name in ("FL","FR","RL","RR"):
            panel=ttk.Frame(robot_tab,style="Side.TFrame")
            panel.pack(fill=tk.X,padx=9,pady=5)
            var=tk.StringVar(value=f"{name}: 0.0 STOP")
            self.wheel_indicators[name]=var
            ttk.Label(panel,textvariable=var,style="Side.TLabel").pack(anchor="w")
            bar=ttk.Progressbar(panel,maximum=50)
            bar.pack(fill=tk.X,pady=3)
            self.wheel_bars[name]=bar
        def update_wheel_bars(*_):
            for i,key in enumerate(("FL","FR","RL","RR")):
                try: self.wheel_bars[key]["value"]=min(50,abs(float(self.wheel_rates[i])))
                except (IndexError,ValueError,tk.TclError): pass
        self.wheel_display.trace_add("write",update_wheel_bars)
        for var in (self.wheel_display,self.mega_diagnostics):
            ttk.Label(robot_tab,textvariable=var,style="Hint.TLabel",wraplength=285).pack(anchor="w",padx=9,pady=5)

        heading(ros_tab,"ROS 2 / NAV2")
        ttk.Label(ros_tab,text="Real-robot execution is disabled unless explicitly armed.",style="Hint.TLabel",wraplength=285).pack(anchor="w",padx=9,pady=6)
        btn(ros_tab,"Publish preview to RViz",self.publish_nav2_preview).pack(fill=tk.X,padx=9,pady=4)
        btn(ros_tab,"Export ROS Path YAML",self.export_nav2_path).pack(fill=tk.X,padx=9,pady=4)
        ttk.Checkbutton(ros_tab,text="ARM REAL ROBOT",variable=self.nav2_execution_enabled,style="Side.TCheckbutton").pack(anchor="w",padx=9,pady=12)
        btn(ros_tab,"SEND PATH TO NAV2",self.send_nav2_path).pack(fill=tk.X,padx=9,pady=4)
        btn(ros_tab,"CANCEL NAV2",self.cancel_nav2_path).pack(fill=tk.X,padx=9,pady=4)
        self.ros_detail=tk.StringVar(value="ROS 2: checking environment")
        for var in (self.nav2_status,self.ros_detail):
            ttk.Label(ros_tab,textvariable=var,style="Hint.TLabel",wraplength=285).pack(anchor="w",padx=9,pady=7)
        ttk.Label(side_content,text="OFFLINE PLANNING  •  NO MOTOR OUTPUT",style="Hint.TLabel").pack(anchor="w",padx=16,pady=20)

        # Map and status share the remaining width of the window.
        viewer=tk.Frame(workspace,bg="#101827")
        viewer.pack(side=tk.LEFT,fill=tk.BOTH,expand=True)
        toolbar=tk.Frame(viewer,bg="#101827")
        toolbar.pack(fill=tk.X,padx=15,pady=(12,4))
        tk.Label(toolbar,text="MAP WORKSPACE",bg="#101827",fg="#eef5ff",font=("DejaVu Sans",13,"bold")).pack(side=tk.LEFT)
        for text,callback in (("Fit",self.fit_map),("−",self.zoom_out),("+",self.zoom_in)):
            btn(toolbar,text,callback).pack(side=tk.RIGHT,padx=3)
        # ====================================================
        # STATUS
        # ====================================================

        self.status_var = tk.StringVar(
            value="Load a ROS map to begin."
        )

        ttk.Label(
            viewer,
            textvariable=self.status_var,
            font=("Arial", 12)
        ).pack(pady=5)

        self.info_var = tk.StringVar(
            value="No map loaded."
        )

        ttk.Label(
            viewer,
            textvariable=self.info_var
        ).pack(pady=3)

        # ====================================================
        # MAP CANVAS
        # ====================================================

        map_frame = ttk.Frame(viewer)

        map_frame.pack(
            fill=tk.BOTH,
            expand=True,
            padx=10,
            pady=10
        )

        self.canvas = tk.Canvas(
            map_frame,
            background="#263346"
        )

        horizontal_scroll = ttk.Scrollbar(
            map_frame,
            orient=tk.HORIZONTAL,
            command=self.canvas.xview
        )

        vertical_scroll = ttk.Scrollbar(
            map_frame,
            orient=tk.VERTICAL,
            command=self.canvas.yview
        )

        self.canvas.configure(
            xscrollcommand=horizontal_scroll.set,
            yscrollcommand=vertical_scroll.set
        )

        self.canvas.grid(
            row=0,
            column=0,
            sticky="nsew"
        )

        vertical_scroll.grid(
            row=0,
            column=1,
            sticky="ns"
        )

        horizontal_scroll.grid(
            row=1,
            column=0,
            sticky="ew"
        )

        map_frame.rowconfigure(
            0,
            weight=1
        )

        map_frame.columnconfigure(
            0,
            weight=1
        )

        self.canvas.bind(
            "<Button-1>",
            self.map_clicked
        )

        self.canvas.bind(
            "<MouseWheel>",
            self.mouse_wheel
        )

        # Right mouse button pans; left mouse button still places waypoints.
        self.canvas.bind("<ButtonPress-3>", self.begin_pan)
        self.canvas.bind("<B3-Motion>", self.drag_pan)
        self.canvas.bind("<ButtonRelease-3>", self.end_pan)
        self.canvas.bind("<Motion>", self.update_mouse_coordinates)
        self.canvas.bind("<Leave>", lambda event: self.mouse_position.set("Cursor: —"))
        self._fit_resize_job = None
        self._fit_on_resize = True
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.canvas.bind("<Button-4>", lambda event: self.zoom_at(event, 1.25))
        self.canvas.bind("<Button-5>", lambda event: self.zoom_at(event, 0.8))
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Escape>", self.leave_fullscreen)
        self.root.bind("<Control-plus>", lambda event: self.zoom_center(1.25))
        self.root.bind("<Control-minus>", lambda event: self.zoom_center(0.8))
        self.root.bind("<Control-0>", lambda event: self.fit_map())
        self.root.bind("<Control-z>", lambda event: self.undo_stop())
        self.root.bind("<Control-p>", lambda event: self.plan_path())
        self.root.bind("<Control-e>", lambda event: self.export_path())
        self.root.bind("<Control-l>", lambda event: self.choose_map())
        self.root.bind("<Control-g>", lambda event: self.set_goal_mode())
        self.root.bind("<Control-s>", lambda event: self.set_start_mode())
        self.root.bind("<Control-Shift-C>", lambda event: self.clear_trail())

        self.pose_var = tk.StringVar(value="Live robot: waiting for ROS TF...")
        ttk.Label(root, textvariable=self.pose_var).pack(side=tk.BOTTOM, pady=2)

        # The Tk event loop owns ROS polling: no background UI thread.
        try:
            if not ROS_AVAILABLE:
                raise RuntimeError(f"ROS 2 not installed ({ROS_ERROR}). Offline mode is available.")
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = Node("agv_path_planner_gui")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.ros_node)
            self.nav2_client = ActionClient(self.ros_node, FollowPath, "/follow_path")
            preview_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.nav2_path_publisher = self.ros_node.create_publisher(
                RosPath, "/planner_preview_path", preview_qos)
        except Exception as exc:
            self.ros_detail.set(f"ROS 2 integration unavailable: {exc}")
            self.pose_var.set("Mode: OFFLINE | Local planning and simulation available")

        if ROS_AVAILABLE and self.ros_node is not None:
            self.ros_detail.set("ROS 2: connected. Check localization and Nav2 before arming real robot.")
        if not ROS_AVAILABLE:
            self.nav2_status.set("Nav2: OFFLINE — export available; execution disabled")
            self.local_status.set("Localization: offline mode (no ROS 2)")
            self.nav2_execution_enabled.set(False)
        self.root.after(200, self.poll_robot_pose)
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)

        # Automatically load real_map.yaml if available.
        if DEFAULT_MAP_YAML.exists():

            try:
                self.load_map(DEFAULT_MAP_YAML)

            except Exception as error:

                messagebox.showwarning(
                    "Map load error",
                    str(error)
                )


    def close_window(self):
        if self.nav2_segments or self.nav2_pending or self.nav2_goal_handle:
            messagebox.showwarning("Nav2", "Cancel the active route and wait for confirmation before closing.")
            return
        self.stop_simulation(redraw=False)
        if self.ros_node is not None:
            self.ros_node.destroy_node()
        self.root.destroy()
        if ROS_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def world_to_pixel(self, wx, wy):
        if self.resolution is None:
            return None
        dx, dy = wx - self.origin_x, wy - self.origin_y
        c, si = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = c * dx + si * dy
        local_y = -si * dx + c * dy
        px = int(round(local_x / self.resolution))
        py = self.map_height - 1 - int(round(local_y / self.resolution))
        if 0 <= px < self.map_width and 0 <= py < self.map_height:
            return (px, py)
        return None

    def toggle_auto_start(self):
        self.auto_start = not self.auto_start
        self.auto_button.configure(
            text=f"AUTO START: {'ON' if self.auto_start else 'OFF'}"
        )
        if self.auto_start:
            self.mode = "goal"
            self.sync_start_to_robot()
        self.redraw()

    def sync_start_to_robot(self):
        if not self.auto_start or self.robot_pose is None or self.inflated_grid is None:
            return False
        pos = self.world_to_pixel(*self.robot_pose[:2])
        if pos is None:
            self.status_var.set("Robot is outside saved map; cannot set START.")
            return False
        if not self.inflated_grid[pos[1], pos[0]]:
            self.status_var.set("Robot is in unknown/blocked/clearance area on saved map.")
            return False
        if pos != self.start:
            self.start = pos
            self.path = None  # old path is no longer from current position
            self.visit_order = []
            return True
        return False

    def poll_robot_pose(self):
        try:
            if self.ros_node is not None:
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "map", "base_link", rclpy.time.Time()
                    )
                    t, q = tf.transform.translation, tf.transform.rotation
                    yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                     1-2*(q.y*q.y + q.z*q.z))
                    new_pose = (t.x, t.y, yaw)
                    moved = (self.robot_pose is None or
                             math.hypot(new_pose[0]-self.robot_pose[0],
                                        new_pose[1]-self.robot_pose[1]) > 0.005 or
                             abs(math.atan2(math.sin(yaw-self.robot_pose[2]),
                                            math.cos(yaw-self.robot_pose[2]))) > 0.01)
                    previous_pose = self.robot_pose
                    self.robot_pose = new_pose
                    self.last_tf_time = time.monotonic()
                    self.local_status.set("Localization: TF available (map → base_link)")
                    if not self.trail or math.hypot(t.x - self.trail[-1][0], t.y - self.trail[-1][1]) >= TRAIL_MIN_DISTANCE_M:
                        self.trail.append((t.x, t.y))
                    self.pose_var.set(
                        f"Live robot: x={t.x:.3f} m  y={t.y:.3f} m  "
                        f"heading={math.degrees(yaw):.1f}°"
                    )
                    if moved:
                        self.sync_start_to_robot()
                        self.redraw()
                except TransformException:
                    self.local_status.set("Localization: TF unavailable")
                    self.pose_var.set("Live robot: waiting for map → base_link TF...")
        except Exception as exc:
            self.local_status.set("Localization: ROS error")
            self.pose_var.set(f"ROS polling error: {exc}")
        finally:
            if self.last_tf_time is not None and time.monotonic() - self.last_tf_time > POSE_STALE_SECONDS:
                self.local_status.set("Localization: STALE — check TF / ROS time")
            self.root.after(200, self.poll_robot_pose)

    # ========================================================
    # MAP LOADING
    # ========================================================

    def choose_map(self):

        filename = filedialog.askopenfilename(
            title="Select ROS Map YAML",
            initialdir=str(
                Path.home()
                / "lidar_slam_ws"
                / "maps"
            ),
            filetypes=[
                (
                    "ROS map YAML",
                    "*.yaml *.yml"
                ),
                (
                    "All files",
                    "*"
                )
            ]
        )

        if not filename:
            return

        try:

            self.load_map(
                Path(filename)
            )

        except Exception as error:

            messagebox.showerror(
                "Could not load map",
                str(error)
            )


    def load_map(self, yaml_path):
        self.stop_simulation(redraw=False)

        yaml_path = Path(yaml_path)

        if not yaml_path.exists():

            raise FileNotFoundError(
                f"YAML file not found:\n{yaml_path}"
            )

        with open(
            yaml_path,
            "r",
            encoding="utf-8"
        ) as file:

            metadata = yaml.safe_load(file)

        required = [
            "image",
            "resolution",
            "origin"
        ]

        for key in required:

            if key not in metadata:

                raise ValueError(
                    f"Map YAML is missing '{key}'."
                )

        # ----------------------------------------------------
        # Image path
        # ----------------------------------------------------

        image_path = Path(
            metadata["image"]
        )

        if not image_path.is_absolute():

            image_path = (
                yaml_path.parent
                / image_path
            )

        image_path = image_path.resolve()

        if not image_path.exists():

            raise FileNotFoundError(
                f"Map image not found:\n{image_path}"
            )

        # ----------------------------------------------------
        # Metadata
        # ----------------------------------------------------

        self.yaml_path = yaml_path.resolve()
        self.image_path = image_path

        self.resolution = float(
            metadata["resolution"]
        )

        origin = metadata["origin"]

        if len(origin) < 3:

            raise ValueError(
                "Map origin must contain [x, y, yaw]."
            )

        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])

        self.negate = int(
            metadata.get(
                "negate",
                0
            )
        )

        self.occupied_thresh = float(
            metadata.get(
                "occupied_thresh",
                0.65
            )
        )

        self.free_thresh = float(
            metadata.get(
                "free_thresh",
                0.25
            )
        )

        # ----------------------------------------------------
        # Load PGM
        # ----------------------------------------------------

        self.trail.clear()
        self.robot_pose = None
        self.last_tf_time = None
        self.original_map = Image.open(
            self.image_path
        ).convert("L")

        self.map_array = np.array(
            self.original_map
        )

        (
            self.map_width,
            self.map_height
        ) = self.original_map.size

        # ----------------------------------------------------
        # Convert ROS map image values to occupancy
        # ----------------------------------------------------

        image_values = (
            self.map_array.astype(np.float32)
            / 255.0
        )

        if self.negate:

            occupancy_probability = (
                image_values
            )

        else:

            occupancy_probability = (
                1.0 - image_values
            )

        # ROS trinary map:
        #
        # occupancy < free_thresh
        #     -> FREE
        #
        # occupancy > occupied_thresh
        #     -> OCCUPIED
        #
        # everything else
        #     -> UNKNOWN
        #
        # Unknown is deliberately NOT traversable.

        self.free_grid = (
            occupancy_probability
            < self.free_thresh
        )

        # ----------------------------------------------------
        # Inflate obstacles
        # ----------------------------------------------------

        radius_pixels = math.ceil(
            self.clearance_var.get()
            / self.resolution
        )

        self.raw_distance_m = obstacle_distance_field(self.free_grid, self.resolution)
        self.static_inflated_grid = inflate_free_grid(self.free_grid, radius_pixels)
        self.dynamic_obstacles.clear()
        self.refresh_dynamic_grid()

        # ----------------------------------------------------
        # Reset planner
        # ----------------------------------------------------

        self.start = None
        self.goal = None
        self.stops = []  # Up to five intermediate destinations.
        self.visit_order = []
        self.path = None

        self.mode = "goal" if self.auto_start else "start"
        self.sync_start_to_robot()

        # Fit after Tk has calculated the canvas dimensions.
        self._fit_on_resize = True
        self.root.after_idle(self.fit_map)

        map_width_m = (
            self.map_width
            * self.resolution
        )

        map_height_m = (
            self.map_height
            * self.resolution
        )

        self.info_var.set(
            f"{self.yaml_path.name}   |   "
            f"{self.map_width} × "
            f"{self.map_height} px   |   "
            f"{self.resolution:.3f} m/px   |   "
            f"{map_width_m:.2f} × "
            f"{map_height_m:.2f} m   |   "
            f"Clearance: "
            f"{self.clearance_var.get():.2f} m"
        )

        self.status_var.set(
            "Map loaded. "
            "Click a free area to set START."
        )

        self.redraw()


    # ========================================================
    # COORDINATE CONVERSION
    # ========================================================

    def pixel_to_world(self, px, py):

        # Convert image coordinates (top-left origin)
        # to ROS map-cell coordinates (bottom-left origin).

        map_x = (
            px * self.resolution
        )

        map_y = (
            (
                self.map_height
                - 1
                - py
            )
            * self.resolution
        )

        # Apply map origin yaw as well.
        cos_yaw = math.cos(
            self.origin_yaw
        )

        sin_yaw = math.sin(
            self.origin_yaw
        )

        world_x = (
            self.origin_x
            + cos_yaw * map_x
            - sin_yaw * map_y
        )

        world_y = (
            self.origin_y
            + sin_yaw * map_x
            + cos_yaw * map_y
        )

        return world_x, world_y


    # ========================================================
    # MODES
    # ========================================================

    def set_start_mode(self):

        if self.original_map is None:
            return

        self.auto_start = False
        self.auto_button.configure(text="AUTO START: OFF")
        self.mode = "start"

        self.status_var.set(
            "START mode: click a free location."
        )


    def set_stop_mode(self):
        if self.original_map is None:
            return
        if len(self.stops) >= 5:
            messagebox.showinfo("Stop limit", "Maximum five intermediate stops.")
            return
        self.mode = "stop"
        self.status_var.set(f"Click intermediate stop {len(self.stops) + 1}/5.")

    def undo_stop(self):
        if self.stops:
            self.stops.pop()
            self.path = None
            self.visit_order = []
            self.status_var.set(f"Removed last stop. {len(self.stops)}/5 remain.")
            self.redraw()

    def set_goal_mode(self):

        if self.original_map is None:
            return

        self.mode = "goal"

        self.status_var.set(
            "GOAL mode: click a free location."
        )


    # ========================================================
    # CLICK HANDLING
    # ========================================================

    def map_clicked(self, event):

        if self.original_map is None:
            return

        canvas_x = self.canvas.canvasx(
            event.x
        )

        canvas_y = self.canvas.canvasy(
            event.y
        )

        px = int(
            (canvas_x - self.map_offset_x) / self.scale
        )

        py = int(
            (canvas_y - self.map_offset_y) / self.scale
        )

        if not (
            0 <= px < self.map_width
            and
            0 <= py < self.map_height
        ):
            return

        if self.mode == "obstacle":
            try:
                radius = float(self.obstacle_radius_var.get())
                if not 0.05 <= radius <= 1.0:
                    raise ValueError()
            except (ValueError, tk.TclError):
                messagebox.showerror("Obstacle", "Radius must be 0.05–1.0 m")
                return
            self.dynamic_obstacles.append((px, py, radius))
            self.refresh_dynamic_grid()
            self.handle_obstacle_change()
            return

        if not self.inflated_grid[
            py,
            px
        ]:

            self.status_var.set(
                "That location is unknown, occupied, "
                "or too close to an obstacle."
            )

            return

        if self.mode == "start":

            self.start = (
                px,
                py
            )

            self.path = None
            self.visit_order = []

            wx, wy = (
                self.pixel_to_world(
                    px,
                    py
                )
            )

            self.status_var.set(
                f"START = "
                f"({wx:.2f}, {wy:.2f}) m. "
                f"Now select GOAL."
            )

            self.mode = "goal"

        elif self.mode == "stop":
            if len(self.stops) >= 5:
                messagebox.showinfo("Stop limit", "Maximum five intermediate stops.")
                return
            candidate = (px, py)
            if candidate == self.start or candidate == self.goal or candidate in self.stops:
                self.status_var.set("Choose a different free cell for this stop.")
                return
            self.stops.append(candidate)
            self.path = None
            self.visit_order = []
            self.status_var.set(f"Added stop {len(self.stops)}/5. Add more or select GOAL.")
            self.redraw()
            return

        else:

            self.goal = (
                px,
                py
            )

            self.path = None

            wx, wy = (
                self.pixel_to_world(
                    px,
                    py
                )
            )

            self.status_var.set(
                f"GOAL = "
                f"({wx:.2f}, {wy:.2f}) m. "
                f"Press PLAN PATH."
            )

        if self.auto_start and self.goal is not None:
            self.sync_start_to_robot()
            self.plan_path()
        else:
            self.redraw()


    # ========================================================
    # PATH PLANNING
    # ========================================================

    def plan_path(self):
        if self.nav2_segments or self.nav2_pending or self.nav2_goal_handle:
            messagebox.showwarning("Nav2", "Cancel the active real-robot route before replanning.")
            return
        self.stop_simulation(redraw=False)

        if self.original_map is None:

            messagebox.showwarning(
                "No map",
                "Load a map first."
            )

            return

        if self.auto_start:
            self.sync_start_to_robot()

        if self.start is None:

            messagebox.showwarning(
                "No start",
                "Set START first."
            )

            return

        if self.goal is None:

            messagebox.showwarning(
                "No goal",
                "Set GOAL first."
            )

            return

        if not self.inflated_grid[self.start[1], self.start[0]] or not self.inflated_grid[self.goal[1], self.goal[0]]:
            self.path = None
            self.status_var.set("Start or goal blocked by obstacle/clearance; move it first.")
            self.redraw()
            return
        if self.algorithm_var.get() == "Theta*" and self.route_mode.get() != "Shortest":
            self.status_var.set("Theta* currently uses shortest-path mode; choose A* for clearance preference.")
        self.status_var.set(
            "Planning..."
        )

        self.root.update_idletasks()

        # Pairwise A* routes provide obstacle-aware distances for the
        # open-path traveling-salesman problem (fixed START and final GOAL).
        nodes = [self.start] + self.stops + [self.goal]
        n = len(nodes)
        edges = {}
        for i in range(n):
            for j in range(i + 1, n):
                segment = optimized_search(self.inflated_grid, nodes[i], nodes[j],
                                          self.algorithm_var.get(), self.distance_field,
                                          {"Shortest": 0.0, "Balanced": 1.0, "Safest": 4.0}[self.route_mode.get()])
                if segment is None:
                    self.path = None
                    self.visit_order = []
                    self.status_var.set(f"NO PATH: locations {i} and {j} disconnected.")
                    self.redraw()
                    return
                distance = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                               for a, b in zip(segment, segment[1:]))
                edges[i, j] = (distance, segment)
                edges[j, i] = (distance, list(reversed(segment)))

        stop_indices = range(1, n - 1)
        best_order = min(
            itertools.permutations(stop_indices),
            key=lambda order: sum(edges[a, b][0] for a, b in zip(
                (0,) + order, order + (n - 1,)))
        )
        ordered_nodes = (0,) + best_order + (n - 1,)
        raw_path = []
        self.visit_order = [nodes[i] for i in best_order]
        for a, b in zip(ordered_nodes, ordered_nodes[1:]):
            segment = edges[a, b][1]
            # Simplify separately so required stops are never skipped.
            smoothed = (segment if self.algorithm_var.get() == "Theta*" or self.route_mode.get() != "Shortest"
                        else simplify_path(self.inflated_grid, segment))
            raw_path.extend(smoothed if not raw_path else smoothed[1:])

        raw_cell_count = sum(len(edges[a, b][1]) for a, b in zip(
            ordered_nodes, ordered_nodes[1:]))
        # Simplify dense grid routes before corner smoothing; preserve stops.
        # This avoids thousands of redundant curve samples on Balanced/Safest.
        raw_path = compact_path(raw_path, self.inflated_grid,
                                required=self.stops, max_segment_px=35)
        if self.smooth_enabled.get():
            try:
                radius_px = max(0, float(self.smooth_radius_var.get())) / self.resolution
            except (ValueError, tk.TclError):
                radius_px = 0
            raw_path = smooth_corners(raw_path, self.inflated_grid, radius_px,
                                      required=self.stops)
        # V7.1: simplify dense smoothing samples without erasing important
        # curvature, cutting across inflated obstacles, or dropping stops.
        raw_path = adaptive_reduce_path(raw_path, self.inflated_grid,
                                        required=self.stops,
                                        tolerance_px=max(0.5, 0.015 / self.resolution),
                                        max_spacing_px=max(2.0, 0.28 / self.resolution))
        try:
            validate_route(raw_path, self.inflated_grid, start=self.start,
                           goal=self.goal, required=self.stops)
        except UnsafePathError as exc:
            self.path = None
            self.visit_order = []
            self.status_var.set(f"UNSAFE PATH rejected: {exc}")
            self.redraw()
            return
        self.path = raw_path
        self.update_route_stats()
        path = self.path

        length_pixels = 0.0

        for index in range(
            1,
            len(path)
        ):

            x1, y1 = path[index - 1]
            x2, y2 = path[index]

            length_pixels += math.hypot(
                x2 - x1,
                y2 - y1
            )

        length_metres = (
            length_pixels
            * self.resolution
        )

        start_world = (
            self.pixel_to_world(
                *self.start
            )
        )

        goal_world = (
            self.pixel_to_world(
                *self.goal
            )
        )

        self.status_var.set(
            f"PATH FOUND   |   "
            f"{length_metres:.2f} m   |   "
            f"{len(path)} waypoints (from {raw_cell_count} cells)   |   "
            f"{len(self.stops)} stops: "
            f"{[self.stops.index(pt)+1 for pt in self.visit_order]}   |   "
            f"Start "
            f"({start_world[0]:.2f}, "
            f"{start_world[1]:.2f})   →   "
            f"Goal "
            f"({goal_world[0]:.2f}, "
            f"{goal_world[1]:.2f})"
        )

        self.redraw()


    # ========================================================
    # EXPORT A* PATH FOR DRY RUN
    # ========================================================

    def validate_current_route(self, *, for_robot=False):
        if self.path is None or self.inflated_grid is None:
            raise UnsafePathError("Plan a route before export or execution")
        validate_route(self.path, self.inflated_grid, start=self.start,
                       goal=self.goal, required=self.stops)
        if for_robot:
            validate_footprint_clearance(self.clearance_var.get(),
                                         ROBOT_LENGTH_M, ROBOT_WIDTH_M)
        return True

    def build_nav2_path(self):
        self.validate_current_route(for_robot=True)
        """Create map-frame ROS Path with one constant chassis heading."""
        if not self.path or self.resolution is None:
            raise ValueError("Plan a path before publishing or exporting.")
        if self.dynamic_obstacles:
            raise ValueError("Virtual obstacles are simulation-only. Remove them before ROS preview.")
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS 2 is unavailable. Use Export ROS Path YAML for offline export.")
        msg = RosPath()
        msg.header.frame_id = "map"
        if self.ros_node is not None:
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        points = list(self.path)
        # Fixed yaw for all path poses: the chassis should translate/strafe,
        # never turn to face the direction of travel.
        # For an offline preview without TF, use map yaw zero.
        yaw = (self.robot_pose[2] if self._nav2_pose_fresh() else 0.0)
        for i, (px, py) in enumerate(points):
            x, y = self.pixel_to_world(px, py)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        return msg

    def publish_nav2_preview(self):
        """RViz-only topic. This does NOT send a Nav2 action or /cmd_vel."""
        try:
            if self.nav2_path_publisher is None:
                raise RuntimeError("ROS 2 is unavailable; source your ROS setup first.")
            msg = self.build_nav2_path()
            self.nav2_path_publisher.publish(msg)
            self.nav2_status.set(f"Nav2: published {len(msg.poses)} preview poses to /planner_preview_path (NO MOTION)")
        except Exception as exc:
            messagebox.showwarning("Nav2 preview", str(exc))

    def _build_nav2_segments(self, full_path):
        """Split the planned route at required stops in optimized visit order."""
        pixels = list(self.path)
        boundaries = [0]
        for stop in self.visit_order:
            # The route planner preserves required stops through smoothing.
            matches = [i for i in range(boundaries[-1] + 1, len(pixels) - 1)
                       if math.hypot(pixels[i][0] - stop[0],
                                     pixels[i][1] - stop[1]) <= 1.5]
            if not matches:
                raise ValueError("A required stop is missing from the planned path. Re-plan the route.")
            boundaries.append(matches[0])
        boundaries.append(len(pixels) - 1)
        segments = []
        for a, b in zip(boundaries, boundaries[1:]):
            if b <= a or b >= len(full_path.poses):
                raise ValueError("Invalid Nav2 segment or incomplete ROS path; re-plan the route.")
            segment = RosPath()
            segment.header = full_path.header
            segment.poses = list(full_path.poses[a:b + 1])
            segments.append(segment)
        return segments

    def _nav2_pose_fresh(self):
        return (self.robot_pose is not None and self.last_tf_time is not None
                and time.monotonic() - self.last_tf_time <= POSE_STALE_SECONDS)

    def send_nav2_path(self):
        """Execute one FollowPath per leg, with a 2-second dwell at each stop."""
        if self.nav2_pending or self.nav2_goal_handle is not None or self.nav2_segments:
            messagebox.showwarning("Nav2", "A navigation route is already active or pending.")
            return
        if not self.nav2_execution_enabled.get():
            messagebox.showwarning("Nav2", "Check ARM REAL ROBOT first.")
            return
        if not self.auto_start:
            messagebox.showwarning("Nav2", "Enable AUTO START and PLAN PATH again.")
            return
        if not self._nav2_pose_fresh():
            messagebox.showwarning("Nav2", "Live map → base_link pose is missing or stale.")
            return
        if self.nav2_client is None or not self.nav2_client.server_is_ready():
            messagebox.showwarning("Nav2", "Nav2 /follow_path action server is not available.")
            return
        try:
            path = self.build_nav2_path()
            segments = self._build_nav2_segments(path)
            sx, sy = path.poses[0].pose.position.x, path.poses[0].pose.position.y
            if math.hypot(sx-self.robot_pose[0], sy-self.robot_pose[1]) > 0.15:
                raise ValueError("Planned start differs from live pose by over 15 cm. Re-plan with AUTO START.")
        except Exception as exc:
            messagebox.showwarning("Nav2", str(exc))
            return
        count = len(segments) - 1
        if not messagebox.askyesno(
            "REAL ROBOT MOVEMENT",
            f"Send {len(segments)} FollowPath legs with {count} intermediate stops "
            f"(2 seconds each)?\n\nConfirm the area is clear, motor cutoff is ready, "
            f"localization is correct, and collision monitoring has been tested."):
            return
        self.nav2_run_id += 1
        self.nav2_cancel_requested = False
        self.nav2_segments = segments
        self.nav2_segment_index = 0
        self._send_nav2_leg(self.nav2_run_id)

    def _send_nav2_leg(self, run_id):
        if run_id != self.nav2_run_id or self.nav2_cancel_requested:
            return
        if not self._nav2_pose_fresh():
            self._finish_nav2_route("Nav2: stopped — localization stale")
            return
        if self.nav2_client is None or not self.nav2_client.server_is_ready():
            self._finish_nav2_route("Nav2: stopped — FollowPath server unavailable")
            return
        path = self.nav2_segments[self.nav2_segment_index]
        start = path.poses[0].pose.position
        if math.hypot(start.x - self.robot_pose[0], start.y - self.robot_pose[1]) > 0.25:
            self._finish_nav2_route("Nav2: stopped — robot too far from next leg start")
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = ""
        self.nav2_pending = True
        self.nav2_status.set(f"Nav2: sending translation-only leg {self.nav2_segment_index + 1}/{len(self.nav2_segments)}")
        try:
            future = self.nav2_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.root.after(0, self._nav2_goal_response, f, run_id))
        except Exception as exc:
            self._finish_nav2_route(f"Nav2: send failed — {exc}")

    def _nav2_goal_response(self, future, run_id):
        if run_id != self.nav2_run_id:
            return
        try:
            handle = future.result()
            self.nav2_pending = False
            if not handle or not handle.accepted:
                self._finish_nav2_route("Nav2: leg rejected — route stopped")
                return
            self.nav2_goal_handle = handle
            self.nav2_status.set(f"Nav2: driving leg {self.nav2_segment_index + 1}/{len(self.nav2_segments)}")
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda f: self.root.after(0, self._nav2_result, f, run_id))
            if self.nav2_cancel_requested:
                handle.cancel_goal_async()
                self.nav2_status.set("Nav2: cancelling active leg...")
        except Exception as exc:
            self._finish_nav2_route(f"Nav2: goal error — {exc}")

    def _nav2_result(self, future, run_id):
        if run_id != self.nav2_run_id:
            return
        self.nav2_goal_handle = None
        self.nav2_pending = False
        try:
            result = future.result()
            if self.nav2_cancel_requested:
                self._finish_nav2_route("Nav2: route cancelled")
                return
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self._finish_nav2_route(f"Nav2: leg failed (status {result.status}); route stopped")
                return
            # Never proceed merely because the action completed: verify actual pose.
            if not self._nav2_pose_fresh():
                self._finish_nav2_route("Nav2: stopped — localization stale after leg")
                return
            endpoint = self.nav2_segments[self.nav2_segment_index].poses[-1].pose.position
            if math.hypot(endpoint.x - self.robot_pose[0], endpoint.y - self.robot_pose[1]) > 0.20:
                self._finish_nav2_route("Nav2: stopped — target not reached within 20 cm")
                return
            self.nav2_segment_index += 1
            if self.nav2_segment_index >= len(self.nav2_segments):
                self._finish_nav2_route("Nav2: route completed successfully")
                return
            self.nav2_status.set(f"Nav2: stop {self.nav2_segment_index}/{len(self.nav2_segments)-1} — dwell 2 s")
            self.nav2_dwell_job = self.root.after(2000, self._nav2_dwell_complete, run_id)
        except Exception as exc:
            self._finish_nav2_route(f"Nav2: result error — {exc}")

    def _nav2_dwell_complete(self, run_id):
        self.nav2_dwell_job = None
        if run_id == self.nav2_run_id and not self.nav2_cancel_requested:
            self._send_nav2_leg(run_id)

    def _finish_nav2_route(self, status):
        if self.nav2_dwell_job is not None:
            self.root.after_cancel(self.nav2_dwell_job)
            self.nav2_dwell_job = None
        self.nav2_segments = []
        self.nav2_segment_index = 0
        self.nav2_pending = False
        self.nav2_goal_handle = None
        self.nav2_cancel_requested = False
        self.nav2_execution_enabled.set(False)
        self.nav2_status.set(status)

    def cancel_nav2_path(self):
        if not (self.nav2_segments or self.nav2_pending or self.nav2_goal_handle):
            self.nav2_status.set("Nav2: no active route")
            return
        self.nav2_cancel_requested = True
        if self.nav2_dwell_job is not None:
            self.root.after_cancel(self.nav2_dwell_job)
            self.nav2_dwell_job = None
            self._finish_nav2_route("Nav2: route cancelled during dwell")
        elif self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()
            self.nav2_status.set("Nav2: cancellation requested — awaiting Nav2 result")
        else:
            self.nav2_status.set("Nav2: cancellation queued until goal accepted")

    def export_nav2_path(self):
        """Portable preview YAML, not a direct Nav2 action goal."""
        try:
            if not self.path or self.resolution is None:
                raise ValueError("Plan a path before exporting.")
            self.validate_current_route(for_robot=True)
            if self.dynamic_obstacles:
                raise ValueError("Remove virtual obstacles before exporting a robot path.")
            yaw = self.robot_pose[2] if self._nav2_pose_fresh() else 0.0
            poses = [self.pixel_to_world(px, py) for px, py in self.path]
        except Exception as exc:
            messagebox.showwarning("Nav2 export", str(exc))
            return
        filename = filedialog.asksaveasfilename(
            title="Export ROS nav_msgs/Path preview YAML",
            initialdir=str(Path.home() / "lidar_slam_ws" / "path_planner"),
            initialfile="nav2_path_preview.yaml", defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml")])
        if not filename:
            return
        stops = [self.pixel_to_world(*p) for p in self.visit_order]
        data = {"message_type": "nav_msgs/msg/Path", "frame_id": "map",
                "execution_enabled": False,
                "notes": "Preview only. This file is not an executable Nav2 action goal.",
                "stops": [{"x": round(x, 5), "y": round(y, 5), "dwell_seconds": 2.0}
                          for x, y in stops],
                "poses": [{"x": round(x, 5), "y": round(y, 5),
                           "qz": round(math.sin(yaw/2), 8),
                           "qw": round(math.cos(yaw/2), 8)} for x, y in poses]}
        try:
            with open(filename, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, sort_keys=False)
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc))
            return
        self.nav2_status.set(f"Nav2: exported {len(poses)} poses (preview only)")

    def export_path(self):
        try:
            self.validate_current_route()
        except UnsafePathError as exc:
            messagebox.showwarning("Unsafe path", str(exc))
            return
        if not self.path or self.resolution is None:
            messagebox.showwarning("No path", "Plan a path before exporting.")
            return

        # Export collision-checked simplified waypoints. Future physical
        # following still requires live obstacle avoidance and safe stopping.
        waypoints = []
        # Mandatory stops are guaranteed to be included because each
        # TSP segment is simplified independently.
        stop_lookup = {pt: i + 1 for i, pt in enumerate(self.visit_order)}
        for px, py in self.path:
            wx, wy = self.pixel_to_world(px, py)
            point = {"x": round(wx, 5), "y": round(wy, 5)}
            if (px, py) in stop_lookup:
                point["stop_id"] = stop_lookup[(px, py)]
                point["dwell_seconds"] = 2.0
            waypoints.append(point)

        filename = filedialog.asksaveasfilename(
            title="Export A* waypoints (dry run only)",
            initialdir=str(Path.home() / "lidar_slam_ws" / "path_planner"),
            initialfile="astar_path.yaml",
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml"), ("All files", "*")]
        )
        if not filename:
            return

        data = {"frame_id": "map", "waypoints": waypoints}
        try:
            with open(filename, "w", encoding="utf-8") as output:
                yaml.safe_dump(data, output, sort_keys=False)
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc))
            return

        self.status_var.set(
            f"Exported {len(waypoints)} waypoints to {Path(filename).name} "
            "(DRY RUN ONLY)."
        )

    # ========================================================
    # DRAWING
    # ========================================================

    def redraw(self):

        if self.original_map is None:

            self.canvas.delete(
                "all"
            )

            return

        # Invalid map metadata must not cause divisions by zero during rendering.
        if self.resolution is None or self.resolution <= 0:
            self.canvas.delete("all")
            return

        display = (
            self.original_map.convert(
                "RGB"
            )
        )

        draw = ImageDraw.Draw(
            display
        )

        # Virtual obstacles are drawn as red translucent-looking disks.
        for ox, oy, radius_m in self.dynamic_obstacles:
            r = max(1, int(round(radius_m / self.resolution)))
            draw.ellipse((ox-r, oy-r, ox+r, oy+r), fill=(230, 90, 60),
                         outline=(130, 30, 20), width=2)

        # Path
        if (
            self.path
            and len(self.path) > 1
        ):

            # Width is scaled relative to map resolution
            # so high-resolution maps remain visible.
            path_width = max(
                1,
                int(
                    0.04
                    / self.resolution
                )
            )

            draw.line(
                self.path,
                fill=(0, 120, 255),
                width=path_width
            )

        # Marker radius approximately 8 cm.
        marker_radius = max(
            2,
            int(
                0.08
                / self.resolution
            )
        )

        # START
        if self.start:

            x, y = self.start

            draw.ellipse(
                (
                    x - marker_radius,
                    y - marker_radius,
                    x + marker_radius,
                    y + marker_radius
                ),
                fill=(0, 220, 0),
                outline=(0, 80, 0)
            )

        # Intermediate stops, numbered in the optimal visiting order.
        for idx, (x, y) in enumerate(self.stops, 1):
            r = marker_radius
            draw.ellipse((x-r, y-r, x+r, y+r), fill=(255, 165, 0), outline=(90, 45, 0))
            visit_num = (self.visit_order.index((x, y)) + 1
                         if (x, y) in self.visit_order else None)
            draw.text((x+r+2, y-r), f"S{idx}" + (f" → {visit_num}" if visit_num else ""),
                      fill=(150, 65, 0), stroke_width=1, stroke_fill="white")

        # GOAL
        if self.goal:

            x, y = self.goal

            draw.ellipse(
                (
                    x - marker_radius,
                    y - marker_radius,
                    x + marker_radius,
                    y + marker_radius
                ),
                fill=(255, 40, 40),
                outline=(100, 0, 0)
            )

        # Travelled route in map coordinates (green, thin line).
        if self.show_trail.get() and len(self.trail) >= 2:
            trail_px = [self.world_to_pixel(x, y) for x, y in self.trail]
            trail_px = [p for p in trail_px if p is not None]
            if len(trail_px) >= 2:
                draw.line(trail_px, fill=(20, 180, 100), width=max(1, int(0.025/self.resolution)))

        # Actual oriented rectangular robot footprint (visualization only).
        if self.show_footprint.get() and self.robot_pose is not None:
            wx, wy, yaw = self.robot_pose
            half_l, half_w = ROBOT_LENGTH_M / 2, ROBOT_WIDTH_M / 2
            corners = []
            for lx, ly in ((half_l, half_w), (half_l, -half_w),
                           (-half_l, -half_w), (-half_l, half_w)):
                pxpy = self.world_to_pixel(wx + lx*math.cos(yaw) - ly*math.sin(yaw),
                                           wy + lx*math.sin(yaw) + ly*math.cos(yaw))
                if pxpy is not None:
                    corners.append(pxpy)
            if len(corners) == 4:
                draw.polygon(corners, outline=(0, 140, 160), width=max(2, int(.02/self.resolution)))

        # Live robot marker (cyan triangle pointing along +X in ROS).
        if self.robot_pose is not None:
            pxpy = self.world_to_pixel(*self.robot_pose[:2])
            if pxpy is not None:
                cx, cy = pxpy
                heading = self.robot_pose[2] - self.origin_yaw
                size = max(5, int(0.16 / self.resolution))
                vertices = []
                for forward, lateral in ((1.0, 0.0), (-0.65, 0.55), (-0.65, -0.55)):
                    vx = cx + size*(forward*math.cos(heading)-lateral*math.sin(heading))
                    vy = cy - size*(forward*math.sin(heading)+lateral*math.cos(heading))
                    vertices.append((vx, vy))
                draw.polygon(vertices, fill=(0, 220, 230), outline=(0, 50, 70))

        # V6 proximity ring: visual indication, NOT live LiDAR sensing.
        if self.sim_pose is not None and self.show_detection.get():
            cx, cy = self.sim_pose[:2]
            try:
                r = max(1, float(self.detection_radius_var.get()) / self.resolution)
            except (ValueError, tk.TclError):
                r = 1
            draw.ellipse((cx-r, cy-r, cx+r, cy+r), outline=(255, 150, 0),
                         width=max(1, int(.025/self.resolution)))

        # Simulated robot is separate from the real TF robot (magenta).
        if self.sim_pose is not None:
            sx, sy, syaw = self.sim_pose
            cx, cy = sx, sy
            radius = max(4, int(0.11 / self.resolution))
            half_l = ROBOT_LENGTH_M/(2*self.resolution)
            half_w = ROBOT_WIDTH_M/(2*self.resolution)
            corners=[]
            for forward,lateral in ((-half_l,-half_w),(half_l,-half_w),(half_l,half_w),(-half_l,half_w)):
                corners.append((cx+forward*math.cos(syaw)-lateral*math.sin(syaw),
                                cy-forward*math.sin(syaw)-lateral*math.cos(syaw)))
            draw.polygon(corners, outline=(0, 160, 210), width=max(2,int(.025/self.resolution)))
            draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius),
                         fill=(220, 40, 200), outline=(80, 0, 90), width=2)
            draw.line((cx, cy, cx + radius*1.5*math.cos(syaw),
                       cy - radius*1.5*math.sin(syaw)), fill=(70, 0, 80), width=3)

        new_width = max(
            1,
            int(
                self.map_width
                * self.scale
            )
        )

        new_height = max(
            1,
            int(
                self.map_height
                * self.scale
            )
        )

        # Metric scale bar: target around 100 source pixels.
        target_m = 100 * self.resolution
        power = 10 ** math.floor(math.log10(max(target_m, 1e-9)))
        scale_m = min((1, 2, 5, 10), key=lambda n: abs(n*power-target_m)) * power
        bar_px = max(1, round(scale_m/self.resolution))
        bx, by = 16, self.map_height - 18
        draw.line(((bx, by), (bx+bar_px, by)), fill=(0,0,0), width=5)
        draw.line(((bx, by), (bx+bar_px, by)), fill=(255,255,255), width=3)
        draw.text((bx, by-17), f"{scale_m:g} m", fill=(0,0,0), stroke_width=2, stroke_fill="white")

        display = display.resize(
            (
                new_width,
                new_height
            ),
            Image.Resampling.NEAREST
        )

        self.tk_image = (
            ImageTk.PhotoImage(
                display
            )
        )

        self.canvas.delete(
            "all"
        )

        # Center the image when it is smaller than the viewport.
        # These offsets are also used for map clicks and cursor coordinates.
        viewport_w = max(1, self.canvas.winfo_width())
        viewport_h = max(1, self.canvas.winfo_height())
        self.map_offset_x = max(0, (viewport_w - new_width) // 2)
        self.map_offset_y = max(0, (viewport_h - new_height) // 2)
        self.canvas.create_image(
            self.map_offset_x,
            self.map_offset_y,
            anchor=tk.NW,
            image=self.tk_image
        )

        self.canvas.configure(
            scrollregion=(
                0,
                0,
                max(viewport_w, new_width),
                max(viewport_h, new_height)
            )
        )


    # ========================================================
    # ZOOM
    # ========================================================

    def begin_pan(self, event):
        self.canvas.scan_mark(event.x, event.y)
        self._drag_origin = (event.x, event.y)

    def drag_pan(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def end_pan(self, event):
        self._drag_origin = None

    def update_mouse_coordinates(self, event):
        if self.original_map is None:
            return
        px = int((self.canvas.canvasx(event.x) - self.map_offset_x) / self.scale)
        py = int((self.canvas.canvasy(event.y) - self.map_offset_y) / self.scale)
        if 0 <= px < self.map_width and 0 <= py < self.map_height:
            wx, wy = self.pixel_to_world(px, py)
            self.mouse_position.set(f"Cursor: x={wx:.2f} m  y={wy:.2f} m  |  pixel ({px}, {py})")
        else:
            self.mouse_position.set("Cursor: outside map")

    def zoom_at(self, event, factor):
        if self.original_map is None:
            return
        old_x = self.canvas.canvasx(event.x)
        old_y = self.canvas.canvasy(event.y)
        self._fit_on_resize = False
        old_scale = self.scale
        old_offset_x, old_offset_y = self.map_offset_x, self.map_offset_y
        self.scale = max(0.2, min(12.0, self.scale * factor))
        if self.scale == old_scale:
            return
        self.redraw()
        new_w = self.map_width * self.scale
        new_h = self.map_height * self.scale
        self.canvas.update_idletasks()
        self.canvas.xview_moveto(max(0, ((old_x-old_offset_x)*self.scale/old_scale+self.map_offset_x-event.x) / max(1,max(new_w,self.canvas.winfo_width()))))
        self.canvas.yview_moveto(max(0, ((old_y-old_offset_y)*self.scale/old_scale+self.map_offset_y-event.y) / max(1,max(new_h,self.canvas.winfo_height()))))

    def zoom_center(self, factor):
        event = type("Event", (), {})()
        event.x = self.canvas.winfo_width() // 2
        event.y = self.canvas.winfo_height() // 2
        self.zoom_at(event, factor)

    def zoom_in(self):
        self.zoom_center(1.25)

    def zoom_out(self):
        self.zoom_center(0.8)

    def mouse_wheel(self, event):
        self.zoom_at(event, 1.25 if event.delta > 0 else 0.8)

    def _on_canvas_resize(self, event):
        if self.original_map is None:
            return
        if self._fit_resize_job is not None:
            self.root.after_cancel(self._fit_resize_job)
        if self._fit_on_resize:
            self._fit_resize_job = self.root.after(150, self.fit_map)
        else:
            self._fit_resize_job = self.root.after(150, self.redraw)

    def fit_map(self):
        if self.original_map is None:
            return
        self._fit_on_resize = True
        width = max(1, self.canvas.winfo_width() - 18)
        height = max(1, self.canvas.winfo_height() - 18)
        if width <= 1 or height <= 1:
            self.root.after(100, self.fit_map)
            return
        self.scale = max(0.05, min(12.0, width / self.map_width, height / self.map_height))
        self.redraw()
        self.canvas.xview_moveto(0)
        self.canvas.yview_moveto(0)

    def toggle_fullscreen(self, event=None):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def leave_fullscreen(self, event=None):
        self.fullscreen = False
        self.root.attributes("-fullscreen", False)

    def set_obstacle_mode(self):
        if self.original_map is not None:
            self.mode = "obstacle"
            self.status_var.set("Click map to add a VIRTUAL circular obstacle.")

    def refresh_dynamic_grid(self):
        if self.static_inflated_grid is None:
            return
        grid = self.static_inflated_grid.copy()
        yy, xx = np.ogrid[:grid.shape[0], :grid.shape[1]]
        clearance = max(0.0, float(self.clearance_var.get()))
        for x, y, radius in self.dynamic_obstacles:
            r = (radius + clearance) / self.resolution
            grid[(xx-x)**2 + (yy-y)**2 <= r*r] = False
        self.inflated_grid = grid
        self.distance_field = distance_transform_edt(grid)

    def undo_obstacle(self):
        if self.dynamic_obstacles:
            self.dynamic_obstacles.pop()
            self.refresh_dynamic_grid()
            self.handle_obstacle_change()

    def clear_obstacles(self):
        self.dynamic_obstacles.clear()
        self.refresh_dynamic_grid()
        self.handle_obstacle_change()

    def handle_obstacle_change(self):
        # Always stop before replanning. Never drive through an unvalidated route.
        was_running = self.sim_state == "running" and self.sim_pose is not None
        current = (round(self.sim_pose[0]), round(self.sim_pose[1])) if was_running else None
        remaining = list(self.visit_order)
        if current is not None:
            # Preserve only stops not yet reached along the simulated route.
            remaining = [p for p in remaining if p in self.sim_route[self.sim_index+1:]]
        self.stop_simulation(redraw=False)
        self.path = None
        self.visit_order = []
        if current is not None:
            if (0 <= current[0] < self.map_width and 0 <= current[1] < self.map_height
                    and self.inflated_grid[current[1], current[0]]):
                self.start = current
                self.stops = remaining
                self.auto_start = False
                self.auto_button.configure(text="AUTO START: OFF")
                self.plan_path()
                self.status_var.set("Obstacle changed: route replanned from virtual position; simulation stopped.")
                return
            self.status_var.set("Obstacle overlaps virtual robot: simulation stopped; move obstacle and replan.")
        else:
            self.status_var.set("Virtual obstacles updated. Press PLAN PATH to replan.")
        self.route_stats.set("Route: not planned")
        self.redraw()

    def update_route_stats(self):
        if not self.path or self.distance_field is None:
            self.route_stats.set("Route: not planned")
            return
        length_px = sum(heuristic(a,b) for a,b in zip(self.path,self.path[1:]))
        # Sample along full line segments, not only at the waypoints.
        minimum_px = float('inf')
        for a,b in zip(self.path,self.path[1:]):
            steps = max(1, int(math.ceil(heuristic(a,b))))
            for i in range(steps+1):
                x = int(round(a[0]+(b[0]-a[0])*i/steps))
                y = int(round(a[1]+(b[1]-a[1])*i/steps))
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    minimum_px = min(minimum_px, self.distance_field[y,x])
        try:
            speed = max(0.05, float(self.sim_speed_var.get()))
        except (ValueError, tk.TclError):
            speed = 0.3
        length_m = length_px * self.resolution
        extra_clearance = max(0.0, (minimum_px-1)*self.resolution)
        self.route_stats.set(f"Route: {length_m:.2f} m | min additional clearance: "
                             f"{extra_clearance:.2f} m | ideal travel: "
                             f"{length_m/speed + 2*len(self.stops):.1f} s incl. dwell (no acceleration)")

    def apply_clearance(self):
        if self.free_grid is None or self.resolution is None:
            return
        try:
            clearance = float(self.clearance_var.get())
            if not math.isfinite(clearance) or not 0 <= clearance <= 1.5:
                raise ValueError()
        except (ValueError, tk.TclError):
            messagebox.showerror("Clearance", "Enter a clearance between 0 and 1.5 m.")
            return
        self.stop_simulation(redraw=False)
        self.static_inflated_grid = inflate_free_grid(
            self.free_grid, math.ceil(clearance / self.resolution))
        self.refresh_dynamic_grid()
        self.path = None
        self.visit_order = []
        self.status_var.set(f"Clearance set to {clearance:.2f} m. Plan again to update the route.")
        self.redraw()

    def start_simulation(self):
        if not self.path or self.resolution is None:
            messagebox.showwarning("Simulation", "Plan a path first.")
            return
        self.stop_simulation(redraw=False)
        self.sim_route = list(self.path)
        self.sim_index = 0
        self.sim_progress = 0.0
        self.sim_velocity = 0.0
        self.sim_distance = 0.0
        self.sim_elapsed = 0.0
        self.sim_dwell_remaining = 0.0
        first, second = self.sim_route[:2] if len(self.sim_route) > 1 else (self.sim_route[0], self.sim_route[0])
        initial_yaw = self.robot_pose[2] if self._nav2_pose_fresh() else 0.0
        self.sim_pose = (*self.sim_route[0], initial_yaw)
        self.sim_index = 1 if len(self.sim_route) > 1 else 0
        self.wheel_rates = [0.0]*4
        self.mega_commands = (0, 0, 0, 0)
        self.sim_state = "running"
        self.sim_last_time = time.monotonic()
        self.sim_status.set("Simulation: running (virtual only)")
        self.simulation_tick()

    def pause_simulation(self):
        if self.sim_state == "running":
            self.sim_state = "paused"
            self.sim_status.set("Simulation: paused")
        elif self.sim_state == "paused":
            self.sim_state = "running"
            self.sim_last_time = time.monotonic()
            self.sim_status.set("Simulation: running (virtual only)")

    def stop_simulation(self, redraw=True):
        if self.sim_after_id is not None:
            try:
                self.root.after_cancel(self.sim_after_id)
            except tk.TclError:
                pass
            self.sim_after_id = None
        self.sim_state = "stopped"
        self.sim_pose = None
        self.sim_route = []
        self.sim_index = 0
        self.sim_velocity = 0.0
        self.wheel_rates = [0.0]*4
        self.mega_commands = (0, 0, 0, 0)
        self.mega_diagnostics.set("Mega virtual: stopped (all commands zero)")
        self.wheel_display.set("FL 0.0 | FR 0.0 | RL 0.0 | RR 0.0 rad/s")
        for name, variable in self.wheel_indicators.items():
            variable.set(f"{name}: 0.0  STOP")
        self.proximity_display.set("Detection zone: inactive")
        self.sim_dwell_remaining = 0.0
        self.sim_status.set("Simulation: stopped")
        self.sim_telemetry.set("Speed: 0.00 m/s | Distance: 0.00 m | Time: 0.0 s")
        if redraw:
            self.redraw()

    def simulation_tick(self):
        """Virtual wheel-driven simulation. Never publishes /cmd_vel or opens serial."""
        self.sim_after_id = None
        if self.sim_state == "stopped":
            return
        now = time.monotonic()
        dt = min(0.08, max(0.0, now - (self.sim_last_time or now)))
        self.sim_last_time = now
        if self.sim_state == "running" and self.sim_pose is not None and dt > 0:
            self.sim_elapsed += dt
            try:
                vmax = max(.05, min(2., float(self.sim_speed_var.get())))
                accel = max(.05, min(2., float(self.sim_accel_var.get())))
                max_yaw = math.radians(max(15., min(360., float(self.sim_yaw_rate_var.get()))))
                radius = max(.01, float(self.wheel_radius_var.get()))
                lever = (max(.01, float(self.wheel_base_var.get())) +
                         max(.01, float(self.track_width_var.get()))) / 2.
                wheel_accel = max(.1, float(self.wheel_accel_var.get()))
            except (ValueError, tk.TclError):
                vmax, accel, max_yaw = .3, .35, math.pi/2
                radius, lever, wheel_accel = .04, .235, 20.

            x, y, yaw = self.sim_pose
            desired_vx = desired_vy = desired_omega = 0.0
            if self.sim_dwell_remaining > 0:
                # Wait only after all four wheels have effectively stopped.
                if max(abs(w) for w in self.wheel_rates) < .12:
                    self.sim_dwell_remaining = max(0., self.sim_dwell_remaining-dt)
                self.sim_status.set(f"Simulation: dwelling ({self.sim_dwell_remaining:.1f} s)")
            elif self.sim_index < len(self.sim_route):
                # Waypoint tracking: world-frame positional error -> body velocity.
                target = self.sim_route[self.sim_index]
                dx = (target[0]-x)*self.resolution
                dy = -(target[1]-y)*self.resolution
                dist = math.hypot(dx, dy)
                is_stop = target in self.stops or self.sim_index == len(self.sim_route)-1
                # Do not skip required stops. Small tolerance accommodates integration.
                threshold = .045 if is_stop else .075
                if dist < threshold and (not is_stop or max(abs(w) for w in self.wheel_rates) < 1.2):
                    if is_stop:
                        self.sim_dwell_remaining = 2.0 if target in self.stops else 0.0
                        if self.sim_index == len(self.sim_route)-1:
                            self.sim_state = "finished"
                            self.wheel_rates = [0.0]*4
                            self.mega_commands = (0, 0, 0, 0)
                            self.mega_diagnostics.set("Mega virtual: completed (all commands zero)")
                            self.sim_status.set("Simulation: completed")
                        else:
                            self.sim_status.set("Simulation: dwelling at stop (2 s)")
                    self.sim_index += 1
                else:
                    # Brake for the next mandatory stop, not for every smoothed point.
                    remaining = dist
                    for j in range(self.sim_index, len(self.sim_route)-1):
                        a, b = self.sim_route[j:j+2]
                        remaining += heuristic(a,b)*self.resolution
                        if b in self.stops or j == len(self.sim_route)-2:
                            break
                    speed_limit = min(vmax, math.sqrt(max(0., 2*accel*remaining)))
                    # Avoid overshooting a waypoint at a sharp bend.
                    if is_stop:
                        speed_limit = min(speed_limit, math.sqrt(max(0., 2*accel*dist)))
                    else:
                        speed_limit = min(speed_limit, max(.08, dist*3.0))
                    # Proportional position controller with bounded world velocity.
                    desired_world_x = speed_limit * dx/max(dist,1e-8)
                    desired_world_y = speed_limit * dy/max(dist,1e-8)
                    desired_vx = math.cos(yaw)*desired_world_x + math.sin(yaw)*desired_world_y
                    desired_vy = -math.sin(yaw)*desired_world_x + math.cos(yaw)*desired_world_y
                    # Translation-only mecanum operation: no commanded yaw.
                    desired_omega = 0.0
                    self.sim_status.set(f"Simulation: waypoint {self.sim_index+1}/{len(self.sim_route)}")

            # All motion goes through wheel-speed commands and wheel acceleration limits.
            if self.mega_mode.get():
                # Exactly the same velocity-to-command mixing and proportional
                # 2000-unit saturation as the uploaded, working Pi bridge.
                # No packets are created or sent to the real Arduino.
                self.mega_commands = mega_bridge_mix(desired_vx, desired_vy, desired_omega)
                wheel_targets = virtual_mega_wheel_targets(self.mega_commands, radius)
                # The Arduino packet uses FL, RL, FR, RR (not the GUI's FL, FR, RL, RR).
                fl, fr, rl, rr = self.mega_commands
                self.mega_diagnostics.set(
                    f"Mega virtual units: FL {fl:+5d} RL {rl:+5d} "
                    f"FR {fr:+5d} RR {rr:+5d} | nominal motor response"
                )
            else:
                self.mega_commands = (0, 0, 0, 0)
                wheel_targets = mecanum_wheel_rates(
                    desired_vx, desired_vy, desired_omega, radius, lever)
                self.mega_diagnostics.set("Direct kinematics (V7.1 mode)")
            self.wheel_rates = [approach(old, goal, wheel_accel*dt)
                                for old,goal in zip(self.wheel_rates,wheel_targets)]
            vx,vy,omega = mecanum_body_velocity(self.wheel_rates,radius,lever)
            yaw_mid = yaw + omega*dt/2
            world_vx = math.cos(yaw_mid)*vx-math.sin(yaw_mid)*vy
            world_vy = math.sin(yaw_mid)*vx+math.cos(yaw_mid)*vy
            new_x = x + world_vx*dt/self.resolution
            new_y = y - world_vy*dt/self.resolution
            # The inflated map remains a hard collision boundary for simulation.
            ix,iy = int(round(new_x)),int(round(new_y))
            next_yaw = math.atan2(math.sin(yaw+omega*dt), math.cos(yaw+omega*dt))
            footprint_safe = (self.free_grid is not None and
                swept_footprint_is_free(self.free_grid, (x,y,yaw),
                    (new_x,new_y,next_yaw), ROBOT_LENGTH_M, ROBOT_WIDTH_M, self.resolution))
            if not footprint_safe:
                self.wheel_rates = [0.0]*4
                self.sim_velocity = 0.0
                self.sim_state = "paused"
                self.sim_status.set("Simulation: ROBOT FOOTPRINT COLLISION RISK — paused")
                self.proximity_display.set("DANGER: predicted footprint overlaps occupied/unknown map cells")
            elif (self.inflated_grid is not None and
                    (ix < 0 or iy < 0 or iy >= self.map_height or ix >= self.map_width or
                     not self.inflated_grid[iy,ix])):
                self.wheel_rates = [0.0]*4
                self.sim_state = "paused"
                self.sim_status.set("Simulation: collision boundary reached — paused")
            else:
                self.sim_pose = (new_x,new_y,math.atan2(math.sin(yaw+omega*dt),
                                                       math.cos(yaw+omega*dt)))
                distance_step = math.hypot(new_x-x,new_y-y)*self.resolution
                self.sim_distance += distance_step
            self.sim_velocity = math.hypot(vx,vy)
            self.sim_telemetry.set(
                f"Speed: {self.sim_velocity:.2f} m/s | Distance: {self.sim_distance:.2f} m | "
                f"Time: {self.sim_elapsed:.1f} s | Wheel accel: {wheel_accel:.1f} rad/s²")
            self.update_wheels_and_detection()
        self.redraw()
        if self.sim_state not in ("stopped", "finished"):
            self.sim_after_id = self.root.after(50,self.simulation_tick)

    def update_wheels_and_detection(self):
        """Read actual simulated wheel states; never recompute them from pose."""
        if self.sim_pose is None or self.resolution is None:
            return
        x,y,_ = self.sim_pose
        names = ("FL","FR","RL","RR")
        self.wheel_display.set(" | ".join(
            f"{name} {rate:+.1f} rad/s ({'FWD' if rate>.02 else 'REV' if rate<-.02 else 'STOP'})"
            for name,rate in zip(names,self.wheel_rates)))
        for name,rate in zip(names,self.wheel_rates):
            direction = "FWD" if rate>.02 else "REV" if rate<-.02 else "STOP"
            self.wheel_indicators[name].set(f"{name}: {rate:+6.1f}  {direction}")
        if self.distance_field is not None:
            ix,iy = int(round(x)),int(round(y))
            if 0 <= iy < self.map_height and 0 <= ix < self.map_width:
                clearance = nearest_obstacle_distance(self.raw_distance_m, x, y) if self.raw_distance_m is not None else 0.0
                try:
                    detection = max(0.,float(self.detection_radius_var.get()))
                except (ValueError,tk.TclError):
                    detection = .45
                state = "OBSTACLE IN ZONE" if clearance <= detection else "clear"
                self.proximity_display.set(
                    f"Detection zone: {state} | nearest inflated obstacle ≈ {clearance:.2f} m "
                    f"| radius {detection:.2f} m (virtual only)")

    def clear_trail(self):
        self.trail.clear()
        self.redraw()

    # ========================================================
    # CLEAR
    # ========================================================

    def clear(self):
        self.stop_simulation(redraw=False)

        self.start = None
        self.goal = None
        self.stops = []  # Up to five intermediate destinations.
        self.visit_order = []
        self.path = None

        self.mode = "start"

        if self.original_map is not None:

            self.status_var.set(
                "Cleared. Click a free area "
                "to set START."
            )

        self.redraw()


# ============================================================
# MAIN
# ============================================================

def main():

    root = tk.Tk()

    PathPlannerGUI(root)

    root.mainloop()


if __name__ == "__main__":
    main()
