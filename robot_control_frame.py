# robot_control_frame.py

import customtkinter as ctk
from tkinter import messagebox, simpledialog, filedialog
import tkinter as tk
from tkinter import ttk
import json
import time
import threading
import serial
import cv2
from PIL import Image, ImageTk
from queue import Queue, Empty
import logging

from constants import STEP_TYPE_COLORS


class RobotControlFrame:
    def __init__(self, parent, com_port, robot_name):
        self.parent = parent
        self.com_port = com_port
        self.robot_name = robot_name

        # Initialize logger
        self.logger = logging.getLogger(robot_name)
        handler = logging.StreamHandler()
        formatter = logging.Formatter(f'%(asctime)s - {robot_name} - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.DEBUG)

        self.current_position = {"x": 235.0, "y": 0.0, "z": 234.0, "t": 3.14}
        self.default_spd = 2.5
        self.default_acc = 10
        self.xyz_increment = 3.0
        self.t_increment = 0.02
        self.update_interval = 0.05

        self.missions = {}
        self.selected_mission = None

        self.ser = self.initialize_serial(self.com_port, 115200)
        self.moving_thread = None
        self.moving_stop_event = threading.Event()

        self.mission_execution_thread = None
        self.mission_execution_stop_event = threading.Event()
        self.current_mission_step = 0

        self.dragging_item = None
        self.dragging_start_index = None

        self.available_cameras = self.detect_cameras()
        self.selected_camera = tk.StringVar(value="None")
        self.camera_label = None
        self.video_capture = None
        self.camera_update_job = None
        self.camera_running = False

        self.speed_scale = 1.0
        self.distance_scale = 100.0
        self.base_delay = 0.5
        self.max_delay = 2.0

        # Queue for thread-safe GUI updates
        self.gui_queue = Queue()

        # Lock for movement synchronization
        self.movement_lock = threading.Lock()

        # Stop event for serial listener
        self.serial_listener_stop_event = threading.Event()

        # Start the serial listener thread if serial is initialized
        if self.ser:
            self.serial_listener_thread = threading.Thread(target=self.serial_listener, daemon=True)
            self.serial_listener_thread.start()
            self.logger.info("Serial listener thread started.")
        else:
            self.logger.error("Serial listener thread not started due to failed serial initialization.")

        self.create_widgets()
        self.move_to_current_position()

        # Start the GUI update loop
        self.parent.after(100, self.process_gui_queue)

    def initialize_serial(self, port, baudrate):
        try:
            s = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Allow time for the serial connection to initialize
            self.logger.info(f"Serial port {port} opened successfully.")
            return s
        except serial.SerialException as e:
            self.show_error("Serial Error", f"Failed to connect {self.robot_name} on {port}: {e}")
            self.logger.error(f"SerialException: {e}")
            return None
        except Exception as e:
            self.show_error("Serial Error", f"Unexpected error connecting {self.robot_name} on {port}: {e}")
            self.logger.error(f"Unexpected exception during serial initialization: {e}")
            return None

    def detect_cameras(self):
        available = []
        for i in range(5):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap and cap.isOpened():
                available.append(str(i))
                cap.release()
        if not available:
            available = ["None"]
        else:
            available.insert(0, "None")
        self.logger.info(f"Available cameras: {available}")
        return available

    def create_widgets(self):
        self.parent.columnconfigure(0, weight=1)
        for r in range(9):
            self.parent.rowconfigure(r, weight=1)

        top_frame = ctk.CTkFrame(self.parent, corner_radius=6)
        top_frame.grid(row=0, column=0, padx=5, pady=10, sticky="nsew")
        self.parent.rowconfigure(0, weight=2)

        title_label = ctk.CTkLabel(top_frame, text=self.robot_name, font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, padx=5, pady=5, sticky="nw")

        self.create_webcam_controls(top_frame, 0, 1)
        self.create_xyz_controls(self.parent, 1, 0)
        self.create_t_control(self.parent, 2, 0)
        self.create_suction_controls(self.parent, 3, 0)
        self.create_extra_controls(self.parent, 4, 0)
        self.create_mission_steps(self.parent, 5, 0)
        self.create_mission_controls(self.parent, 6, 0)
        self.create_status_bar(self.parent, 7, 0)
        self.create_bottom_buttons(self.parent, 8, 0)

    def serial_listener(self):
        """Continuously listen for incoming serial data."""
        while not self.serial_listener_stop_event.is_set():
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.logger.debug(f"Received data: {line}")
                        try:
                            data = json.loads(line)
                            self.handle_serial_data(data)
                        except json.JSONDecodeError:
                            self.logger.warning(f"Invalid JSON received: {line}")
                    else:
                        continue
                except serial.SerialException as e:
                    self.logger.error(f"SerialException in listener: {e}")
                    self.show_error("Serial Error", f"Serial exception: {e}")
                    break  # Exit the listener thread on serial exception
                except Exception as e:
                    self.logger.error(f"Unexpected error in serial listener: {e}")
                    self.show_error("Listener Error", f"Unexpected error: {e}")
                    break  # Exit the listener thread on unexpected errors
            else:
                time.sleep(1)  # Wait before retrying if serial is not open
        self.logger.info("Serial listener thread exiting.")

    def handle_serial_data(self, data):
        """Process incoming serial data."""
        t_value = data.get("T")
        if t_value == 1051:
            with self.movement_lock:
                self.current_position["x"] = data.get("x", self.current_position["x"])
                self.current_position["y"] = data.get("y", self.current_position["y"])
                self.current_position["z"] = data.get("z", self.current_position["z"])
                self.current_position["t"] = data.get("t", self.current_position["t"])
                self.logger.info(f"Position updated from T=1051: {self.current_position}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Position updated: {self.current_position}."))
        elif t_value == 105:  # Assuming T=105 is another type of acknowledgment
            # Handle other types of acknowledgments if necessary
            self.logger.debug("Received T=105 acknowledgment.")
            pass
        else:
            # Handle other message types if necessary
            self.logger.warning(f"Unknown message type received: {data}")

    def move_to_current_position(self):
        cmd = {
            "T": 104,
            "x": self.current_position["x"],
            "y": self.current_position["y"],
            "z": self.current_position["z"],
            "t": self.current_position["t"],
            "spd": self.default_spd,
            "acc": self.default_acc
        }
        self.send_command_in_thread(cmd)
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Movement command sent."))
        self.logger.info(f"Movement command sent: {cmd}")

    def create_webcam_controls(self, parent, row, column):
        """Creates UI elements for selecting and displaying webcam feeds."""
        # Create a frame for webcam controls
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")

        # Expand this row/column in the parent for layout
        parent.rowconfigure(row, weight=1)
        parent.columnconfigure(column, weight=1)

        # Label for the camera selection
        cam_label = ctk.CTkLabel(frame, text="Select Webcam:", font=("Arial", 12))
        cam_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")

        # OptionMenu for available cameras
        self.selected_camera = tk.StringVar(value="None")
        self.camera_dropdown = ctk.CTkOptionMenu(
            frame,
            variable=self.selected_camera,
            values=self.available_cameras,
            command=self.on_camera_select
        )
        self.camera_dropdown.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        # A Label (using tkinter.Label) to display the webcam feed frame
        self.camera_label = tk.Label(frame)
        self.camera_label.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

        # Adjust layout weights so the camera feed can expand
        frame.columnconfigure(0, weight=0)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(1, weight=1)

    def on_camera_select(self, selection):
        """Called when the user selects a camera from the dropdown."""
        if selection == "None":
            self.stop_camera()
        else:
            self.start_camera(int(selection))

    def start_camera(self, camera_index):
        """Start capturing from the selected camera index."""
        self.stop_camera()
        self.video_capture = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not self.video_capture.isOpened():
            self.show_error("Camera Error", f"Unable to access camera {camera_index}.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to start camera {camera_index}."))
            return
        self.camera_running = True
        self.update_camera_feed()
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Camera {camera_index} started."))

    def stop_camera(self):
        """Stop capturing from the current camera feed."""
        if self.camera_running:
            self.camera_running = False
            if self.video_capture and self.video_capture.isOpened():
                self.video_capture.release()
            self.camera_label.config(image='')
            if self.camera_update_job:
                try:
                    self.parent.after_cancel(self.camera_update_job)
                except RuntimeError:
                    pass
                self.camera_update_job = None
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Camera stopped."))

    def update_camera_feed(self):
        """Continuously update the camera feed in the GUI."""
        if self.camera_running and self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.resize(frame, (400, 300))
                image = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=image)
                self.camera_label.imgtk = imgtk
                self.camera_label.configure(image=imgtk)
            else:
                self.stop_camera()
            try:
                self.camera_update_job = self.parent.after(33, self.update_camera_feed)
            except RuntimeError:
                pass

    def create_xyz_controls(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")

        title_label = ctk.CTkLabel(frame, text="XYZ Controls", font=("Arial", 12, "bold"))
        title_label.pack(pady=2)

        # X Control
        x_frame = ctk.CTkFrame(frame)
        x_frame.pack(padx=5, pady=2, fill="x")
        ctk.CTkLabel(x_frame, text="X:").pack(side="left", padx=5)
        plus_x = ctk.CTkButton(x_frame, text="+", command=lambda: self.start_continuous_move("x", +self.xyz_increment),
                               width=25, height=25, font=("Arial", 10))
        plus_x.pack(side="left", padx=2)
        minus_x = ctk.CTkButton(x_frame, text="-", command=lambda: self.start_continuous_move("x", -self.xyz_increment),
                                width=25, height=25, font=("Arial", 10))
        minus_x.pack(side="left", padx=2)
        plus_x.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("x", +self.xyz_increment))
        plus_x.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())
        minus_x.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("x", -self.xyz_increment))
        minus_x.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())

        # Y Control
        y_frame = ctk.CTkFrame(frame)
        y_frame.pack(padx=5, pady=2, fill="x")
        ctk.CTkLabel(y_frame, text="Y:").pack(side="left", padx=5)
        plus_y = ctk.CTkButton(y_frame, text="+", command=lambda: self.start_continuous_move("y", +self.xyz_increment),
                               width=25, height=25, font=("Arial", 10))
        plus_y.pack(side="left", padx=2)
        minus_y = ctk.CTkButton(y_frame, text="-", command=lambda: self.start_continuous_move("y", -self.xyz_increment),
                                width=25, height=25, font=("Arial", 10))
        minus_y.pack(side="left", padx=2)
        plus_y.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("y", +self.xyz_increment))
        plus_y.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())
        minus_y.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("y", -self.xyz_increment))
        minus_y.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())

        # Z Control
        z_frame = ctk.CTkFrame(frame)
        z_frame.pack(padx=5, pady=2, fill="x")
        ctk.CTkLabel(z_frame, text="Z:").pack(side="left", padx=5)
        plus_z = ctk.CTkButton(z_frame, text="+", command=lambda: self.start_continuous_move("z", +self.xyz_increment),
                               width=25, height=25, font=("Arial", 10))
        plus_z.pack(side="left", padx=2)
        minus_z = ctk.CTkButton(z_frame, text="-", command=lambda: self.start_continuous_move("z", -self.xyz_increment),
                                width=25, height=25, font=("Arial", 10))
        minus_z.pack(side="left", padx=2)
        plus_z.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("z", +self.xyz_increment))
        plus_z.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())
        minus_z.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("z", -self.xyz_increment))
        minus_z.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())

    def create_t_control(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")

        title_label = ctk.CTkLabel(frame, text="T Axis Control", font=("Arial", 12, "bold"))
        title_label.pack(pady=2)

        t_frame = ctk.CTkFrame(frame)
        t_frame.pack(padx=5, pady=2, fill="x")
        ctk.CTkLabel(t_frame, text="T:").pack(side="left", padx=5)
        plus_t = ctk.CTkButton(t_frame, text="+", command=lambda: self.start_continuous_move("t", +self.t_increment),
                               width=25, height=25, font=("Arial", 10))
        plus_t.pack(side="left", padx=2)
        minus_t = ctk.CTkButton(t_frame, text="-", command=lambda: self.start_continuous_move("t", -self.t_increment),
                                width=25, height=25, font=("Arial", 10))
        minus_t.pack(side="left", padx=2)
        plus_t.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("t", +self.t_increment))
        plus_t.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())
        minus_t.bind("<ButtonPress-1>", lambda e: self.start_continuous_move("t", -self.t_increment))
        minus_t.bind("<ButtonRelease-1>", lambda e: self.stop_continuous_move())

    def create_suction_controls(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")

        title_label = ctk.CTkLabel(frame, text="Suction & Relay", font=("Arial", 12, "bold"))
        title_label.pack(pady=2)

        btn_frame = ctk.CTkFrame(frame)
        btn_frame.pack(pady=5, fill="x", padx=10)

        ctk.CTkButton(btn_frame, text="Suction On", command=self.suction_on,
                      width=100, height=30, font=("Arial", 10)).pack(side="left", padx=5, pady=5, expand=True, fill="x")
        ctk.CTkButton(btn_frame, text="Suction Off", command=self.suction_off,
                      width=100, height=30, font=("Arial", 10)).pack(side="left", padx=5, pady=5, expand=True, fill="x")
        ctk.CTkButton(btn_frame, text="Relay Off", command=self.relay_off,
                      width=100, height=30, font=("Arial", 10)).pack(side="left", padx=5, pady=5, expand=True, fill="x")

    def suction_on(self):
        cmd = {"T": 113, "pwm_a": 0, "pwm_b": 255}
        self.send_command_in_thread(cmd)
        self.logger.info("Suction turned ON.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Suction turned ON."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Suction On command added to mission.")

    def suction_off(self):
        cmd = {"T": 113, "pwm_a": 255, "pwm_b": 0}
        self.send_command_in_thread(cmd)
        self.logger.info("Suction turned OFF.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Suction turned OFF."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Suction Off command added to mission.")

    def relay_off(self):
        cmd = {"T": 113, "pwm_a": 0, "pwm_b": 0}
        self.send_command_in_thread(cmd)
        self.logger.info("Relay turned OFF.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Relay turned OFF."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Relay Off command added to mission.")

    def create_extra_controls(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")

        title_label = ctk.CTkLabel(frame, text="Extra Controls", font=("Arial", 12, "bold"))
        title_label.pack(pady=2)

        btn_frame = ctk.CTkFrame(frame)
        btn_frame.pack(pady=5, fill="x", padx=10)

        ctk.CTkButton(btn_frame, text="Enable Torque", command=self.enable_torque,
                      width=120, height=30, font=("Arial", 10)).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(btn_frame, text="Disable Torque", command=self.disable_torque,
                      width=120, height=30, font=("Arial", 10)).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(btn_frame, text="Enable Dyn Adapt", command=self.enable_dynamic_adaptation,
                      width=140, height=30, font=("Arial", 10)).grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(btn_frame, text="Disable Dyn Adapt", command=self.disable_dynamic_adaptation,
                      width=140, height=30, font=("Arial", 10)).grid(row=0, column=3, padx=5, pady=5, sticky="ew")

    def enable_torque(self):
        cmd = {"T": 210, "cmd": 1}
        self.send_command_in_thread(cmd)
        self.logger.info("Torque enabled.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Torque enabled."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Torque Enable command added to mission.")

    def disable_torque(self):
        cmd = {"T": 210, "cmd": 0}
        self.send_command_in_thread(cmd)
        self.logger.info("Torque disabled.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Torque disabled."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Torque Disable command added to mission.")

    def enable_dynamic_adaptation(self):
        cmd = {"T": 112, "mode": 1, "b": 60, "s": 110, "e": 50, "h": 50}
        self.send_command_in_thread(cmd)
        self.logger.info("Dynamic Adaptation enabled.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Dynamic Adaptation enabled."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Dynamic Adaptation Enable command added to mission.")

    def disable_dynamic_adaptation(self):
        cmd = {"T": 112, "mode": 0, "b": 1000, "s": 1000, "e": 1000, "h": 1000}
        self.send_command_in_thread(cmd)
        self.logger.info("Dynamic Adaptation disabled.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Dynamic Adaptation disabled."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Dynamic Adaptation Disable command added to mission.")

    def create_mission_steps(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")

        title_label = ctk.CTkLabel(frame, text="Mission Steps", font=("Arial", 12, "bold"))
        title_label.pack(pady=2)

        btn_frame = ctk.CTkFrame(frame)
        btn_frame.pack(pady=5, fill="x", padx=10)

        buttons = [
            ("Add Step", self.add_step),
            ("Edit Step", self.edit_step),
            ("Delete Step", self.delete_step),
            ("Set Speed", self.set_speed),
            ("Add Delay", self.add_delay),
            ("Play Step", self.play_step),
            ("Add LED On", self.add_led_on_step),
            ("Add LED Off", self.add_led_off_step),
            ("Update Step", self.update_step),
            ("Insert Step", self.insert_step),
            ("Duplicate Step", self.duplicate_step)
        ]
        for idx, (txt, cmd) in enumerate(buttons):
            ctk.CTkButton(btn_frame, text=txt, command=cmd, width=120, height=30, font=("Arial", 10))\
                .grid(row=0, column=idx, padx=2, pady=2, sticky="ew")

        self.mission_steps_tree = ttk.Treeview(frame, columns=("Step Number", "Details"), show="headings", selectmode="browse")
        self.mission_steps_tree.heading("Step Number", text="Step Number")
        self.mission_steps_tree.heading("Details", text="Details")
        self.mission_steps_tree.column("Step Number", width=100, anchor="center")
        self.mission_steps_tree.column("Details", width=500, anchor="w")
        self.mission_steps_tree.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.mission_steps_tree.yview)
        scroll.pack(side="right", fill="y")
        self.mission_steps_tree.config(yscrollcommand=scroll.set)

        self.mission_steps_tree.bind("<Double-1>", self.edit_step_event)
        self.mission_steps_tree.bind("<ButtonPress-1>", self.on_treeview_click)
        self.mission_steps_tree.bind("<B1-Motion>", self.on_treeview_drag)
        self.mission_steps_tree.bind("<ButtonRelease-1>", self.on_treeview_drop)

        style = ttk.Style()
        style.configure("Treeview", rowheight=25)
        style.map("Treeview", background=[('selected', '#347083')], foreground=[('selected', 'white')])

        self.progress_frame = ctk.CTkFrame(frame, corner_radius=6)
        self.progress_frame.pack(pady=5, fill="x", padx=10)

        progress_label = ctk.CTkLabel(self.progress_frame, text="Mission Progress:", font=("Arial", 12))
        progress_label.pack(side="left", padx=(0, 10))

        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(self.progress_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.pack(side="left", fill="x", expand=True)

    def add_step(self):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return
        step = {
            "T": 104,
            "x": self.current_position["x"],
            "y": self.current_position["y"],
            "z": self.current_position["z"],
            "t": self.current_position["t"],
            "spd": self.default_spd,
            "acc": self.default_acc
        }
        self.missions[self.selected_mission]["steps"].append(step)
        self.update_mission_steps_viewer()
        self.logger.info(f"Step added with position: {self.current_position}")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step added to mission."))

    def edit_step_event(self, event):
        self.edit_step()

    def edit_step(self):
        selected_item = self.mission_steps_tree.selection()
        if not selected_item:
            self.show_error("Error", f"No step selected for {self.robot_name}.")
            return
        index = self.mission_steps_tree.index(selected_item)
        step = self.missions[self.selected_mission]["steps"][index]
        x = simpledialog.askfloat("Edit Step", "X:", initialvalue=step.get("x", self.current_position["x"]), parent=self.parent)
        if x is None: return
        y = simpledialog.askfloat("Edit Step", "Y:", initialvalue=step.get("y", self.current_position["y"]), parent=self.parent)
        if y is None: return
        z = simpledialog.askfloat("Edit Step", "Z:", initialvalue=step.get("z", self.current_position["z"]), parent=self.parent)
        if z is None: return
        t = simpledialog.askfloat("Edit Step", "T:", initialvalue=step.get("t", self.current_position["t"]), parent=self.parent)
        if t is None: return
        spd = simpledialog.askfloat("Edit Step", "Speed:", initialvalue=step.get("spd", self.default_spd), parent=self.parent)
        if not spd or spd <= 0:
            self.show_error("Invalid Input", "Speed must be a positive number.")
            return
        acc = simpledialog.askfloat("Edit Step", "Acceleration:", initialvalue=step.get("acc", self.default_acc), parent=self.parent)
        if not acc or acc <= 0:
            self.show_error("Invalid Input", "Acceleration must be a positive number.")
            return
        new_step = {"T": 104, "x": x, "y": y, "z": z, "t": t, "spd": spd, "acc": acc}
        self.missions[self.selected_mission]["steps"][index] = new_step
        self.update_mission_steps_viewer()
        self.logger.info(f"Step {index + 1} updated: {new_step}")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step {index + 1} updated."))

    def delete_step(self):
        selected_item = self.mission_steps_tree.selection()
        if not selected_item:
            self.show_error("Error", f"No step selected for {self.robot_name}.")
            return
        index = self.mission_steps_tree.index(selected_item)
        del self.missions[self.selected_mission]["steps"][index]
        self.update_mission_steps_viewer()
        self.logger.info(f"Step {index + 1} deleted.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step {index + 1} deleted."))

    def set_speed(self):
        spd = simpledialog.askfloat("Set Speed", "Enter speed factor:", initialvalue=self.default_spd, parent=self.parent)
        if spd and spd > 0:
            self.default_spd = spd
            self.logger.info(f"Speed set to {self.default_spd}")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Speed set to {self.default_spd}"))
        else:
            self.show_error("Invalid Input", "Speed must be a positive number.")

    def add_delay(self):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return
        delay = simpledialog.askinteger("Add Delay", "ms:", initialvalue=1000, parent=self.parent)
        if delay is not None and delay >= 0:
            self.missions[self.selected_mission]["steps"].append({"T": 111, "cmd": delay})
            self.update_mission_steps_viewer()
            self.logger.info(f"Delay of {delay} ms added.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Delay of {delay} ms added."))
        else:
            self.show_error("Invalid Input", "Delay must be a non-negative integer.")

    def play_step(self):
        selected_item = self.mission_steps_tree.selection()
        if not selected_item:
            self.show_error("Error", f"No step selected for {self.robot_name}.")
            return
        index = self.mission_steps_tree.index(selected_item)
        step = self.missions[self.selected_mission]["steps"][index]

        def run_step():
            try:
                self.execute_step(step)
                self.queue_gui_update(lambda: self.update_progress_bar(100))
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step {index + 1} played successfully."))
            except Exception as e:
                self.show_error("Error", f"Failed to execute step: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to execute step {index + 1}."))
    
        threading.Thread(target=run_step, daemon=True).start()
        self.logger.info(f"Playing step {index + 1}.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Playing step {index + 1}."))

    def add_led_on_step(self):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return
        self.missions[self.selected_mission]["steps"].append({"T": 114, "led": 255})
        self.update_mission_steps_viewer()
        self.logger.info("LED On step added.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: LED On step added to mission."))

    def add_led_off_step(self):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return
        self.missions[self.selected_mission]["steps"].append({"T": 114, "led": 0})
        self.update_mission_steps_viewer()
        self.logger.info("LED Off step added.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: LED Off step added to mission."))

    def update_step(self):
        selected_item = self.mission_steps_tree.selection()
        if not selected_item:
            self.show_error("Error", f"No step selected for {self.robot_name}.")
            return
        index = self.mission_steps_tree.index(selected_item)
        step = self.missions[self.selected_mission]["steps"][index]
        if self.get_step_type(step) != "movement":
            self.show_error("Error", "Selected step is not a position step.")
            return
        updated_step = {
            "T": 104,
            "x": self.current_position["x"],
            "y": self.current_position["y"],
            "z": self.current_position["z"],
            "t": self.current_position["t"],
            "spd": self.default_spd,
            "acc": self.default_acc
        }
        self.missions[self.selected_mission]["steps"][index] = updated_step
        self.update_mission_steps_viewer()
        self.logger.info(f"Step {index + 1} updated: {updated_step}")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step {index + 1} updated."))

    def insert_step(self):
        selected_item = self.mission_steps_tree.selection()
        if not selected_item:
            self.show_error("Error", f"No step selected for {self.robot_name}.")
            return
        index = self.mission_steps_tree.index(selected_item)
        new_step = {
            "T": 104,
            "x": self.current_position["x"],
            "y": self.current_position["y"],
            "z": self.current_position["z"],
            "t": self.current_position["t"],
            "spd": self.default_spd,
            "acc": self.default_acc
        }
        self.missions[self.selected_mission]["steps"].insert(index + 1, new_step)
        self.update_mission_steps_viewer()
        self.logger.info(f"New step inserted after step {index + 1} with current position: {new_step}")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: New step inserted after step {index + 1}."))
    
    def duplicate_step(self):
        selected_item = self.mission_steps_tree.selection()
        if not selected_item:
            self.show_error("Error", f"No step selected for {self.robot_name}.")
            return
        index = self.mission_steps_tree.index(selected_item)
        step_to_dup = self.missions[self.selected_mission]["steps"][index]
        dup_step = step_to_dup.copy()
        self.missions[self.selected_mission]["steps"].insert(index + 1, dup_step)
        self.update_mission_steps_viewer()
        self.logger.info(f"Step {index + 1} duplicated and inserted as step {index + 2}.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step {index + 1} duplicated as step {index + 2}."))

    def get_step_type(self, step):
        t_value = step.get("T")
        if t_value == 104:
            return "movement"
        elif t_value == 113:
            if "pwm_a" in step and "pwm_b" in step:
                return "suction"
            return "relay"
        elif t_value == 111:
            return "delay"
        elif t_value == 114:
            return "led"
        elif t_value == 210:
            return "torque"
        elif t_value == 112:
            return "dynamic_adaptation"
        return "other"

    def create_mission_controls(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")
        title_label = ctk.CTkLabel(frame, text="Mission Controls", font=("Arial", 12, "bold"))
        title_label.pack(pady=2)

        btn_frame = ctk.CTkFrame(frame)
        btn_frame.pack(pady=5, fill="x", padx=10)

        ctk.CTkButton(btn_frame, text="Create Mission", command=self.create_mission,
                      width=120, height=30, font=("Arial", 10))\
            .grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        ctk.CTkButton(btn_frame, text="Save Mission", command=self.save_mission,
                      width=120, height=30, font=("Arial", 10))\
            .grid(row=0, column=1, padx=2, pady=2, sticky="ew")
        ctk.CTkButton(btn_frame, text="Play Mission", command=self.play_mission,
                      width=120, height=30, font=("Arial", 10))\
            .grid(row=0, column=2, padx=2, pady=2, sticky="ew")
        ctk.CTkButton(btn_frame, text="Load Mission", command=self.load_mission,
                      width=120, height=30, font=("Arial", 10))\
            .grid(row=0, column=3, padx=2, pady=2, sticky="ew")

    def update_mission_steps_viewer(self):
        self.mission_steps_tree.delete(*self.mission_steps_tree.get_children())
        if self.selected_mission in self.missions:
            for idx, s in enumerate(self.missions[self.selected_mission]["steps"]):
                step_num = idx + 1
                stype = self.get_step_type(s)
                if stype == "movement":
                    details = f"Move to (X:{s['x']}, Y:{s['y']}, Z:{s['z']}, T:{s['t']}) | Speed: {s['spd']} | Acc: {s['acc']}"
                elif stype == "suction":
                    st = "ON" if s.get("pwm_b", 0) > 0 else "OFF"
                    details = f"Suction {st}"
                elif stype == "relay":
                    details = f"Relay Control"
                elif stype == "delay":
                    details = f"Delay for {s['cmd']} ms"
                elif stype == "led":
                    st = "ON" if s.get("led", 0) > 0 else "OFF"
                    details = f"LED {st}"
                elif stype == "torque":
                    en = "Enabled" if s.get("cmd", 0) == 1 else "Disabled"
                    details = f"Torque {en}"
                elif stype == "dynamic_adaptation":
                    m = "Enabled" if s.get("mode", 0) == 1 else "Disabled"
                    details = f"Dynamic Adaptation {m}"
                else:
                    details = f"Unknown Step Type: {s}"
                iid = f"step_{idx+1}"
                self.mission_steps_tree.insert("", "end", iid=iid, values=(step_num, details), tags=(stype,))
        for tp, clr in STEP_TYPE_COLORS.items():
            self.mission_steps_tree.tag_configure(tp, background=clr)
        style = ttk.Style()
        style.configure("Treeview", rowheight=25)
        style.map("Treeview", background=[('selected', '#347083')], foreground=[('selected', 'white')])

    def create_bottom_buttons(self, parent, row, column):
        frame = ctk.CTkFrame(parent, corner_radius=6)
        frame.grid(row=row, column=column, padx=5, pady=10, sticky="ew")

        ctk.CTkButton(frame, text="Update Position", command=self.update_position_once,
                      width=120, height=40, font=("Arial", 12))\
            .pack(side="left", padx=10, pady=5, expand=True, fill="x")
        ctk.CTkButton(frame, text="Init", command=self.init_robot,
                      width=120, height=40, font=("Arial", 12))\
            .pack(side="left", padx=10, pady=5, expand=True, fill="x")
        ctk.CTkButton(frame, text="Close", command=self.close_connection,
                      width=120, height=40, font=("Arial", 12))\
            .pack(side="left", padx=10, pady=5, expand=True, fill="x")

    def create_status_bar(self, parent, row, column):
        sf = ctk.CTkFrame(parent, corner_radius=6)
        sf.grid(row=row, column=column, padx=5, pady=5, sticky="ew")
        parent.rowconfigure(row, weight=0)
        parent.columnconfigure(column, weight=1)
        self.status_var = tk.StringVar()
        lab = ctk.CTkLabel(sf, textvariable=self.status_var, anchor="w", font=("Arial", 10))
        lab.pack(fill="both", expand=True)

    def update_status_bar(self, message):
        self.status_var.set(message)
        self.logger.info(message)

    def queue_gui_update(self, func):
        """Adds a GUI update function to the queue."""
        self.gui_queue.put(func)

    def process_gui_queue(self):
        """Processes all pending GUI update functions."""
        try:
            while True:
                func = self.gui_queue.get_nowait()
                func()
        except Empty:
            pass
        finally:
            self.parent.after(100, self.process_gui_queue)

    def create_mission(self):
        name = simpledialog.askstring("Create Mission", "Name:", parent=self.parent)
        if name:
            if name in self.missions:
                self.show_error("Error", f"Mission '{name}' already exists.")
                return
            self.missions[name] = {"name": name, "intro": "Mission", "steps": []}
            self.selected_mission = name
            self.update_mission_steps_viewer()
            self.logger.info(f"Mission '{name}' created.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Mission '{name}' created."))

    def save_mission(self):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return
        file_path = filedialog.asksaveasfilename(defaultextension=".mission", filetypes=[("Mission Files", "*.mission")], parent=self.parent)
        if file_path:
            try:
                data = self.missions[self.selected_mission]
                with open(file_path, 'w') as f:
                    header = {"name": data["name"], "intro": "Mission"}
                    f.write(json.dumps(header, separators=(',', ':')) + "\n")
                    for s in data["steps"]:
                        f.write(json.dumps(s, separators=(',', ':')) + "\n")
                cmd = {"T": 221, "name": self.selected_mission}
                self.send_command_in_thread(cmd)
                self.logger.info(f"Mission '{self.selected_mission}' saved to {file_path}.")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Mission '{self.selected_mission}' saved to {file_path}."))  # Removed extra ')'
            except Exception as e:
                self.show_error("Error", f"Failed to save mission: {e}")
                self.logger.error(f"Failed to save mission: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to save mission: {e}."))
    
    def load_mission(self):
        file_path = filedialog.askopenfilename(defaultextension=".mission", filetypes=[("Mission Files", "*.mission")], parent=self.parent)
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                header = json.loads(lines[0].strip())
                name = header.get("name", "unnamed_mission")
                steps = [json.loads(l.strip()) for l in lines[1:]]
                if name in self.missions:
                    ow = messagebox.askyesno("Overwrite Mission", f"Mission '{name}' already exists. Overwrite?", parent=self.parent)
                    if not ow:
                        return
                self.missions[name] = {"name": name, "intro": header.get("intro", "Mission"), "steps": steps}
                self.selected_mission = name
                self.update_mission_steps_viewer()
                self.logger.info(f"Mission '{name}' loaded from {file_path}.")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Mission '{name}' loaded from {file_path}."))  # Removed extra ')'
            except Exception as e:
                self.show_error("Error", f"Failed to load mission: {e}")
                self.logger.error(f"Failed to load mission: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to load mission: {e}."))
    
    def play_mission(self):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return
        times = simpledialog.askinteger("Play Mission", "Enter number of repetitions:", initialvalue=1, parent=self.parent)
        if not times or times < 1:
            return
        self.play_mission_specific(times)

    def close_connection(self):
        # Signal the serial listener to stop
        self.serial_listener_stop_event.set()
        self.logger.info("Stop event set for serial listener.")

        # Wait briefly to allow the listener thread to exit
        if hasattr(self, 'serial_listener_thread') and self.serial_listener_thread.is_alive():
            self.serial_listener_thread.join(timeout=2)
            self.logger.info("Serial listener thread joined.")
        else:
            self.logger.info("Serial listener thread already stopped or was never started.")

        # Proceed to stop other components
        self.stop_continuous_move()
        self.stop_camera()

        # Close the serial connection
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.logger.info("Serial connection closed.")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Serial connection closed."))
            except Exception as e:
                self.show_error("Error", f"Error closing serial connection: {e}")
                self.logger.error(f"Error closing serial connection: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Error closing serial connection: {e}."))
        else:
            self.logger.info("Serial connection already closed or was never opened.")

    def play_mission_specific(self, times):
        if not self.selected_mission:
            self.show_error("Error", f"No mission selected for {self.robot_name}.")
            return

        def run_mission():
            try:
                total_steps = len(self.missions[self.selected_mission]["steps"]) * times
                executed_steps = 0
                for _ in range(times):
                    for stp in self.missions[self.selected_mission]["steps"]:
                        if self.mission_execution_stop_event.is_set():
                            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Mission execution stopped."))
                            return
                        self.execute_step(stp)
                        executed_steps += 1
                        prog = (executed_steps / total_steps) * 100
                        self.queue_gui_update(lambda p=prog: self.update_progress_bar(p))
                        # Movement commands are handled with sleep in execute_step
                self.queue_gui_update(lambda: self.update_progress_bar(100))
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Mission '{self.selected_mission}' completed."))
                self.logger.info(f"Mission '{self.selected_mission}' completed.")
            except Exception as e:
                self.show_error("Error", f"Mission execution failed: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Mission execution failed: {e}."))
                self.logger.error(f"Mission execution failed: {e}.")

        self.queue_gui_update(lambda: self.update_progress_bar(0))
        self.mission_execution_stop_event.clear()
        self.mission_execution_thread = threading.Thread(target=run_mission, daemon=True)
        self.mission_execution_thread.start()
        self.logger.info(f"Playing mission '{self.selected_mission}' for {times} times.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Playing mission '{self.selected_mission}' for {times} times."))

    def update_progress_bar(self, value):
        def update():
            self.progress_var.set(value)
        try:
            self.parent.after(0, update)
        except RuntimeError:
            pass

    def send_command_in_thread(self, command):
        def send():
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()  # Clear input buffer before sending
                    cmd_str = json.dumps(command) + "\n"  # Ensure proper termination
                    self.ser.write(cmd_str.encode('utf-8'))
                    self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Command sent: {command}"))
                    self.logger.info(f"Command sent: {command}")
                except Exception as e:
                    self.show_error("Serial Error", f"Failed to send command: {e}")
                    self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to send command: {e}."))
                    self.logger.error(f"Error sending command: {e}")
            else:
                self.show_error("Serial Error", "Serial not open.")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Serial not open."))
                self.logger.warning("Attempted to send command while serial port is not open.")
        threading.Thread(target=send, daemon=True).start()

    def execute_step(self, step):
        if step["T"] == 104:
            # Calculate the distance to move
            dx = abs(self.current_position["x"] - step["x"])
            dy = abs(self.current_position["y"] - step["y"])
            dz = abs(self.current_position["z"] - step["z"])
            mx = max(dx, dy, dz)
            
            # Calculate the time required for movement
            spd = step.get("spd", self.default_spd)
            tc = (mx / self.distance_scale) * (self.speed_scale / spd) + self.base_delay
            tc = min(tc, self.max_delay)  # Ensure tc does not exceed max_delay
            
            # Send the movement command
            self.send_command_in_thread(step)
            self.logger.info(f"Executed movement command: {step}")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Movement command executed."))
            
            # Sleep to allow the robot to execute the movement
            time.sleep(tc)
            
            # Update the current position
            with self.movement_lock:
                self.current_position.update({
                    "x": step["x"],
                    "y": step["y"],
                    "z": step["z"],
                    "t": step["t"]
                })
                self.logger.info(f"Position updated to: {self.current_position}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Position updated to: {self.current_position}."))
            
        elif step["T"] == 111:
            # Delay command
            delay_ms = step.get("cmd", 1000)
            self.logger.info(f"Executing delay for {delay_ms} ms.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Executing delay for {delay_ms} ms."))
            time.sleep(delay_ms / 1000.0)
            self.logger.info(f"Delay of {delay_ms} ms completed.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Delay of {delay_ms} ms completed."))
            
        elif step["T"] in [113, 114, 210, 112]:
            # Other commands (suction, relay, torque, dynamic adaptation)
            self.send_command_in_thread(step)
            self.logger.info(f"Executed command: {step}")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Command executed."))
            
        else:
            self.logger.warning(f"Unknown Step Type: {step}")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Unknown Step Type: {step}."))
    
    def update_position_once(self):
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            fb_cmd = {"T": 105}
            try:
                cmd_str = json.dumps(fb_cmd) + "\n"
                self.ser.write(cmd_str.encode('utf-8'))
                self.logger.info("Sent position update command.")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Position update command sent."))
                # The response will be handled by the serial listener
            except Exception as e:
                self.show_error("Serial Error", f"Failed to send position update command: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to send position update command: {e}."))
                self.logger.error(f"Failed to send position update command: {e}")
        else:
            self.show_error("Serial Error", "Serial not open.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Serial not open."))
            self.logger.warning("Attempted to update position while serial port is not open.")

    def start_continuous_move(self, axis, delta):
        self.stop_continuous_move()
        self.moving_stop_event.clear()
        self.moving_thread = threading.Thread(target=self.continuous_move_loop, args=(axis, delta), daemon=True)
        self.moving_thread.start()
        self.logger.info(f"Started continuous move on {axis}-axis with delta {delta}.")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Started continuous move on {axis}-axis."))

    def stop_continuous_move(self):
        if self.moving_thread and self.moving_thread.is_alive():
            self.moving_stop_event.set()
            self.moving_thread.join()
            self.logger.info("Stopped continuous move.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Stopped continuous move."))
        self.moving_thread = None

    def continuous_move_loop(self, axis, delta):
        while not self.moving_stop_event.is_set():
            with self.movement_lock:
                self.current_position[axis] += delta
                cmd = {
                    "T": 104,
                    "x": self.current_position["x"],
                    "y": self.current_position["y"],
                    "z": self.current_position["z"],
                    "t": self.current_position["t"],
                    "spd": self.default_spd,
                    "acc": self.default_acc
                }
                self.send_command_in_thread(cmd)
                self.logger.info(f"Continuous move command sent: {cmd}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Moving {axis} by {delta}."))
            time.sleep(self.update_interval)

    def init_robot(self):
        """Resets the robot to its startup position."""
        startup_position = {"x": 235.0, "y": 0.0, "z": 234.0, "t": 3.14}
        self.current_position = startup_position.copy()
        cmd = {
            "T": 104,
            "x": self.current_position["x"],
            "y": self.current_position["y"],
            "z": self.current_position["z"],
            "t": self.current_position["t"],
            "spd": self.default_spd,
            "acc": self.default_acc
        }
        self.send_command_in_thread(cmd)
        self.logger.info(f"Robot initialized to startup position: {self.current_position}")
        self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Robot initialized to startup position."))

        if self.selected_mission:
            self.missions[self.selected_mission]["steps"].append(cmd)
            self.update_mission_steps_viewer()
            self.logger.info("Initialization command added to mission.")

    def on_treeview_click(self, event):
        item = self.mission_steps_tree.identify_row(event.y)
        if item:
            self.dragging_item = item
            self.dragging_start_index = self.mission_steps_tree.index(item)
            self.logger.info(f"Dragging item: {item} at index {self.dragging_start_index}")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Dragging step {self.dragging_start_index + 1}."))
        else:
            self.dragging_item = None
            self.dragging_start_index = None
            self.logger.info("No item found at click position.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: No step selected for dragging."))

    def on_treeview_drag(self, event):
        if self.dragging_item:
            t_item = self.mission_steps_tree.identify_row(event.y)
            if t_item and t_item != self.dragging_item:
                self.mission_steps_tree.tag_remove("drag_target", *self.mission_steps_tree.tag_names())
                self.mission_steps_tree.item(t_item, tags=("drag_target",))
                self.mission_steps_tree.tag_configure("drag_target", background="#FFD700")
                self.logger.info(f"Hovering over item: {t_item}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Hovering over step {self.mission_steps_tree.index(t_item) + 1}."))
            else:
                self.mission_steps_tree.tag_remove("drag_target", *self.mission_steps_tree.tag_names())
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Dragging step."))

    def on_treeview_drop(self, event):
        if self.dragging_item:
            try:
                drop_item = self.mission_steps_tree.identify_row(event.y)
                if not drop_item:
                    drop_index = len(self.missions[self.selected_mission]["steps"])
                else:
                    drop_index = self.mission_steps_tree.index(drop_item)
                start_index = self.dragging_start_index
                if drop_index > start_index:
                    drop_index -= 1
                stp = self.missions[self.selected_mission]["steps"].pop(start_index)
                self.missions[self.selected_mission]["steps"].insert(drop_index, stp)
                self.update_mission_steps_viewer()
                new_iid = f"step_{drop_index + 1}"
                self.mission_steps_tree.selection_set(new_iid)
                self.mission_steps_tree.focus(new_iid)
                self.mission_steps_tree.see(new_iid)
                self.logger.info(f"Step moved from {start_index + 1} to {drop_index + 1}.")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Step moved from {start_index + 1} to {drop_index + 1}."))
            except Exception as e:
                self.show_error("Error", f"Failed to move step: {e}")
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Failed to move step: {e}."))
            finally:
                self.dragging_item = None
                self.dragging_start_index = None
                self.mission_steps_tree.tag_remove("drag_target", *self.mission_steps_tree.tag_names())
                self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Drag-and-drop operation completed."))

    def show_error(self, title, message):
        def show():
            messagebox.showerror(title, message, parent=self.parent)
        self.queue_gui_update(show)
        self.logger.error(f"{title}: {message}")

    def stop_continuous_move(self):
        if self.moving_thread and self.moving_thread.is_alive():
            self.moving_stop_event.set()
            self.moving_thread.join()
            self.logger.info("Stopped continuous move.")
            self.queue_gui_update(lambda: self.update_status_bar(f"{self.robot_name}: Stopped continuous move."))
        self.moving_thread = None
