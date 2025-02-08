# multi_robot_app.py

import customtkinter as ctk
from tkinter import messagebox, simpledialog
import threading

from robot_control_frame import RobotControlFrame  # Import the RobotControlFrame class
from scheduler import SchedulerFrame               # Import the SchedulerFrame class
from constants import STEP_TYPE_COLORS            # Import shared constants if needed


class MultiRobotApp:
    def __init__(self, master):
        """
        Initializes the MultiRobotApp.

        Args:
            master (ctk.CTk): The main application window.
        """
        print("[DEBUG] Initializing MultiRobotApp")
        self.master = master
        self.master.title("RoArm M2 IK Control - Multi-Robot Control")
        self.master.geometry("1600x1200")  # Increased window size for better fit
        self.master.minsize(1400, 1000)    # Set a minimum window size

        # Set theme
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        # Create a Scrollable Frame to hold all content
        self.scrollable_frame = ctk.CTkScrollableFrame(self.master, width=1600, height=1200)
        self.scrollable_frame.pack(expand=True, fill="both", padx=10, pady=10)
        self.scrollable_frame.columnconfigure(0, weight=1)
        self.scrollable_frame.rowconfigure(0, weight=1)

        # Create a TabView inside the Scrollable Frame
        self.tabview = ctk.CTkTabview(self.scrollable_frame)
        self.tabview.grid(row=0, column=0, sticky="nsew")

        # Add tabs
        self.tab1 = self.tabview.add("Robot 1")
        self.tab2 = self.tabview.add("Robot 2")
        self.tab3 = self.tabview.add("Global Controls")
        self.tab4 = self.tabview.add("Scheduler")  # New Scheduler Tab

        # Initialize RobotControlFrame instances within their respective tabs
        self.robot1 = self.create_robot_frame(self.tabview.tab("Robot 1"), "Robot 1", "COM9")
        self.robot2 = self.create_robot_frame(self.tabview.tab("Robot 2"), "Robot 2", "COM10")

        # Initialize Global Mission Controls within the "Global Controls" tab
        self.create_global_mission_controls(self.tabview.tab("Global Controls"))

        # Initialize SchedulerFrame within the "Scheduler" tab
        self.scheduler = self.create_scheduler_frame(self.tabview.tab("Scheduler"))

        print("[DEBUG] MultiRobotApp initialized successfully")

    def create_robot_frame(self, parent, robot_name, com_port):
        """
        Creates a RobotControlFrame within the specified parent frame.

        Args:
            parent (ctk.CTkFrame): The parent frame (a tab in TabView).
            robot_name (str): Name of the robot (e.g., "Robot 1").
            com_port (str): COM port for serial communication.

        Returns:
            RobotControlFrame: The created RobotControlFrame instance.
        """
        print(f"[DEBUG] Creating RobotControlFrame for {robot_name} on {com_port}")
        robot = RobotControlFrame(parent, com_port, robot_name)
        print(f"[DEBUG] RobotControlFrame for {robot_name} created successfully")
        return robot

    def create_scheduler_frame(self, parent):
        """
        Creates the SchedulerFrame within the specified parent frame.

        Args:
            parent (ctk.CTkFrame): The parent frame (Scheduler tab).

        Returns:
            SchedulerFrame: The created SchedulerFrame instance.
        """
        print("[DEBUG] Creating SchedulerFrame")
        scheduler = SchedulerFrame(parent, self.robot1, self.robot2)
        scheduler.pack(padx=10, pady=10, fill="both", expand=True)
        print("[DEBUG] SchedulerFrame created successfully")
        return scheduler

    ##################################################################
    #                Global Mission Controls                          #
    ##################################################################
    def create_global_mission_controls(self, parent):
        print("[DEBUG] Creating Global Mission Controls")  # Debugging statement
        frame = ctk.CTkFrame(parent, corner_radius=10)
        frame.pack(padx=10, pady=10, fill="both", expand=True)

        title_label = ctk.CTkLabel(frame, text="Global Mission Controls", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)

        # Play Both Missions Button
        play_both_button = ctk.CTkButton(
            frame,
            text="Play Both Missions",
            command=self.play_both_missions,
            width=300,
            height=60,
            font=("Arial", 16)
        )
        play_both_button.pack(padx=20, pady=20, fill="x")

    def play_both_missions(self):
        """
        Handles the "Play Both Missions" button click.
        Executes missions for both robots concurrently.
        """
        # Check if both robots have a selected mission
        if not self.robot1.selected_mission or not self.robot2.selected_mission:
            messagebox.showerror("Error", "Both robots must have a mission selected.")
            print("[ERROR] Both robots must have a mission selected.")
            return

        # Ask for repetitions once for both missions
        times = simpledialog.askinteger("Play Both Missions", "Enter number of repetitions:", initialvalue=1, parent=self.master)
        if times is None or times < 1:
            print("[DEBUG] Invalid number of repetitions entered.")
            return

        # Define a function to play both missions concurrently
        def play_both():
            # Start playing Robot 1's mission
            def play_robot1():
                print("[DEBUG] Starting Robot 1's mission")
                self.robot1.play_mission_specific(times)

            # Start playing Robot 2's mission
            def play_robot2():
                print("[DEBUG] Starting Robot 2's mission")
                self.robot2.play_mission_specific(times)

            # Create threads for both missions
            thread1 = threading.Thread(target=play_robot1, daemon=True)
            thread2 = threading.Thread(target=play_robot2, daemon=True)

            # Start both threads
            thread1.start()
            thread2.start()

            print("Both missions have been started.")

        # Start the play_both function in a new thread to prevent blocking
        threading.Thread(target=play_both, daemon=True).start()
        print("play_both_missions function executed.")

    def close_all_connections(self):
        """
        Handles the window close event by closing all serial connections.
        """
        print("[DEBUG] Closing all serial connections")
        # Close connections for both robots
        self.robot1.close_connection()
        self.robot2.close_connection()
        self.master.quit()
        print("[DEBUG] All serial connections closed and application exited")
