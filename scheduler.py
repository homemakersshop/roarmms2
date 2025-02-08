# scheduler.py

import customtkinter as ctk
from tkinter import messagebox, filedialog, simpledialog
import tkinter as tk  # Import tkinter for StringVar and other components
from tkinter import ttk
import threading
import json
import time
from datetime import datetime, timedelta  # Import timedelta for scheduling next occurrences
import os

class SchedulerFrame(ctk.CTkFrame):
    def __init__(self, parent, robot1, robot2):
        """
        Initializes the SchedulerFrame.

        Args:
            parent (ctk.CTkFrame): The parent frame (Scheduler tab).
            robot1 (RobotControlFrame): Instance of Robot 1's control frame.
            robot2 (RobotControlFrame): Instance of Robot 2's control frame.
        """
        super().__init__(parent, corner_radius=10)
        self.parent = parent
        self.robot1 = robot1
        self.robot2 = robot2

        # Scheduled tasks list
        self.scheduled_tasks = []

        self.create_widgets()

    def create_widgets(self):
        # Title
        title_label = ctk.CTkLabel(self, text="Mission Scheduler", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        # Frame for scheduling options
        options_frame = ctk.CTkFrame(self, corner_radius=8)
        options_frame.pack(padx=20, pady=10, fill="x")

        # Robot Selection
        robot_label = ctk.CTkLabel(options_frame, text="Select Robot:", font=("Arial", 12))
        robot_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")

        self.robot_var = tk.StringVar(value="Robot 1")
        robot_options = ["Robot 1", "Robot 2"]
        robot_dropdown = ctk.CTkOptionMenu(options_frame, variable=self.robot_var, values=robot_options)
        robot_dropdown.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        # Mission File Selection
        mission_label = ctk.CTkLabel(options_frame, text="Mission File:", font=("Arial", 12))
        mission_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")

        self.mission_path_var = tk.StringVar()
        mission_entry = ctk.CTkEntry(options_frame, textvariable=self.mission_path_var, width=300)
        mission_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        browse_button = ctk.CTkButton(options_frame, text="Browse", command=self.browse_mission_file)
        browse_button.grid(row=1, column=2, padx=5, pady=5, sticky="w")

        # Schedule Time Selection (Graphical Time Picker)
        time_label = ctk.CTkLabel(options_frame, text="Scheduled Time:", font=("Arial", 12))
        time_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")

        # Time Picker Frame
        time_picker_frame = ctk.CTkFrame(options_frame)
        time_picker_frame.grid(row=2, column=1, padx=5, pady=5, sticky="w")

        # Hours Dropdown (1-12)
        self.hour_var = tk.StringVar(value="12")
        hour_options = [f"{i:02d}" for i in range(1, 13)]
        hour_dropdown = ctk.CTkOptionMenu(time_picker_frame, variable=self.hour_var, values=hour_options, width=60)
        hour_dropdown.pack(side="left", padx=(0, 5))

        # Minutes Dropdown (00-59)
        self.minute_var = tk.StringVar(value="00")
        minute_options = [f"{i:02d}" for i in range(0, 60)]
        minute_dropdown = ctk.CTkOptionMenu(time_picker_frame, variable=self.minute_var, values=minute_options, width=60)
        minute_dropdown.pack(side="left", padx=(0, 5))

        # Seconds Dropdown (00-59)
        self.second_var = tk.StringVar(value="00")
        second_options = [f"{i:02d}" for i in range(0, 60)]
        second_dropdown = ctk.CTkOptionMenu(time_picker_frame, variable=self.second_var, values=second_options, width=60)
        second_dropdown.pack(side="left", padx=(0, 5))

        # AM/PM Dropdown
        self.am_pm_var = tk.StringVar(value="AM")
        am_pm_options = ["AM", "PM"]
        am_pm_dropdown = ctk.CTkOptionMenu(time_picker_frame, variable=self.am_pm_var, values=am_pm_options, width=60)
        am_pm_dropdown.pack(side="left", padx=(0, 5))

        # Recurrence Pattern Selection
        recurrence_label = ctk.CTkLabel(options_frame, text="Recurrence:", font=("Arial", 12))
        recurrence_label.grid(row=3, column=0, padx=5, pady=5, sticky="w")

        self.recurrence_var = tk.StringVar(value="Once")
        recurrence_options = ["Once", "Daily", "Weekly"]
        recurrence_dropdown = ctk.CTkOptionMenu(options_frame, variable=self.recurrence_var, values=recurrence_options, width=100)
        recurrence_dropdown.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        # Add to Schedule Button
        add_button = ctk.CTkButton(options_frame, text="Add to Schedule", command=self.add_to_schedule)
        add_button.grid(row=4, column=1, padx=5, pady=5, sticky="w")

        # Save and Load Schedule Buttons
        save_button = ctk.CTkButton(options_frame, text="Save Schedule", command=self.save_schedule)
        save_button.grid(row=4, column=2, padx=5, pady=5, sticky="w")

        load_button = ctk.CTkButton(options_frame, text="Load Schedule", command=self.load_schedule)
        load_button.grid(row=5, column=2, padx=5, pady=5, sticky="w")

        # Scheduled Tasks Treeview
        tasks_frame = ctk.CTkFrame(self, corner_radius=8)
        tasks_frame.pack(padx=20, pady=10, fill="both", expand=True)

        columns = ("Robot", "Mission File", "Scheduled Time", "Recurrence", "Status")
        self.tasks_tree = ttk.Treeview(tasks_frame, columns=columns, show="headings", selectmode="browse")
        for col in columns:
            self.tasks_tree.heading(col, text=col)
            self.tasks_tree.column(col, width=120, anchor="center")

        self.tasks_tree.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        # Scrollbar for Treeview
        tasks_scrollbar = ttk.Scrollbar(tasks_frame, orient="vertical", command=self.tasks_tree.yview)
        tasks_scrollbar.pack(side="right", fill="y")
        self.tasks_tree.config(yscrollcommand=tasks_scrollbar.set)

        # Context Menu for Deleting Tasks
        self.tasks_tree.bind("<Button-3>", self.show_context_menu)
        self.context_menu = tk.Menu(self, tearoff=0)
        self.context_menu.add_command(label="Delete Task", command=self.delete_selected_task)

    def browse_mission_file(self):
        file_path = filedialog.askopenfilename(
            title="Select Mission File",
            filetypes=[("Mission Files", "*.mission"), ("All Files", "*.*")]
        )
        if file_path:
            self.mission_path_var.set(file_path)

    def add_to_schedule(self):
        robot = self.robot_var.get()
        mission_file = self.mission_path_var.get()
        hour = self.hour_var.get()
        minute = self.minute_var.get()
        second = self.second_var.get()
        am_pm = self.am_pm_var.get()
        recurrence = self.recurrence_var.get()

        # Validate inputs
        if not mission_file or not os.path.isfile(mission_file):
            messagebox.showerror("Error", "Please select a valid mission file.")
            return

        # Construct time string
        scheduled_time_str = f"{hour}:{minute}:{second} {am_pm}"

        try:
            # Parse the scheduled time in 12-hour format
            scheduled_time = datetime.strptime(scheduled_time_str, "%I:%M:%S %p").time()
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid time.")
            return

        # Create a scheduled task dictionary
        task = {
            "robot": robot,
            "mission_file": mission_file,
            "scheduled_time": scheduled_time_str,
            "recurrence": recurrence,
            "status": "Pending"
        }

        # Insert into the Treeview
        self.tasks_tree.insert("", "end", values=(robot, os.path.basename(mission_file), scheduled_time_str, recurrence, "Pending"))

        # Add to the scheduled_tasks list
        self.scheduled_tasks.append(task)

        # Start a thread to monitor and execute the task
        threading.Thread(target=self.monitor_task, args=(task,), daemon=True).start()

        # Clear input fields
        self.mission_path_var.set("")
        self.hour_var.set("12")
        self.minute_var.set("00")
        self.second_var.set("00")
        self.am_pm_var.set("AM")
        self.recurrence_var.set("Once")

        print(f"Scheduled mission for {robot} at {scheduled_time_str} with recurrence '{recurrence}'.")

    def monitor_task(self, task):
        """
        Monitors the current time and executes the mission when the scheduled time is reached.
        Handles recurrence by rescheduling the task based on its recurrence pattern.

        Args:
            task (dict): The scheduled task dictionary.
        """
        while True:
            # Calculate the time difference
            now = datetime.now()
            try:
                scheduled_time = datetime.strptime(task["scheduled_time"], "%I:%M:%S %p").time()
            except ValueError:
                print(f"Invalid time format for task: {task}")
                self.update_task_status(task, "Invalid Time Format")
                return

            scheduled_datetime = datetime.combine(now.date(), scheduled_time)

            # If the scheduled time is earlier than now, adjust based on recurrence
            if scheduled_datetime < now:
                if task["recurrence"] == "Daily":
                    scheduled_datetime += timedelta(days=1)
                elif task["recurrence"] == "Weekly":
                    scheduled_datetime += timedelta(weeks=1)
                else:
                    # Once and time has passed
                    self.update_task_status(task, "Time Passed")
                    return

            time_to_wait = (scheduled_datetime - now).total_seconds()
            print(f"Task for {task['robot']} scheduled to run in {time_to_wait} seconds.")

            # Wait until the scheduled time
            if time_to_wait > 0:
                time.sleep(time_to_wait)

            # Update the task status to Running
            self.update_task_status(task, "Running")

            # Load the mission file
            try:
                with open(task["mission_file"], 'r') as f:
                    lines = f.readlines()
                header = json.loads(lines[0].strip())
                mission_name = header.get("name", "Unnamed Mission")
                print(f"Executing mission '{mission_name}' for {task['robot']}")
            except Exception as e:
                print(f"Failed to load mission file '{task['mission_file']}': {e}")
                self.update_task_status(task, "Failed to Load")
                return

            # Execute the mission
            if task["robot"] == "Robot 1":
                self.robot1.play_mission_specific(1)
            elif task["robot"] == "Robot 2":
                self.robot2.play_mission_specific(1)
            else:
                print(f"Unknown robot: {task['robot']}")
                self.update_task_status(task, "Unknown Robot")
                return

            # Update the task status to Completed
            self.update_task_status(task, "Completed")
            print(f"Mission '{mission_name}' for {task['robot']} completed.")

            # Handle recurrence
            if task["recurrence"] == "Once":
                break
            elif task["recurrence"] in ["Daily", "Weekly"]:
                # Reschedule the task
                self.update_task_status(task, "Pending")
                print(f"Rescheduling task for {task['robot']} with recurrence '{task['recurrence']}'.")

    def update_task_status(self, task, status):
        """
        Updates the status of a scheduled task in the Treeview.

        Args:
            task (dict): The scheduled task dictionary.
            status (str): The new status (e.g., "Running", "Completed").
        """
        for item in self.tasks_tree.get_children():
            values = self.tasks_tree.item(item, "values")
            if (values[0] == task["robot"] and
                values[1] == os.path.basename(task["mission_file"]) and
                values[2] == task["scheduled_time"] and
                values[3] == task["recurrence"]):
                self.tasks_tree.set(item, column="Status", value=status)
                break

    def show_context_menu(self, event):
        """
        Displays the context menu on right-click.

        Args:
            event: The Tkinter event object.
        """
        selected_item = self.tasks_tree.identify_row(event.y)
        if selected_item:
            self.tasks_tree.selection_set(selected_item)
            self.context_menu.post(event.x_root, event.y_root)

    def delete_selected_task(self):
        """
        Deletes the selected task from the schedule.
        """
        selected_item = self.tasks_tree.selection()
        if not selected_item:
            messagebox.showerror("Error", "No task selected.")
            return
        values = self.tasks_tree.item(selected_item, "values")
        robot, mission_file, scheduled_time, recurrence, status = values

        if status == "Running":
            messagebox.showerror("Error", "Cannot delete a running task.")
            return

        # Remove from scheduled_tasks list
        for task in self.scheduled_tasks:
            if (task["robot"] == robot and
                os.path.basename(task["mission_file"]) == mission_file and
                task["scheduled_time"] == scheduled_time and
                task["recurrence"] == recurrence):
                self.scheduled_tasks.remove(task)
                break

        # Remove from Treeview
        self.tasks_tree.delete(selected_item)
        print(f"Deleted scheduled task for {robot} at {scheduled_time} with recurrence '{recurrence}'.")

    def save_schedule(self):
        """
        Saves the current schedule to a JSON file.
        """
        if not self.scheduled_tasks:
            messagebox.showinfo("Info", "No scheduled tasks to save.")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json")],
            title="Save Schedule",
            initialfile="schedule.json",
            parent=self.parent
        )

        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(self.scheduled_tasks, f, indent=4)
                messagebox.showinfo("Success", f"Schedule saved to {file_path}")
                print(f"Schedule saved to {file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save schedule: {e}")
                print(f"Failed to save schedule: {e}")

    def load_schedule(self):
        """
        Loads a schedule from a JSON file.
        """
        file_path = filedialog.askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json")],
            title="Load Schedule",
            parent=self.parent
        )

        if file_path:
            try:
                with open(file_path, 'r') as f:
                    loaded_tasks = json.load(f)

                # Clear existing tasks
                self.tasks_tree.delete(*self.tasks_tree.get_children())
                self.scheduled_tasks = []

                for task in loaded_tasks:
                    robot = task.get("robot", "Robot 1")
                    mission_file = task.get("mission_file", "")
                    scheduled_time_str = task.get("scheduled_time", "12:00:00 AM")
                    recurrence = task.get("recurrence", "Once")
                    status = task.get("status", "Pending")

                    # Insert into Treeview
                    self.tasks_tree.insert("", "end", values=(robot, os.path.basename(mission_file), scheduled_time_str, recurrence, status))

                    # Add to scheduled_tasks list
                    self.scheduled_tasks.append(task)

                    # If the task was pending, start monitoring it
                    if status == "Pending":
                        threading.Thread(target=self.monitor_task, args=(task,), daemon=True).start()

                messagebox.showinfo("Success", f"Schedule loaded from {file_path}")
                print(f"Schedule loaded from {file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load schedule: {e}")
                print(f"Failed to load schedule: {e}")
