# main.py

import customtkinter as ctk
from multi_robot_app import MultiRobotApp

if __name__ == "__main__":
    root = ctk.CTk()
    app = MultiRobotApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close_all_connections)
    root.mainloop()
