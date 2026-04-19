#!/usr/bin/env python3
"""
TidyBot Gazebo Controller
=========================
Keyboard and GUI controller for base motion and arm joints.

Run with:
    ros2 run drift_tidybot arm.py
    or: python3 arm.py

Controls:
    W / Up Arrow      -> Forward
    S / Down Arrow    -> Backward
    A / Left Arrow    -> Turn Left
    D / Right Arrow   -> Turn Right
    Space             -> Emergency stop

Arm controls:
    Shoulder, elbow, wrist, and gripper sliders publish directly to
    /arm_controller/commands.
"""

import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Drive tuning
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1.0
PUB_RATE_HZ = 20


class TidybotNode(Node):
    """ROS 2 node that publishes base and arm commands."""

    def __init__(self):
        super().__init__('tidybot_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10
        )
        self._keys = set()
        self.create_timer(1.0 / PUB_RATE_HZ, self._publish_twist)

    def set_keys(self, keys):
        self._keys = keys

    def _publish_twist(self):
        """Publish drive commands from currently pressed keys."""
        twist = Twist()
        keys = self._keys

        if 'a' in keys or 'left' in keys:
            twist.linear.x = -LINEAR_SPEED
        if 'd' in keys or 'right' in keys:
            twist.linear.x = LINEAR_SPEED
        if 'w' in keys or 'up' in keys:
            twist.angular.z = -ANGULAR_SPEED
        if 's' in keys or 'down' in keys:
            twist.angular.z = ANGULAR_SPEED

        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def send_arm(self, shoulder, elbow, wrist, gripper):
        """Publish arm joint targets for both visible arms."""
        msg = Float64MultiArray()
        msg.data = [
            shoulder,
            elbow,
            wrist,
            gripper,
            gripper,
            gripper,
            -gripper,
            shoulder,
            elbow,
            wrist,
            gripper,
            gripper,
            gripper,
            -gripper,
        ]
        self.arm_pub.publish(msg)


class TidybotGUI:
    """Tkinter GUI for drive and arm control."""

    BG = "#0d0d0f"
    PANEL = "#14141a"
    ACCENT = "#00e5ff"
    ACCENT2 = "#ff6b35"
    TEXT = "#e8e8f0"
    MUTED = "#555568"
    BTN_DRIVE = "#1a2a1a"
    BTN_PRESS = "#00c853"

    def __init__(self, node: TidybotNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("TidyBot Controller")
        self.root.configure(bg=self.BG)
        self.root.resizable(False, False)

        self._pressed = set()
        self._btn_refs = {}

        self._shoulder = tk.DoubleVar(value=0.0)
        self._elbow = tk.DoubleVar(value=0.0)
        self._wrist = tk.DoubleVar(value=0.0)
        self._gripper = tk.DoubleVar(value=0.0)

        self._build_ui()
        self._bind_keys()

    def _build_ui(self):
        root = self.root

        header = tk.Frame(root, bg=self.BG)
        header.pack(fill='x', padx=20, pady=(18, 4))
        tk.Label(
            header,
            text="TIDYBOT",
            font=("Courier New", 22, "bold"),
            bg=self.BG,
            fg=self.ACCENT,
        ).pack(side='left')
        tk.Label(
            header,
            text="GAZEBO CONTROLLER",
            font=("Courier New", 9),
            bg=self.BG,
            fg=self.MUTED,
        ).pack(side='left', padx=(10, 0), anchor='s', pady=(0, 4))

        tk.Frame(root, bg=self.ACCENT, height=1).pack(fill='x', padx=20, pady=(0, 12))

        columns = tk.Frame(root, bg=self.BG)
        columns.pack(padx=20, fill='both')

        self._build_drive_panel(columns)
        tk.Frame(columns, bg=self.MUTED, width=1).pack(side='left', fill='y', padx=12)
        self._build_arm_panel(columns)

        tk.Frame(root, bg=self.MUTED, height=1).pack(fill='x', padx=20, pady=(12, 0))
        self._status_var = tk.StringVar(value="● IDLE")
        tk.Label(
            root,
            textvariable=self._status_var,
            font=("Courier New", 9),
            bg=self.BG,
            fg=self.MUTED,
        ).pack(anchor='w', padx=22, pady=(4, 14))

    def _build_drive_panel(self, parent):
        panel = tk.Frame(parent, bg=self.BG)
        panel.pack(side='left', anchor='n')

        tk.Label(
            panel,
            text="DRIVE",
            font=("Courier New", 11, "bold"),
            bg=self.BG,
            fg=self.ACCENT2,
        ).pack(anchor='w', pady=(0, 10))

        self._speed_var = tk.StringVar(value="STOP")
        tk.Label(
            panel,
            textvariable=self._speed_var,
            font=("Courier New", 13, "bold"),
            bg=self.PANEL,
            fg=self.ACCENT,
            width=10,
            pady=6,
        ).pack(pady=(0, 12))

        dpad = tk.Frame(panel, bg=self.BG)
        dpad.pack()

        def button(text, key, row, column):
            widget = tk.Button(
                dpad,
                text=text,
                width=4,
                height=2,
                font=("Courier New", 14, "bold"),
                bg=self.BTN_DRIVE,
                fg=self.TEXT,
                activebackground=self.BTN_PRESS,
                relief='flat',
                bd=0,
                cursor='hand2',
            )
            widget.grid(row=row, column=column, padx=3, pady=3, ipadx=4)
            widget.bind('<ButtonPress-1>', lambda _e, k=key: self._key_down(k))
            widget.bind('<ButtonRelease-1>', lambda _e, k=key: self._key_up(k))
            self._btn_refs[key] = widget

        button("▲", 'w', 0, 1)
        button("◄", 'a', 1, 0)
        button("■", 'space', 1, 1)
        button("►", 'd', 1, 2)
        button("▼", 's', 2, 1)

        help_box = tk.Frame(panel, bg=self.BG)
        help_box.pack(pady=(8, 0))
        for text, color in [
            ("W/S = Fwd/Back", self.MUTED),
            ("A/D = Turn L/R", self.MUTED),
            ("SPACE = Stop", self.ACCENT2),
        ]:
            tk.Label(help_box, text=text, font=("Courier New", 8), bg=self.BG, fg=color).pack(anchor='w')

        for label, var, low, high in [
            ("LINEAR SPEED", self._make_speed_var('lin', LINEAR_SPEED), 0.1, 2.0),
            ("ANGULAR SPEED", self._make_speed_var('ang', ANGULAR_SPEED), 0.1, 3.0),
        ]:
            tk.Label(panel, text=label, font=("Courier New", 8), bg=self.BG, fg=self.MUTED).pack(anchor='w', pady=(10, 2))
            tk.Scale(
                panel,
                variable=var,
                from_=low,
                to=high,
                resolution=0.05,
                orient='horizontal',
                length=160,
                bg=self.PANEL,
                fg=self.TEXT,
                troughcolor=self.BG,
                highlightthickness=0,
                activebackground=self.ACCENT,
                sliderlength=16,
                bd=0,
            ).pack()

        tk.Button(
            panel,
            text="⛔  E-STOP",
            font=("Courier New", 10, "bold"),
            bg="#3a0000",
            fg="#ff4444",
            activebackground="#660000",
            relief='flat',
            bd=0,
            cursor='hand2',
            command=self._estop,
            pady=6,
            padx=10,
        ).pack(fill='x', pady=(16, 0))

    def _make_speed_var(self, which, default):
        var = tk.DoubleVar(value=default)
        if which == 'lin':
            self._lin_speed = var
        else:
            self._ang_speed = var
        return var

    def _build_arm_panel(self, parent):
        panel = tk.Frame(parent, bg=self.BG)
        panel.pack(side='left', anchor='n')

        tk.Label(
            panel,
            text="ROBOTIC ARM",
            font=("Courier New", 11, "bold"),
            bg=self.BG,
            fg=self.ACCENT2,
        ).pack(anchor='w', pady=(0, 10))

        for label, var, low, high in [
            ("Shoulder", self._shoulder, -1.57, 1.57),
            ("Elbow", self._elbow, -1.57, 1.57),
            ("Wrist", self._wrist, -1.57, 1.57),
        ]:
            self._make_slider_row(panel, label, var, low, high)

        tk.Frame(panel, bg=self.MUTED, height=1).pack(fill='x', pady=(12, 8))

        tk.Label(
            panel,
            text="GRIPPER",
            font=("Courier New", 10, "bold"),
            bg=self.BG,
            fg=self.ACCENT,
        ).pack(anchor='w', pady=(0, 4))

        gripper_row = tk.Frame(panel, bg=self.BG)
        gripper_row.pack(fill='x', pady=3)

        left_frame = tk.Frame(gripper_row, bg=self.BG, width=120)
        left_frame.pack(side='left')
        left_frame.pack_propagate(False)

        tk.Label(
            left_frame,
            text="OPEN / CLOSE",
            font=("Courier New", 8, "bold"),
            bg=self.BG,
            fg=self.TEXT,
            anchor='w',
        ).pack(fill='x')

        self._grip_val_var = tk.StringVar(value="0.00")
        tk.Label(
            left_frame,
            textvariable=self._grip_val_var,
            font=("Courier New", 8),
            bg=self.BG,
            fg=self.ACCENT,
            anchor='w',
        ).pack(fill='x')

        tk.Scale(
            gripper_row,
            variable=self._gripper,
            from_=-1.57,
            to=1.57,
            resolution=0.01,
            orient='horizontal',
            length=220,
            bg=self.PANEL,
            fg=self.TEXT,
            troughcolor="#1a1a28",
            highlightthickness=0,
            activebackground=self.ACCENT,
            sliderlength=18,
            bd=0,
            showvalue=False,
            command=lambda value: (
                self._grip_val_var.set(f"{float(value):+.2f} rad"),
                self._send_arm(),
            ),
        ).pack(side='left', padx=(8, 0))

        hint = tk.Frame(panel, bg=self.BG)
        hint.pack(fill='x')
        tk.Label(hint, text="◄ CLOSE", font=("Courier New", 8), bg=self.BG, fg=self.MUTED).pack(side='left')
        tk.Label(hint, text="OPEN ►", font=("Courier New", 8), bg=self.BG, fg=self.MUTED).pack(side='right')

        tk.Label(panel, text="PRESETS", font=("Courier New", 8, "bold"), bg=self.BG, fg=self.MUTED).pack(anchor='w', pady=(14, 4))

        presets = tk.Frame(panel, bg=self.BG)
        presets.pack(anchor='w')

        for text, values in [
            ("HOME", (0.0, 0.0, 0.0, 0.0)),
            ("REACH", (0.0, -1.0, 0.8, 0.0)),
            ("GRIP", (0.0, -0.8, 0.6, 0.8)),
            ("STOW", (1.2, 1.2, -1.0, 0.0)),
        ]:
            tk.Button(
                presets,
                text=text,
                font=("Courier New", 9),
                bg=self.PANEL,
                fg=self.TEXT,
                activebackground=self.ACCENT,
                relief='flat',
                bd=0,
                cursor='hand2',
                padx=10,
                pady=4,
                command=lambda vals=values: self._apply_preset(vals),
            ).pack(side='left', padx=(0, 6))

        tk.Button(
            panel,
            text="↺  RESET ALL",
            font=("Courier New", 9),
            bg=self.PANEL,
            fg=self.MUTED,
            activebackground="#222230",
            relief='flat',
            bd=0,
            cursor='hand2',
            pady=4,
            command=lambda: self._apply_preset((0.0, 0.0, 0.0, 0.0)),
        ).pack(fill='x', pady=(8, 0))

    def _make_slider_row(self, parent, label, var, low, high):
        row = tk.Frame(parent, bg=self.BG)
        row.pack(fill='x', pady=3)

        left_frame = tk.Frame(row, bg=self.BG, width=120)
        left_frame.pack(side='left')
        left_frame.pack_propagate(False)

        tk.Label(
            left_frame,
            text=label.upper(),
            font=("Courier New", 8, "bold"),
            bg=self.BG,
            fg=self.TEXT,
            anchor='w',
        ).pack(fill='x')

        value_var = tk.StringVar(value="0.00")
        tk.Label(
            left_frame,
            textvariable=value_var,
            font=("Courier New", 8),
            bg=self.BG,
            fg=self.ACCENT,
            anchor='w',
        ).pack(fill='x')

        tk.Scale(
            row,
            variable=var,
            from_=low,
            to=high,
            resolution=0.01,
            orient='horizontal',
            length=220,
            bg=self.PANEL,
            fg=self.TEXT,
            troughcolor="#1a1a28",
            highlightthickness=0,
            activebackground=self.ACCENT,
            sliderlength=18,
            bd=0,
            showvalue=False,
            command=lambda value, vv=value_var: (
                vv.set(f"{float(value):+.2f} rad"),
                self._send_arm(),
            ),
        ).pack(side='left', padx=(8, 0))

    def _bind_keys(self):
        self.root.bind('<KeyPress>', self._on_key_press)
        self.root.bind('<KeyRelease>', self._on_key_release)
        self.root.focus_set()

    def _on_key_press(self, event):
        key = event.keysym.lower() if len(event.keysym) > 1 else event.char.lower()
        if key == 'space':
            self._estop()
            return
        self._key_down(key)

    def _on_key_release(self, event):
        key = event.keysym.lower() if len(event.keysym) > 1 else event.char.lower()
        self._key_up(key)

    def _key_down(self, key):
        global LINEAR_SPEED, ANGULAR_SPEED
        self._pressed.add(key)
        self._update_drive_display()
        self.node.set_keys(self._pressed)
        LINEAR_SPEED = self._lin_speed.get()
        ANGULAR_SPEED = self._ang_speed.get()

    def _key_up(self, key):
        self._pressed.discard(key)
        self._update_drive_display()
        self.node.set_keys(self._pressed)

    def _update_drive_display(self):
        keys = self._pressed
        if 'w' in keys or 'up' in keys:
            text, color = "FORWARD ▲", self.BTN_PRESS
        elif 's' in keys or 'down' in keys:
            text, color = "BACKWARD ▼", "#ff6b35"
        elif 'a' in keys or 'left' in keys:
            text, color = "◄ LEFT", self.ACCENT
        elif 'd' in keys or 'right' in keys:
            text, color = "RIGHT ►", self.ACCENT
        else:
            text, color = "STOP", self.MUTED

        self._speed_var.set(text)
        self._status_var.set(
            f"● {text}  |  lin={self._lin_speed.get():.2f}  ang={self._ang_speed.get():.2f}"
        )

    def _send_arm(self):
        self.node.send_arm(
            self._shoulder.get(),
            self._elbow.get(),
            self._wrist.get(),
            self._gripper.get(),
        )

    def _apply_preset(self, values):
        shoulder, elbow, wrist, gripper = values
        self._shoulder.set(shoulder)
        self._elbow.set(elbow)
        self._wrist.set(wrist)
        self._gripper.set(gripper)
        self._grip_val_var.set(f"{gripper:+.2f} rad")
        self._send_arm()

    def _estop(self):
        self._pressed.clear()
        self.node.stop()
        self._speed_var.set("STOP")
        self._status_var.set("⛔ E-STOP ENGAGED")

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.node.stop()
        self.root.destroy()


def main():
    rclpy.init()
    node = TidybotNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        gui = TidybotGUI(node)
        gui.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
