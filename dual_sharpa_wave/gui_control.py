"""Small Tk slider panel; commands start only after feedback and an explicit click."""

import argparse
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args

from .control_model import SIDES, bounded_positions, load_limits, positive
from .control_ros import CommandClient
from .joint_names import JOINT_SUFFIXES


class ControlPanel:
    def __init__(self, root, node, rate=30.0):
        self.root, self.node = root, node
        self.period = 1.0 / positive(rate, 'rate')
        self.next_tick = 0.0
        self.targets = {}
        self.active = {s: False for s in SIDES}
        self.scales, self.actual, self.status, self.start_buttons = {}, {}, {}, {}
        self.closed = False
        self.after_id = None
        root.title('Sharpa Wave joint control - radians')
        root.geometry('1120x820')
        ttk.Label(root, text='Launch the hands, then click Start Sending. Stop Sending is not an emergency stop.').pack(pady=8)
        columns = ttk.Frame(root)
        columns.pack(fill='both', expand=True)
        for side, title in (('left', 'Left Hand'), ('right', 'Right Hand')):
            outer = ttk.LabelFrame(columns, text=title)
            outer.pack(side='left', fill='both', expand=True, padx=5, pady=5)
            self.status[side] = tk.StringVar(value='Waiting for feedback and hand connection')
            ttk.Label(outer, textvariable=self.status[side], wraplength=480).pack(fill='x')
            buttons = ttk.Frame(outer)
            buttons.pack(fill='x')
            start = ttk.Button(buttons, text='Start Sending', command=lambda s=side: self.start(s))
            start.pack(side='left')
            start.state(['disabled'])
            self.start_buttons[side] = start
            ttk.Button(buttons, text='Stop Sending', command=lambda s=side: self.stop(s)).pack(side='left')
            ttk.Button(buttons, text='Load Current Pose', command=lambda s=side: self.reload(s)).pack(side='left')
            container = ttk.Frame(outer)
            container.pack(fill='both', expand=True)
            canvas = tk.Canvas(container, highlightthickness=0)
            scroll = ttk.Scrollbar(container, orient='vertical', command=canvas.yview)
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side='right', fill='y')
            canvas.pack(side='left', fill='both', expand=True)
            rows = ttk.Frame(canvas)
            window = canvas.create_window((0, 0), window=rows, anchor='nw')
            rows.bind('<Configure>', lambda event, c=canvas: c.configure(scrollregion=c.bbox('all')))
            canvas.bind('<Configure>', lambda event, c=canvas, w=window: c.itemconfigure(w, width=event.width))
            self.scales[side], self.actual[side] = [], []
            for index, (name, (lo, hi)) in enumerate(zip(JOINT_SUFFIXES, node.limits[side])):
                row = ttk.Frame(rows)
                row.pack(fill='x', padx=6, pady=3)
                text = tk.StringVar(value=f'{name}  |  actual: -- rad')
                ttk.Label(row, textvariable=text).pack(anchor='w')
                slider = tk.Scale(row, from_=lo, to=hi, orient='horizontal', resolution=-1, digits=10,
                                  length=430, state='disabled',
                                  command=lambda value, s=side, i=index: self.move(s, i, value))
                slider.pack(fill='x')
                self.scales[side].append(slider)
                self.actual[side].append(text)
        root.protocol('WM_DELETE_WINDOW', self.close)

    def move(self, side, index, value):
        if side in self.targets:
            self.targets[side][index] = float(value)

    def reload(self, side):
        self.stop(side)
        try:
            self.node.require_ready((side,))
            positions = bounded_positions(side, self.node.positions[side], self.node.limits)
            # Scale resolution must not round untouched feedback targets.
            self.targets.pop(side, None)
            for slider, value in zip(self.scales[side], positions):
                slider.configure(state='normal')
                slider.set(value)
            self.root.update_idletasks()
            self.targets[side] = positions.copy()
            self.status[side].set('Current pose loaded; not sending')
        except (ValueError, RuntimeError) as error:
            self.status[side].set(str(error))

    def start(self, side):
        try:
            self.node.require_ready((side,))
            if side not in self.targets:
                raise RuntimeError('Load the current pose first')
            bounded_positions(side, self.targets[side], self.node.limits)
            self.active[side] = True
            self.status[side].set('Sending target joint angles')
        except (ValueError, RuntimeError) as error:
            self.status[side].set(str(error))

    def stop(self, side):
        self.active[side] = False
        self.status[side].set('Sending stopped; last target is unchanged')

    def tick(self):
        if self.closed:
            return
        if not rclpy.ok():
            self.close()
            return
        try:
            # Bound event processing so a busy ROS graph cannot freeze Tk.
            for _ in range(4):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            for side in SIDES:
                ready = self.node.ready(side)
                if not ready:
                    self.active[side] = False
                    self.status[side].set('Waiting for valid feedback and hand connection; sending disabled')
                elif side not in self.targets:
                    self.reload(side)
                self.start_buttons[side].state(['!disabled'] if ready and side in self.targets else ['disabled'])
                for slider in self.scales[side]:
                    slider.configure(state='normal' if ready and side in self.targets else 'disabled')
                if side in self.node.positions:
                    for name, value, label in zip(JOINT_SUFFIXES, self.node.positions[side], self.actual[side]):
                        label.set(f'{name}  |  actual: {value:.4f} rad')
            now = time.monotonic()
            if now >= self.next_tick:
                targets = {s: self.targets[s] for s in SIDES if self.active[s]}
                if targets:
                    self.node.send(targets)
                self.next_tick = now + self.period
        except (ValueError, RuntimeError) as error:
            for side in SIDES:
                self.active[side] = False
                self.status[side].set(str(error))
        if not self.closed:
            self.after_id = self.root.after(10, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        for side in SIDES:
            self.active[side] = False
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        self.root.destroy()


def main(args=None):
    raw = sys.argv if args is None else ['gui_control.py', *args]
    parser = argparse.ArgumentParser(description='Sharpa joint sliders; all angles in radians')
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--state-timeout', type=float, default=1.0)
    options = parser.parse_args(remove_ros_args(raw)[1:])
    try:
        positive(options.rate, 'rate')
        positive(options.state_timeout, 'state-timeout')
        limits = load_limits(Path(get_package_share_directory('dual_sharpa_wave')))
    except (ValueError, OSError) as error:
        parser.error(str(error))
    node = root = panel = None
    rclpy.init(args=raw[1:])
    try:
        node = CommandClient('sharpa_gui_control', limits, options.state_timeout)
        root = tk.Tk()
        panel = ControlPanel(root, node, options.rate)
        panel.tick()
        root.mainloop()
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as error:
        print(f'GUI control failed: {error}', file=sys.stderr)
        return 1
    finally:
        if panel is not None:
            panel.close()
        elif root is not None:
            root.destroy()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
