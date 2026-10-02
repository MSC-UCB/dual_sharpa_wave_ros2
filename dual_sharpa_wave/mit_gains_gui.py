"""Simple ROS-only MIT gain editor. Opening/editing the GUI never writes gains."""

import argparse
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.utilities import remove_ros_args
from sharpa_control_interfaces.srv import MitGains

from .joint_names import joint_names
from .mit_gains import gains, scaled_gains, PERIOD


class GainPanel:
    def __init__(self, root, node, side):
        self.root, self.node = root, node
        self.client = node.create_client(MitGains, f'/sharpa/{side}_hand/mit_gains')
        self.pending = None
        self.loaded = False
        self.next_poll = 0.0
        root.title(f'Sharpa {side} — MIT gains')
        ttk.Label(root, text='2 steps (50%, 100%) · approx. 0.8–0.9 s · click Apply to write').grid(
            row=0, column=0, columnspan=4, padx=8, pady=8)
        for col, text in enumerate(('Joint', 'Device Kp / Kd', 'Target Kp', 'Target Kd')):
            ttk.Label(root, text=text).grid(row=1, column=col, padx=8)
        self.actual, self.kp, self.kd = [], [], []
        for row, name in enumerate(joint_names(side), start=2):
            ttk.Label(root, text=name).grid(row=row, column=0, sticky='w', padx=8)
            actual = tk.StringVar(value='—')
            ttk.Label(root, textvariable=actual, width=24).grid(row=row, column=1)
            self.actual.append(actual)
            for col, values in ((2, self.kp), (3, self.kd)):
                value = tk.StringVar()
                ttk.Entry(root, textvariable=value, width=12).grid(row=row, column=col, padx=3)
                values.append(value)
        ratio_row = ttk.Frame(root)
        ratio_row.grid(row=24, column=0, columnspan=4, pady=8)
        self.kp_ratio = tk.StringVar(value='1.0')
        self.kd_ratio = tk.StringVar(value='1.0')
        for label, value in (('Kp ratio', self.kp_ratio), ('Kd ratio', self.kd_ratio)):
            ttk.Label(ratio_row, text=label).pack(side='left')
            ttk.Entry(ratio_row, textvariable=value, width=8).pack(side='left', padx=6)
        ttk.Label(ratio_row, text='1 = fixed tuning baseline').pack(side='left')
        buttons = ttk.Frame(root)
        buttons.grid(row=25, column=0, columnspan=4, pady=8)
        ttk.Button(buttons, text='Read device gains', command=lambda: self.send(
            MitGains.Request.READ)).pack(side='left')
        self.apply_button = ttk.Button(buttons, text='Apply', command=lambda: self.send(
            MitGains.Request.APPLY))
        self.apply_button.pack(side='left')
        self.apply_button.state(['disabled'])
        ttk.Button(buttons, text='Stop transition', command=lambda: self.send(
            MitGains.Request.CANCEL)).pack(side='left')
        self.status = tk.StringVar(value='Connect the MIT driver, then click Read device gains')
        ttk.Label(root, textvariable=self.status, wraplength=650).grid(
            row=26, column=0, columnspan=4, padx=8, pady=8)
        ttk.Label(root, text='Closing does not cancel. SDK latency can extend the transition and pause ROS callbacks.').grid(
            row=27, column=0, columnspan=4, pady=4)
        self.kp_ratio.trace_add('write', self.scale_targets)
        self.kd_ratio.trace_add('write', self.scale_targets)
        self.scale_targets()
        self.status.set('Targets use the fixed baseline; read device gains before applying')
        root.after(20, self.tick)

    def scaled_gains(self):
        return scaled_gains(float(self.kp_ratio.get()), float(self.kd_ratio.get()))

    def scale_targets(self, *_):
        try:
            kp, kd = self.scaled_gains()
            for fields, values in ((self.kp, kp), (self.kd, kd)):
                for field, value in zip(fields, values):
                    field.set(repr(value))
            self.status.set('All targets scaled from fixed tuning baseline; click Apply to write')
        except ValueError as error:
            self.status.set(str(error))
        self.next_poll = time.monotonic() + 3.0

    def send(self, operation):
        if self.pending is not None:
            return
        if not self.client.service_is_ready():
            self.status.set('MIT gain service unavailable')
            self.apply_button.state(['disabled'])
            return
        request = MitGains.Request(operation=operation)
        try:
            if operation == MitGains.Request.APPLY:
                if not self.loaded:
                    raise ValueError('Read gains first')
                self.scaled_gains()  # Reject invalid ratio even if the previous preview remains.
                request.kp = gains(float(v.get()) for v in self.kp)
                request.kd = gains(float(v.get()) for v in self.kd)
            self.pending = (self.client.call_async(request), operation, time.monotonic())
        except (ValueError, TypeError) as error:
            self.status.set(str(error))
            self.next_poll = time.monotonic() + 3.0

    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now = time.monotonic()
        if self.pending is not None:
            future, operation, sent = self.pending
            if future.done():
                self.pending = None
                try:
                    result = future.result()
                    if len(result.kp) == len(result.kd) == 22:
                        for label, kp, kd in zip(self.actual, result.kp, result.kd):
                            label.set(f'{kp:.6g} / {kd:.6g}')
                        if operation == MitGains.Request.READ and result.success:
                            self.loaded = True
                    else:
                        for label in self.actual:
                            label.set('—')
                        self.loaded = False
                    self.status.set(result.message)
                    self.apply_button.state(['!disabled'] if self.loaded else ['disabled'])
                except Exception as error:
                    self.status.set(str(error))
                    self.loaded = False
                    self.apply_button.state(['disabled'])
                self.next_poll = now + PERIOD
            elif now - sent > 5.0:
                future.cancel()
                self.pending = None
                self.loaded = False
                self.apply_button.state(['disabled'])
                self.status.set('Service timed out; result unknown. Read gains before retrying.')
        if self.pending is None and self.loaded and now >= self.next_poll:
            # STATUS returns driver cache, never a hardware parameter read.
            self.send(MitGains.Request.STATUS)
            self.next_poll = now + PERIOD
        self.root.after(20, self.tick)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = rclpy.create_node('sharpa_mit_gain_editor')
    try:
        root = tk.Tk()
        GainPanel(root, node, options.side)
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
