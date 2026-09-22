"""Tk interaction test; requires a display (WSLg or an X server)."""

from pathlib import Path
import tkinter as tk
from unittest.mock import Mock

import pytest

from dual_sharpa_wave import gui_control
from dual_sharpa_wave.control_model import load_limits


@pytest.mark.integration
def test_gui_feedback_initialization_slider_and_disconnect(monkeypatch):
    node = Mock()
    node.limits = load_limits(Path(__file__).parents[1])
    node.positions = {'left': [0.01234567] * 22, 'right': [0.03] * 22}
    online = {'left': False, 'right': False}
    node.ready.side_effect = lambda side: online[side]
    def require_ready(sides):
        if not all(online[s] for s in sides):
            raise RuntimeError('feedback missing')
    node.require_ready.side_effect = require_ready
    monkeypatch.setattr(gui_control.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(gui_control.rclpy, 'spin_once', lambda *a, **kw: None)
    root = tk.Tk()
    root.withdraw()
    panel = gui_control.ControlPanel(root, node)
    try:
        panel.tick()
        assert len(panel.scales['left']) == len(panel.scales['right']) == 22
        assert not panel.targets
        node.send.assert_not_called()
        online.update(left=True, right=True)
        root.update()
        panel.tick()
        root.update()
        assert panel.targets == node.positions
        node.send.assert_not_called()
        panel.scales['left'][0].set(0.1)
        root.update()
        assert panel.targets['left'][0] == pytest.approx(0.1)
        assert panel.targets['left'][1:] == node.positions['left'][1:]
        assert panel.targets['right'] == node.positions['right']
        panel.start('left')
        panel.next_tick = 0
        panel.tick()
        assert set(node.send.call_args.args[0]) == {'left'}
        panel.stop('left')
        node.send.reset_mock()
        panel.next_tick = 0
        panel.tick()
        node.send.assert_not_called()
        panel.start('left')
        online['left'] = False
        panel.tick()
        assert not panel.active['left']
        online['left'] = True
        panel.tick()
        assert not panel.active['left']
        panel.reload('left')
        root.update()
        assert panel.targets['left'] == node.positions['left']
    finally:
        panel.close()
        panel.close()
