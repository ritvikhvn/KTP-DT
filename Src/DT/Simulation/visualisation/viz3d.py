"""Realtime 3D drone visualiser — runs in a separate process, never blocks the sim loop.

Entry point: run_viz(queue, span_m). Receives snapshots from main proc via
multiprocessing.Queue(maxsize=1). Drops stale frames (put_nowait on sender side).

Display convention: East=X, North=Y, Up=Z (NED → right-hand display).
World frame is fixed: axes centered on origin with half-extent `span_m`. Drone
translates through the world; axes do not follow it. Auto-grows (never shrinks)
if drone exits bounds so it stays visible.

Body axes arrows: red=forward(+Xb), green=right(+Yb), blue=down(+Zb).
World-origin triad is drawn fixed at display (0,0,0) for reference.
"""
from __future__ import annotations

import queue as _queue

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation


def run_viz(q, span_m: float = 20.0) -> None:
    fig = plt.figure(figsize=(7, 6))
    fig.canvas.manager.set_window_title("Digital Twin — 3D Viz")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("E (m)")
    ax.set_ylabel("N (m)")
    ax.set_zlabel("Up (m)")

    # Fixed world frame: bounds set once, centered on origin.
    ax.set_xlim(-span_m, span_m)
    ax.set_ylim(-span_m, span_m)
    ax.set_zlim(0.0, 2.0 * span_m)
    ax.set_box_aspect((1.0, 1.0, 1.0))

    # Ground plane grid at Up=0 — fixed visual reference.
    grid_n = 11
    g_lin = np.linspace(-span_m, span_m, grid_n)
    g_xx, g_yy = np.meshgrid(g_lin, g_lin)
    g_zz = np.zeros_like(g_xx)
    ax.plot_wireframe(g_xx, g_yy, g_zz, color="#888888", linewidth=0.4, alpha=0.6)

    # World-origin axis triad — drawn once, fixed.
    world_arrow_len = max(0.5, 0.05 * span_m)
    ax.quiver(0, 0, 0, world_arrow_len, 0, 0, color="#cc4444", linewidth=1.5, alpha=0.7)
    ax.quiver(0, 0, 0, 0, world_arrow_len, 0, color="#44aa44", linewidth=1.5, alpha=0.7)
    ax.quiver(0, 0, 0, 0, 0, world_arrow_len, color="#4477cc", linewidth=1.5, alpha=0.7)
    ax.text(world_arrow_len, 0, 0, "E", color="#cc4444", fontsize=8)
    ax.text(0, world_arrow_len, 0, "N", color="#44aa44", fontsize=8)
    ax.text(0, 0, world_arrow_len, "Up", color="#4477cc", fontsize=8)

    pos = np.zeros(3)
    quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0])

    arrow_len = 0.4
    body_local = np.eye(3) * arrow_len
    colors = ["r", "g", "b"]
    labels = ["+X fwd", "+Y right", "+Z down"]

    arrows = [
        ax.quiver(0, 0, 0, 0, 0, 0, color=c, linewidth=2, label=lb)
        for c, lb in zip(colors, labels)
    ]

    # quadcopter icon: X-frame, 4 arms + rotor rings + hub
    arm_len = 0.35
    _s2 = arm_len / np.sqrt(2)
    arm_dirs_body = np.array([
        [+_s2, +_s2, 0.0],
        [+_s2, -_s2, 0.0],
        [-_s2, -_s2, 0.0],
        [-_s2, +_s2, 0.0],
    ]).T  # (3, 4)
    arm_color = "#222222"
    rotor_color = "#444444"
    arm_lines = [
        ax.plot([0, 0], [0, 0], [0, 0], color=arm_color, linewidth=2)[0]
        for _ in range(4)
    ]
    rotor_dots = [
        ax.plot([0], [0], [0], "o", color=rotor_color,
                markersize=10, markerfacecolor="none", markeredgewidth=2)[0]
        for _ in range(4)
    ]
    hub_dot = ax.plot([0], [0], [0], "s", color=arm_color, markersize=5)[0]

    ax.legend(loc="upper left", fontsize=8)

    sim_time_text = ax.text2D(0.02, 0.95, "t=0.00s", transform=ax.transAxes, fontsize=9)

    def _ned_to_display(v_ned: np.ndarray):
        return v_ned[1], v_ned[0], -v_ned[2]  # E, N, Up

    def _drain():
        nonlocal pos, quat_xyzw
        msg = None
        try:
            while True:
                msg = q.get_nowait()
        except _queue.Empty:
            pass
        if msg is not None:
            pos[:] = msg["pos_ned"]
            w, x, y, z = msg["quat_wxyz"]
            quat_xyzw[:] = [x, y, z, w]
            return msg.get("t", 0.0)
        return None

    def _update(_frame):
        t = _drain()
        px, py, pz = _ned_to_display(pos)
        R = Rotation.from_quat(quat_xyzw).as_matrix()  # body → earth (NED)
        for i, _ in enumerate(arrows):
            v_ned = R @ body_local[:, i]
            vx, vy, vz = _ned_to_display(v_ned)
            arrows[i].remove()
            arrows[i] = ax.quiver(px, py, pz, vx, vy, vz,
                                  color=colors[i], linewidth=2)
        for i in range(4):
            v_ned = R @ arm_dirs_body[:, i]
            vx, vy, vz = _ned_to_display(v_ned)
            arm_lines[i].set_data([px, px + vx], [py, py + vy])
            arm_lines[i].set_3d_properties([pz, pz + vz])
            rotor_dots[i].set_data([px + vx], [py + vy])
            rotor_dots[i].set_3d_properties([pz + vz])
        hub_dot.set_data([px], [py])
        hub_dot.set_3d_properties([pz])

        # Auto-grow guard: if drone exits world, expand axis (never shrink).
        xl = ax.get_xlim(); yl = ax.get_ylim(); zl = ax.get_zlim()
        if px < xl[0] or px > xl[1]:
            new_half = 1.5 * abs(px)
            ax.set_xlim(-new_half, new_half)
        if py < yl[0] or py > yl[1]:
            new_half = 1.5 * abs(py)
            ax.set_ylim(-new_half, new_half)
        if pz < zl[0] or pz > zl[1]:
            ax.set_zlim(min(zl[0], 1.5 * pz if pz < 0 else zl[0]),
                        max(zl[1], 1.5 * pz if pz > 0 else zl[1]))

        if t is not None:
            sim_time_text.set_text(f"t={t:.2f}s")
        return [hub_dot, sim_time_text] + arrows + arm_lines + rotor_dots

    _anim = FuncAnimation(fig, _update, interval=33, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
