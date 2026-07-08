#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAVROS Area Planner GUI - enhanced version

功能：
1. 任意多边形区域规划；
2. 矩形区域模式：点击两个点确定矩形长边，输入宽度，自动得到斜矩形和扫描角度；
3. 支持撤销点、按编号删除点、右键删除最近点；
4. 支持指定飞行起始参考点，生成航线时自动选择最近端作为起点；
5. 生成割草机/蛇形面状航线；
6. 导出 CSV: index,x,y,z,speed,yaw。

坐标说明：
- 本工具使用 MAVROS 本地 ENU 坐标系，单位 m。
- x/y 是本地坐标，不是经纬度。
- z 为向上高度。
"""

import csv
import math
import os
import tkinter as tk
from tkinter import filedialog, messagebox
from dataclasses import dataclass
from typing import List, Optional, Tuple

Point = Tuple[float, float]


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    speed: float
    yaw: float


class AreaPlannerGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MAVROS 面状航线规划上位机 - 增强版")
        self.root.geometry("1280x800")

        self.polygon: List[Point] = []
        self.waypoints: List[Waypoint] = []
        self.scale_px_per_m = 70.0
        self.origin_px = (640, 390)

        # point_mode: polygon | rect_edge | start_point
        self.point_mode = tk.StringVar(value="polygon")
        self.start_point: Optional[Point] = None

        self._build_ui()
        self._draw_grid()
        self.update_info()

    def _build_ui(self):
        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)

        left = tk.Frame(main, width=340)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=8)

        right = tk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=8, pady=8)

        title = tk.Label(left, text="面状航线参数", font=("Arial", 15, "bold"))
        title.pack(anchor="w", pady=(0, 8))

        mode_frame = tk.LabelFrame(left, text="绘制模式")
        mode_frame.pack(fill=tk.X, pady=(0, 8))
        tk.Radiobutton(mode_frame, text="任意多边形：左键添加顶点", variable=self.point_mode, value="polygon").pack(anchor="w")
        tk.Radiobutton(mode_frame, text="矩形模式：点击两个点确定长边", variable=self.point_mode, value="rect_edge").pack(anchor="w")
        tk.Radiobutton(mode_frame, text="设置起始参考点：点击画布", variable=self.point_mode, value="start_point").pack(anchor="w")

        self.entries = {}
        self._add_entry(left, "矩形宽度 rect_width (m)", "rect_width", "1.0")
        self._add_entry(left, "扫描间距 row_spacing (m)", "row_spacing", "0.5")
        self._add_entry(left, "飞行高度 z (m)", "altitude", "1.0")
        self._add_entry(left, "巡航速度 speed (m/s)", "speed", "0.3")
        self._add_entry(left, "扫描角度 angle (deg)", "angle", "0.0")
        self._add_entry(left, "航向 yaw (deg)", "yaw", "0.0")
        self._add_entry(left, "画布比例 px/m", "scale", "70")

        btn_frame = tk.Frame(left)
        btn_frame.pack(fill=tk.X, pady=8)
        tk.Button(btn_frame, text="用矩形参数生成区域", command=self.build_rectangle_from_two_points, height=2).pack(fill=tk.X, pady=3)
        tk.Button(btn_frame, text="生成航线", command=self.generate_route, height=2).pack(fill=tk.X, pady=3)
        tk.Button(btn_frame, text="反转航线", command=self.reverse_route, height=2).pack(fill=tk.X, pady=3)
        tk.Button(btn_frame, text="导出 CSV", command=self.export_csv, height=2).pack(fill=tk.X, pady=3)

        edit_frame = tk.LabelFrame(left, text="点编辑")
        edit_frame.pack(fill=tk.X, pady=8)
        tk.Button(edit_frame, text="撤销最后一个点", command=self.undo_last_point).pack(fill=tk.X, pady=3)
        row = tk.Frame(edit_frame)
        row.pack(fill=tk.X, pady=3)
        tk.Label(row, text="删除点编号 P").pack(side=tk.LEFT)
        self.delete_index_var = tk.StringVar(value="")
        tk.Entry(row, textvariable=self.delete_index_var, width=8).pack(side=tk.LEFT, padx=4)
        tk.Button(row, text="删除", command=self.delete_point_by_index).pack(side=tk.LEFT, padx=4)
        tk.Button(edit_frame, text="清空区域/起点/航线", command=self.clear_all).pack(fill=tk.X, pady=3)
        tk.Button(edit_frame, text="示例斜矩形", command=self.load_demo_rectangle).pack(fill=tk.X, pady=3)

        help_text = (
            "操作说明：\n"
            "1. 任意多边形模式：左键依次添加区域顶点。\n"
            "2. 右键点击某个顶点附近可删除该点。\n"
            "3. 矩形模式：先点长边起点，再点长边终点，\n"
            "   输入 rect_width 后点“用矩形参数生成区域”。\n"
            "   软件会自动生成长方形，并把 angle 设置为长边角度。\n"
            "4. 起始参考点：点击画布设置，生成航线时会自动\n"
            "   选择离它最近的一端作为第一个航点。\n"
            "5. angle 是扫描线方向相对本地 x 轴的角度。\n\n"
            "坐标系：MAVROS 本地 ENU\n"
            "x: 本地 x / East，y: 本地 y / North，z: Up"
        )
        tk.Label(left, text=help_text, justify=tk.LEFT, fg="#333333").pack(anchor="w", pady=8)

        self.info_var = tk.StringVar(value="")
        tk.Label(left, textvariable=self.info_var, font=("Arial", 11, "bold"), fg="#0055aa").pack(anchor="w", pady=6)

        self.coord_text = tk.Text(left, height=15, width=43)
        self.coord_text.pack(fill=tk.BOTH, expand=False, pady=4)

        self.canvas = tk.Canvas(right, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_left_click)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        self.canvas.bind("<Configure>", lambda event: self.redraw())

    def _add_entry(self, parent, label, key, default):
        tk.Label(parent, text=label).pack(anchor="w")
        var = tk.StringVar(value=default)
        ent = tk.Entry(parent, textvariable=var)
        ent.pack(fill=tk.X, pady=(0, 5))
        self.entries[key] = var

    def get_float(self, key: str) -> float:
        return float(self.entries[key].get())

    def set_float(self, key: str, value: float):
        self.entries[key].set(f"{value:.6f}")

    def px_to_world(self, px: float, py: float) -> Point:
        scale = self.get_float("scale")
        self.scale_px_per_m = scale
        ox, oy = self.origin_px
        x = (px - ox) / scale
        y = (oy - py) / scale
        return (x, y)

    def world_to_px(self, x: float, y: float) -> Tuple[float, float]:
        scale = self.get_float("scale")
        self.scale_px_per_m = scale
        ox, oy = self.origin_px
        px = ox + x * scale
        py = oy - y * scale
        return (px, py)

    def on_canvas_left_click(self, event):
        try:
            p = self.px_to_world(event.x, event.y)
        except ValueError:
            messagebox.showerror("参数错误", "画布比例必须是数字")
            return

        mode = self.point_mode.get()
        if mode == "start_point":
            self.start_point = p
        else:
            self.polygon.append(p)
            self.waypoints = []
        self.redraw()
        self.update_info()

    def on_canvas_right_click(self, event):
        """右键删除最近的区域点。"""
        if not self.polygon:
            return
        try:
            p = self.px_to_world(event.x, event.y)
        except ValueError:
            return
        idx, dist = self.nearest_polygon_index(p)
        # 右键距离 0.25m 以内才删除，避免误删
        if idx is not None and dist <= 0.25:
            self.polygon.pop(idx)
            self.waypoints = []
            self.redraw()
            self.update_info()

    def nearest_polygon_index(self, p: Point) -> Tuple[Optional[int], float]:
        if not self.polygon:
            return None, float("inf")
        best_i = 0
        best_d = float("inf")
        for i, q in enumerate(self.polygon):
            d = math.hypot(p[0] - q[0], p[1] - q[1])
            if d < best_d:
                best_i, best_d = i, d
        return best_i, best_d

    def undo_last_point(self):
        if self.polygon:
            self.polygon.pop()
            self.waypoints = []
            self.redraw()
            self.update_info()

    def delete_point_by_index(self):
        try:
            idx = int(self.delete_index_var.get())
        except ValueError:
            messagebox.showerror("编号错误", "请输入要删除的点编号，例如 0、1、2。")
            return
        if idx < 0 or idx >= len(self.polygon):
            messagebox.showerror("编号错误", f"当前点编号范围是 0 到 {len(self.polygon)-1}。")
            return
        self.polygon.pop(idx)
        self.waypoints = []
        self.redraw()
        self.update_info()

    def clear_all(self):
        self.polygon = []
        self.waypoints = []
        self.start_point = None
        self.redraw()
        self.update_info()

    def load_demo_rectangle(self):
        self.polygon = [(0.0, 0.0), (2.5, 0.8)]
        self.entries["rect_width"].set("1.0")
        self.build_rectangle_from_two_points(show_message=False)

    def build_rectangle_from_two_points(self, show_message: bool = True):
        if len(self.polygon) < 2:
            messagebox.showwarning("点不足", "矩形模式需要至少两个点：长边起点和长边终点。")
            return
        try:
            width = self.get_float("rect_width")
        except ValueError:
            messagebox.showerror("参数错误", "rect_width 必须是数字。")
            return
        if abs(width) <= 1e-6:
            messagebox.showerror("参数错误", "rect_width 不能为 0。")
            return

        a = self.polygon[0]
        b = self.polygon[1]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            messagebox.showerror("参数错误", "两个矩形长边点不能重合。")
            return

        # 单位法向量：左侧为正宽度，右侧为负宽度
        nx = -dy / length
        ny = dx / length
        c = (b[0] + nx * width, b[1] + ny * width)
        d = (a[0] + nx * width, a[1] + ny * width)
        self.polygon = [a, b, c, d]
        self.waypoints = []

        angle_deg = math.degrees(math.atan2(dy, dx))
        self.set_float("angle", angle_deg)
        # 默认让机头也沿矩形长边方向，可手动改 yaw
        self.set_float("yaw", angle_deg)

        self.redraw()
        self.update_info()
        if show_message:
            messagebox.showinfo(
                "矩形区域已生成",
                f"已生成斜矩形。长边角度 angle = {angle_deg:.2f}°，并已同步到扫描角度。"
            )

    def reverse_route(self):
        if not self.waypoints:
            messagebox.showwarning("无航线", "请先生成航线。")
            return
        self.waypoints.reverse()
        self.redraw()
        self.update_info()

    def update_info(self):
        start_str = "未设置"
        if self.start_point is not None:
            start_str = f"({self.start_point[0]:.2f}, {self.start_point[1]:.2f})"
        self.info_var.set(f"顶点数: {len(self.polygon)} | 航点数: {len(self.waypoints)} | 起始参考点: {start_str}")
        self.coord_text.delete("1.0", tk.END)
        self.coord_text.insert(tk.END, "区域顶点 x,y：\n")
        for i, (x, y) in enumerate(self.polygon):
            self.coord_text.insert(tk.END, f"P{i}: {x:.3f}, {y:.3f}\n")
        if self.start_point is not None:
            self.coord_text.insert(tk.END, f"\n起始参考点 S: {self.start_point[0]:.3f}, {self.start_point[1]:.3f}\n")
        if self.waypoints:
            self.coord_text.insert(tk.END, "\n航点 x,y,z,speed,yaw：\n")
            for i, wp in enumerate(self.waypoints[:80]):
                self.coord_text.insert(tk.END, f"W{i}: {wp.x:.3f}, {wp.y:.3f}, {wp.z:.3f}, {wp.speed:.2f}, {wp.yaw:.2f}\n")
            if len(self.waypoints) > 80:
                self.coord_text.insert(tk.END, f"... 共 {len(self.waypoints)} 个航点\n")

    def redraw(self):
        self.canvas.delete("all")
        w = max(self.canvas.winfo_width(), 100)
        h = max(self.canvas.winfo_height(), 100)
        self.origin_px = (w / 2, h / 2)
        self._draw_grid()
        self._draw_polygon()
        self._draw_start_point()
        self._draw_waypoints()

    def _draw_grid(self):
        w = max(self.canvas.winfo_width(), 100)
        h = max(self.canvas.winfo_height(), 100)
        try:
            scale = self.get_float("scale")
        except Exception:
            scale = self.scale_px_per_m
        ox, oy = self.origin_px
        step = scale

        x = ox % step
        while x < w:
            self.canvas.create_line(x, 0, x, h, fill="#eeeeee")
            x += step
        y = oy % step
        while y < h:
            self.canvas.create_line(0, y, w, y, fill="#eeeeee")
            y += step

        self.canvas.create_line(0, oy, w, oy, fill="#999999", width=2)
        self.canvas.create_line(ox, 0, ox, h, fill="#999999", width=2)
        self.canvas.create_text(ox + 30, oy - 15, text="+X", fill="#444444")
        self.canvas.create_text(ox + 20, oy - 35, text="+Y", fill="#444444")

    def _draw_polygon(self):
        if not self.polygon:
            return
        pts = [self.world_to_px(x, y) for x, y in self.polygon]
        for i, (px, py) in enumerate(pts):
            color = "#111111"
            self.canvas.create_oval(px - 5, py - 5, px + 5, py + 5, fill=color)
            self.canvas.create_text(px + 14, py - 12, text=f"P{i}", fill=color)
        if len(pts) >= 2:
            for i in range(len(pts) - 1):
                self.canvas.create_line(*pts[i], *pts[i + 1], fill="#0066cc", width=2)
        if len(pts) >= 3:
            self.canvas.create_line(*pts[-1], *pts[0], fill="#0066cc", width=2)

    def _draw_start_point(self):
        if self.start_point is None:
            return
        px, py = self.world_to_px(*self.start_point)
        r = 8
        self.canvas.create_oval(px - r, py - r, px + r, py + r, outline="#00aa00", width=3)
        self.canvas.create_line(px - 12, py, px + 12, py, fill="#00aa00", width=2)
        self.canvas.create_line(px, py - 12, px, py + 12, fill="#00aa00", width=2)
        self.canvas.create_text(px + 18, py - 14, text="Start Ref", fill="#008800")

    def _draw_waypoints(self):
        if not self.waypoints:
            return
        pts = [self.world_to_px(wp.x, wp.y) for wp in self.waypoints]
        for i in range(len(pts) - 1):
            self.canvas.create_line(*pts[i], *pts[i + 1], fill="#cc0000", width=2, arrow=tk.LAST)
        for i, (px, py) in enumerate(pts):
            fill = "#00aa00" if i == 0 else ("#8800cc" if i == len(pts) - 1 else "#cc0000")
            self.canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill=fill)
            label = "Start" if i == 0 else ("End" if i == len(pts) - 1 else str(i))
            self.canvas.create_text(px + 16, py + 10, text=label, fill=fill)

    @staticmethod
    def rotate_point(p: Point, angle_rad: float) -> Point:
        x, y = p
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return (x * c - y * s, x * s + y * c)

    @staticmethod
    def inverse_rotate_point(p: Point, angle_rad: float) -> Point:
        return AreaPlannerGUI.rotate_point(p, -angle_rad)

    @staticmethod
    def line_polygon_intersections_y(poly: List[Point], scan_y: float) -> List[float]:
        xs = []
        n = len(poly)
        eps = 1e-9
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            if abs(y2 - y1) < eps:
                continue
            ymin = min(y1, y2)
            ymax = max(y1, y2)
            if scan_y >= ymin and scan_y < ymax:
                t = (scan_y - y1) / (y2 - y1)
                x = x1 + t * (x2 - x1)
                xs.append(x)
        xs.sort()
        return xs

    def maybe_apply_start_point(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        if not waypoints or self.start_point is None:
            return waypoints
        sx, sy = self.start_point
        d_front = math.hypot(waypoints[0].x - sx, waypoints[0].y - sy)
        d_back = math.hypot(waypoints[-1].x - sx, waypoints[-1].y - sy)
        if d_back < d_front:
            waypoints = list(reversed(waypoints))
        return waypoints

    def generate_route(self):
        if len(self.polygon) < 3:
            messagebox.showwarning("区域不足", "至少需要 3 个顶点才能生成面状航线。")
            return
        try:
            spacing = self.get_float("row_spacing")
            altitude = self.get_float("altitude")
            speed = self.get_float("speed")
            angle_deg = self.get_float("angle")
            yaw_deg = self.get_float("yaw")
        except ValueError:
            messagebox.showerror("参数错误", "扫描间距、高度、速度、角度必须是数字。")
            return

        if spacing <= 0:
            messagebox.showerror("参数错误", "扫描间距必须大于 0。")
            return
        if altitude <= 0:
            messagebox.showerror("参数错误", "高度必须大于 0。")
            return
        if speed <= 0:
            messagebox.showerror("参数错误", "速度必须大于 0。")
            return

        angle_rad = math.radians(angle_deg)
        rotated_poly = [self.inverse_rotate_point(p, angle_rad) for p in self.polygon]
        min_y = min(p[1] for p in rotated_poly)
        max_y = max(p[1] for p in rotated_poly)

        lines = []
        y = min_y
        while y <= max_y + 1e-6:
            xs = self.line_polygon_intersections_y(rotated_poly, y)
            if len(xs) >= 2:
                segments = []
                for k in range(0, len(xs) - 1, 2):
                    if xs[k + 1] - xs[k] > 1e-6:
                        segments.append((xs[k], xs[k + 1]))
                if segments:
                    lines.append((y, segments))
            y += spacing

        if not lines:
            messagebox.showwarning("生成失败", "没有生成有效扫描线，请检查区域和间距。")
            return

        waypoints: List[Waypoint] = []
        yaw = math.radians(yaw_deg)
        reverse = False
        for scan_y, segments in lines:
            if reverse:
                segments = list(reversed(segments))
                for x1, x2 in segments:
                    p1 = self.rotate_point((x2, scan_y), angle_rad)
                    p2 = self.rotate_point((x1, scan_y), angle_rad)
                    waypoints.append(Waypoint(p1[0], p1[1], altitude, speed, yaw))
                    waypoints.append(Waypoint(p2[0], p2[1], altitude, speed, yaw))
            else:
                for x1, x2 in segments:
                    p1 = self.rotate_point((x1, scan_y), angle_rad)
                    p2 = self.rotate_point((x2, scan_y), angle_rad)
                    waypoints.append(Waypoint(p1[0], p1[1], altitude, speed, yaw))
                    waypoints.append(Waypoint(p2[0], p2[1], altitude, speed, yaw))
            reverse = not reverse

        filtered: List[Waypoint] = []
        for wp in waypoints:
            if not filtered:
                filtered.append(wp)
                continue
            prev = filtered[-1]
            if math.hypot(wp.x - prev.x, wp.y - prev.y) > 1e-4:
                filtered.append(wp)

        filtered = self.maybe_apply_start_point(filtered)
        self.waypoints = filtered
        self.redraw()
        self.update_info()
        msg = f"已生成 {len(self.waypoints)} 个航点。"
        if self.start_point is not None:
            msg += "\n已根据起始参考点自动选择航线方向。"
        messagebox.showinfo("生成完成", msg)

    def export_csv(self):
        if not self.waypoints:
            messagebox.showwarning("无航点", "请先生成航线。")
            return
        default_path = os.path.expanduser("~/area_waypoints.csv")
        path = filedialog.asksaveasfilename(
            title="保存航点 CSV",
            defaultextension=".csv",
            initialfile=os.path.basename(default_path),
            filetypes=[("CSV files", "*.csv"), ("All files", "*")]
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "x", "y", "z", "speed", "yaw"])
            for i, wp in enumerate(self.waypoints):
                writer.writerow([i, f"{wp.x:.6f}", f"{wp.y:.6f}", f"{wp.z:.6f}", f"{wp.speed:.6f}", f"{wp.yaw:.6f}"])
        messagebox.showinfo("导出完成", f"航点已保存到：\n{path}")


def main():
    root = tk.Tk()
    AreaPlannerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
