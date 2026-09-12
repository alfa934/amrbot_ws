#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import CompressedImage

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np
import math
import threading

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.camera_service = self.create_client(SetBool, '/camera_control')
        self.lidar_service = self.create_client(SetBool, '/lidar_control')

        self.annotated_image = None
        self.annotated_lock = threading.Lock()
        self.detection_info = "No tags detected"
        self.detection_lock = threading.Lock()

        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera_image/compressed',
            self.image_callback,
            10
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info('Integrated GUI + AprilTag node ready.')

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            annotated = frame.copy()
            cv2.drawMarker(annotated, (cx, cy), (0, 255, 0),
                           markerType=cv2.MARKER_CROSS, thickness=1)

            tags = []
            if ids is not None:
                for i, tag_id in enumerate(ids.flatten()):
                    pts = corners[i][0]
                    tx = np.mean(pts[:, 0])
                    ty = np.mean(pts[:, 1])
                    dist = math.hypot(tx - cx, ty - cy)

                    w_px = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
                    h_px = math.hypot(pts[0][0] - pts[3][0], pts[0][1] - pts[3][1])
                    area = w_px * h_px

                    tags.append({
                        'id': int(tag_id),
                        'cx': tx, 'cy': ty,
                        'dx': tx - cx,
                        'dy': ty - cy,
                        'dist': dist,
                        'area': area,
                        'pts': pts
                    })

            closest = None
            if tags:
                closest = max(tags, key=lambda t: t['area'])

            for tag in tags:
                pts_int = tag['pts'].astype(np.int32)
                color = (0, 255, 0)
                if closest and tag['id'] == closest['id']:
                    color = (0, 255, 255) 
                cv2.polylines(annotated, [pts_int], True, color, 2)
                cv2.circle(annotated, (int(tag['cx']), int(tag['cy'])), 4, (0, 0, 255), -1)
                cv2.putText(annotated, f"ID:{tag['id']}",
                            (int(tag['pts'][0][0]), int(tag['pts'][0][1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if closest:
                pt1 = (cx, cy)
                pt2 = (int(closest['cx']), int(closest['cy']))
                cv2.line(annotated, pt1, pt2, (0, 255, 255), 3)
            lines = []
            if closest:
                lines.append(f"★ Closest ID {closest['id']} (largest area): "
                             f"dx={closest['dx']:+.1f}px, dy={closest['dy']:+.1f}px, "
                             f"dist={closest['dist']:.1f}px, area={closest['area']:.0f}px²")
            if tags:
                for tag in tags:
                    if closest and tag['id'] == closest['id']:
                        continue
                    lines.append(f"ID {tag['id']}: dx={tag['dx']:+.1f}px, dy={tag['dy']:+.1f}px, "
                                 f"dist={tag['dist']:.1f}px, area={tag['area']:.0f}px²")
            info = "\n".join(lines) if lines else "No tags detected"

            with self.detection_lock:
                self.detection_info = info

            with self.annotated_lock:
                self.annotated_image = annotated

        except Exception as e:
            self.get_logger().warn(f"Detection error: {e}")


class GuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("AMRBOT Control")
        self.root.geometry("900x700")

        rclpy.init(args=None)
        self.node = GuiNode()
        self.spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.spin_thread.start()

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)

        self.camera_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.camera_frame, text="Camera")
        self._setup_camera_tab()

        self.lidar_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.lidar_frame, text="LiDAR")
        self._setup_lidar_tab()

        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ttk.Label(root, textvariable=self.status_var, relief='sunken')
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        self.video_running = False
        self.update_ui()

    def _spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _setup_camera_tab(self):
        # Control row
        ctrl_frame = ttk.Frame(self.camera_frame)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)

        self.camera_button = ttk.Button(ctrl_frame, text="Start Camera", command=self._toggle_camera)
        self.camera_button.pack(side=tk.LEFT, padx=5)

        self.show_video_var = tk.BooleanVar(value=True)
        self.show_video_cb = ttk.Checkbutton(ctrl_frame, text="Show Camera Feed", variable=self.show_video_var)
        self.show_video_cb.pack(side=tk.LEFT, padx=20)

        self.camera_status_label = ttk.Label(ctrl_frame, text="Status: Stopped", foreground="red")
        self.camera_status_label.pack(side=tk.LEFT, padx=20)

        paned = ttk.PanedWindow(self.camera_frame, orient=tk.VERTICAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.canvas_frame = ttk.Frame(paned)
        paned.add(self.canvas_frame, weight=3)
        self.canvas = tk.Canvas(self.canvas_frame, bg='gray')
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.log_frame = ttk.Frame(paned)
        paned.add(self.log_frame, weight=1)
        self.log_text = scrolledtext.ScrolledText(self.log_frame, wrap=tk.WORD, state='normal', height=10)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.config(state='disabled')

    def _setup_lidar_tab(self):
        ctrl_frame = ttk.Frame(self.lidar_frame)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)

        self.lidar_button = ttk.Button(ctrl_frame, text="Start LiDAR", command=self._toggle_lidar)
        self.lidar_button.pack(side=tk.LEFT, padx=5)

        self.lidar_status_label = ttk.Label(ctrl_frame, text="Status: Stopped", foreground="red")
        self.lidar_status_label.pack(side=tk.LEFT, padx=20)

        info_label = ttk.Label(self.lidar_frame, text="LiDAR control only (visualization later)")
        info_label.pack(pady=20)

    def _toggle_camera(self):
        if self.camera_button['text'] == "Start Camera":
            self._call_service('/camera_control', True, self._camera_started)
        else:
            self._call_service('/camera_control', False, self._camera_stopped)

    def _camera_started(self, response):
        if response.success:
            self.camera_button.config(text="Stop Camera")
            self.camera_status_label.config(text="Status: Running", foreground="green")
            self.status_var.set("Camera started")
            self.video_running = True
        else:
            messagebox.showerror("Error", f"Failed to start camera: {response.message}")

    def _camera_stopped(self, response):
        if response.success:
            self.camera_button.config(text="Start Camera")
            self.camera_status_label.config(text="Status: Stopped", foreground="red")
            self.status_var.set("Camera stopped")
            self.video_running = False
            self.canvas.delete("all")
            self.log_text.config(state='normal')
            self.log_text.delete(1.0, tk.END)
            self.log_text.config(state='disabled')
            with self.node.annotated_lock:
                self.node.annotated_image = None
        else:
            messagebox.showerror("Error", f"Failed to stop camera: {response.message}")

    def _toggle_lidar(self):
        if self.lidar_button['text'] == "Start LiDAR":
            self._call_service('/lidar_control', True, self._lidar_started)
        else:
            self._call_service('/lidar_control', False, self._lidar_stopped)

    def _lidar_started(self, response):
        if response.success:
            self.lidar_button.config(text="Stop LiDAR")
            self.lidar_status_label.config(text="Status: Running", foreground="green")
            self.status_var.set("LiDAR started")
        else:
            messagebox.showerror("Error", f"Failed to start LiDAR: {response.message}")

    def _lidar_stopped(self, response):
        if response.success:
            self.lidar_button.config(text="Start LiDAR")
            self.lidar_status_label.config(text="Status: Stopped", foreground="red")
            self.status_var.set("LiDAR stopped")
        else:
            messagebox.showerror("Error", f"Failed to stop LiDAR: {response.message}")

    def _call_service(self, service_name, data, callback):
        client = self.node.camera_service if service_name == '/camera_control' else self.node.lidar_service
        if not client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", f"Service {service_name} not available")
            self.status_var.set(f"{service_name} not available")
            return
        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        future.add_done_callback(lambda f: self._service_done(f, callback))

    def _service_done(self, future, callback):
        try:
            response = future.result()
            self.root.after(0, lambda: callback(response))
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Service Error", str(e)))

    def update_ui(self):
        if self.video_running and self.show_video_var.get():
            with self.node.detection_lock:
                info = self.node.detection_info
            self.log_text.config(state='normal')
            self.log_text.delete(1.0, tk.END)
            self.log_text.insert(tk.END, info)
            self.log_text.config(state='disabled')

            annotated_img = None
            with self.node.annotated_lock:
                annotated_img = self.node.annotated_image

            if annotated_img is not None:
                canvas_w = self.canvas.winfo_width()
                canvas_h = self.canvas.winfo_height()
                if canvas_w > 1 and canvas_h > 1:
                    h, w = annotated_img.shape[:2]
                    scale = min(canvas_w/w, canvas_h/h)
                    new_w = int(w * scale)
                    new_h = int(h * scale)
                    if new_w > 0 and new_h > 0:
                        img_resized = cv2.resize(annotated_img, (new_w, new_h))
                        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
                        img_pil = Image.fromarray(img_rgb)
                        imgtk = ImageTk.PhotoImage(image=img_pil)
                        self.canvas.delete("all")
                        self.canvas.create_image(canvas_w//2, canvas_h//2, anchor=tk.CENTER, image=imgtk)
                        self.canvas.image = imgtk
            else:
                self.canvas.delete("all")
                self.canvas.create_text(self.canvas.winfo_width()//2, self.canvas.winfo_height()//2,
                                        text="No image available", fill="white")

        self.root.after(50, self.update_ui)

    def on_closing(self):
        self.video_running = False
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = GuiApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()