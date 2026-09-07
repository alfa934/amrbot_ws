#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import math


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        self.declare_parameter('show_display', False)
        self.declare_parameter('tag_family', 'DICT_APRILTAG_36h11')
        self.declare_parameter('tag_physical_size', 0.12)

        self.show_display = self.get_parameter('show_display').value
        tag_family_str = self.get_parameter('tag_family').value
        self.physical_size = self.get_parameter('tag_physical_size').value

        self.subscription = self.create_subscription(
            CompressedImage,
            'camera_image/compressed',
            self.image_callback,
            10
        )

        dict_map = {
            'DICT_APRILTAG_16h5': cv2.aruco.DICT_APRILTAG_16h5,
            'DICT_APRILTAG_25h9': cv2.aruco.DICT_APRILTAG_25h9,
            'DICT_APRILTAG_36h10': cv2.aruco.DICT_APRILTAG_36h10,
            'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
        }
        dictionary = dict_map.get(tag_family_str, cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info('AprilTag node ready')

    def image_callback(self, msg):
        frame = self._decode_image(msg)
        if frame is None:
            return

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        display = frame.copy() if self.show_display else None

        if self.show_display:
            self._draw_reticle(display, cx, cy)

        tags_info = []
        if ids is not None:
            for i, tag_id in enumerate(ids.flatten()):
                pts = corners[i][0]
                tx = np.mean(pts[:, 0])
                ty = np.mean(pts[:, 1])
                dist = math.hypot(tx - cx, ty - cy)

                w_px = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
                h_px = math.hypot(pts[0][0] - pts[3][0], pts[0][1] - pts[3][1])
                area = w_px * h_px

                tags_info.append({
                    'id': tag_id,
                    'cx': tx, 'cy': ty,
                    'dx': tx - cx, 'dy': ty - cy,
                    'dist': dist,
                    'area': area,
                    'pts': pts
                })

                self.get_logger().info(
                    f'Tag {tag_id}: dx={tx-cx:+.1f}px, dy={ty-cy:+.1f}px, area={area:.0f}px^2'
                )

                if self.show_display:
                    self._draw_tag_annotations(display, pts, tag_id)

        if self.show_display and tags_info:
            closest = min(tags_info, key=lambda t: t['dist'])
            self._draw_closest_info(display, closest, cx, cy, h)

        if self.show_display:
            cv2.imshow('AprilTag Detection', display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def _decode_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode frame')
        return frame

    def _draw_reticle(self, img, cx, cy):
        cv2.drawMarker(img, (cx, cy), (0, 255, 0),
                       markerType=cv2.MARKER_CROSS, thickness=2)

    def _draw_tag_annotations(self, img, pts, tag_id):
        pts_int = pts.astype(np.int32)
        cv2.polylines(img, [pts_int], True, (0, 255, 0), 2)
        tx = int(np.mean(pts[:, 0]))
        ty = int(np.mean(pts[:, 1]))
        cv2.circle(img, (tx, ty), 4, (0, 0, 255), -1)
        cv2.putText(img, f'ID:{tag_id}',
                    (int(pts[0][0]), int(pts[0][1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def _draw_closest_info(self, img, closest, cx, cy, height):
        cv2.line(img, (cx, cy),
                 (int(closest['cx']), int(closest['cy'])),
                 (255, 255, 0), 2)

        lines = [
            f'ID: {closest["id"]}',
            f'dx: {closest["dx"]:+.1f}px',
            f'dy: {closest["dy"]:+.1f}px',
            f'area: {closest["area"]:.0f}px^2'
        ]

        fs = 0.45
        lh = 18
        x0, y0 = 10, height - 10
        for line in lines:
            cv2.putText(img, line, (x0, y0),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 0), 1)
            y0 -= lh

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()