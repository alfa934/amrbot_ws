#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import signal
import os
import time

class CameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager')

        self.camera_srv = self.create_service(SetBool, 'camera_control', self.camera_callback)
        self.lidar_srv = self.create_service(SetBool, 'lidar_control', self.lidar_callback)

        self.active_process = None 
        self.active_type = None         

        self.get_logger().info('Manager started. Use /camera_control or /lidar_control services.')

    def camera_callback(self, request, response):
        if request.data:  
            if self.active_type == 'camera':
                response.success = True
                response.message = 'Camera already running'
                return response

            if self.active_type == 'lidar':
                self._stop_active_node()

            success, msg = self._start_node(
                'camera',
                ['ros2', 'run', 'mini_agv_vision', 'camera_node']
            )
            response.success = success
            response.message = msg
            return response

        else:
            if self.active_type != 'camera':
                response.success = False
                response.message = 'Camera is not running'
                return response

            success, msg = self._stop_active_node()
            response.success = success
            response.message = msg
            return response

    def lidar_callback(self, request, response):
        if request.data:
            if self.active_type == 'lidar':
                response.success = True
                response.message = 'LiDAR already running'
                return response

            if self.active_type == 'camera':
                self._stop_active_node()

            success, msg = self._start_node(
                'lidar',
                ['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py', 'serial_port:=/dev/ttyUSB1', 'frame_id:=lidar_link',
                 'inverted:=true'
                 ]
            )
            response.success = success
            response.message = msg
            return response

        else:
            if self.active_type != 'lidar':
                response.success = False
                response.message = 'LiDAR is not running'
                return response

            success, msg = self._stop_active_node()
            response.success = success
            response.message = msg
            return response

    def _start_node(self, node_type, command):
        try:
            proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            time.sleep(0.5)

            if proc.poll() is None:
                self.active_process = proc
                self.active_type = node_type
                self.get_logger().info(f'{node_type.capitalize()} started')
                return (True, f'{node_type.capitalize()} started successfully')
            else:
                _, stderr = proc.communicate()
                self.active_process = None
                self.active_type = None
                return (False, f'Failed to start {node_type}: {stderr.decode().strip()}')
        except Exception as e:
            self.active_process = None
            self.active_type = None
            return (False, f'Exception starting {node_type}: {str(e)}')


    def _stop_active_node(self):
        if self.active_process is None or self.active_process.poll() is not None:
            self.active_process = None
            self.active_type = None
            return (False, 'No active node to stop')

        node_type = self.active_type
        try:
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)
            self.active_process.wait(timeout=2)
            self.active_process = None
            self.active_type = None
            self.get_logger().info(f'{node_type.capitalize()} stopped gracefully')
            return (True, f'{node_type.capitalize()} stopped')
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGKILL)
            self.active_process = None
            self.active_type = None
            self.get_logger().warn(f'{node_type.capitalize()} force-killed')
            return (True, f'{node_type.capitalize()} force-stopped')
        except Exception as e:
            self.active_process = None
            self.active_type = None
            return (False, f'Error stopping {node_type}: {str(e)}')

    def destroy_node(self):
        if self.active_process and self.active_process.poll() is None:
            self.get_logger().info('Shutting down active node...')
            try:
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)
                self.active_process.wait(timeout=1)
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()