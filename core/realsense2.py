import numpy as np
import pyrealsense2 as rs
import cv2
import time
import threading
from threading import Thread


class Realsense2(object):
    def __init__(
        self,
        id=0,
        rgb_width=640,
        rgb_height=480,
        rgb_fps=30,
        is_depth=True,
        is_colormap=True,
        is_filter=True,
        is_fps=False,
    ):
        self.id_camera = id
        self.im_width = rgb_width
        self.im_height = rgb_height
        self.im_fps = rgb_fps

        self.is_depth = is_depth
        self.is_colormap = is_colormap
        self.is_filter = is_filter
        self.is_fps = is_fps

        # RealSense objects
        self.pipeline = None
        self.config = None
        self.align = None

        # 数据缓存
        self.rgb_image = None
        self.depth_raw = None        # ✅ 原始深度（uint16）
        self.depth_vis = None        # 仅用于显示
        self.intrinsics = None
        self.aligned_depth_frame = None

        # 同步机制
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame_id = 0
        self._last_read_id = -1

        # FPS
        self._start_time = time.time()
        self._frame_count = 0

        self._camera_config()
        self._start_camera()

    # ------------------------------------------------------------------
    # 相机配置
    # ------------------------------------------------------------------
    def _camera_config(self):
        self.align = rs.align(rs.stream.color)

        self.config = rs.config()
        self.config.enable_stream(
            rs.stream.color,
            self.im_width,
            self.im_height,
            rs.format.bgr8,
            self.im_fps,
        )
        self.config.enable_stream(
            rs.stream.depth,
            self.im_width,
            self.im_height,
            rs.format.z16,
            self.im_fps,
        )

        # 选择设备
        devices = []
        for d in rs.context().devices:
            if d.get_info(rs.camera_info.name).lower() != "platform camera":
                devices.append(d.get_info(rs.camera_info.serial_number))

        if not devices:
            raise RuntimeError("No RealSense device found")

        self.config.enable_device(devices[self.id_camera])

        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)

        time.sleep(2)

    # ------------------------------------------------------------------
    # 后台采集线程
    # ------------------------------------------------------------------
    def _start_camera(self):
        Thread(target=self._get_data, daemon=True).start()

    def _get_data(self):
        hole_filling = rs.hole_filling_filter() if self.is_filter else None

        while True:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            rgb = np.asanyarray(color_frame.get_data()).copy()
            depth_raw = np.asanyarray(depth_frame.get_data()).copy()

            # 深度仅用于显示的版本
            depth_vis = depth_raw
            if hole_filling:
                depth_vis = np.asanyarray(
                    hole_filling.process(depth_frame).get_data()
                ).copy()

            if self.is_colormap:
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_vis, alpha=0.1),
                    cv2.COLORMAP_JET,
                )

            with self._cond:
                self.rgb_image = rgb
                self.depth_raw = depth_raw
                self.depth_vis = depth_vis
                self.intrinsics = (
                    color_frame.profile.as_video_stream_profile().intrinsics
                )
                self.aligned_depth_frame = depth_frame

                self._frame_id += 1
                self._cond.notify_all()

    # ------------------------------------------------------------------
    # Iterator 接口（关键）
    # ------------------------------------------------------------------
    def __iter__(self):
        return self

    def __next__(self):
        with self._cond:
            while self._frame_id == self._last_read_id:
                self._cond.wait()

            self._last_read_id = self._frame_id

            return (
                self.rgb_image.copy(),
                self.depth_vis.copy(),
                self.depth_raw.copy(),
            )

    # ------------------------------------------------------------------
    # 像素 -> 三维点
    # ------------------------------------------------------------------    
    def pixel2point(self, xy):
        with self._lock:
            z = self.aligned_depth_frame.get_distance(xy[0], xy[1])
            print(f"原始测量深度是：{z}")            
            return rs.rs2_deproject_pixel_to_point(self.intrinsics, xy, z)
