#!/usr/bin/env python3

"""Optional thermal-map display for :mod:`thermal_spotter`."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rasterio
from affine import Affine
from pyproj import Geod, Transformer
from rasterio.errors import RasterioIOError
from thermal_common import (
    THERMAL_SENSOR_ROTATION_DEG,
    ThermalCameraModel,
    projected_image_corner_offsets_m,
)
MIN_IMAGE_VALUE = 0.0
MAX_IMAGE_VALUE = 255.0
DISPLAY_SCALE = 3
INPUT_IMAGE_SCALE = 4
WINDOW_NAME = "MLX90640 grayscale"
MAP_WINDOW_NAME = "Thermal global map"
HEADING_PREVIEW_WINDOW_NAME = "Heading image preview"
AREA_XMIN = 10
AREA_XMAX = 45
AREA_YMIN = -50
AREA_YMAX = -10

MIN_TEMPERATURE_C = 30
MAX_TEMPERATURE_C = 60
TEMPERATURE_SLIDER_MAX_C = 120.0
TEMPERATURE_SLIDER_STEP_C = 0.1
# meters per pixel
AREA_RESOLUTION = 0.1
MAP_BLEND_REPLACE = "replace"
MAP_BLEND_WEIGHTED = "weighted"
MAP_BLEND_MODE = MAP_BLEND_REPLACE
MAP_BLEND_ALPHA = 0.4
DEFAULT_HEAT_ALPHA = 0.55
DRONE_MARKER_RADIUS_PX = 6
HOT_SPOT_MARKER_RADIUS_PX = 7
HOT_SPOT_MARKER_COLOR = (255, 0, 255)
CLUSTER_MARKER_RADIUS_PX = 9
CLUSTER_MARKER_COLOR = (48, 220, 48)
MAP_CONTROL_MARGIN_PX = 8
MAP_CONTROL_ROW_HEIGHT_PX = 24
DRONE_HEADING_LENGTH_M = 2.0
DRONE_HEADING_THICKNESS_PX = 2
PROJECTED_IMAGE_OUTLINE_THICKNESS_PX = 2
PROJECTED_BLEND_MASK_ERODE_PX = 1
TEMPERATURE_LABEL_ORIGIN_PX = (8, 22)
DRONE_POSITION_BOUNDS_LABEL_ORIGIN_PX = (8, 44)
THERMAL_PALETTE_RGB = (
    (0.00, (18, 20, 58)),
    (0.18, (31, 87, 151)),
    (0.38, (46, 161, 171)),
    (0.58, (240, 202, 74)),
    (0.78, (226, 103, 51)),
    (1.00, (250, 241, 220)),
)


class ThermalMap:
    """Render thermal frames and aircraft state supplied by ThermalSpotter."""

    def __init__(
        self,
        ground_altitude_m: float,
        geotiff_path: Path | None = None,
        heat_alpha: float = DEFAULT_HEAT_ALPHA,
    ):
        self.ground_altitude_m = ground_altitude_m
        self.geotiff_path = geotiff_path
        self.heat_alpha = heat_alpha
        self._closed = False
        self.last_position_m: np.ndarray | None = None
        self.last_attitude_deg: np.ndarray | None = None
        self.last_gps_lat_deg: float | None = None
        self.last_gps_lon_deg: float | None = None
        self.last_gps_hmsl_m: float | None = None
        self.drone_position_min_east: float | None = None
        self.drone_position_max_east: float | None = None
        self.drone_position_min_north: float | None = None
        self.drone_position_max_north: float | None = None
        self.last_image: np.ndarray | None = None
        self.last_subpage: int | None = None
        self.hot_spots = []
        self.spot_clusters = []
        self.show_spots = True
        self.show_clusters = True
        self.map_control_hitboxes: dict[str, tuple[int, int, int, int]] = {}
        self.thermal_sensor_rotation_deg = THERMAL_SENSOR_ROTATION_DEG.copy()
        self.camera_model = ThermalCameraModel(self.thermal_sensor_rotation_deg)
        self.temperature_min_c = float(MIN_TEMPERATURE_C)
        self.temperature_max_c = float(MAX_TEMPERATURE_C)
        self.temperature_sliders_ready = False
        self.satellite_image: np.ndarray | None = None
        self.geotiff_transformer: Transformer | None = None
        self.geotiff_inverse_transform: Affine | None = None
        self.geod = Geod(ellps="WGS84")
        self.area_xmin = AREA_XMIN
        self.area_xmax = AREA_XMAX
        self.area_ymin = AREA_YMIN
        self.area_ymax = AREA_YMAX
        self.area_resolution = AREA_RESOLUTION
        if self.geotiff_path is not None:
            self.load_geotiff(self.geotiff_path)
        self.area_image = self.create_area_image()
        self.map_blend_mode = MAP_BLEND_MODE
        self.map_blend_alpha = MAP_BLEND_ALPHA

    @property
    def geotiff_mode(self) -> bool:
        return self.satellite_image is not None

    def load_geotiff(self, geotiff_path: Path) -> None:
        try:
            dataset = rasterio.open(geotiff_path)
        except RasterioIOError as exc:
            raise RuntimeError(f"Could not open GeoTIFF: {geotiff_path}") from exc

        with dataset:
            if dataset.crs is None:
                raise RuntimeError(f"GeoTIFF has no CRS/projection: {geotiff_path}")
            if dataset.transform == Affine.identity():
                raise RuntimeError(f"GeoTIFF has no geotransform: {geotiff_path}")

            self.geotiff_transformer = Transformer.from_crs(
                "EPSG:4326",
                dataset.crs,
                always_xy=True,
            )
            self.geotiff_inverse_transform = ~dataset.transform
            self.satellite_image = self.read_geotiff_image(dataset)

    @staticmethod
    def read_geotiff_image(dataset) -> np.ndarray:
        band_count = dataset.count
        if band_count <= 0:
            raise RuntimeError("GeoTIFF has no raster bands")

        if band_count == 1:
            band = ThermalMap.normalize_raster_band(dataset.read(1))
            return cv2.cvtColor(band, cv2.COLOR_GRAY2BGR)

        channels = [
            ThermalMap.normalize_raster_band(dataset.read(index))
            for index in range(1, min(3, band_count) + 1)
        ]
        while len(channels) < 3:
            channels.append(channels[-1])
        rgb = np.dstack(channels[:3])
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    @staticmethod
    def normalize_raster_band(band: np.ndarray) -> np.ndarray:
        if band.dtype == np.uint8:
            return band

        band = band.astype(np.float32)
        finite = band[np.isfinite(band)]
        if finite.size == 0:
            return np.zeros(band.shape, dtype=np.uint8)

        low = float(np.percentile(finite, 2.0))
        high = float(np.percentile(finite, 98.0))
        if np.isclose(low, high):
            low = float(np.min(finite))
            high = float(np.max(finite))
        if np.isclose(low, high):
            return np.zeros(band.shape, dtype=np.uint8)

        normalized = (band - low) / (high - low)
        return (np.clip(normalized, 0.0, 1.0) * 255.0).astype(np.uint8)

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        cv2.destroyAllWindows()

    def update_aircraft_state(
        self,
        position_m: np.ndarray,
        attitude_deg: np.ndarray,
    ) -> None:
        self.last_position_m = position_m.copy()
        self.last_attitude_deg = attitude_deg.copy()
        self.update_drone_position_bounds(position_m)

    def update_gps(self, latitude_deg: float, longitude_deg: float, hmsl_m: float) -> None:
        self.last_gps_lat_deg = latitude_deg
        self.last_gps_lon_deg = longitude_deg
        self.last_gps_hmsl_m = hmsl_m

    def process_frame(self, frame: dict) -> None:
        self.last_image = self.create_grayscale_image(frame)
        self.last_subpage = int(frame["subpage"])
        self.project_last_image_to_area()

    def mark_hot_spot(self, hot_spot) -> None:
        """Keep a detected ground point visible as new thermal frames arrive."""
        self.hot_spots.append(hot_spot)

    def update_spot_clusters(self, spot_clusters) -> None:
        self.spot_clusters = list(spot_clusters)

    def reset_spots(self) -> None:
        """Remove detected spots and clusters from the map overlay."""
        self.hot_spots.clear()
        self.spot_clusters.clear()

    def update_drone_position_bounds(self, position_m: np.ndarray) -> None:
        east = float(position_m[0])
        north = float(position_m[1])
        if self.drone_position_min_east is None:
            self.drone_position_min_east = east
            self.drone_position_max_east = east
            self.drone_position_min_north = north
            self.drone_position_max_north = north
            return

        self.drone_position_min_east = min(self.drone_position_min_east, east)
        self.drone_position_max_east = max(self.drone_position_max_east, east)
        self.drone_position_min_north = min(self.drone_position_min_north, north)
        self.drone_position_max_north = max(self.drone_position_max_north, north)

    def create_grayscale_image(self, frame: dict) -> np.ndarray:
        width = int(frame["width"])
        height = int(frame["height"])
        temperatures = np.array(frame["temperatures"], dtype=np.float32).reshape(
            height, width
        )
        temperatures = temperatures[:, ::-1]
        temperatures = np.nan_to_num(temperatures, nan=MIN_IMAGE_VALUE)
        temperatures = np.clip(temperatures, MIN_IMAGE_VALUE, MAX_IMAGE_VALUE)
        if INPUT_IMAGE_SCALE > 1:
            temperatures = cv2.resize(
                temperatures,
                None,
                fx=INPUT_IMAGE_SCALE,
                fy=INPUT_IMAGE_SCALE,
                interpolation=cv2.INTER_CUBIC,
            )
            temperatures = np.clip(temperatures, MIN_IMAGE_VALUE, MAX_IMAGE_VALUE)
        return temperatures.astype(np.float32)

    def create_area_image(self) -> np.ndarray:
        if self.satellite_image is not None:
            height, width = self.satellite_image.shape[:2]
            return np.zeros((height, width), dtype=np.float32)

        if self.area_resolution <= 0:
            raise ValueError("area resolution must be positive")

        width = int(np.ceil((self.area_xmax - self.area_xmin) / self.area_resolution))
        height = int(np.ceil((self.area_ymax - self.area_ymin) / self.area_resolution))
        if width <= 0 or height <= 0:
            raise ValueError("area bounds must produce a positive image size")
        return np.zeros((height, width), dtype=np.float32)

    def project_last_image_to_area(self) -> bool:
        if (
            self.last_image is None
            or self.last_position_m is None
            or self.last_attitude_deg is None
        ):
            return False

        destination_points = self.projected_image_corners_px()
        if destination_points is None:
            return False
        if not self.projected_bounds_intersect_area(destination_points):
            return False

        height, width = self.last_image.shape
        source_points = np.array(
            [
                [0.0, 0.0],
                [width - 1.0, 0.0],
                [width - 1.0, height - 1.0],
                [0.0, height - 1.0],
            ],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(source_points, destination_points)
        area_height, area_width = self.area_image.shape
        warped_image = cv2.warpPerspective(
            self.last_image,
            transform,
            (area_width, area_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        source_footprint_mask = np.full((height, width), 255, dtype=np.uint8)
        warped_footprint_mask = cv2.warpPerspective(
            source_footprint_mask,
            transform,
            (area_width, area_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        source_update_mask = self.create_subpage_update_mask(height, width)
        warped_update_mask = cv2.warpPerspective(
            source_update_mask,
            transform,
            (area_width, area_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

        valid_pixels = (
            (self.erode_projected_mask(warped_footprint_mask) > 0)
            & (warped_update_mask > 0)
        )
        self.blend_area_image(warped_image, valid_pixels)
        return True

    @staticmethod
    def erode_projected_mask(mask: np.ndarray) -> np.ndarray:
        if PROJECTED_BLEND_MASK_ERODE_PX <= 0:
            return mask

        kernel_size = 2 * PROJECTED_BLEND_MASK_ERODE_PX + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        return cv2.erode(mask, kernel, iterations=1)

    def blend_area_image(self, new_image: np.ndarray, update_mask: np.ndarray) -> None:
        empty_pixels = update_mask & (self.area_image == 0)
        existing_pixels = update_mask & (self.area_image != 0)

        self.area_image[empty_pixels] = new_image[empty_pixels]
        if self.map_blend_mode == MAP_BLEND_REPLACE:
            self.area_image[existing_pixels] = new_image[existing_pixels]
        elif self.map_blend_mode == MAP_BLEND_WEIGHTED:
            blended = (
                (1.0 - self.map_blend_alpha)
                * self.area_image[existing_pixels].astype(np.float32)
                + self.map_blend_alpha * new_image[existing_pixels].astype(np.float32)
            )
            self.area_image[existing_pixels] = np.clip(
                blended, MIN_IMAGE_VALUE, MAX_IMAGE_VALUE
            )
        else:
            raise ValueError(f"unknown map blend mode: {self.map_blend_mode}")

    def create_subpage_update_mask(self, height: int, width: int) -> np.ndarray:
        #return np.full((height, width), 255, dtype=np.uint8)
        if self.last_subpage is None:
            return np.full((height, width), 255, dtype=np.uint8)

        rows, cols = np.indices((height, width))
        updated_pixels = (rows + cols) % 2 == self.last_subpage % 2
        return (updated_pixels * 255).astype(np.uint8)

    def projected_image_corners_px(self) -> np.ndarray | None:
        ground_offsets = self.projected_image_corner_offsets_m()
        if ground_offsets is None:
            return None

        if self.geotiff_mode:
            destination_points = np.array(
                [
                    self.gps_offset_to_area_pixel(offset[0], offset[1])
                    for offset in ground_offsets
                ],
                dtype=np.float32,
            )
        else:
            ground_points = self.last_position_m + ground_offsets
            destination_points = np.array(
                [
                    self.world_to_area_pixel(point[0], point[1])
                    for point in ground_points
                ],
                dtype=np.float32,
            )
        if not np.all(np.isfinite(destination_points)):
            return None
        return destination_points

    def projected_image_corner_offsets_m(self) -> np.ndarray | None:
        return projected_image_corner_offsets_m(
            self.camera_model,
            self.last_position_m,
            self.last_attitude_deg,
            self.ground_altitude_m,
        )

    def world_to_area_pixel(self, east: float, north: float) -> tuple[float, float]:
        col = (east - self.area_xmin) / self.area_resolution
        row = (self.area_ymax - north) / self.area_resolution
        return col, row

    def current_drone_area_pixel(self) -> tuple[float, float] | None:
        if self.geotiff_mode:
            if self.last_gps_lat_deg is None or self.last_gps_lon_deg is None:
                return None
            return self.gps_to_area_pixel(
                self.last_gps_lat_deg,
                self.last_gps_lon_deg,
            )

        if self.last_position_m is None:
            return None
        return self.world_to_area_pixel(
            float(self.last_position_m[0]),
            float(self.last_position_m[1]),
        )

    def gps_offset_to_area_pixel(
        self,
        east_m: float,
        north_m: float,
    ) -> tuple[float, float]:
        if (
            self.last_gps_lat_deg is None
            or self.last_gps_lon_deg is None
            or self.geotiff_transformer is None
            or self.geotiff_inverse_transform is None
        ):
            return np.nan, np.nan

        distance_m = float(np.hypot(east_m, north_m))
        if distance_m == 0.0:
            return self.gps_to_area_pixel(self.last_gps_lat_deg, self.last_gps_lon_deg)

        azimuth_deg = float(np.degrees(np.arctan2(east_m, north_m)))
        lon, lat, _ = self.geod.fwd(
            self.last_gps_lon_deg,
            self.last_gps_lat_deg,
            azimuth_deg,
            distance_m,
        )
        return self.gps_to_area_pixel(lat, lon)

    def gps_to_area_pixel(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        if self.geotiff_transformer is None or self.geotiff_inverse_transform is None:
            return np.nan, np.nan

        x, y = self.geotiff_transformer.transform(lon_deg, lat_deg)
        col, row = self.geotiff_inverse_transform * (x, y)
        return col, row

    def projected_bounds_intersect_area(self, points: np.ndarray) -> bool:
        height, width = self.area_image.shape
        min_col = float(np.min(points[:, 0]))
        max_col = float(np.max(points[:, 0]))
        min_row = float(np.min(points[:, 1]))
        max_row = float(np.max(points[:, 1]))
        return (
            min_col < width
            and max_col >= 0.0
            and min_row < height
            and max_row >= 0.0
        )

    def temperature_scale_bounds(self, image: np.ndarray) -> tuple[float, float] | None:
        if self.temperature_min_c == 0 and self.temperature_max_c == 0:
            bounds = self.temperature_data_bounds(image)
            if bounds is None:
                return None
            low, high = bounds
        else:
            low = self.temperature_min_c
            high = self.temperature_max_c

        if np.isclose(low, high):
            low -= 1.0
            high += 1.0
        elif low > high:
            low, high = high, low
        return low, high

    def setup_temperature_sliders(self) -> None:
        if self.temperature_sliders_ready:
            return

        cv2.namedWindow(MAP_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar(
            "min C x10",
            MAP_WINDOW_NAME,
            self.temperature_to_slider_position(self.temperature_min_c),
            self.temperature_slider_max_position(),
            self.on_temperature_min_slider,
        )
        cv2.createTrackbar(
            "max C x10",
            MAP_WINDOW_NAME,
            self.temperature_to_slider_position(self.temperature_max_c),
            self.temperature_slider_max_position(),
            self.on_temperature_max_slider,
        )
        cv2.setMouseCallback(MAP_WINDOW_NAME, self.on_map_mouse)
        self.temperature_sliders_ready = True

    def on_temperature_min_slider(self, position: int) -> None:
        self.temperature_min_c = self.slider_position_to_temperature(position)

    def on_temperature_max_slider(self, position: int) -> None:
        self.temperature_max_c = self.slider_position_to_temperature(position)

    @staticmethod
    def temperature_slider_max_position() -> int:
        return int(round(TEMPERATURE_SLIDER_MAX_C / TEMPERATURE_SLIDER_STEP_C))

    @staticmethod
    def temperature_to_slider_position(temperature_c: float) -> int:
        clamped = float(np.clip(temperature_c, 0.0, TEMPERATURE_SLIDER_MAX_C))
        return int(round(clamped / TEMPERATURE_SLIDER_STEP_C))

    @staticmethod
    def slider_position_to_temperature(position: int) -> float:
        return position * TEMPERATURE_SLIDER_STEP_C

    @staticmethod
    def temperature_data_bounds(image: np.ndarray) -> tuple[float, float] | None:
        valid_values = image[image > 0]
        if valid_values.size == 0:
            return None
        return float(np.min(valid_values)), float(np.max(valid_values))

    def scale_temperature_image_for_display(
        self,
        image: np.ndarray,
    ) -> tuple[np.ndarray, tuple[float, float] | None]:
        valid_mask = image > 0
        bounds = self.temperature_scale_bounds(image)
        if bounds is None:
            return np.zeros((*image.shape, 3), dtype=np.uint8), None

        low, high = bounds
        normalized = (image.astype(np.float32) - low) / (high - low)
        normalized = np.clip(normalized, 0.0, 1.0)
        return self.apply_thermal_palette(normalized, valid_mask), bounds

    @staticmethod
    def apply_thermal_palette(normalized: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
        color_image = np.zeros((*normalized.shape, 3), dtype=np.uint8)
        for index in range(len(THERMAL_PALETTE_RGB) - 1):
            left_t, left_rgb = THERMAL_PALETTE_RGB[index]
            right_t, right_rgb = THERMAL_PALETTE_RGB[index + 1]
            segment_mask = (
                valid_mask
                & (normalized >= left_t)
                & (normalized <= right_t)
            )
            if not np.any(segment_mask):
                continue

            local = (normalized[segment_mask] - left_t) / (right_t - left_t)
            left = np.array(left_rgb, dtype=np.float32)
            right = np.array(right_rgb, dtype=np.float32)
            rgb = left + (right - left) * local[:, None]
            color_image[segment_mask] = rgb[:, ::-1].astype(np.uint8)
        return color_image

    def display_images(self) -> bool:
        self.setup_temperature_sliders()
        if self.last_image is not None:
            self.display_scaled_image(WINDOW_NAME, self.last_image, DISPLAY_SCALE)
            self.display_heading_preview()
        self.display_global_map()
        return cv2.waitKey(1) == ord("q")

    def display_global_map(self) -> None:
        if self.satellite_image is None:
            self.display_scaled_image(
                MAP_WINDOW_NAME,
                self.area_image,
                DISPLAY_SCALE,
                draw_drone=True,
                draw_temperature_label=True,
            )
            return

        heat_image, _ = self.scale_temperature_image_for_display(self.area_image)
        valid_heat = self.area_image > 0
        map_image = self.satellite_image.copy()
        if np.any(valid_heat):
            heat_alpha = max(0.0, min(1.0, self.heat_alpha))
            satellite_alpha = 1.0 - heat_alpha
            blended = (
                satellite_alpha * map_image[valid_heat].astype(np.float32)
                + heat_alpha * heat_image[valid_heat].astype(np.float32)
            )
            map_image[valid_heat] = np.clip(blended, 0, 255).astype(np.uint8)

        scaled_image = cv2.resize(
            map_image,
            None,
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE,
            interpolation=cv2.INTER_AREA,
        )
        self.draw_projected_image_shape(scaled_image, DISPLAY_SCALE)
        if self.show_spots:
            self.draw_hot_spots(scaled_image, DISPLAY_SCALE)
        if self.show_clusters:
            self.draw_spot_clusters(scaled_image, DISPLAY_SCALE)
        self.draw_drone_position(scaled_image, DISPLAY_SCALE)
        self.draw_drone_position_bounds_label(scaled_image)
        data_bounds = self.temperature_data_bounds(self.area_image)
        if data_bounds is not None:
            self.draw_temperature_label(scaled_image, data_bounds)
        self.draw_map_controls(scaled_image)
        cv2.imshow(MAP_WINDOW_NAME, scaled_image)

    def display_scaled_image(
        self,
        window_name: str,
        image: np.ndarray,
        scale: int,
        draw_drone: bool = False,
        draw_temperature_label: bool = False,
    ) -> None:
        display_image, scale_bounds = self.scale_temperature_image_for_display(image)
        scaled_image = cv2.resize(
            display_image,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST,
        )
        if draw_drone:
            self.draw_projected_image_shape(scaled_image, scale)
            if self.show_spots:
                self.draw_hot_spots(scaled_image, scale)
            if self.show_clusters:
                self.draw_spot_clusters(scaled_image, scale)
            self.draw_drone_position(scaled_image, scale)
            self.draw_drone_position_bounds_label(scaled_image)
        if draw_temperature_label:
            data_bounds = self.temperature_data_bounds(image)
            if data_bounds is not None:
                self.draw_temperature_label(scaled_image, data_bounds)
        if draw_drone:
            self.draw_map_controls(scaled_image)
        cv2.imshow(window_name, scaled_image)

    def draw_map_controls(self, image: np.ndarray) -> None:
        controls = (
            ("spots", "Spots", self.show_spots),
            ("clusters", "Clusters", self.show_clusters),
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        label_width = max(
            cv2.getTextSize(label, font, font_scale, thickness)[0][0]
            for _, label, _ in controls
        )
        control_width = label_width + 32
        origin_x = max(
            MAP_CONTROL_MARGIN_PX,
            image.shape[1] - control_width - MAP_CONTROL_MARGIN_PX,
        )
        self.map_control_hitboxes.clear()

        for index, (name, label, checked) in enumerate(controls):
            top = MAP_CONTROL_MARGIN_PX + index * MAP_CONTROL_ROW_HEIGHT_PX
            bottom = top + MAP_CONTROL_ROW_HEIGHT_PX - 2
            cv2.rectangle(
                image,
                (origin_x - 4, top - 2),
                (origin_x + control_width, bottom),
                (0, 0, 0),
                -1,
            )
            checkbox_top_left = (origin_x, top + 3)
            checkbox_bottom_right = (origin_x + 14, top + 17)
            cv2.rectangle(image, checkbox_top_left, checkbox_bottom_right, (255, 255, 255), 1)
            if checked:
                cv2.line(image, (origin_x + 3, top + 10), (origin_x + 6, top + 14), (255, 255, 255), 1, cv2.LINE_AA)
                cv2.line(image, (origin_x + 6, top + 14), (origin_x + 12, top + 6), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(
                image,
                label,
                (origin_x + 20, top + 15),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            self.map_control_hitboxes[name] = (
                origin_x - 4,
                top - 2,
                origin_x + control_width,
                bottom,
            )

    def on_map_mouse(self, event, x: int, y: int, flags, param) -> None:
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        for name, (left, top, right, bottom) in self.map_control_hitboxes.items():
            if left <= x <= right and top <= y <= bottom:
                if name == "spots":
                    self.show_spots = not self.show_spots
                elif name == "clusters":
                    self.show_clusters = not self.show_clusters
                return

    def draw_drone_position_bounds_label(self, image: np.ndarray) -> None:
        if self.drone_position_min_east is None:
            return

        text = (
            f"x [{self.drone_position_min_east:.1f} ... "
            f"{self.drone_position_max_east:.1f}]  "
            f"y [{self.drone_position_min_north:.1f} ... "
            f"{self.drone_position_max_north:.1f}]"
        )
        self.draw_label(image, text, DRONE_POSITION_BOUNDS_LABEL_ORIGIN_PX)

    def draw_temperature_label(
        self,
        image: np.ndarray,
        data_bounds: tuple[float, float],
    ) -> None:
        low, high = data_bounds
        text = f"min {low:.1f} C  max {high:.1f} C"
        self.draw_label(image, text, TEMPERATURE_LABEL_ORIGIN_PX)

    @staticmethod
    def draw_label(
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
    ) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size, baseline = cv2.getTextSize(text, font, font_scale, thickness)
        top_left = (origin[0] - 4, origin[1] - text_size[1] - 4)
        bottom_right = (origin[0] + text_size[0] + 4, origin[1] + baseline + 4)
        cv2.rectangle(image, top_left, bottom_right, (0, 0, 0), -1)
        cv2.putText(
            image,
            text,
            origin,
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def draw_projected_image_shape(self, image: np.ndarray, scale: int) -> None:
        if (
            self.last_image is None
            or self.last_position_m is None
            or self.last_attitude_deg is None
        ):
            return

        corners = self.projected_image_corners_px()
        if corners is None:
            return
        if not self.projected_bounds_intersect_area(corners):
            return

        scaled_corners = np.round(corners * scale).astype(np.int32)
        cv2.polylines(
            image,
            [scaled_corners],
            isClosed=True,
            color=(0, 0, 0),
            thickness=PROJECTED_IMAGE_OUTLINE_THICKNESS_PX + 2,
            lineType=cv2.LINE_AA,
        )
        cv2.polylines(
            image,
            [scaled_corners],
            isClosed=True,
            color=(51, 103, 226),
            thickness=PROJECTED_IMAGE_OUTLINE_THICKNESS_PX,
            lineType=cv2.LINE_AA,
        )

    def draw_hot_spots(self, image: np.ndarray, scale: int) -> None:
        for hot_spot in self.hot_spots:
            position = self.hot_spot_area_pixel(hot_spot)
            if position is None:
                continue

            col, row = position
            height, width = self.area_image.shape
            if col < 0.0 or col >= width or row < 0.0 or row >= height:
                continue

            center = (int(round(col * scale)), int(round(row * scale)))
            cv2.drawMarker(
                image,
                center,
                (0, 0, 0),
                cv2.MARKER_TILTED_CROSS,
                HOT_SPOT_MARKER_RADIUS_PX * 2 + 4,
                3,
                cv2.LINE_AA,
            )
            cv2.drawMarker(
                image,
                center,
                HOT_SPOT_MARKER_COLOR,
                cv2.MARKER_TILTED_CROSS,
                HOT_SPOT_MARKER_RADIUS_PX * 2,
                1,
                cv2.LINE_AA,
            )
            self.draw_label(
                image,
                f"#{hot_spot.sequence} {hot_spot.temperature:.1f} C",
                (center[0] + HOT_SPOT_MARKER_RADIUS_PX, center[1] - HOT_SPOT_MARKER_RADIUS_PX),
            )

    def draw_spot_clusters(self, image: np.ndarray, scale: int) -> None:
        for cluster_index, cluster in enumerate(self.spot_clusters, start=1):
            position = self.hot_spot_area_pixel(cluster)
            if position is None:
                continue

            col, row = position
            height, width = self.area_image.shape
            if col < 0.0 or col >= width or row < 0.0 or row >= height:
                continue

            center = (int(round(col * scale)), int(round(row * scale)))
            cv2.drawMarker(
                image,
                center,
                (0, 0, 0),
                cv2.MARKER_STAR,
                CLUSTER_MARKER_RADIUS_PX * 2 + 4,
                3,
                cv2.LINE_AA,
            )
            cv2.drawMarker(
                image,
                center,
                CLUSTER_MARKER_COLOR,
                cv2.MARKER_STAR,
                CLUSTER_MARKER_RADIUS_PX * 2,
                1,
                cv2.LINE_AA,
            )
            self.draw_label(
                image,
                f"cluster {cluster_index} ({cluster.member_count}) {cluster.temperature:.1f} C",
                (
                    center[0] + CLUSTER_MARKER_RADIUS_PX,
                    center[1] + CLUSTER_MARKER_RADIUS_PX + 14,
                ),
            )

    def hot_spot_area_pixel(self, hot_spot) -> tuple[float, float] | None:
        if self.geotiff_mode:
            if not np.isfinite(hot_spot.latitude) or not np.isfinite(hot_spot.longitude):
                return None
            return self.gps_to_area_pixel(hot_spot.latitude, hot_spot.longitude)

        return self.world_to_area_pixel(hot_spot.east, hot_spot.north)

    def draw_drone_position(self, image: np.ndarray, scale: int) -> None:
        position = self.current_drone_area_pixel()
        if position is None:
            return

        col, row = position
        height, width = self.area_image.shape
        if col < 0.0 or col >= width or row < 0.0 or row >= height:
            return

        center = (int(round(col * scale)), int(round(row * scale)))
        self.draw_drone_heading(image, center, scale)
        cv2.circle(image, center, DRONE_MARKER_RADIUS_PX + 2, (0, 0, 0), -1)
        cv2.circle(image, center, DRONE_MARKER_RADIUS_PX, (255, 255, 255), -1)
        cv2.circle(image, center, 2, (51, 103, 226), -1)

    def draw_drone_heading(
        self,
        image: np.ndarray,
        center: tuple[int, int],
        scale: int,
    ) -> None:
        if self.last_attitude_deg is None:
            return

        heading_rad = np.deg2rad(float(self.last_attitude_deg[2]))
        if self.geotiff_mode:
            east_m = np.sin(heading_rad) * DRONE_HEADING_LENGTH_M
            north_m = np.cos(heading_rad) * DRONE_HEADING_LENGTH_M
            end_col, end_row = self.gps_offset_to_area_pixel(east_m, north_m)
            if not np.isfinite(end_col) or not np.isfinite(end_row):
                return
            end = (
                int(round(end_col * scale)),
                int(round(end_row * scale)),
            )
        else:
            length_px = DRONE_HEADING_LENGTH_M / self.area_resolution * scale
            end = (
                int(round(center[0] + np.sin(heading_rad) * length_px)),
                int(round(center[1] - np.cos(heading_rad) * length_px)),
            )
        cv2.arrowedLine(
            image,
            center,
            end,
            (0, 0, 0),
            DRONE_HEADING_THICKNESS_PX + 2,
            cv2.LINE_AA,
            0,
            0.25,
        )
        cv2.arrowedLine(
            image,
            center,
            end,
            (51, 103, 226),
            DRONE_HEADING_THICKNESS_PX,
            cv2.LINE_AA,
            0,
            0.25,
        )

    def display_heading_preview(self) -> None:
        if self.last_image is None or self.last_attitude_deg is None:
            return

        display_image, _ = self.scale_temperature_image_for_display(self.last_image)
        scaled_image = cv2.resize(
            display_image,
            None,
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE,
            interpolation=cv2.INTER_NEAREST,
        )
        preview_heading_deg = (
            float(self.last_attitude_deg[2])
            + float(self.thermal_sensor_rotation_deg[2])
        )
        rotated_image = self.rotate_image_by_heading(
            scaled_image,
            preview_heading_deg,
        )
        cv2.imshow(HEADING_PREVIEW_WINDOW_NAME, rotated_image)

    @staticmethod
    def rotate_image_by_heading(image: np.ndarray, heading_deg: float) -> np.ndarray:
        height, width = image.shape[:2]
        center = (width / 2.0, height / 2.0)
        rotation = cv2.getRotationMatrix2D(center, -heading_deg, 1.0)
        cos_angle = abs(rotation[0, 0])
        sin_angle = abs(rotation[0, 1])
        rotated_width = int((height * sin_angle) + (width * cos_angle))
        rotated_height = int((height * cos_angle) + (width * sin_angle))
        rotation[0, 2] += (rotated_width / 2.0) - center[0]
        rotation[1, 2] += (rotated_height / 2.0) - center[1]
        return cv2.warpAffine(
            image,
            rotation,
            (rotated_width, rotated_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def main() -> None:
    from thermal_spotter import main as thermal_spotter_main

    thermal_spotter_main(enable_map=True)


if __name__ == "__main__":
    main()
