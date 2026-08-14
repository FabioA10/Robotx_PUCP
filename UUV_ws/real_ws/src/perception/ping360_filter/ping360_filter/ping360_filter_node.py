#!/usr/bin/env python3

import math
from typing import Dict, List

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class Ping360PointCloudFilterNode(Node):
    """
    Filtra la nube PointCloud2 generada por el nodo de adquisición del Ping360.

    Etapas:
      1. Máscara por distancia.
      2. Umbral robusto de intensidad por bandas de distancia.
      3. Agrupación de retornos consecutivos en dirección radial.
      4. Confirmación entre sectores angulares vecinos.
      5. Publicación de la nube filtrada.

    El nodo de adquisición debe publicar casi todas las muestras. Se recomienda
    ejecutarlo con threshold:=0 o threshold:=1.
    """

    def __init__(self):
        super().__init__('ping360_pointcloud_filter_node')

        self.declare_parameter('input_topic', '/ping360/points')
        self.declare_parameter('output_topic', '/ping360/filtered_points')

        self.declare_parameter('min_range_m', 0.25)
        self.declare_parameter('max_range_m', 5.0)

        self.declare_parameter('adaptive_threshold', True)
        self.declare_parameter('min_intensity', 20.0)
        self.declare_parameter('range_band_m', 0.25)
        self.declare_parameter('mad_scale', 4.0)
        self.declare_parameter('intensity_margin', 5.0)
        self.declare_parameter('min_points_per_band', 20)

        self.declare_parameter('angular_bin_deg', 1.8)
        self.declare_parameter('radial_bin_m', 0.025)
        self.declare_parameter('min_radial_cells', 2)
        self.declare_parameter('max_radial_gap_cells', 1)
        self.declare_parameter('first_return_only', True)

        self.declare_parameter('angular_window_bins', 2)
        self.declare_parameter('min_angular_support', 2)
        self.declare_parameter('neighbor_range_tolerance_m', 0.12)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)

        self.adaptive_threshold = bool(
            self.get_parameter('adaptive_threshold').value
        )
        self.min_intensity = float(self.get_parameter('min_intensity').value)
        self.range_band_m = float(self.get_parameter('range_band_m').value)
        self.mad_scale = float(self.get_parameter('mad_scale').value)
        self.intensity_margin = float(
            self.get_parameter('intensity_margin').value
        )
        self.min_points_per_band = int(
            self.get_parameter('min_points_per_band').value
        )

        self.angular_bin_deg = float(
            self.get_parameter('angular_bin_deg').value
        )
        self.radial_bin_m = float(self.get_parameter('radial_bin_m').value)
        self.min_radial_cells = int(
            self.get_parameter('min_radial_cells').value
        )
        self.max_radial_gap_cells = int(
            self.get_parameter('max_radial_gap_cells').value
        )
        self.first_return_only = bool(
            self.get_parameter('first_return_only').value
        )

        self.angular_window_bins = int(
            self.get_parameter('angular_window_bins').value
        )
        self.min_angular_support = int(
            self.get_parameter('min_angular_support').value
        )
        self.neighbor_range_tolerance_m = float(
            self.get_parameter('neighbor_range_tolerance_m').value
        )

        self._validate_parameters()

        self.angular_bin_count = max(
            1,
            int(round(360.0 / self.angular_bin_deg))
        )
        self.angular_bin_width_rad = (
            2.0 * math.pi / self.angular_bin_count
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        self.message_count = 0

        self.get_logger().info(
            f'Filtro Ping360 activo: {self.input_topic} -> {self.output_topic}'
        )
        self.get_logger().info(
            f'Rango útil: {self.min_range_m:.2f} a '
            f'{self.max_range_m:.2f} m | '
            f'bin angular: {360.0 / self.angular_bin_count:.2f}° | '
            f'bin radial: {self.radial_bin_m:.3f} m'
        )

    def _validate_parameters(self):
        if self.min_range_m < 0.0:
            raise ValueError('min_range_m no puede ser negativo')
        if self.max_range_m <= self.min_range_m:
            raise ValueError('max_range_m debe ser mayor que min_range_m')
        if self.range_band_m <= 0.0:
            raise ValueError('range_band_m debe ser mayor que cero')
        if self.angular_bin_deg <= 0.0:
            raise ValueError('angular_bin_deg debe ser mayor que cero')
        if self.radial_bin_m <= 0.0:
            raise ValueError('radial_bin_m debe ser mayor que cero')
        if self.min_radial_cells < 1:
            raise ValueError('min_radial_cells debe ser al menos 1')
        if self.max_radial_gap_cells < 0:
            raise ValueError('max_radial_gap_cells no puede ser negativo')
        if self.min_angular_support < 1:
            raise ValueError('min_angular_support debe ser al menos 1')
        if self.angular_window_bins < 0:
            raise ValueError('angular_window_bins no puede ser negativo')

    def pointcloud_callback(self, msg: PointCloud2):
        points = self._pointcloud_to_numpy(msg)

        if points is None:
            return

        if points.shape[0] == 0:
            self.publisher.publish(self._create_cloud(msg.header, points))
            return

        finite_mask = np.isfinite(points).all(axis=1)
        points = points[finite_mask]

        ranges = np.hypot(points[:, 0], points[:, 1])
        range_mask = (
            (ranges >= self.min_range_m)
            & (ranges <= self.max_range_m)
        )

        points = points[range_mask]
        ranges = ranges[range_mask]

        if points.shape[0] == 0:
            self.publisher.publish(self._create_cloud(msg.header, points))
            return

        intensity_mask = self._compute_intensity_mask(
            ranges,
            points[:, 3]
        )

        candidate_points = points[intensity_mask]
        candidate_ranges = ranges[intensity_mask]

        if candidate_points.shape[0] == 0:
            self.publisher.publish(
                self._create_cloud(msg.header, candidate_points)
            )
            return

        accepted_indices = self._spatial_consistency_filter(
            candidate_points,
            candidate_ranges
        )

        filtered_points = candidate_points[accepted_indices]

        self.publisher.publish(
            self._create_cloud(msg.header, filtered_points)
        )

        self.message_count += 1
        if self.message_count % 20 == 0:
            self.get_logger().info(
                f'Puntos entrada={msg.width * msg.height}, '
                f'candidatos={candidate_points.shape[0]}, '
                f'filtrados={filtered_points.shape[0]}'
            )

    def _compute_intensity_mask(
        self,
        ranges: np.ndarray,
        intensities: np.ndarray
    ) -> np.ndarray:
        if not self.adaptive_threshold:
            return intensities >= self.min_intensity

        keep = np.zeros(intensities.shape[0], dtype=bool)
        band_ids = np.floor(ranges / self.range_band_m).astype(np.int32)

        for band_id in np.unique(band_ids):
            indices = np.flatnonzero(band_ids == band_id)
            values = intensities[indices]

            if values.size < self.min_points_per_band:
                threshold = self.min_intensity
            else:
                median = float(np.median(values))
                mad = float(np.median(np.abs(values - median)))
                robust_sigma = 1.4826 * mad

                adaptive_part = max(
                    self.intensity_margin,
                    self.mad_scale * robust_sigma
                )
                threshold = max(
                    self.min_intensity,
                    median + adaptive_part
                )

            keep[indices] = values >= threshold

        return keep

    def _spatial_consistency_filter(
        self,
        points: np.ndarray,
        ranges: np.ndarray
    ) -> np.ndarray:
        angles = np.mod(
            np.arctan2(points[:, 1], points[:, 0]),
            2.0 * math.pi
        )
        angle_bins = np.floor(
            angles / self.angular_bin_width_rad
        ).astype(np.int32)
        angle_bins %= self.angular_bin_count

        radial_bins = np.floor(
            ranges / self.radial_bin_m
        ).astype(np.int32)

        candidates: List[Dict] = []
        candidates_by_angle: Dict[int, List[int]] = {}

        for angle_bin in np.unique(angle_bins):
            point_indices = np.flatnonzero(angle_bins == angle_bin)

            ordered = point_indices[
                np.argsort(radial_bins[point_indices], kind='stable')
            ]
            ordered_radial_bins = radial_bins[ordered]

            split_locations = np.flatnonzero(
                np.diff(ordered_radial_bins)
                > (self.max_radial_gap_cells + 1)
            ) + 1

            groups = np.split(ordered, split_locations)
            angle_candidates = []

            for group in groups:
                unique_radial_cells = np.unique(radial_bins[group]).size

                if unique_radial_cells < self.min_radial_cells:
                    continue

                candidate = {
                    'angle_bin': int(angle_bin),
                    'range_m': float(np.median(ranges[group])),
                    'min_range_m': float(np.min(ranges[group])),
                    'point_indices': group,
                }
                angle_candidates.append(candidate)

            if self.first_return_only and angle_candidates:
                angle_candidates = [
                    min(
                        angle_candidates,
                        key=lambda item: item['min_range_m']
                    )
                ]

            for candidate in angle_candidates:
                candidate_index = len(candidates)
                candidates.append(candidate)
                candidates_by_angle.setdefault(
                    candidate['angle_bin'],
                    []
                ).append(candidate_index)

        if not candidates:
            return np.empty(0, dtype=np.int64)

        accepted_point_indices = []

        for candidate in candidates:
            support = 1
            center_bin = candidate['angle_bin']
            center_range = candidate['range_m']

            for offset in range(1, self.angular_window_bins + 1):
                for sign in (-1, 1):
                    neighbor_bin = (
                        center_bin + sign * offset
                    ) % self.angular_bin_count

                    neighbor_candidates = candidates_by_angle.get(
                        neighbor_bin,
                        []
                    )

                    has_match = any(
                        abs(candidates[index]['range_m'] - center_range)
                        <= self.neighbor_range_tolerance_m
                        for index in neighbor_candidates
                    )

                    if has_match:
                        support += 1

            if support >= self.min_angular_support:
                accepted_point_indices.extend(
                    candidate['point_indices'].tolist()
                )

        if not accepted_point_indices:
            return np.empty(0, dtype=np.int64)

        return np.unique(
            np.asarray(accepted_point_indices, dtype=np.int64)
        )

    def _pointcloud_to_numpy(
        self,
        msg: PointCloud2
    ):
        field_map = {field.name: field for field in msg.fields}
        required_names = ('x', 'y', 'z', 'intensity')

        if not all(name in field_map for name in required_names):
            self.get_logger().error(
                'La nube debe contener x, y, z e intensity'
            )
            return None

        if any(
            field_map[name].datatype != PointField.FLOAT32
            for name in required_names
        ):
            self.get_logger().error(
                'Los campos x, y, z e intensity deben ser FLOAT32'
            )
            return None

        point_count = int(msg.width * msg.height)

        if point_count == 0:
            return np.empty((0, 4), dtype=np.float32)

        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({
            'names': list(required_names),
            'formats': [f'{endian}f4'] * 4,
            'offsets': [
                field_map[name].offset
                for name in required_names
            ],
            'itemsize': msg.point_step,
        })

        expected_bytes = point_count * msg.point_step
        if len(msg.data) < expected_bytes:
            self.get_logger().error(
                'El tamaño de msg.data no coincide con la nube'
            )
            return None

        structured = np.frombuffer(
            msg.data,
            dtype=dtype,
            count=point_count
        )

        return np.column_stack([
            structured['x'],
            structured['y'],
            structured['z'],
            structured['intensity'],
        ]).astype(np.float32, copy=False)

    def _create_cloud(
        self,
        source_header: Header,
        points: np.ndarray
    ) -> PointCloud2:
        output = PointCloud2()
        output.header = source_header
        output.height = 1
        output.width = int(points.shape[0])
        output.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            ),
        ]
        output.is_bigendian = False
        output.point_step = 16
        output.row_step = output.point_step * output.width
        output.is_dense = True

        if output.width == 0:
            output.data = b''
        else:
            output.data = np.asarray(
                points,
                dtype='<f4'
            ).reshape(-1, 4).tobytes()

        return output


def main(args=None):
    rclpy.init(args=args)
    node = Ping360PointCloudFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
