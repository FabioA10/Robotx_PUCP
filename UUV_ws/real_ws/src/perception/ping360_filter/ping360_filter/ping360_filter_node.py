"""ROS 2 Ping360 filter with simulation-compatible tracking outputs."""

import json
import math
import time

from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker

from .filter_core import (
    filter_polar_grid,
    mask_to_laserscan_ranges,
    points_to_polar_grid,
    rotation_warmup_complete,
    select_contour_components,
    select_target_candidate,
    TemporalPointTracker,
    wrap_degrees,
)

FLOAT32 = PointField.FLOAT32


class Ping360FilterNode(Node):
    def __init__(self):
        super().__init__('ping360_filter')
        self._declare_parameters()
        self.angle_count = int(self.get_parameter('angle_count').value)
        self.range_bin_count = int(self.get_parameter('range_bin_count').value)
        self.angle_step_deg = float(self.get_parameter('angle_step_deg').value)
        self.range_step_m = float(self.get_parameter('range_step_m').value)
        self.range_step_initialized = not bool(self.get_parameter('auto_infer_range_step').value)
        self.previous_grid = None
        self.last_updated_angle = None
        self.wrap_count = 0
        self.message_count = 0
        self.first_contour_stamp = None
        self.first_contour_monotonic = None
        self.contour_elapsed_sec = 0.0
        self.contour_warmup_reported = False
        self.last_observation_monotonic = None
        self.last_observation_stamp = None
        self.last_target_xy = None
        self.last_candidate = None
        self.last_state = None
        self.tracker = TemporalPointTracker(
            association_distance_m=float(
                self.get_parameter('tracker_association_distance_m').value
            ),
            confirmation_hits=int(self.get_parameter('tracker_confirmation_rotations').value),
            maximum_misses=int(self.get_parameter('tracker_maximum_misses').value),
            smoothing_alpha=float(self.get_parameter('tracker_smoothing_alpha').value),
            minimum_valid_confidence=float(
                self.get_parameter('tracker_minimum_valid_confidence').value
            ),
        )

        self.filtered_publisher = self.create_publisher(PointCloud2, self._p('filtered_topic'), 10)
        self.legacy_filtered_publisher = self.create_publisher(
            PointCloud2, self._p('legacy_filtered_topic'), 10
        )
        self.scan_publisher = self.create_publisher(LaserScan, self._p('filtered_scan_topic'), 10)
        self.target_publisher = self.create_publisher(PointStamped, self._p('target_topic'), 10)
        self.distance_publisher = self.create_publisher(
            Float32, self._p('compatibility_distance_topic'), 10
        )
        self.bearing_publisher = self.create_publisher(
            Float32, self._p('compatibility_bearing_topic'), 10
        )
        self.heading_publisher = self.create_publisher(
            Float32, self._p('compatibility_heading_topic'), 10
        )
        self.status_publisher = self.create_publisher(String, self._p('tracking_status_topic'), 10)
        self.compatibility_status_publisher = self.create_publisher(
            String, self._p('compatibility_status_topic'), 10
        )
        self.stats_publisher = self.create_publisher(String, self._p('stats_topic'), 10)
        self.marker_publisher = self.create_publisher(Marker, self._p('tracking_marker_topic'), 10)
        self.contour_status_publisher = self.create_publisher(
            String, self._p('contour_status_topic'), 10
        )
        input_topic = self._p('input_topic')
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos_profile_sensor_data
        )
        self.status_timer = None
        if self._p('filter_mode') == 'compact_target':
            self.status_timer = self.create_timer(
                0.5,
                self._publish_freshness_status,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
        self.get_logger().info(
            f'Esperando {input_topic}; salida compatible: /ping360/real_scan y '
            '/pipeline/{distance,bearing,heading}.'
        )

    def _declare_parameters(self):
        defaults = {
            'input_topic': '/ping360/points',
            'filtered_topic': '/ping360/points_filtered',
            'legacy_filtered_topic': '/ping360/filtered_points',
            'filtered_scan_topic': '/ping360/real_scan',
            'target_topic': '/ping360/target',
            'tracking_marker_topic': '/ping360/tracking_marker',
            'contour_status_topic': '/ping360/pool_contour_status',
            'tracking_status_topic': '/ping360/tracking_status',
            'compatibility_status_topic': '/pipeline/status',
            'stats_topic': '/ping360/filter_stats',
            'compatibility_distance_topic': '/pipeline/distance',
            'compatibility_bearing_topic': '/pipeline/bearing',
            'compatibility_heading_topic': '/pipeline/heading',
            'angle_count': 100,
            'range_bin_count': 400,
            'angle_step_deg': 3.6,
            'range_step_m': 0.0124875,
            'auto_infer_range_step': True,
            'full_rotation_period_sec': 15.0,
            'filter_mode': 'pool_contour',
            'pool_length_m': 4.0,
            'pool_width_m': 2.0,
            'pool_range_margin_m': 0.15,
            'contour_max_range_m': 2.0,
            'contour_min_range_m': 0.70,
            'contour_min_intensity': 90.0,
            'contour_residual_threshold': 45.0,
            'contour_background_window_bins': 41,
            'contour_min_component_pixels': 4,
            'contour_min_points': 16,
            'contour_min_angular_span_deg': 15.0,
            'min_range_m': 0.70,
            'max_range_m': 4.80,
            'minimum_intensity': 90.0,
            'residual_threshold': 45.0,
            'background_window_bins': 41,
            'min_component_pixels': 4,
            'warmup_wraps': 1,
            'target_min_range_m': 0.80,
            'target_max_range_m': 4.00,
            'target_search_center_deg': 0.0,
            'target_search_half_width_deg': 70.0,
            'target_min_points': 4,
            'target_max_points': 180,
            'target_max_angular_span_deg': 25.0,
            'target_max_radial_span_m': 0.50,
            'target_max_extent_m': 0.55,
            'tracker_association_distance_m': 0.45,
            'tracker_confirmation_rotations': 3,
            'tracker_maximum_misses': 1,
            'tracker_smoothing_alpha': 0.45,
            'tracker_minimum_valid_confidence': 0.55,
            'tracking_timeout_sec': 22.5,
            'orbit_desired_bearing_deg': 90.0,
            'publish_simulation_compatibility': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _p(self, name):
        return str(self.get_parameter(name).value)

    @staticmethod
    def _field_offset(message, name):
        for field in message.fields:
            if field.name == name:
                if field.datatype != FLOAT32 or field.count != 1:
                    raise ValueError(f"El campo '{name}' debe ser FLOAT32")
                return int(field.offset)
        raise ValueError(f"PointCloud2 no contiene el campo '{name}'")

    def _read_cloud(self, message):
        count = int(message.width * message.height)
        endian = '>f4' if message.is_bigendian else '<f4'
        point_step = int(message.point_step)
        values = {}
        for name in ('x', 'y', 'intensity'):
            offset = self._field_offset(message, name)
            values[name] = np.ndarray(
                shape=(count,),
                dtype=endian,
                buffer=message.data,
                offset=offset,
                strides=(point_step,),
            ).astype(np.float32, copy=True)
        return values['x'], values['y'], values['intensity']

    def _infer_range_step(self, x, y):
        ranges = np.hypot(x, y)
        finite = ranges[np.isfinite(ranges)]
        if finite.size == 0:
            return
        inferred = float(np.max(finite) / max(1, self.range_bin_count - 1))
        if inferred <= 0.0:
            return
        self.range_step_m = inferred
        self.range_step_initialized = True
        self.get_logger().info(f'Paso radial inferido: {inferred:.7f} m/bin')

    def _cloud_to_grid(self, message):
        x, y, intensity = self._read_cloud(message)
        if not self.range_step_initialized:
            self._infer_range_step(x, y)
        return points_to_polar_grid(
            x=x,
            y=y,
            intensity=intensity,
            angle_count=self.angle_count,
            angle_step_deg=self.angle_step_deg,
            range_bin_count=self.range_bin_count,
            range_step_m=self.range_step_m,
        )

    def _contour_warmup_complete(self, message):
        stamp = message.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now = time.monotonic()
        if stamp_sec > 0.0:
            if self.first_contour_stamp is None or stamp_sec < self.first_contour_stamp:
                self.first_contour_stamp = stamp_sec
                self.first_contour_monotonic = now
                self.contour_elapsed_sec = 0.0
                self.contour_warmup_reported = False
            period = float(self.get_parameter('full_rotation_period_sec').value)
            rotations = int(self.get_parameter('warmup_wraps').value)
            self.contour_elapsed_sec = max(0.0, stamp_sec - self.first_contour_stamp)
            return rotation_warmup_complete(self.first_contour_stamp, stamp_sec, period, rotations)

        if self.first_contour_monotonic is None:
            self.first_contour_monotonic = now
            self.contour_warmup_reported = False
        required_sec = max(
            0.0,
            float(self.get_parameter('full_rotation_period_sec').value)
            * int(self.get_parameter('warmup_wraps').value),
        )
        self.contour_elapsed_sec = max(0.0, now - self.first_contour_monotonic)
        return self.contour_elapsed_sec >= required_sec

    def cloud_callback(self, message):
        self.message_count += 1
        try:
            grid = self._cloud_to_grid(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        # Each recorded Ping360 message contains a complete 360-degree cloud.
        # Do not look for a partial-angle wrap between messages.
        if self._p('filter_mode') == 'pool_contour':
            if not self._contour_warmup_complete(message):
                if not self.contour_warmup_reported:
                    self.get_logger().info('Acumulando la primera vuelta completa del Ping360.')
                    self.contour_warmup_reported = True
                return
            self.wrap_count = max(
                1,
                int(
                    self.contour_elapsed_sec
                    / max(
                        0.001,
                        float(self.get_parameter('full_rotation_period_sec').value),
                    )
                ),
            )
            self._process_pool_contour(message, grid)
            return

        if self.previous_grid is None:
            self.previous_grid = grid
            return
        changed_angles = np.flatnonzero(np.any(grid != self.previous_grid, axis=1))
        self.previous_grid = grid
        if changed_angles.size == 0:
            return
        updated_angle = int(changed_angles[-1])
        wrapped = self.last_updated_angle is not None and updated_angle < self.last_updated_angle
        self.last_updated_angle = updated_angle
        if not wrapped:
            return
        self.wrap_count += 1
        if self.wrap_count <= int(self.get_parameter('warmup_wraps').value):
            self.get_logger().info(f'Vuelta incompleta {self.wrap_count} descartada.')
            return
        self._process_complete_scan(message, grid)

    def _process_complete_scan(self, source_message, grid):
        if self._p('filter_mode') == 'pool_contour':
            self._process_pool_contour(source_message, grid)
            return

        smoothed, residual, mask, components = filter_polar_grid(
            grid=grid,
            angle_step_deg=self.angle_step_deg,
            range_step_m=self.range_step_m,
            min_range_m=float(self.get_parameter('min_range_m').value),
            max_range_m=float(self.get_parameter('max_range_m').value),
            minimum_intensity=float(self.get_parameter('minimum_intensity').value),
            residual_threshold=float(self.get_parameter('residual_threshold').value),
            background_window_bins=int(self.get_parameter('background_window_bins').value),
            min_component_pixels=int(self.get_parameter('min_component_pixels').value),
        )
        candidate, quality = select_target_candidate(
            components,
            min_range_m=float(self.get_parameter('target_min_range_m').value),
            max_range_m=float(self.get_parameter('target_max_range_m').value),
            search_center_deg=float(self.get_parameter('target_search_center_deg').value),
            search_half_width_deg=float(self.get_parameter('target_search_half_width_deg').value),
            min_points=int(self.get_parameter('target_min_points').value),
            max_points=int(self.get_parameter('target_max_points').value),
            max_angular_span_deg=float(self.get_parameter('target_max_angular_span_deg').value),
            max_radial_span_m=float(self.get_parameter('target_max_radial_span_m').value),
            max_extent_m=float(self.get_parameter('target_max_extent_m').value),
        )
        state = self.tracker.update(
            None if candidate is None else (candidate.centroid_x, candidate.centroid_y),
            quality,
        )

        cloud = self._make_filtered_cloud(source_message, smoothed, mask)
        self.filtered_publisher.publish(cloud)
        if self._p('legacy_filtered_topic') != self._p('filtered_topic'):
            self.legacy_filtered_publisher.publish(cloud)
        self.scan_publisher.publish(self._make_filtered_scan(source_message, smoothed, mask))
        self._publish_tracking(source_message, candidate, state)
        stats = {
            'input_messages': self.message_count,
            'completed_rotation': self.wrap_count,
            'raw_nonzero_points': int(np.count_nonzero(grid)),
            'filtered_points': int(np.count_nonzero(mask)),
            'components': len(components),
            'residual_max': round(float(np.max(residual)), 1),
            'candidate_quality': round(quality, 3),
            'target_valid': bool(state is not None and state.valid),
        }
        self.stats_publisher.publish(String(data=json.dumps(stats)))
        self.get_logger().info(json.dumps(stats))

    def _process_pool_contour(self, source_message, grid):
        pool_diagonal = math.hypot(
            float(self.get_parameter('pool_length_m').value),
            float(self.get_parameter('pool_width_m').value),
        )
        pool_range_cap = pool_diagonal + float(self.get_parameter('pool_range_margin_m').value)
        range_limit = min(
            max(0.0, float(self.get_parameter('contour_max_range_m').value)),
            pool_range_cap,
        )
        smoothed, residual, raw_mask, components = filter_polar_grid(
            grid=grid,
            angle_step_deg=self.angle_step_deg,
            range_step_m=self.range_step_m,
            min_range_m=float(self.get_parameter('contour_min_range_m').value),
            max_range_m=range_limit,
            minimum_intensity=float(self.get_parameter('contour_min_intensity').value),
            residual_threshold=float(self.get_parameter('contour_residual_threshold').value),
            background_window_bins=int(self.get_parameter('contour_background_window_bins').value),
            min_component_pixels=int(self.get_parameter('contour_min_component_pixels').value),
        )
        mask, contour_components = select_contour_components(
            raw_mask,
            components,
            min_points=int(self.get_parameter('contour_min_points').value),
            min_angular_span_deg=float(self.get_parameter('contour_min_angular_span_deg').value),
        )
        cloud = self._make_filtered_cloud(source_message, smoothed, mask)
        self.filtered_publisher.publish(cloud)
        if self._p('legacy_filtered_topic') != self._p('filtered_topic'):
            self.legacy_filtered_publisher.publish(cloud)
        self.scan_publisher.publish(self._make_filtered_scan(source_message, smoothed, mask))
        accepted_ranges = np.flatnonzero(mask.any(axis=0)) * self.range_step_m
        bearing_coverage = float(np.count_nonzero(mask.any(axis=1))) / max(1, self.angle_count)
        status = {
            'mode': 'pool_contour',
            'state': 'CONTOUR_CANDIDATES' if contour_components else 'NO_CONTOUR',
            'has_candidates': bool(contour_components),
            'contour_points': int(np.count_nonzero(mask)),
            'contour_components': len(contour_components),
            'bearing_coverage_fraction': round(bearing_coverage, 3),
            'range_limit_m': round(range_limit, 3),
            'range_min_m': (
                None if accepted_ranges.size == 0 else round(float(np.min(accepted_ranges)), 3)
            ),
            'range_max_m': (
                None if accepted_ranges.size == 0 else round(float(np.max(accepted_ranges)), 3)
            ),
            'scan_period_sec': float(self.get_parameter('full_rotation_period_sec').value),
            'motion_compensated': False,
        }
        self.contour_status_publisher.publish(String(data=json.dumps(status)))
        stats = {
            'input_messages': self.message_count,
            'completed_rotation': self.wrap_count,
            'raw_nonzero_points': int(np.count_nonzero(grid)),
            'contour_points': int(np.count_nonzero(mask)),
            'contour_components': len(contour_components),
            'bearing_coverage_fraction': round(bearing_coverage, 3),
            'range_limit_m': round(range_limit, 3),
            'residual_max': round(float(np.max(residual)), 1),
        }
        self.stats_publisher.publish(String(data=json.dumps(stats)))
        self.get_logger().info(json.dumps(stats))

    def _make_filtered_cloud(self, source_message, intensity_grid, mask):
        angle_indices, range_indices = np.where(mask)
        angles = np.deg2rad(angle_indices * self.angle_step_deg)
        ranges = range_indices * self.range_step_m
        points = np.empty((angle_indices.size, 4), dtype=np.float32)
        points[:, 0] = ranges * np.cos(angles)
        points[:, 1] = ranges * np.sin(angles)
        points[:, 2] = 0.0
        points[:, 3] = intensity_grid[angle_indices, range_indices]
        output = PointCloud2()
        output.header = source_message.header
        output.height = 1
        output.width = int(points.shape[0])
        output.fields = [
            PointField(name='x', offset=0, datatype=FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=FLOAT32, count=1),
        ]
        output.is_bigendian = False
        output.point_step = 16
        output.row_step = int(output.width * output.point_step)
        output.data = points.tobytes()
        output.is_dense = True
        return output

    def _make_filtered_scan(self, source_message, intensity_grid, mask):
        ranges, intensities = mask_to_laserscan_ranges(mask, intensity_grid, self.range_step_m)
        split = self.angle_count // 2
        ranges = np.concatenate((ranges[split:], ranges[:split]))
        intensities = np.concatenate((intensities[split:], intensities[:split]))
        message = LaserScan()
        message.header = source_message.header
        message.angle_min = -math.pi
        message.angle_increment = math.radians(self.angle_step_deg)
        message.angle_max = message.angle_min + (self.angle_count - 1) * message.angle_increment
        message.time_increment = 0.0
        message.scan_time = float(self.get_parameter('full_rotation_period_sec').value)
        message.range_min = float(self.get_parameter('min_range_m').value)
        if self._p('filter_mode') == 'pool_contour':
            message.range_min = float(self.get_parameter('contour_min_range_m').value)
            message.range_max = math.hypot(
                float(self.get_parameter('pool_length_m').value),
                float(self.get_parameter('pool_width_m').value),
            ) + float(self.get_parameter('pool_range_margin_m').value)
        else:
            message.range_max = float(self.get_parameter('max_range_m').value)
        message.ranges = ranges.tolist()
        message.intensities = intensities.tolist()
        return message

    def _publish_tracking(self, source_message, candidate, state):
        self.last_candidate = candidate
        self.last_state = state
        if candidate is not None:
            if state is not None:
                self.last_target_xy = (float(state.x), float(state.y))
            else:
                self.last_target_xy = (float(candidate.centroid_x), float(candidate.centroid_y))
            self.last_observation_monotonic = time.monotonic()
            stamp = source_message.header.stamp
            self.last_observation_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        valid = state is not None and state.valid
        status = self._tracking_status_payload()
        if valid:
            target = PointStamped()
            target.header = source_message.header
            target.point.x = float(state.x)
            target.point.y = float(state.y)
            target.point.z = 0.0
            self.target_publisher.publish(target)
            if bool(self.get_parameter('publish_simulation_compatibility').value):
                distance = float(np.hypot(state.x, state.y))
                bearing = wrap_degrees(float(np.degrees(np.arctan2(state.y, state.x))))
                desired = float(self.get_parameter('orbit_desired_bearing_deg').value)
                tangent_error = wrap_degrees(bearing - desired)
                self.distance_publisher.publish(Float32(data=distance))
                self.bearing_publisher.publish(Float32(data=bearing))
                self.heading_publisher.publish(Float32(data=tangent_error))
                status['compatibility_heading_deg'] = round(tangent_error, 2)
        self._publish_status_payload(status)
        self.marker_publisher.publish(self._make_target_marker(source_message, candidate, state))

    def _tracking_status_payload(self):
        state = self.last_state
        now = time.monotonic()
        age = None
        if self.last_observation_monotonic is not None:
            age = max(0.0, now - self.last_observation_monotonic)
        timeout = float(self.get_parameter('tracking_timeout_sec').value)
        fresh = age is not None and age <= timeout
        valid = state is not None and state.valid and fresh
        if valid:
            state_name = 'TARGET_TRACKED'
        elif age is not None and not fresh:
            state_name = 'TARGET_STALE'
        elif self.last_candidate is not None and fresh:
            state_name = 'TARGET_CANDIDATE'
        elif state is not None and state.misses:
            state_name = 'TARGET_OCCLUDED'
        else:
            state_name = 'TARGET_NOT_DETECTED'

        payload = {
            'state': state_name,
            'valid': bool(valid),
            'distance_m': None,
            'angle_deg': None,
            'bearing_deg': None,
            'confidence': 0.0,
            'age_sec': None if age is None else round(age, 3),
            'hits': 0 if state is None else int(state.hits),
            'misses': 0 if state is None else int(state.misses),
            'stamp_sec': self.last_observation_stamp,
        }
        if self.last_target_xy is not None:
            x, y = self.last_target_xy
            distance = float(np.hypot(x, y))
            angle = wrap_degrees(float(np.degrees(np.arctan2(y, x))))
            payload.update(
                x_m=round(x, 3),
                y_m=round(y, 3),
                distance_m=round(distance, 3),
                angle_deg=round(angle, 2),
                bearing_deg=round(angle, 2),
            )
        if state is not None and fresh:
            payload['confidence'] = round(float(state.confidence), 3)
        return payload

    def _publish_status_payload(self, payload):
        message = String(data=json.dumps(payload, separators=(',', ':')))
        self.status_publisher.publish(message)
        if self._p('compatibility_status_topic') != self._p('tracking_status_topic'):
            self.compatibility_status_publisher.publish(message)

    def _publish_freshness_status(self):
        self._publish_status_payload(self._tracking_status_payload())

    @staticmethod
    def _make_target_marker(source_message, candidate, state):
        marker = Marker()
        marker.header = source_message.header
        marker.ns = 'ping360_tracking'
        marker.id = 0
        marker.pose.orientation.w = 1.0
        if candidate is None and state is None:
            marker.action = Marker.DELETE
            return marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(state.x if state is not None else candidate.centroid_x)
        marker.pose.position.y = float(state.y if state is not None else candidate.centroid_y)
        marker.pose.position.z = 0.10
        marker.scale.x = marker.scale.y = marker.scale.z = 0.20
        if state is not None and state.valid:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.1
        else:
            marker.color.r = 1.0
            marker.color.g = 0.75
            marker.color.b = 0.0
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = Ping360FilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
