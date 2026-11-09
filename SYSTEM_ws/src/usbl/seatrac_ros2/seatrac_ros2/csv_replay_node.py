#!/usr/bin/env python3
"""Replay normalized USBL fixes without requiring SeaTrac hardware."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from seatrac_ros2.csv_fixes import CsvFormatError, load_csv_fixes
from seatrac_ros2.fix_validation import (
    FixValidator,
    ValidationConfig,
    pose_covariance,
)


class CsvReplayNode(Node):
    """Replay CSV rows through the same validation used by a real driver."""

    def __init__(self) -> None:
        super().__init__("seatrac_csv_replay")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("topic", "/usbl/relative_pose")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("stamp_mode", "now")
        self.declare_parameter("expected_beacon_id", -1)
        self.declare_parameter("warn_fit_error", 2.0)
        self.declare_parameter("reject_fit_error", 3.0)
        self.declare_parameter("max_horizontal_speed_mps", 3.0)
        self.declare_parameter("max_vertical_speed_mps", 1.5)

        csv_path = str(self.get_parameter("csv_path").value)
        if not csv_path:
            raise RuntimeError("csv_path parameter must not be empty")
        try:
            self.samples = load_csv_fixes(csv_path)
        except (OSError, CsvFormatError) as error:
            raise RuntimeError(f"cannot load USBL CSV: {error}") from error

        self.speed = float(self.get_parameter("speed").value)
        if not math.isfinite(self.speed) or self.speed <= 0.0:
            raise RuntimeError("speed must be finite and greater than zero")
        self.loop = bool(self.get_parameter("loop").value)
        self.stamp_mode = str(self.get_parameter("stamp_mode").value)
        if self.stamp_mode not in {"now", "recorded"}:
            raise RuntimeError("stamp_mode must be 'now' or 'recorded'")

        expected_id = int(self.get_parameter("expected_beacon_id").value)
        config = ValidationConfig(
            expected_beacon_id=None if expected_id < 0 else expected_id,
            warn_fit_error=float(
                self.get_parameter("warn_fit_error").value
            ),
            reject_fit_error=float(
                self.get_parameter("reject_fit_error").value
            ),
            max_horizontal_speed_mps=float(
                self.get_parameter("max_horizontal_speed_mps").value
            ),
            max_vertical_speed_mps=float(
                self.get_parameter("max_vertical_speed_mps").value
            ),
        )
        self.validator = FixValidator(config)
        topic = str(self.get_parameter("topic").value)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, topic, qos_profile_sensor_data
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.index = 0
        self.wall_start_s = time.monotonic()
        self.record_start_s = self.samples[0].timestamp_s
        self.record_stamp_offset_s = 0.0
        self.loop_span_s = self._calculate_loop_span()
        self.accepted_count = 0
        self.rejected_count = 0
        self.finished_reported = False
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            f"Loaded {len(self.samples)} USBL fixes from {csv_path}; "
            f"publishing {topic} in frame {self.frame_id}."
        )

    def tick(self) -> None:
        """Publish all records due at this replay instant."""
        if self.index >= len(self.samples):
            if self.loop:
                self._restart_loop()
            elif not self.finished_reported:
                self._publish_diagnostic(
                    DiagnosticStatus.WARN, "replay_complete", None
                )
                self.finished_reported = True
            return

        elapsed_record_s = (
            time.monotonic() - self.wall_start_s
        ) * self.speed
        while self.index < len(self.samples):
            sample = self.samples[self.index]
            due_s = sample.timestamp_s - self.record_start_s
            if due_s > elapsed_record_s:
                break
            self._process(sample)
            self.index += 1

    def _process(self, sample) -> None:
        result = self.validator.validate(sample)
        if not result.accepted:
            self.rejected_count += 1
            self._publish_diagnostic(
                DiagnosticStatus.WARN, result.reason, sample
            )
            return

        message = PoseWithCovarianceStamped()
        if self.stamp_mode == "now":
            message.header.stamp = self.get_clock().now().to_msg()
        else:
            stamp_s = sample.timestamp_s + self.record_stamp_offset_s
            seconds = math.floor(stamp_s)
            nanoseconds = int(
                round((stamp_s - seconds) * 1.0e9)
            )
            if nanoseconds == 1_000_000_000:
                seconds += 1
                nanoseconds = 0
            message.header.stamp.sec = int(seconds)
            message.header.stamp.nanosec = nanoseconds
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x = sample.x_m
        message.pose.pose.position.y = sample.y_m
        message.pose.pose.position.z = sample.z_m
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance = pose_covariance(sample)
        self.pose_publisher.publish(message)
        self.accepted_count += 1
        level = DiagnosticStatus.WARN if result.degraded else DiagnosticStatus.OK
        self._publish_diagnostic(level, result.reason, sample)

    def _restart_loop(self) -> None:
        self.index = 0
        self.wall_start_s = time.monotonic()
        self.record_stamp_offset_s += self.loop_span_s
        self.validator.reset()

    def _calculate_loop_span(self) -> float:
        if len(self.samples) == 1:
            return 1.0
        positive_intervals = [
            right.timestamp_s - left.timestamp_s
            for left, right in zip(self.samples, self.samples[1:])
            if right.timestamp_s > left.timestamp_s
        ]
        final_interval = positive_intervals[-1] if positive_intervals else 1.0
        return (
            self.samples[-1].timestamp_s
            - self.samples[0].timestamp_s
            + final_interval
        )

    def _publish_diagnostic(self, level, reason, sample) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "usbl/seatrac_csv_replay"
        status.hardware_id = "replay"
        status.message = reason
        values = {
            "state": "accepted" if level == DiagnosticStatus.OK else reason,
            "accepted_count": str(self.accepted_count),
            "rejected_count": str(self.rejected_count),
            "row_index": str(self.index),
        }
        if sample is not None:
            values["beacon_id"] = str(sample.beacon_id)
            values["fit_error"] = str(sample.fit_error)
            values["range_m"] = str(sample.range_m)
            values["rssi_db"] = str(sample.rssi_db)
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        array.status = [status]
        self.diagnostics_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsvReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
