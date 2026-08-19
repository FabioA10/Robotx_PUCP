import math
import os
import struct
import subprocess
import tempfile
import wave

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class AudioFeedbackNode(Node):

    def __init__(self):
        super().__init__('uuv_audio_feedback')

        self.declare_parameter('enabled', True)
        self.declare_parameter('volume', 0.35)

        self.enabled = (
            self.get_parameter('enabled')
            .get_parameter_value()
            .bool_value
        )

        self.volume = (
            self.get_parameter('volume')
            .get_parameter_value()
            .double_value
        )

        self.volume = max(
            0.0,
            min(1.0, self.volume)
        )

        self.previous_armed = None

        self.arm_sound = os.path.join(
            tempfile.gettempdir(),
            'uuv_arm_sound.wav'
        )

        self.disarm_sound = os.path.join(
            tempfile.gettempdir(),
            'uuv_disarm_sound.wav'
        )

        self.create_arm_sound(
            self.arm_sound
        )

        self.create_disarm_sound(
            self.disarm_sound
        )

        self.subscription = self.create_subscription(
            Bool,
            '/uuv/armed',
            self.armed_callback,
            10
        )

        self.get_logger().info(
            'UUV audio feedback ready'
        )

    # ---------------------------------------------------------
    # WAV generation
    # ---------------------------------------------------------

    def generate_tone_sequence(
        self,
        filename,
        tones,
        sample_rate=44100
    ):

        samples = []

        for frequency, duration in tones:

            total_samples = int(
                sample_rate * duration
            )

            for i in range(total_samples):

                # Small fade to avoid clicks
                fade_samples = int(
                    sample_rate * 0.015
                )

                gain = 1.0

                if i < fade_samples:
                    gain = i / fade_samples

                elif i > (
                    total_samples
                    - fade_samples
                ):
                    gain = (
                        total_samples - i
                    ) / fade_samples

                value = (
                    math.sin(
                        2.0
                        * math.pi
                        * frequency
                        * i
                        / sample_rate
                    )
                    * self.volume
                    * gain
                )

                sample = int(
                    value * 32767
                )

                samples.append(sample)

            # Short silence between notes

            silence_samples = int(
                sample_rate * 0.04
            )

            samples.extend(
                [0] * silence_samples
            )

        with wave.open(
            filename,
            'w'
        ) as wav_file:

            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(
                sample_rate
            )

            data = b''.join(
                struct.pack('<h', sample)
                for sample in samples
            )

            wav_file.writeframes(data)

    def create_arm_sound(self, filename):

        # Ascending startup tone
        tones = [
            (650, 0.12),
            (850, 0.12),
            (1100, 0.20),
        ]

        self.generate_tone_sequence(
            filename,
            tones
        )

    def create_disarm_sound(
        self,
        filename
    ):

        # Descending shutdown tone
        tones = [
            (1000, 0.12),
            (750, 0.12),
            (500, 0.22),
        ]

        self.generate_tone_sequence(
            filename,
            tones
        )

    # ---------------------------------------------------------
    # Playback
    # ---------------------------------------------------------

    def play_sound(self, filename):

        if not self.enabled:
            return

        try:

            subprocess.Popen(
                [
                    'paplay',
                    filename
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

        except FileNotFoundError:

            self.get_logger().warning(
                'paplay not found. '
                'Audio feedback unavailable.'
            )

    # ---------------------------------------------------------
    # Armed state
    # ---------------------------------------------------------

    def armed_callback(self, msg):

        armed = msg.data

        # Ignore initial state so starting ROS while already
        # disarmed does not produce a shutdown sound.

        if self.previous_armed is None:

            self.previous_armed = armed

            self.get_logger().info(
                f'Initial vehicle state: '
                f'{"ARMED" if armed else "DISARMED"}'
            )

            return

        if armed == self.previous_armed:
            return

        if armed:

            self.get_logger().warning(
                'Vehicle ARMED - playing startup sound'
            )

            self.play_sound(
                self.arm_sound
            )

        else:

            self.get_logger().info(
                'Vehicle DISARMED - playing shutdown sound'
            )

            self.play_sound(
                self.disarm_sound
            )

        self.previous_armed = armed


def main(args=None):

    rclpy.init(args=args)

    node = AudioFeedbackNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
