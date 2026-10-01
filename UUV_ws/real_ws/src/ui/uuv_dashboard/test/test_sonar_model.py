import struct
from types import SimpleNamespace
import unittest

from uuv_dashboard.sonar_model import decode_xy_intensity, FLOAT32


def field(name, offset, datatype=FLOAT32):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype)


def cloud(points, bigendian=False):
    order = '>' if bigendian else '<'
    data = b''.join(
        struct.pack(order + 'ffff', x, y, z, intensity) for x, y, z, intensity in points
    )
    return SimpleNamespace(
        fields=[field('x', 0), field('y', 4), field('z', 8), field('intensity', 12)],
        width=len(points),
        height=1,
        point_step=16,
        row_step=16 * len(points),
        is_bigendian=bigendian,
        data=data,
    )


class SonarModelTests(unittest.TestCase):
    def test_decodes_little_endian_cloud(self):
        result = decode_xy_intensity(
            cloud(
                [
                    (1.0, -2.0, 0.0, 25.0),
                    (3.0, 4.0, 0.0, 90.0),
                ]
            )
        )
        self.assertEqual(result, [(1.0, -2.0, 25.0), (3.0, 4.0, 90.0)])

    def test_decodes_big_endian_cloud(self):
        result = decode_xy_intensity(cloud([(1.5, 2.5, 0.0, 30.0)], bigendian=True))
        self.assertEqual(result, [(1.5, 2.5, 30.0)])

    def test_bounds_number_of_decoded_points(self):
        result = decode_xy_intensity(
            cloud([(float(index), 0.0, 0.0, 1.0) for index in range(100)]), max_points=10
        )
        self.assertEqual(len(result), 10)

    def test_rejects_missing_xy_fields(self):
        msg = cloud([(1.0, 2.0, 0.0, 3.0)])
        msg.fields = [field('intensity', 12)]
        self.assertEqual(decode_xy_intensity(msg), [])

    def test_rejects_invalid_maximum(self):
        with self.assertRaises(ValueError):
            decode_xy_intensity(cloud([]), max_points=0)


if __name__ == '__main__':
    unittest.main()
