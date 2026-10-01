"""Pure helpers for bounded PointCloud2 decoding used by the Ping360 view."""

import math
import struct

FLOAT32 = 7


def decode_xy_intensity(msg, max_points=16000):
    """
    Return finite ``(x, y, intensity)`` samples without blocking the GUI.

    The Ping360 cloud is normally unorganized, but this decoder also respects
    ``row_step`` for organized PointCloud2 messages.  Large clouds are sampled
    uniformly so painting the dashboard cannot starve ROS callbacks.
    """
    if max_points < 1:
        raise ValueError('max_points debe ser al menos 1')

    fields = {field.name: field for field in msg.fields}
    if 'x' not in fields or 'y' not in fields:
        return []
    if fields['x'].datatype != FLOAT32 or fields['y'].datatype != FLOAT32:
        return []

    intensity_field = fields.get('intensity')
    if intensity_field is not None and intensity_field.datatype != FLOAT32:
        intensity_field = None

    width = max(0, int(msg.width))
    height = max(0, int(msg.height))
    point_step = int(msg.point_step)
    row_step = int(msg.row_step)
    if width == 0 or height == 0 or point_step <= 0 or row_step <= 0:
        return []

    total = width * height
    stride = max(1, int(math.ceil(total / float(max_points))))
    byte_order = '>' if msg.is_bigendian else '<'
    raw = memoryview(msg.data)
    points = []

    for flat_index in range(0, total, stride):
        row, column = divmod(flat_index, width)
        base = row * row_step + column * point_step
        try:
            x = struct.unpack_from(byte_order + 'f', raw, base + fields['x'].offset)[0]
            y = struct.unpack_from(byte_order + 'f', raw, base + fields['y'].offset)[0]
            intensity = (
                struct.unpack_from(byte_order + 'f', raw, base + intensity_field.offset)[0]
                if intensity_field is not None
                else 1.0
            )
        except (struct.error, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(intensity):
            points.append((x, y, intensity))

    return points
