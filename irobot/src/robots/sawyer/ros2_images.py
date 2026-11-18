"""Fast standard sensor_msgs/Image CDR encoding for rclpy raw publication."""

from __future__ import annotations

import struct

from .core.communication import wire

CDR_LITTLE_ENDIAN = b'\x00\x01\x00\x00'


def serialize_image(image, stamp):
    """
    Encode the standard XCDR1 layout without converting every pixel to Python.

    rclpy Publisher.publish accepts serialized bytes. Metadata stays identical
    to sensor_msgs/Image; cameras retain their full resolution and encoding.
    """
    payload = bytearray(CDR_LITTLE_ENDIAN)

    def integer(value):
        payload.extend(b'\x00' * (-len(payload) % 4))
        payload.extend(struct.pack('<I', value))

    def string(value):
        if '\x00' in value:
            msg = 'CDR strings cannot contain embedded nulls'
            raise ValueError(msg)
        encoded = value.encode('utf-8') + b'\x00'
        integer(len(encoded))
        payload.extend(encoded)

    payload.extend(struct.pack('<iI', stamp.sec, stamp.nanosec))
    string(image.header.frame_id)
    integer(image.height)
    integer(image.width)
    string(image.encoding)
    payload.extend(struct.pack('B', image.is_bigendian))
    integer(image.step)
    integer(len(image.data))
    payload.extend(image.data)
    return bytes(payload)


def deserialize_image(payload):
    """Read a serialized Image efficiently for rclpy raw=True consumers."""
    data = memoryview(payload)
    if data[:4] != CDR_LITTLE_ENDIAN:
        msg = 'Expected little-endian XCDR1 Image'
        raise ValueError(msg)
    offset = 4

    def integer():
        nonlocal offset
        offset += -offset % 4
        value = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        return value

    def string():
        nonlocal offset
        length = integer()
        if (
            not length
            or offset + length > len(data)
            or data[offset + length - 1] != 0
        ):
            msg = 'Malformed CDR string'
            raise ValueError(msg)
        value = bytes(data[offset : offset + length - 1]).decode('utf-8')
        offset += length
        return value

    stamp = wire.Time(*struct.unpack_from('<iI', data, offset))
    offset += 8
    frame = string()
    height, width, encoding = integer(), integer(), string()
    bigendian = data[offset]
    offset += 1
    step, length = integer(), integer()
    if offset + length != len(data):
        msg = 'Malformed Image data length'
        raise ValueError(msg)
    image = wire.Image(
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=bigendian,
        step=step,
        data=bytes(data[offset:]),
    )
    image.header.frame_id, image.header.stamp = frame, stamp
    return image
