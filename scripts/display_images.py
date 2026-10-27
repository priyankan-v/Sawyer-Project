"""PNG loading and ROS display-image conversion without Pillow or OpenCV."""

from pathlib import Path
import struct
import zlib

import numpy as np
from sensor_msgs.msg import Image

if __package__:
    from .clock_display import HEIGHT, WIDTH
else:
    from clock_display import HEIGHT, WIDTH


def read_png(path, background=(255, 255, 255)):
    """Decode noninterlaced 8-bit RGB/RGBA/grayscale/palette PNGs to uint8 BGR.

    Transparent pixels are composited over the specified RGB background.
    Unsupported formats fail explicitly instead of displaying corrupt pixels.
    """
    contents = Path(path).read_bytes()
    if not contents.startswith(b'\x89PNG\r\n\x1a\n'):
        raise ValueError(f'Not a PNG image: {path}')
    offset, compressed, palette, transparency, header = 8, bytearray(), None, None, None
    ended = False
    while offset + 12 <= len(contents):
        length = struct.unpack_from('!I', contents, offset)[0]
        kind = contents[offset+4:offset+8]
        payload = contents[offset+8:offset+8+length]
        if offset+12+length > len(contents):
            raise ValueError('Truncated PNG chunk')
        checksum = struct.unpack_from('!I', contents, offset+8+length)[0]
        if zlib.crc32(kind+payload) != checksum:
            raise ValueError('PNG checksum failed')
        if kind == b'IHDR':
            header = struct.unpack('!IIBBBBB', payload)
        elif kind == b'PLTE':
            palette = np.frombuffer(payload, dtype=np.uint8).reshape(-1, 3)
        elif kind == b'tRNS':
            transparency = payload
        elif kind == b'IDAT':
            compressed.extend(payload)
        elif kind == b'IEND':
            ended = True
            break
        offset += length + 12
    if header is None or not ended:
        raise ValueError('Incomplete PNG image')
    width, height, depth, color, compression, filtering, interlace = header
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(color)
    if (depth != 8 or channels is None or compression or filtering or interlace
            or width < 1 or height < 1 or width*height > 16_777_216):
        raise ValueError('PNG must be noninterlaced, 8-bit, and at most 16 megapixels')
    stride = width*channels
    expected = height*(stride+1)
    decoder = zlib.decompressobj()
    decoded = decoder.decompress(bytes(compressed), expected+1)
    if len(decoded) != expected or not decoder.eof:
        raise ValueError('PNG pixel data has an invalid length')
    output = np.empty((height, stride), dtype=np.uint8)
    for y in range(height):
        start = y*(stride+1)
        mode = decoded[start]
        raw = np.frombuffer(decoded, dtype=np.uint8, count=stride, offset=start+1)
        previous = output[y-1] if y else np.zeros(stride, dtype=np.uint8)
        if mode == 0:
            output[y] = raw
        elif mode == 1:
            output[y] = np.cumsum(raw.reshape(width, channels), axis=0, dtype=np.uint32).astype(np.uint8).ravel()
        elif mode == 2:
            output[y] = raw + previous
        elif mode in (3, 4):
            row = output[y]
            for x, value in enumerate(raw):
                a, b, c = (int(row[x-channels]) if x >= channels else 0), int(previous[x]), (int(previous[x-channels]) if x >= channels else 0)
                if mode == 3:
                    prediction = (a+b)//2
                else:
                    p = a+b-c
                    pa, pb, pc = abs(p-a), abs(p-b), abs(p-c)
                    prediction = a if pa <= pb and pa <= pc else b if pb <= pc else c
                row[x] = (int(value)+prediction) & 255
        else:
            raise ValueError(f'Invalid PNG filter {mode}')
    pixels = output.reshape(height, width, channels)
    alpha = None
    if color == 3:
        if palette is None or int(pixels.max()) >= len(palette):
            raise ValueError('Invalid PNG palette')
        indices = pixels[:, :, 0]
        rgb = palette[indices]
        if transparency:
            opacity = np.full(len(palette), 255, dtype=np.uint8)
            opacity[:len(transparency)] = np.frombuffer(transparency, dtype=np.uint8)
            alpha = opacity[indices]
    elif color in (0, 4):
        rgb = np.repeat(pixels[:, :, :1], 3, axis=2)
        if color == 4:
            alpha = pixels[:, :, 1]
    else:
        rgb = pixels[:, :, :3]
        if color == 6:
            alpha = pixels[:, :, 3]
    if alpha is not None:
        a = alpha[:, :, None].astype(np.uint16)
        rgb = ((rgb.astype(np.uint16)*a + np.asarray(background, dtype=np.uint16)*(255-a) + 127)//255).astype(np.uint8)
    return np.ascontiguousarray(rgb[:, :, ::-1])


def screen_image(path, background=(255, 255, 255)):
    image = read_png(path, background)
    height, width = image.shape[:2]
    if (height, width) == (HEIGHT, WIDTH):
        return image
    scale = min(WIDTH/width, HEIGHT/height)
    w, h = max(1, round(width*scale)), max(1, round(height*scale))
    resized = image[(np.arange(h)*height//h)[:, None], (np.arange(w)*width//w)[None, :]]
    canvas = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    canvas[:] = background[::-1]
    x, y = (WIDTH-w)//2, (HEIGHT-h)//2
    canvas[y:y+h, x:x+w] = resized
    return canvas


def image_message(bgr):
    if bgr.shape != (HEIGHT, WIDTH, 3) or bgr.dtype != np.uint8:
        raise ValueError('Head display requires a 600x1024x3 uint8 BGR image')
    message = Image()
    message.width, message.height, message.step = WIDTH, HEIGHT, WIDTH*3
    message.encoding, message.is_bigendian = 'bgr8', 0
    message.data = bgr.tobytes()
    return message
