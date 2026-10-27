"""Dependency-light digital clock rendering for Sawyer's 1024x600 head screen."""

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import struct
from zoneinfo import ZoneInfo
import zlib

import numpy as np


WIDTH, HEIGHT = 1024, 600
# Colors are BGR, matching the Intera SDK's head-display image encoding.
BACKGROUND = (22, 16, 10)
PANEL = (31, 23, 16)
TEXT = (242, 246, 249)
ACCENT = (224, 207, 75)
MUTED = (151, 134, 111)
INACTIVE = (48, 37, 27)

# Compact bitmap glyphs keep this renderer independent of Pillow/OpenCV/fonts.
GLYPHS = {
    'A': '01110 10001 10001 11111 10001 10001 10001',
    'B': '11110 10001 10001 11110 10001 10001 11110',
    'C': '01111 10000 10000 10000 10000 10000 01111',
    'D': '11110 10001 10001 10001 10001 10001 11110',
    'E': '11111 10000 10000 11110 10000 10000 11111',
    'F': '11111 10000 10000 11110 10000 10000 10000',
    'G': '01111 10000 10000 10111 10001 10001 01111',
    'H': '10001 10001 10001 11111 10001 10001 10001',
    'I': '11111 00100 00100 00100 00100 00100 11111',
    'J': '00111 00010 00010 00010 00010 10010 01100',
    'K': '10001 10010 10100 11000 10100 10010 10001',
    'L': '10000 10000 10000 10000 10000 10000 11111',
    'M': '10001 11011 10101 10101 10001 10001 10001',
    'N': '10001 11001 10101 10011 10001 10001 10001',
    'O': '01110 10001 10001 10001 10001 10001 01110',
    'P': '11110 10001 10001 11110 10000 10000 10000',
    'Q': '01110 10001 10001 10001 10101 10010 01101',
    'R': '11110 10001 10001 11110 10100 10010 10001',
    'S': '01111 10000 10000 01110 00001 00001 11110',
    'T': '11111 00100 00100 00100 00100 00100 00100',
    'U': '10001 10001 10001 10001 10001 10001 01110',
    'V': '10001 10001 10001 10001 10001 01010 00100',
    'W': '10001 10001 10001 10101 10101 10101 01010',
    'X': '10001 10001 01010 00100 01010 10001 10001',
    'Y': '10001 10001 01010 00100 00100 00100 00100',
    'Z': '11111 00001 00010 00100 01000 10000 11111',
    '0': '01110 10001 10011 10101 11001 10001 01110',
    '1': '00100 01100 00100 00100 00100 00100 01110',
    '2': '01110 10001 00001 00010 00100 01000 11111',
    '3': '11110 00001 00001 01110 00001 00001 11110',
    '4': '00010 00110 01010 10010 11111 00010 00010',
    '5': '11111 10000 10000 11110 00001 00001 11110',
    '6': '01110 10000 10000 11110 10001 10001 01110',
    '7': '11111 00001 00010 00100 01000 01000 01000',
    '8': '01110 10001 10001 01110 10001 10001 01110',
    '9': '01110 10001 10001 01111 00001 00001 01110',
    ':': '00000 00100 00100 00000 00100 00100 00000',
    '/': '00001 00001 00010 00100 01000 10000 10000',
    '-': '00000 00000 00000 11111 00000 00000 00000',
    '+': '00000 00100 00100 11111 00100 00100 00000',
    '_': '00000 00000 00000 00000 00000 00000 11111',
    ' ': '00000 00000 00000 00000 00000 00000 00000',
    '?': '01110 10001 00001 00010 00100 00000 00100',
}
DIGITS = {'0': 'abcdef', '1': 'bc', '2': 'abdeg', '3': 'abcdg', '4': 'bcfg',
          '5': 'acdfg', '6': 'acdefg', '7': 'abc', '8': 'abcdefg', '9': 'abcdfg'}
WEEKDAYS = ('MONDAY', 'TUESDAY', 'WEDNESDAY', 'THURSDAY', 'FRIDAY', 'SATURDAY', 'SUNDAY')
MONTHS = ('JANUARY', 'FEBRUARY', 'MARCH', 'APRIL', 'MAY', 'JUNE', 'JULY',
          'AUGUST', 'SEPTEMBER', 'OCTOBER', 'NOVEMBER', 'DECEMBER')


@dataclass(frozen=True)
class ClockFields:
    time: str
    date: str
    zone: str


def clock_fields(instant, zone):
    if instant.tzinfo is None or instant.utcoffset() is None:
        raise ValueError('Clock timestamps must be timezone-aware')
    local = instant.astimezone(zone)
    offset = local.strftime('%z')
    return ClockFields(
        local.strftime('%H:%M:%S'),
        f'{WEEKDAYS[local.weekday()]} {local.day:02d} {MONTHS[local.month-1]} {local.year}',
        f'{zone.key.upper()}  {local.tzname()}  UTC{offset[:3]}:{offset[3:]}',
    )


def rectangle(canvas, x, y, width, height, color):
    canvas[max(0,y):min(HEIGHT,y+height), max(0,x):min(WIDTH,x+width)] = color


def text(canvas, value, y, scale=3, color=TEXT, center=WIDTH//2):
    scale = max(1, min(scale, (WIDTH-40)//max(1, len(value)*6)))
    x = center - (len(value)*6-1)*scale//2
    for character in value.upper():
        for row, pattern in enumerate(GLYPHS.get(character, GLYPHS['?']).split()):
            for column, pixel in enumerate(pattern):
                if pixel == '1':
                    rectangle(canvas, x+column*scale, y+row*scale, scale, scale, color)
        x += 6*scale


def digit(canvas, value, x, y, color):
    width, height, thickness, gap = 108, 196, 14, 4
    middle = height//2
    segments = {
        'a': (thickness, 0, width-2*thickness, thickness),
        'b': (width-thickness, thickness+gap, thickness, middle-thickness-2*gap),
        'c': (width-thickness, middle+gap, thickness, middle-thickness-2*gap),
        'd': (thickness, height-thickness, width-2*thickness, thickness),
        'e': (0, middle+gap, thickness, middle-thickness-2*gap),
        'f': (0, thickness+gap, thickness, middle-thickness-2*gap),
        'g': (thickness+gap, middle-thickness//2, width-2*thickness-2*gap, thickness),
    }
    for name, (dx, dy, w, h) in segments.items():
        rectangle(canvas, x+dx, y+dy, w, h, color if name in DIGITS[value] else INACTIVE)


def render_clock(instant=None, zone=None):
    """Return a contiguous uint8 BGR image, with no ROS or image-library dependency."""
    zone = zone or ZoneInfo('America/Chicago')
    fields = clock_fields(instant or datetime.now(timezone.utc), zone)
    canvas = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    canvas[:] = BACKGROUND
    rectangle(canvas, 48, 150, 928, 260, PANEL)
    rectangle(canvas, 48, 150, 928, 2, INACTIVE)
    rectangle(canvas, 48, 408, 928, 2, INACTIVE)
    text(canvas, 'SAWYER', 55, scale=6, color=ACCENT)
    text(canvas, 'LOCAL TIME', 117, scale=2, color=MUTED)
    x = (WIDTH - (6*108 + 2*30 + 7*18))//2
    for index, character in enumerate(fields.time):
        color = ACCENT if index >= 6 else TEXT
        if character == ':':
            for cy in (235, 295):
                rectangle(canvas, x+8, cy, 14, 14, MUTED)
            x += 30 + 18
        else:
            digit(canvas, character, x, 175, color)
            x += 108 + 18
    for label, center in (('HOURS', 212), ('MINUTES', 512), ('SECONDS', 812)):
        text(canvas, label, 385, scale=2, color=MUTED, center=center)
    text(canvas, fields.date, 444, scale=4)
    text(canvas, fields.zone, 519, scale=3, color=ACCENT)
    return canvas


def write_png(path, bgr):
    """Write an offline preview using the standard library PNG/zlib formats."""
    def chunk(kind, payload):
        return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind+payload))
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    height, width = rgb.shape[:2]
    scanlines = b''.join(b'\0' + row.tobytes() for row in rgb)
    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!IIBBBBB', width, height, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(scanlines)) + chunk(b'IEND', b''))
    Path(path).write_bytes(png)
