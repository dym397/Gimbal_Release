#!/usr/bin/env python3
"""Generate a black/white chessboard image for camera calibration."""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(chunk_type)
    crc = zlib.crc32(data, crc)
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", crc & 0xFFFFFFFF)


def write_grayscale_png(path: Path, width: int, height: int, pixels: bytearray) -> None:
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # PNG filter type: None
        start = y * width
        raw.extend(pixels[start : start + width])

    data = b"".join(
        [
            b"\x89PNG\r\n\x1a\n",
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)),
            _png_chunk(b"IDAT", zlib.compress(bytes(raw), level=9)),
            _png_chunk(b"IEND", b""),
        ]
    )
    path.write_bytes(data)


def generate_chessboard(
    output: Path,
    cols: int,
    rows: int,
    square_px: int,
    margin_px: int,
    invert: bool,
) -> tuple[int, int]:
    if cols <= 1 or rows <= 1:
        raise ValueError("cols and rows must both be greater than 1")
    if square_px <= 0:
        raise ValueError("square-px must be greater than 0")
    if margin_px < 0:
        raise ValueError("margin-px must be 0 or greater")

    board_w = cols * square_px
    board_h = rows * square_px
    width = board_w + 2 * margin_px
    height = board_h + 2 * margin_px

    pixels = bytearray([255] * (width * height))
    for y in range(board_h):
        row = y // square_px
        for x in range(board_w):
            col = x // square_px
            is_black = (row + col) % 2 == 0
            if invert:
                is_black = not is_black
            pixels[(y + margin_px) * width + (x + margin_px)] = 0 if is_black else 255

    output.parent.mkdir(parents=True, exist_ok=True)
    write_grayscale_png(output, width, height, pixels)
    return width, height


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a chessboard PNG. For OpenCV pattern_size=(15, 11), "
            "use the default 16 x 12 squares."
        )
    )
    parser.add_argument("--cols", type=int, default=16, help="number of chessboard squares horizontally")
    parser.add_argument("--rows", type=int, default=12, help="number of chessboard squares vertically")
    parser.add_argument("--square-px", type=int, default=100, help="pixels per square")
    parser.add_argument("--margin-px", type=int, default=0, help="white border around the chessboard")
    parser.add_argument("--invert", action="store_true", help="start with a white square in the top-left")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("calibration_chessboard_16x12.png"),
        help="output PNG path",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    width, height = generate_chessboard(
        output=args.output,
        cols=args.cols,
        rows=args.rows,
        square_px=args.square_px,
        margin_px=args.margin_px,
        invert=args.invert,
    )
    print(f"Wrote {args.output} ({width} x {height}px)")
    print(f"OpenCV pattern_size = ({args.cols - 1}, {args.rows - 1})")


if __name__ == "__main__":
    main()
