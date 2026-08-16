#!/usr/bin/env python3
"""
Convert a PNG image into a single-character GFXfont for Arduino GFX / Adafruit GFX.

- The image is turned into a monochrome bitmap.
- The single glyph will be mapped to the character 'A' (0x41).

Usage:
    python png_to_gfxfont.py logo.png LogoFont LogoFont.h
"""

import sys
from pathlib import Path

from PIL import Image

def image_to_monobitmap(img, threshold=128):
    """
    Convert an image to a 1-bit per pixel bitmap (list of bytes),
    packed MSB-first, row by row, as expected by GFXfont.
    """
    width, height = img.size

    # If the image has alpha, use it to decide foreground/background:
    if img.mode in ("RGBA", "LA"):
        # Convert to RGBA to be safe
        img = img.convert("RGBA")
        pixels = img.load()
        bits = []
        for y in range(height):
            for x in range(width):
                r, g, b, a = pixels[x, y]
                # Treat transparent or light pixels as background (0), others as foreground (1)
                # Adjust logic if your logo colors are inverted.
                if a < threshold:
                    bits.append(0)
                else:
                    # Decide based on brightness
                    brightness = (r + g + b) / 3
                    bits.append(1 if brightness < threshold else 0)
    else:
        # Convert to L (grayscale) and threshold
        img = img.convert("L")
        pixels = img.load()
        bits = []
        for y in range(height):
            for x in range(width):
                # Dark pixels => foreground (1), light => background (0)
                bits.append(1 if pixels[x, y] < threshold else 0)

    # Pack bits into bytes, MSB first
    bitmap_bytes = []
    byte = 0
    bit_index = 0  # 0..7

    for bit in bits:
        byte = (byte << 1) | (bit & 1)
        bit_index += 1
        if bit_index == 8:
            bitmap_bytes.append(byte)
            byte = 0
            bit_index = 0

    # If the last byte is not complete, shift it to the left
    if bit_index != 0:
        byte = byte << (8 - bit_index)
        bitmap_bytes.append(byte)

    return bitmap_bytes, width, height


def generate_gfxfont_header(bitmap_bytes, width, height, font_name, char='A'):
    """
    Generate the C header text for a single-character GFXfont.

    - char is the glyph character ('A' by default).
    - We use a simple metric setup suitable for a logo.
    """
    bitmap_name = f"{font_name}Bitmaps"
    glyphs_name = f"{font_name}Glyphs"

    # Metrics: you can tweak these if needed.
    x_advance = width + 1  # space to move cursor after drawing 'A'
    x_offset = 0          # horizontal offset from cursor to bitmap
    y_offset = -(height*2//3)    # top of bitmap is height pixels above baseline
    y_advance = height    # line height

    # Character code
    char_code = ord(char)

    # Format bitmap bytes as hex
    bitmap_hex_lines = []
    line = []
    for i, b in enumerate(bitmap_bytes):
        line.append(f"0x{b:02X}")
        if len(line) == 12:
            bitmap_hex_lines.append(", ".join(line))
            line = []
    if line:
        bitmap_hex_lines.append(", ".join(line))

    bitmap_array_str = ",\n  ".join(bitmap_hex_lines) if bitmap_hex_lines else ""

    header = f"""#pragma once

#include <stdint.h>

// Auto-generated from PNG by png_to_gfxfont.py
// Font name: {font_name}
// Single glyph: '{char}' (0x{char_code:02X})

const uint8_t {bitmap_name}[] PROGMEM = {{
  {bitmap_array_str}
}};

const GFXglyph {glyphs_name}[] PROGMEM = {{
  // bitmapOffset, width, height, xAdvance, xOffset, yOffset
  {{ 0, {width}, {height}, {x_advance}, {x_offset}, {y_offset} }}  // '{char}'
}};

const GFXfont {font_name} PROGMEM = {{
  (uint8_t*){bitmap_name},
  (GFXglyph*){glyphs_name},
  0x{char_code:02X}, 0x{char_code:02X}, {y_advance}
}};

// Usage example (Arduino):
//   gfx->setFont(&{font_name});
//   gfx->setCursor(x, y);
//   gfx->print("{char}");
"""

    return header


def main():
    if len(sys.argv) != 4:
        print("Usage: python png_to_gfxfont.py <input.png> <FontName> <output.h>")
        sys.exit(1)

    input_png = Path(sys.argv[1])
    font_name = sys.argv[2]
    output_h = Path(sys.argv[3])

    if not input_png.exists():
        print(f"Input file not found: {input_png}")
        sys.exit(1)

    img = Image.open(input_png)
    bitmap_bytes, width, height = image_to_monobitmap(img)

    header_text = generate_gfxfont_header(bitmap_bytes, width, height, font_name, char='A')

    output_h.write_text(header_text, encoding="utf-8")
    print(f"Generated {output_h} with font '{font_name}' for character 'A'.")
    print(f"Image size: {width}x{height}, bitmap bytes: {len(bitmap_bytes)}")


if __name__ == "__main__":
    main()

