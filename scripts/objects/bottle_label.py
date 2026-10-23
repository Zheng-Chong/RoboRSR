"""Wrap-around label artwork for make_plastic_bottle.py (numpy + PIL only, so it can be previewed without pxr).

The image is the label unrolled: u (left to right) runs counter-clockwise around the bottle seen from above, which is
left to right for a viewer outside; v runs bottom to top. Bottom `share` of the height is the label's main colour, the
rest its second colour. Front panel: picture, brand, Latin brand, product and volume; back panel: composition table,
barcode, fine print and a QR code. Everything printed is generic or fictional except the 00000 twin's brand words.
"""

import os
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

# (file name, face index) per role; Noto CJK collections hold JP, KR, SC, TC, HK faces, SC is index 2
FONTS = {
    "brand": [("NotoSerifCJK-Bold.ttc", 2), ("Songti.ttc", 0), ("STHeiti Medium.ttc", 0)],
    "bold": [("NotoSansCJK-Bold.ttc", 2), ("STHeiti Medium.ttc", 0)],
    "regular": [("NotoSansCJK-Regular.ttc", 2), ("STHeiti Light.ttc", 0)],
}
FONT_DIRS = [
    os.environ.get("CJK_FONT_DIR"),
    "/usr/share/fonts/opentype/noto",
    "/System/Library/Fonts",
    "/System/Library/Fonts/Supplemental",
]
WATER_TABLE = [
    ("偏硅酸", "≥1.8 mg/L"),
    ("钙", "≥400 μg/L"),
    ("钾", "≥35 μg/L"),
    ("钠", "≥80 μg/L"),
    ("镁", "≥50 μg/L"),
    ("pH值 (25℃)", "7.3±0.5"),
]
NUTRITION_TABLE = [("能量", "180 kJ"), ("蛋白质", "0 g"), ("脂肪", "0 g"), ("碳水化合物", "10.6 g"), ("钠", "20 mg")]
FINE_PRINT = [
    "配料：{ingredients}",
    "保质期：{shelf}",
    "贮存条件：避免阳光直射，常温保存",
    "生产日期：见瓶盖",
    "执行标准：{standard}",
    "产地：中国",
    "开启后请尽快饮用",
    "客服热线：400-000-0000",
]


def font_path(role):
    for d in filter(None, FONT_DIRS):
        for name, index in FONTS[role]:
            p = Path(d) / name
            if p.exists():
                return str(p), index
    raise FileNotFoundError(
        f"no CJK font for '{role}' ({[n for n, _ in FONTS[role]]}) in {FONT_DIRS}: set CJK_FONT_DIR, e.g. to the "
        "fonts-noto-cjk files unpacked with `apt-get download fonts-noto-cjk && dpkg -x fonts-noto-cjk_*.deb x`"
    )


def font(role, px):
    path, index = font_path(role)
    return ImageFont.truetype(path, max(int(px), 6), index=index)


def srgb(linear):
    return tuple(int(round(255 * min(max(c, 0.0), 1.0) ** (1 / 2.2))) for c in linear)


def fit_text(draw, text, role, box_w, box_h, spacing=0.0):
    """Largest font of `role` whose text (with letter spacing in em) fits box_w x box_h."""
    px = box_h
    while px > 6:
        f = font(role, px)
        w = text_width(draw, text, f, spacing)
        b = draw.textbbox((0, 0), text, font=f)
        if w <= box_w and b[3] - b[1] <= box_h:
            return f
        px *= 0.93
    return font(role, 6)


def text_width(draw, text, f, spacing):
    return sum(draw.textlength(ch, font=f) for ch in text) + spacing * f.size * (len(text) - 1)


def draw_text(draw, text, f, cx, top, fill, spacing=0.0, anchor="c"):
    """Draw `text` with letter spacing; cx is the centre (anchor c) or left edge (anchor l); top is the ink top."""
    w = text_width(draw, text, f, spacing)
    x = cx - w / 2 if anchor == "c" else cx
    y = top - draw.textbbox((0, 0), text, font=f)[1]
    for ch in text:
        draw.text((x, y), ch, font=f, fill=fill)
        x += draw.textlength(ch, font=f) + spacing * f.size


def ridge(rng, n, base, amp, rough):
    x = np.linspace(0, 1, n)
    y = np.zeros(n)
    for k in range(1, 7):
        y += amp / k**rough * np.sin(2 * np.pi * (k * x * rng.uniform(0.6, 1.4) + rng.uniform(0, 1)))
    return base - np.abs(y)


def picture(kind, w, h, main, accent, rng):
    """Front-panel picture, RGB w x h, in the label's main colour with accent highlights."""
    img = Image.new("RGB", (w, h))
    px = np.zeros((h, w, 3), np.float32)
    yy = np.linspace(0, 1, h)[:, None, None]
    if kind == "landscape":  # sky, three mountain ridges, a lake with their reflection
        px[:] = (1 - yy) * np.array([150, 196, 232]) + yy * np.array([236, 244, 250])
        img = Image.fromarray(px.astype(np.uint8))
        d = ImageDraw.Draw(img)
        shore = int(0.68 * h)
        layers = [(0.42, 0.20, (122, 150, 176)), (0.52, 0.16, (70, 110, 120)), (0.62, 0.10, (32, 78, 64))]
        ridges = []
        for base, amp, col in layers:
            y = ridge(rng, w, base * h, amp * h, 1.2)
            ridges.append((y, col))
            d.polygon([(0, shore)] + [(i, float(v)) for i, v in enumerate(y)] + [(w - 1, shore)], fill=col)
        lake = np.array(img).astype(np.float32)
        refl = lake[max(0, 2 * shore - h) : shore][::-1]
        refl = refl[: h - shore]
        water = np.linspace(1, 0.55, h - shore)[:, None, None]
        lake[shore:] = refl * 0.55 * water + np.array([120, 170, 200]) * (1 - 0.55 * water)
        img = Image.fromarray(np.clip(lake, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6))
        d = ImageDraw.Draw(img)
        for _ in range(w // 3):  # ripples
            x, y = rng.uniform(0, w), rng.uniform(shore + 2, h)
            d.line([(x, y), (x + rng.uniform(4, 14), y)], fill=(210, 230, 240), width=1)
        return img
    hi = np.array(main) * 0.35 + 255 * 0.65
    px[:] = (1 - yy) * hi + yy * np.array(main)
    img = Image.fromarray(px.astype(np.uint8))
    d = ImageDraw.Draw(img, "RGBA")
    if kind == "bubbles":
        for _ in range(40):
            r = rng.uniform(0.02, 0.12) * h
            x, y = rng.uniform(0, w), rng.uniform(0, h)
            d.ellipse([x - r, y - r, x + r, y + r], outline=(255, 255, 255, 200), width=max(2, int(r / 6)))
            d.ellipse([x - r * 0.5, y - r * 0.6, x - r * 0.2, y - r * 0.3], fill=(255, 255, 255, 180))
    elif kind == "fruit":  # orange slices
        for _ in range(4):
            r = rng.uniform(0.22, 0.34) * h
            x, y = rng.uniform(0.15, 0.85) * w, rng.uniform(0.3, 0.8) * h
            d.ellipse([x - r, y - r, x + r, y + r], fill=(240, 150, 30, 255), outline=(250, 230, 160, 255), width=6)
            for a in np.linspace(0, 2 * np.pi, 10, endpoint=False):
                d.line([(x, y), (x + 0.85 * r * np.cos(a), y + 0.85 * r * np.sin(a))], fill=(252, 220, 140), width=3)
            d.ellipse([x - 0.12 * r, y - 0.12 * r, x + 0.12 * r, y + 0.12 * r], fill=(252, 230, 170, 255))
    elif kind == "stripes":
        for k in range(-6, 12):
            x0 = k * w / 6
            col = (*accent, 200) if k % 2 else (255, 255, 255, 90)
            d.polygon([(x0, h), (x0 + w / 12, h), (x0 + w / 12 + h * 0.6, 0), (x0 + h * 0.6, 0)], fill=col)
    return img


def barcode(draw, x0, y0, w, h, digits, ink, rng):
    bits = [1, 0, 1] + list(rng.integers(0, 2, 42)) + [0, 1, 0, 1, 0] + list(rng.integers(0, 2, 42)) + [1, 0, 1]
    bw = w / len(bits)
    for i, b in enumerate(bits):
        if b:
            draw.rectangle([x0 + i * bw, y0, x0 + (i + 1) * bw - 0.2, y0 + h * 0.8], fill=ink)
    f = fit_text(draw, digits, "regular", w, h * 0.17)
    draw_text(draw, digits, f, x0 + w / 2, y0 + h * 0.83, ink)


def qr(draw, x0, y0, s, ink, rng, n=25):
    m = rng.integers(0, 2, (n, n))
    for fx, fy in ((0, 0), (n - 7, 0), (0, n - 7)):  # finder patterns
        m[fy : fy + 7, fx : fx + 7] = 1
        m[fy + 1 : fy + 6, fx + 1 : fx + 6] = 0
        m[fy + 2 : fy + 5, fx + 2 : fx + 5] = 1
    c = s / n
    for j, i in zip(*np.nonzero(m)):
        draw.rectangle([x0 + i * c, y0 + j * c, x0 + (i + 1) * c - 0.5, y0 + (j + 1) * c - 0.5], fill=ink)


def draw_label(art, main, second, share, size, seed):
    """Label image (PIL RGB, size = (w, h)). main / second: linear RGB of the bottom / top colour bands."""
    w, h = size
    rng = np.random.default_rng(seed)
    c_main, c_second = srgb(main), srgb(second)
    img = Image.new("RGB", size, c_second)
    d = ImageDraw.Draw(img)
    split = int(round(h * (1 - share)))
    d.rectangle([0, split, w, h], fill=c_main)
    trim = srgb(art.get("trim", main))
    d.rectangle([0, 0, w, int(0.022 * h)], fill=trim)
    d.rectangle([0, split - int(0.012 * h), w, split], fill=trim)
    low, top = h - split, split

    # front panel, centred at u = 0.38
    fx0, fx1 = 0.20 * w, 0.56 * w
    cx = (fx0 + fx1) / 2
    pic_w, pic_h = int(0.15 * w), int(top * 0.80 + low * 0.30)
    pic = picture(art["picture"], pic_w, pic_h, c_main, c_second, rng)
    px0, py0 = int(0.035 * w), int(0.06 * h)
    d.rectangle([px0 - 6, py0 - 6, px0 + pic_w + 6, py0 + pic_h + 6], fill=(250, 250, 248))
    img.paste(pic, (px0, py0))
    f = fit_text(d, art["brand"], "brand", (fx1 - fx0) * 0.92, low * 0.46, spacing=0.06)
    draw_text(d, art["brand"], f, cx, split + low * 0.10, c_second, spacing=0.06)
    f = fit_text(d, art["latin"], "bold", (fx1 - fx0) * 0.80, low * 0.11, spacing=0.25)
    draw_text(d, art["latin"], f, cx, split + low * 0.64, c_second, spacing=0.25)
    f = fit_text(d, art["tagline"], "regular", (fx1 - fx0) * 0.70, low * 0.08)
    draw_text(d, art["tagline"], f, cx, split + low * 0.82, c_second)
    f = fit_text(d, art["product"], "bold", (fx1 - fx0) * 0.50, top * 0.30, spacing=0.12)
    draw_text(d, art["product"], f, fx0 + 0.01 * (fx1 - fx0), top * 0.28, c_main, spacing=0.12, anchor="l")
    f = fit_text(d, art["volume"], "bold", (fx1 - fx0) * 0.30, top * 0.30)
    draw_text(d, art["volume"], f, fx1 - text_width(d, art["volume"], f, 0), top * 0.28, c_main, anchor="l")
    f = fit_text(d, art["note"], "regular", (fx1 - fx0) * 0.9, top * 0.12)
    draw_text(d, art["note"], f, cx, top * 0.72, c_main)

    # back panel: composition table and barcode on the top band, fine print and QR code on the bottom band
    bx0, bx1 = 0.62 * w, 0.98 * w
    tx1 = bx0 + 0.21 * w
    rows = WATER_TABLE if art["table"] == "water" else NUTRITION_TABLE
    title = "饮用天然水物质成分表" if art["table"] == "water" else "营养成分表 (每100mL)"
    ink = c_main if sum(c_main) < 600 else (40, 40, 40)
    f = fit_text(d, title, "bold", tx1 - bx0, top * 0.13)
    draw_text(d, title, f, bx0, top * 0.10, ink, anchor="l")
    rh = top * 0.66 / len(rows)
    fr = font("regular", rh * 0.62)
    for i, (k, v) in enumerate(rows):
        y = top * 0.28 + i * rh
        d.line([(bx0, y), (tx1, y)], fill=ink, width=1)
        draw_text(d, k, fr, bx0 + 4, y + rh * 0.18, ink, anchor="l")
        draw_text(d, v, fr, tx1 - text_width(d, v, fr, 0) - 4, y + rh * 0.18, ink, anchor="l")
    d.rectangle([tx1 + 0.02 * w, top * 0.12, bx1, top * 0.88], fill=(255, 255, 255))
    barcode(d, tx1 + 0.03 * w, top * 0.18, bx1 - tx1 - 0.04 * w, top * 0.66, art["ean"], (15, 15, 15), rng)
    qs, m = min(low * 0.55, 0.10 * w), 0.006 * w
    lines = [line.format(**art["facts"]) for line in FINE_PRINT]
    longest = max(lines, key=lambda s: text_width(d, s, font("regular", 100), 0))
    fp = fit_text(d, longest, "regular", bx1 - qs - 3 * m - bx0, low * 0.065)
    for i, line in enumerate(lines):
        draw_text(d, line, fp, bx0, split + low * (0.08 + 0.105 * i), c_second, anchor="l")
    qy = split + (low - qs) / 2
    d.rectangle([bx1 - qs - 2 * m, qy - m, bx1, qy + qs + m], fill=(255,) * 3)
    qr(d, bx1 - qs - m, qy, qs, (15, 15, 15), rng)
    return img
