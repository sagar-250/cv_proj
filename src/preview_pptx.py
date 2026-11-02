"""Preview a .pptx as PNGs without LibreOffice, and report text overflow and text overlaps.

This machine has no LibreOffice, so this draws each slide from the pptx file itself with Pillow:
shapes, pictures, text (Liberation Sans, metric-compatible with Arial) and a rough drawing of
native column/line charts from their data. It is a layout check, not a pixel-exact render, so
flip through the real deck in PowerPoint once as well.

Run: .venv/bin/python src/preview_pptx.py deck.pptx outdir [slide numbers ...]
Writes outdir/slide-NN.png and prints one line per problem found.
"""
import io
import os
import sys

from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation
from pptx.enum.dml import MSO_FILL
from pptx.enum.shapes import MSO_SHAPE, MSO_SHAPE_TYPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN

PX = 100.0                      # pixels per inch
FD = "/usr/share/fonts/truetype/liberation/"
_cache = {}
report_legend = []


def font(size_pt, bold=False):
    k = (round(size_pt * 4), bold)
    if k not in _cache:
        _cache[k] = ImageFont.truetype(FD + ("LiberationSans-Bold.ttf" if bold else "LiberationSans-Regular.ttf"),
                                       max(1, int(round(size_pt * PX / 72))))
    return _cache[k]


def px(emu):
    return emu / 914400 * PX


def rgb_of(fill):
    try:
        if fill.type == MSO_FILL.SOLID:
            c = fill.fore_color.rgb
            return (c[0], c[1], c[2])
    except Exception:
        pass
    return None


def layout_text(shape, report, sn):
    """Lay out a text frame; return the glyph runs to draw and the used rectangle."""
    tf = shape.text_frame
    ml, mr, mt, mb = px(tf.margin_left), px(tf.margin_right), px(tf.margin_top), px(tf.margin_bottom)
    x0, y0, w, h = px(shape.left), px(shape.top), px(shape.width), px(shape.height)
    inner_w = w - ml - mr
    lines_all, used_w = [], 0
    for p in tf.paragraphs:
        runs = [(r.text, r.font.size.pt if r.font.size else 18, bool(r.font.bold),
                 tuple(r.font.color.rgb) if r.font.color and r.font.color.type is not None else (0, 0, 0)) for r in p.runs]
        if not runs:
            lines_all.append(([], 18 * PX / 72 * 1.2, p, 0, 0))
            continue
        toks = []
        for t, sz, b, c in runs:
            parts = t.replace("\n", " ").split(" ")
            for i, w_ in enumerate(parts):
                toks.append((w_ + (" " if i < len(parts) - 1 else ""), sz, b, c))
        lines, cur, cw = [], [], 0
        for tk in toks:
            tw = font(tk[1], tk[2]).getlength(tk[0])
            tw_ns = font(tk[1], tk[2]).getlength(tk[0].rstrip(" "))
            if cur and cw + tw_ns > inner_w + 0.5:
                lines.append((cur, cw))
                cur, cw = [], 0
            if tw_ns > inner_w + 0.5:
                report.append(f"slide {sn}: word wider than box ({tk[0]!r}) in {shape.name}")
            cur.append(tk)
            cw += tw
        lines.append((cur, cw))
        ls = p.line_spacing if isinstance(p.line_spacing, float) else 1.0
        sa = p.space_after.pt * PX / 72 if p.space_after is not None else 0
        for li, (ln, lw) in enumerate(lines):
            mx = max(tk[1] for tk in ln) if ln else 18
            lines_all.append((ln, mx * PX / 72 * 1.2 * ls, p, lw, sa if li == len(lines) - 1 else 0))
    need = sum(l[1] + l[4] for l in lines_all) - (lines_all[-1][4] if lines_all else 0)
    avail = h - mt - mb
    anchor = tf.vertical_anchor
    y = (avail - need) / 2 if anchor == MSO_ANCHOR.MIDDLE else (avail - need if anchor == MSO_ANCHOR.BOTTOM else 0.0)
    y_start, out = y, []
    for ln, lh, p, lw, sa in lines_all:
        al = p.alignment
        xs = ml if al in (None, PP_ALIGN.LEFT) else (ml + (inner_w - lw) / 2 if al == PP_ALIGN.CENTER else ml + inner_w - lw)
        cx = xs
        for tk in ln:
            out.append((x0 + cx, y0 + mt + y, tk[0].rstrip(" "), font(tk[1], tk[2]), tk[3], lh))
            cx += font(tk[1], tk[2]).getlength(tk[0])
        used_w = max(used_w, lw)
        y += lh + sa
    if need > avail + 2:
        txt = " ".join(p.text for p in tf.paragraphs)[:40]
        report.append(f"slide {sn}: TEXT OVERFLOW in '{shape.name}' ({txt!r}): needs {need / PX:.2f} in, box {avail / PX:.2f} in")
    return out, (x0 + ml, y0 + mt + y_start, used_w, need)


def draw_chart(d, shape, sn):
    ch = shape.chart
    x, y, w, h = px(shape.left), px(shape.top), px(shape.width), px(shape.height)
    plot = ch.plots[0]
    cats, sers = list(plot.categories), list(plot.series)
    fnt = font(11)
    leg_h = 40 if ch.has_legend else 0
    left, right, top, bottom = x + 60, x + w - 15, y + 20, y + h - leg_h - 55
    vmin = ch.value_axis.minimum_scale if ch.value_axis.minimum_scale is not None else 0
    vmax = ch.value_axis.maximum_scale if ch.value_axis.maximum_scale is not None else max(max(s.values) for s in sers)

    def vy(v):
        return bottom - (v - vmin) / (vmax - vmin) * (bottom - top)

    step = ch.value_axis.major_unit or (vmax - vmin) / 5
    v = vmin
    while v <= vmax + 1e-9:
        d.line([(left, vy(v)), (right, vy(v))], fill=(213, 220, 227), width=1)
        d.text((left - 8, vy(v)), f"{v:g}", font=fnt, fill=(27, 39, 51), anchor="rm")
        v += step
    n = len(cats)
    slot = (right - left) / n
    is_bar = "COLUMN" in str(ch.chart_type) or "BAR" in str(ch.chart_type)
    colors = []
    for s_ in sers:
        c = None
        try:
            if is_bar and s_.format.fill.type == MSO_FILL.SOLID:
                c = tuple(s_.format.fill.fore_color.rgb)
            elif not is_bar:
                c = tuple(s_.format.line.color.rgb)
        except Exception:
            pass
        colors.append(c or (120, 120, 120))
    ns = len(sers)
    gap = (plot.gap_width if is_bar else 0) / 100
    for i, cat in enumerate(cats):
        cxm = left + slot * (i + 0.5)
        words, lines, cur = str(cat).split(" "), [], ""
        for w_ in words:
            if fnt.getlength((cur + " " + w_).strip()) > slot - 6 and cur:
                lines.append(cur)
                cur = w_
            else:
                cur = (cur + " " + w_).strip()
        lines.append(cur)
        for k, ln in enumerate(lines):
            d.text((cxm, bottom + 6 + k * 14), ln, font=fnt, fill=(27, 39, 51), anchor="mt")
    for si, (s_, col) in enumerate(zip(sers, colors)):
        pts = []
        for i, val in enumerate(list(s_.values)):
            cxm = left + slot * (i + 0.5)
            if is_bar:
                bw = slot / (ns + gap)
                bx = cxm - bw * ns / 2 + si * bw
                d.rectangle([bx, vy(val), bx + bw, vy(0 if vmin <= 0 else vmin)], fill=col)
                if plot.has_data_labels:
                    lab = f"{val:.1f}" if "0.0" in plot.data_labels.number_format else f"{val:.0f}"
                    d.text((bx + bw / 2, vy(val) - 3), lab, font=fnt, fill=(27, 39, 51), anchor="ms")
            else:
                pts.append((cxm, vy(val)))
        if pts:
            thick = s_.format.line.width and s_.format.line.width.pt > 2
            d.line(pts, fill=col, width=3 if thick else 1)
    if ch.has_legend:
        lx, ly = left, y + h - 22
        for s_, col in zip(sers, colors):
            d.rectangle([lx, ly, lx + 14, ly + 10], fill=col)
            d.text((lx + 20, ly - 2), s_.name, font=fnt, fill=(27, 39, 51))
            lx += 40 + fnt.getlength(s_.name)
            if lx > right:
                report_legend.append(f"slide {sn}: legend wider than chart")


def render(deck, outdir, only=None):
    prs = Presentation(deck)
    os.makedirs(outdir, exist_ok=True)
    report = []
    W, H = int(px(prs.slide_width)), int(px(prs.slide_height))
    for sn, slide in enumerate(prs.slides, 1):
        if only and sn not in only:
            continue
        im = Image.new("RGB", (W, H), rgb_of(slide.background.fill) or (255, 255, 255))
        d = ImageDraw.Draw(im)
        text_rects = []
        for shape in slide.shapes:
            x, y, w, h = px(shape.left), px(shape.top), px(shape.width), px(shape.height)
            if x < -1 or y < -1 or x + w > W + 1 or y + h > H + 1:
                report.append(f"slide {sn}: '{shape.name}' extends past the slide edge")
            if shape.shape_type == MSO_SHAPE_TYPE.PICTURE:
                pic = Image.open(io.BytesIO(shape.image.blob)).convert("RGB").resize((max(1, int(w)), max(1, int(h))), Image.LANCZOS)
                mask = Image.new("L", pic.size, 0)
                ImageDraw.Draw(mask).rounded_rectangle([0, 0, pic.size[0] - 1, pic.size[1] - 1], radius=12, fill=255)
                im.paste(pic, (int(x), int(y)), mask)
                d.rounded_rectangle([x, y, x + w, y + h], radius=12, outline=(213, 220, 227), width=1)
                continue
            if getattr(shape, "has_chart", False):
                draw_chart(d, shape, sn)
                continue
            fill = rgb_of(shape.fill) if hasattr(shape, "fill") and shape.shape_type != MSO_SHAPE_TYPE.TEXT_BOX else None
            if fill is not None:
                try:
                    st = shape.auto_shape_type
                except Exception:
                    st = MSO_SHAPE.RECTANGLE
                if st == MSO_SHAPE.ROUNDED_RECTANGLE:
                    d.rounded_rectangle([x, y, x + w, y + h], radius=shape.adjustments[0] * min(w, h), fill=fill)
                elif st == MSO_SHAPE.OVAL:
                    d.ellipse([x, y, x + w, y + h], fill=fill)
                elif st == MSO_SHAPE.RIGHT_ARROW:
                    d.polygon([(x, y + h * 0.25), (x + w * 0.55, y + h * 0.25), (x + w * 0.55, y), (x + w, y + h / 2),
                               (x + w * 0.55, y + h), (x + w * 0.55, y + h * 0.75), (x, y + h * 0.75)], fill=fill)
                else:
                    d.rectangle([x, y, x + w, y + h], fill=fill)
            if shape.has_text_frame and shape.text_frame.text.strip():
                items, rect = layout_text(shape, report, sn)
                for tx, ty, t, f, c, lh in items:
                    d.text((tx, ty + (lh - f.size * 1.2) / 2 + f.size * 0.02), t, font=f, fill=c)
                if shape.shape_type == MSO_SHAPE_TYPE.TEXT_BOX or shape.is_placeholder:
                    text_rects.append((shape.name, rect, shape.text_frame.text[:25]))
        for i in range(len(text_rects)):
            for j in range(i + 1, len(text_rects)):
                a, b = text_rects[i][1], text_rects[j][1]
                ox = min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])
                oy = min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1])
                if ox > 4 and oy > 4:
                    report.append(f"slide {sn}: text overlap '{text_rects[i][2]}' / '{text_rects[j][2]}'")
        im.save(os.path.join(outdir, f"slide-{sn:02d}.png"))
    return report + report_legend


if __name__ == "__main__":
    only = set(int(a) for a in sys.argv[3:]) or None
    rep = render(sys.argv[1], sys.argv[2], only)
    print("\n".join(rep) if rep else "no overflow / overlap found")
