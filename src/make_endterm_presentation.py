"""Build the end-term presentation from the project's own results and pictures.

Story: what the project is and why, how it works, and what we improved since the
mid-term (realistic simulation, a registration that works at GPS-sized offsets, the
visibility check, error bars) plus the three experiments (colour, AI clutter removal,
COLMAP photogrammetry). Numbers come from results/experiments/*.csv and
results/improvement_ladder.csv; pictures from media/ and results/dart/. Charts are
native PowerPoint charts, so they can be edited.

Run: .venv/bin/python src/make_endterm_presentation.py
Output: results/presentation/BIM_Progress_Monitoring_EndTerm.pptx
"""
import json
import os

import pandas as pd
from PIL import Image
from pptx import Presentation
from pptx.chart.data import CategoryChartData
from pptx.dml.color import RGBColor
from pptx.enum.chart import XL_CHART_TYPE, XL_LEGEND_POSITION, XL_LABEL_POSITION, XL_TICK_LABEL_POSITION
from pptx.enum.dml import MSO_LINE
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt
from lxml import etree

OUT = "results/presentation/BIM_Progress_Monitoring_EndTerm.pptx"
ASSETS = "results/presentation/assets"
E = "results/experiments/"

# ---- look -----------------------------------------------------------------
def rgb(h):
    return RGBColor.from_string(h)


NAVY, INK, MUTED = "16212E", "1B2733", "5B6B7A"
BRICK, STEEL, GREEN, AMBER = "C4572F", "2F6690", "3E8E5A", "D9962B"
CARD, LINE, WHITE, SOFT_BRICK, SOFT_STEEL = "EEF2F6", "D5DCE3", "FFFFFF", "F9E8E2", "E3EDF5"
FONT = "Arial"
W, H = 13.333, 7.5
MX = 0.6                      # side margin
TOP = 1.7                     # content starts here
PILL = {"NEW": (BRICK, WHITE), "IMPROVED": (STEEL, WHITE), "EXPERIMENT": (AMBER, INK), "RECAP": (MUTED, WHITE)}

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(W), Inches(H)
TITLE_ONLY = prs.slide_layouts[5]
os.makedirs(ASSETS, exist_ok=True)
_page = [0]


# ---- helpers ----------------------------------------------------------------
def style_run(run, size, bold=False, color=INK, italic=False):
    run.font.name = FONT
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.italic = italic
    run.font.color.rgb = rgb(color)


def text(slide, x, y, w, h, paras, size=16, color=INK, bold=False, align=PP_ALIGN.LEFT,
         anchor=MSO_ANCHOR.TOP, space_after=0, line=1.0, name=None):
    """paras: str or list of paragraphs; a paragraph is a str or a list of (text, {opts}) runs."""
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        tb.name = name
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = anchor
    if isinstance(paras, str):
        paras = [paras]
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.line_spacing = line
        p.space_after = Pt(space_after)
        runs = [(para, {})] if isinstance(para, str) else para
        for t, o in runs:
            r = p.add_run()
            r.text = t
            style_run(r, o.get("size", size), o.get("bold", bold), o.get("color", color), o.get("italic", False))
    return tb


def box(slide, x, y, w, h, fill=CARD, shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.08, line=None, name=None):
    s = slide.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        s.name = name
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE:
        s.adjustments[0] = radius
    s.fill.solid()
    s.fill.fore_color.rgb = rgb(fill)
    if line:
        s.line.color.rgb = rgb(line)
        s.line.width = Pt(1)
    else:
        s.line.fill.background()
    s.shadow.inherit = False
    return s


def pill(slide, kind, x=None, y=0.55, w=1.5):
    fill, fg = PILL[kind]
    x = W - MX - w if x is None else x
    s = box(slide, x, y, w, 0.36, fill=fill, radius=0.5, name=f"tag {kind}")
    tf = s.text_frame
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.CENTER
    r = p.add_run()
    r.text = kind
    style_run(r, 12, True, fg)
    return s


def picture(slide, path, x, y, w, h, alt, align="center", frame=True):
    """Place a picture inside the box (x, y, w, h), keeping its aspect ratio. Returns (x, y, w, h) used."""
    iw, ih = Image.open(path).size
    s = min(w / iw, h / ih)
    pw, ph = iw * s, ih * s
    px = x + (w - pw) / 2 if align == "center" else (x if align == "left" else x + w - pw)
    py = y + (h - ph) / 2 if align == "center" else y
    pic = slide.shapes.add_picture(path, Inches(px), Inches(py), Inches(pw), Inches(ph))
    pic.auto_shape_type = MSO_SHAPE.ROUNDED_RECTANGLE
    geom = pic._element.spPr.find(qn("a:prstGeom"))
    av = geom.find(qn("a:avLst"))
    if av is None:
        av = etree.SubElement(geom, qn("a:avLst"))
    gd = etree.SubElement(av, qn("a:gd"))
    gd.set("name", "adj")
    gd.set("fmla", "val 3000")
    if frame:
        pic.line.color.rgb = rgb(LINE)
        pic.line.width = Pt(1)
    pic._element.nvPicPr.cNvPr.set("descr", alt)
    return px, py, pw, ph


def crop(path, name, box_frac):
    """Save a cropped copy (box_frac = left, top, right, bottom as fractions) and return its path."""
    im = Image.open(path)
    w, h = im.size
    l, t, r, b = box_frac
    out = os.path.join(ASSETS, name)
    im.crop((int(l * w), int(t * h), int(r * w), int(b * h))).save(out)
    return out


def new_slide(title, subtitle=None, tag=None, notes=""):
    s = prs.slides.add_slide(TITLE_ONLY)
    _page[0] += 1
    t = s.shapes.title
    t.left, t.top, t.width, t.height = Inches(MX), Inches(0.4), Inches(W - 2 * MX - 1.8), Inches(0.7)
    tf = t.text_frame
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.LEFT
    r = p.add_run()
    r.text = title
    style_run(r, 30, True, INK)
    if subtitle:
        text(s, MX, 1.12, W - 2 * MX, 0.4, subtitle, size=16, color=MUTED, name="message")
    if tag:
        pill(s, tag)
    text(s, MX, 7.05, 8, 0.25, "BIM-based construction progress monitoring  |  End-term review", size=10, color=MUTED,
         name="footer")
    text(s, W - MX - 1, 7.05, 1, 0.25, str(_page[0] + 1), size=10, color=MUTED, align=PP_ALIGN.RIGHT, name="page number")
    s.notes_slide.notes_text_frame.text = notes
    return s


def stat(slide, x, y, w, big, label, color=BRICK, h=1.25, fill=CARD, big_size=34):
    box(slide, x, y, w, h, fill=fill)
    text(slide, x + 0.2, y + 0.12, w - 0.4, 0.62, big, size=big_size, bold=True, color=color, anchor=MSO_ANCHOR.MIDDLE)
    text(slide, x + 0.2, y + 0.76, w - 0.4, h - 0.82, label, size=12, color=MUTED)


def numbered(slide, x, y, n, color=STEEL, d=0.46):
    c = box(slide, x, y, d, d, fill=color, shape=MSO_SHAPE.OVAL, name=f"number {n}")
    tf = c.text_frame
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.CENTER
    r = p.add_run()
    r.text = str(n)
    style_run(r, 16, True, WHITE)


def caption(slide, x, y, w, t):
    text(slide, x, y, w, 0.4, t, size=11, color=MUTED, name="caption")


def style_chart(chart, size=12):
    chart.font.size = Pt(size)
    chart.font.name = FONT
    chart.font.color.rgb = rgb(INK)
    chart.has_title = False


def grid(axis, color=LINE):
    axis.has_major_gridlines = True
    axis.major_gridlines.format.line.color.rgb = rgb(color)
    axis.major_gridlines.format.line.width = Pt(0.75)
    axis.format.line.fill.background()


def alt_chart(gf, descr):
    gf._element.nvGraphicFramePr.cNvPr.set("descr", descr)


# ---- data -------------------------------------------------------------------
t13 = pd.read_csv(E + "main_table_13_busy_late_misplaced_rule_on.csv")
ladder = pd.read_csv("results/improvement_ladder.csv").set_index("step")
sfm = pd.read_csv(E + "main_table_sfm.csv").set_index("stage")
sfm_ideal = pd.read_csv(E + "main_table_sfm_ideal.csv").set_index("stage")
t13i = t13.set_index("stage")


def lad(step):
    return float(ladder.loc[step, "mean_error"])


DESIGN_IMG = crop("media/02_ground_truth_stages/complete_design_model.png", "design_model.png", (0.0, 0.07, 1.0, 0.92))

# =============================================================================
# 1. title
# =============================================================================
s = prs.slides.add_slide(prs.slide_layouts[6])
s.background.fill.solid()
s.background.fill.fore_color.rgb = rgb(NAVY)
text(s, 0.8, 1.35, 7.4, 0.4, "CV5570   |   END-TERM PROJECT REVIEW", size=14, bold=True, color=AMBER)
text(s, 0.8, 2.0, 7.4, 2.2, ["BIM-Based Construction", "Progress Monitoring"], size=44, bold=True, color=WHITE, line=1.05,
     name="title")
text(s, 0.8, 3.95, 7.4, 0.6, "via Drone Simulation", size=26, color="CFE0EE")
text(s, 0.8, 5.35, 6.8, 0.9, "From a clean test to a realistic one: what we changed since the mid-term, and what it bought us",
     size=16, color="9FB3C6")
picture(s, DESIGN_IMG, 8.35, 1.2, 4.4, 5.1,
        "The complete design model of the apartment building, coloured by element type", frame=False)
s.notes_slide.notes_text_frame.text = (
    "Welcome. This is the end-term review of our project: monitoring construction progress by comparing scans of a "
    "building with its design model. At the mid-term we showed a working pipeline on clean, easy data. Today we show "
    "what changed since then: a much more realistic test, a registration method that works when the scan starts metres "
    "off, a visibility check, a progress number with a range, and three experiments: colour, AI clutter removal and "
    "COLMAP photogrammetry.")

# =============================================================================
# 2. problem
# =============================================================================
s = new_slide("What we set out to do", "A design model says what should be built, not what is built", "RECAP", notes=(
    "Recap of the problem. A BIM encodes every wall, slab, beam and column exactly as planned, but says nothing about "
    "what exists on site. Checking by hand does not scale. The idea: a drone and a scanner collect a point cloud, we lay "
    "it on the design, decide which elements are built, and report one progress number. Real sites have no exact answer "
    "key, so we simulate: we take a real design and un-build it into 16 stages. Then we know the true progress and can "
    "grade every step of the method."))
steps = [("The design (BIM) says what should be there", "Every wall, slab, beam and column as planned."),
         ("A drone and a scanner see what is there", "They collect a point cloud: millions of 3D dots on the surfaces."),
         ("Compare the two", "Which elements are built, and how far along is the whole project?")]
for i, (h1, h2) in enumerate(steps):
    y = TOP + i * 1.3
    box(s, MX, y, 7.0, 1.12)
    numbered(s, MX + 0.22, y + 0.33, i + 1)
    text(s, MX + 0.95, y + 0.17, 5.85, 0.35, h1, size=18, bold=True)
    text(s, MX + 0.95, y + 0.58, 5.85, 0.5, h2, size=14, color=MUTED)
box(s, MX, TOP + 3.95, 7.0, 1.0, fill=SOFT_STEEL)
text(s, MX + 0.25, TOP + 4.07, 6.5, 0.8,
     [[("Why simulate? ", {"bold": True}),
       ("Real sites have no answer key. We un-build a real design into stages, so the true progress is known exactly.", {})]],
     size=14, anchor=MSO_ANCHOR.MIDDLE)
picture(s, DESIGN_IMG, 8.05, TOP, 4.68, 4.2,
        "The complete design model of the apartment building")
caption(s, 8.05, TOP + 4.3, 4.68, "Test building: Schependomlaan, a Dutch apartment block. 1,355 elements, 6 storeys.")

# =============================================================================
# 3. what changed
# =============================================================================
s = new_slide("What changed since the mid-term", "Seven things are different, and each one makes the test harder or the method better",
              notes=("This is the summary slide for the whole talk. At the mid-term the scans were clean and almost "
                     "aligned, we used plain ICP, and an element we did not find was simply called not built. Now the "
                     "scans have realistic noise, the site is busy, the scan starts metres off like drone GPS, the "
                     "alignment is a new two-step method, we check whether a missing element could even have been seen, "
                     "and the progress number comes with a range. We also tried three ways of getting better data: "
                     "colour, AI clutter removal and photogrammetry. The rest of the talk goes through these."))
rows = [("Test scans", "Clean ray-cast scan, tiny noise", "Realistic sensor: distance noise, holes, stray points"),
        ("The site", "Empty lot with trees", "Scaffolding, crane, workers, off-design and late elements"),
        ("Scan position", "Starts almost aligned (centimetres)", "Starts metres off and turned, like drone GPS"),
        ("Alignment", "ICP only", "Floor-plan search, then ICP, with a warning when unsure"),
        ("Element not found", "Counted as not built", "Visibility check: not built, or just not seen?"),
        ("Result", "One progress number", "Progress number with a 90% range"),
        ("Data source", "Ray-casting only", "Also tested: colour, AI clutter removal, COLMAP photos")]
text(s, 3.0, 1.55, 4.3, 0.3, "AT MID-TERM", size=12, bold=True, color=MUTED)
text(s, 7.85, 1.55, 4.8, 0.3, "NOW", size=12, bold=True, color=BRICK)
rh, gap = 0.64, 0.09
for i, (lab, old, new) in enumerate(rows):
    y = 1.9 + i * (rh + gap)
    text(s, MX, y, 2.3, rh, lab, size=14, bold=True, anchor=MSO_ANCHOR.MIDDLE)
    box(s, 3.0, y, 4.3, rh, fill=CARD)
    text(s, 3.2, y, 3.95, rh, old, size=14, color=MUTED, anchor=MSO_ANCHOR.MIDDLE)
    arrow = box(s, 7.4, y + rh / 2 - 0.14, 0.34, 0.28, fill=BRICK, shape=MSO_SHAPE.RIGHT_ARROW)
    box(s, 7.85, y, 4.88, rh, fill=SOFT_BRICK)
    text(s, 8.05, y, 4.55, rh, new, size=14, bold=True, anchor=MSO_ANCHOR.MIDDLE)

# =============================================================================
# 4. how it works
# =============================================================================
s = new_slide("How it works, end to end", "Seven steps from the design model to a progress number", notes=(
    "Here is the whole pipeline. One: the design model. Two: we un-build it into 16 construction stages; that is our "
    "answer key. Three: a simulated drone orbit and indoor scanner produce a point cloud for each stage, now with realistic "
    "noise and a busy site. Four: alignment, because the scan starts metres off the design. Five: every scan point is "
    "assigned to the nearest design element, using a KD-tree for fast nearest-point lookup. Six: for elements we did not "
    "find, we ask whether the cameras could have seen them. Seven: elements become a progress percentage, with a range. "
    "The orange and blue tags mark what is new or improved since mid-term."))
nodes = [("Design model (BIM)", "Every wall, slab, beam and column as planned.", None),
         ("16 stages", "The design is un-built floor by floor. This is the answer key.", None),
         ("Simulated scans", "Drone orbit and indoor scans, with realistic noise and a busy site.", "NEW"),
         ("Alignment", "Lay the scan on the design: floor-plan search, then ICP.", "IMPROVED"),
         ("Match to elements", "Each scan point goes to the nearest design element (KD-tree).", None),
         ("Visibility check", "Element not found: not built, or just not seen?", "NEW"),
         ("Progress and range", "Elements add up to a progress %, with a 90% range.", "NEW")]
nw, nh, ng = 2.75, 2.15, 0.37
pos = [(MX + i * (nw + ng), 1.75) for i in range(4)] + [(MX + i * (nw + ng), 4.3) for i in range(3)]
for i, ((x, y), (h1, h2, tg)) in enumerate(zip(pos, nodes)):
    box(s, x, y, nw, nh, fill=CARD)
    numbered(s, x + 0.2, y + 0.2, i + 1, color=NAVY)
    if tg:
        pill(s, tg, x=x + nw - 1.3 - 0.15, y=y + 0.26, w=1.3)
    text(s, x + 0.2, y + 0.82, nw - 0.4, 0.4, h1, size=17, bold=True)
    text(s, x + 0.2, y + 1.22, nw - 0.4, 0.9, h2, size=13, color=MUTED)
    if i not in (3, 6):
        box(s, x + nw + 0.06, y + nh / 2 - 0.14, 0.25, 0.28, fill=MUTED, shape=MSO_SHAPE.RIGHT_ARROW)
box(s, MX + 3 * (nw + ng), 4.3, nw, nh, fill=SOFT_STEEL)
text(s, MX + 3 * (nw + ng) + 0.2, 4.45, nw - 0.4, 1.9,
     [[("Tried along the way", {"bold": True, "size": 16})],
      "Colour for alignment",
      "AI clutter removal (DART)",
      "COLMAP point clouds from photos"],
     size=14, space_after=5)

# =============================================================================
# 5. test world
# =============================================================================
s = new_slide("A test world with a known answer", "We un-build a real design into 16 stages, so every result can be graded exactly",
              "RECAP", notes=(
    "The test world. We take the complete design and rebuild it in 16 stages: foundation first, then for each floor the "
    "structure, the outer walls and the inner walls. Some walls are only partly built, cut at a height. For every stage "
    "we store exactly which elements exist. That gives exact grading for every later step. We never show this answer key "
    "to the method itself."))
picture(s, "media/02_ground_truth_stages/all_16_stages.png", MX, 1.6, 6.3, 5.3, "The 16 construction stages: built elements blue, half-built orange, rest of the building a pale ghost")
stat(s, 7.3, 1.7, 1.7, "16", "stages", color=STEEL, h=1.1)
stat(s, 9.15, 1.7, 1.7, "1,355", "elements", color=STEEL, h=1.1)
stat(s, 11.0, 1.7, 1.73, "6", "storeys", color=STEEL, h=1.1)
for i, (h1, h2) in enumerate([("Floor by floor", "Foundation, then each floor: structure, outer walls, inner walls."),
                              ("Partly built walls", "Some walls are cut to a height, as on a real site."),
                              ("An exact answer key", "The true status of every element at every stage is stored. Only the grading uses it.")]):
    y = 3.2 + i * 1.2
    numbered(s, 7.3, y + 0.05, i + 1)
    text(s, 8.0, y, 4.7, 0.35, h1, size=17, bold=True)
    text(s, 8.0, y + 0.38, 4.7, 0.7, h2, size=14, color=MUTED)

# =============================================================================
# 6. realistic simulation
# =============================================================================
s = new_slide("Making the simulation realistic", "Real scans are noisy, start off-position and see a messy site", "NEW", notes=(
    "At mid-term the simulated scans were too clean. We made four changes. A realistic sensor: noise that grows with "
    "distance and slanted views, holes on textureless surfaces for the photos, surfaces seen at a grazing angle drop out, and "
    "stray points at edges. A GPS-sized starting offset: the scan begins metres off and turned. Camera colour with sun, "
    "shadows, dirt and brick patterns, and some elements built from a different material than designed. And some "
    "elements built a few centimetres off the design. On the picture: the same wall slice with the simple sensor on the "
    "left and the realistic one on the right; the red dots are junk points. Along the way we found and fixed two bugs in our "
    "own simulator. After these changes the progress error rose from 0.8 to 1.7 points, but registration stayed within 3 cm."))
img = crop("media/03_simulated_scans/sensor_simple_vs_realistic_slice.png", "sensor_top.png", (0.0, 0.07, 1.0, 0.53))
picture(s, img, MX, TOP, 6.9, 3.5, "Top view of a slice through the first-floor walls with a simple and a realistic sensor")
caption(s, MX, TOP + 3.6, 6.9, "Top view of a wall slice. Left: simple sensor. Right: realistic sensor, junk points in red.")
stat(s, MX, 5.55, 3.3, "0.8 → 1.7", "average progress error (points). Registration stays within 3 cm.", color=BRICK, h=1.3)
stat(s, MX + 3.6, 5.55, 3.3, "2 bugs", "found and fixed in our own simulator along the way", color=STEEL, h=1.3)
feats = [("Realistic sensor", "Noise grows with distance and slant. Holes, missing grazing surfaces, stray points."),
         ("GPS-sized start offset", "The scan begins metres off and turned, like a drone survey."),
         ("Camera colour", "Sun, shadows, dirt, brick pattern. 6% of elements use a different material."),
         ("Off-design elements", "4% of elements are built 4-12 cm away from the design.")]
for i, (h1, h2) in enumerate(feats):
    y = TOP + i * 1.28
    box(s, 7.85, y, 4.88, 1.12)
    text(s, 8.1, y + 0.14, 4.4, 0.35, h1, size=16, bold=True)
    text(s, 8.1, y + 0.5, 4.4, 0.6, h2, size=13, color=MUTED)

# =============================================================================
# 7. busy site
# =============================================================================
s = new_slide("A busy construction site", "None of this is in the design, so the method has to cope with it", "NEW", notes=(
    "A real site is not an empty lot. We added scaffolding along the walls, a tower crane, a site fence, shoring props, "
    "material piles and workers. We also made the as-built differ from the design: 4% of elements are built 4 to 12 cm "
    "off, 3% arrive two stages late, and 1.5% of walls are built about a metre from their place; the ground truth counts "
    "those as not built. In the picture: pink elements are slightly off, black ones are about a metre off, and the pale "
    "blue ghost is a late element. The cost was small for the progress number, about 1.8 to 2.0 points, and registration "
    "stayed within 5 cm. But one weakness showed up: scaffold decks, workers and piles can look like unbuilt elements, "
    "and 28 of 67 false 'built' calls came from them."))
picture(s, crop("media/09_busy_site/busy_site_world.png", "busy_site.png", (0.0, 0.05, 1.0, 1.0)), MX, 1.6, 6.0, 5.3, "A busy construction site with scaffolding, crane, fence, piles and workers around the building")
items = [("Scaffolding, crane, fence", "Scaffolding stands 0.3-1.0 m off the walls and is removed at the end."),
         ("Props, piles, workers", "New ones appear every stage, on the ground and on the floors."),
         ("Built off design", "4% of elements 4-12 cm off, 1.5% of walls about 1 m off."),
         ("Late elements", "3% arrive two stages after the schedule says.")]
for i, (h1, h2) in enumerate(items):
    y = TOP + i * 1.0
    text(s, 7.0, y, 5.7, 0.35, h1, size=16, bold=True)
    text(s, 7.0, y + 0.36, 5.7, 0.55, h2, size=13, color=MUTED)
stat(s, 7.0, 5.65, 2.75, "1.8 → 2.0", "average error (points), registration within 5 cm", color=STEEL, h=1.25, big_size=28)
stat(s, 9.95, 5.65, 2.78, "28 of 67", "false 'built' calls came from scaffolding, workers and piles", color=BRICK, h=1.25, big_size=28)

# =============================================================================
# 8. registration idea
# =============================================================================
s = new_slide("Alignment: why ICP alone failed", "The scan starts metres off, and ICP only works when it starts close", "IMPROVED", notes=(
    "Alignment, or registration, means laying the scan on top of the design. ICP, iterative closest point, is the classic "
    "method: for every scan point it finds the nearest point on the model, using a KD-tree so the search is fast; then it "
    "moves the scan a little to shrink those distances, and repeats. The catch is that it only finds a nearby answer. If "
    "the scan starts metres off or turned, it settles in the wrong place. In our test it was right 0% of the time at "
    "GPS-sized offsets. So we added a coarse step before ICP: level the scan using the floors, draw a top-down floor plan "
    "of the walls, and try every turn and shift to find where the two floor plans overlap best. Then ICP only has to "
    "fine-tune. The method also raises a warning when the answer looks doubtful."))
box(s, MX, TOP, 4.7, 2.3)
text(s, MX + 0.25, TOP + 0.15, 4.2, 0.4, "ICP in one minute", size=18, bold=True)
text(s, MX + 0.25, TOP + 0.62, 4.2, 1.6,
     ["For every scan point, find the nearest point on the design. A KD-tree makes that fast.",
      "Shift the scan a little to shrink those distances, then repeat."], size=14, space_after=8)
box(s, MX, TOP + 2.5, 4.7, 2.2, fill=SOFT_BRICK)
text(s, MX + 0.25, TOP + 2.65, 4.2, 0.85, "0%", size=44, bold=True, color=BRICK)
text(s, MX + 0.25, TOP + 3.6, 4.2, 0.9, "success for the old ICP when the scan starts metres off, and it never warned", size=14)
picture(s, "media/05_registration/how_the_search_works.png", 5.6, TOP, 7.13, 2.4, "How the floor-plan search finds the heading and position of the scan")
for i, (h1, h2) in enumerate([("Level the scan", "Use the floors to find which way is up."),
                              ("Search every turn and shift", "Compare floor plans of the walls and keep the best overlap."),
                              ("Fine-tune with ICP, flag doubt", "ICP does the last centimetres. Doubtful results raise a warning.")]):
    y = TOP + 2.65 + i * 0.68
    numbered(s, 5.6, y, i + 1, d=0.42)
    text(s, 6.2, y - 0.02, 6.5, 0.55, [[(h1 + ".  ", {"bold": True}), (h2, {"color": MUTED})]], size=14)

# =============================================================================
# 9. registration results
# =============================================================================
s = new_slide("Alignment: from 0% to 98% success", "Random starting offsets: GPS-sized for a drone survey, up to 30 m for an indoor scan without GPS", "IMPROVED", notes=(
    "Results of the alignment test bench, with 50 random starting offsets per case. The old ICP succeeded 0% of the "
    "time, and every failure was silent. Feature matching with RANSAC got 30 to 40%. Our floor-plan search plus ICP got "
    "97 to 98% in both cases, a drone survey with GPS and an indoor scan without. Where it failed it usually flagged the "
    "problem itself. In the full pipeline every stage ends up within about 1 to 3 cm. On the left you can see before and "
    "after for one stage: the scan in red starts about a metre off, and after alignment it sits on the design in grey."))
picture(s, "media/05_registration/before_after_alignment.png", MX, 1.6, 4.5, 5.3, "A scan before and after alignment to the design: about a metre off before, on top of the design after")
cd = CategoryChartData()
cd.categories = ["Old ICP", "Feature matching", "Floor-plan search + ICP"]
cd.add_series("Drone survey with GPS offset", [0, 40, 98])
cd.add_series("Indoor scan, no GPS", [0, 30, 98])
text(s, 5.3, TOP - 0.05, 7.4, 0.3, "Success rate (%): scan ends within 5 cm of the right place", size=14, bold=True)
gf = s.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(5.3), Inches(TOP + 0.25), Inches(7.43), Inches(2.75), cd)
alt_chart(gf, "Column chart: success rate of old ICP 0%, feature matching 30 to 40%, floor-plan search 98%")
ch = gf.chart
style_chart(ch)
ch.has_legend = True
ch.legend.position = XL_LEGEND_POSITION.BOTTOM
ch.legend.include_in_layout = False
ch.legend.font.size = Pt(12)
grid(ch.value_axis)
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale, ch.value_axis.major_unit = 0, 120, 20
ch.value_axis.tick_labels.font.size = Pt(12)
ch.category_axis.tick_labels.font.size = Pt(12)
pl = ch.plots[0]
pl.gap_width, pl.overlap = 60, -5
pl.has_data_labels = True
pl.data_labels.number_format = '0"%"'
pl.data_labels.number_format_is_linked = False
pl.data_labels.position = XL_LABEL_POSITION.OUTSIDE_END
pl.data_labels.font.size = Pt(12)
for ser, col in zip(pl.series, (STEEL, AMBER)):
    ser.format.fill.solid()
    ser.format.fill.fore_color.rgb = rgb(col)
stat(s, 5.3, 4.95, 2.3, "0%", "old ICP, and every failure was silent", color=BRICK, h=1.3)
stat(s, 7.8, 4.95, 2.3, "98%", "floor-plan search + ICP", color=GREEN, h=1.3)
stat(s, 10.3, 4.95, 2.43, "1-3 cm", "error at every stage in the full run", color=STEEL, h=1.3)
text(s, 5.3, 6.4, 7.4, 0.5, "Failures of the new method: 4 of 170 trials. Two were flagged by the method itself, two were near misses of 7 and 9 cm.",
     size=11, color=MUTED)

# =============================================================================
# 10. matching + half-built wall fix
# =============================================================================
s = new_slide("Matching scan points to the design", "The design tells us where every element should be, so we match instead of discovering", "IMPROVED", notes=(
    "After alignment we match. We do not try to discover walls from scratch, because the design already tells us where "
    "every element should be. We sample the design surface densely, tag each sample with its element, and put them in a "
    "KD-tree. A scan point joins an element if it is within 8 cm and clearly nearer to it than to any other element, by a "
    "4 cm margin. An element with enough points counts as detected, and how high its points reach tells us whether it "
    "is complete or only partly built. While doing this we found a bug: our simulator dropped the side faces of "
    "half-built walls, so they almost vanished from the scan. Fixing it cut the progress error from 4.8 to 2.1 points."))
for i, (h1, h2) in enumerate([("Tag the design", "Sample the design surface densely. Every sample knows its element."),
                              ("Look up fast", "A KD-tree finds the nearest design sample for every scan point."),
                              ("Assign with a margin", "A point joins an element if it is within 8 cm and clearly nearer to it than to any other."),
                              ("Read the height", "Enough points means detected. How high they reach tells complete from partly built.")]):
    y = TOP + i * 1.28
    numbered(s, MX, y + 0.05, i + 1)
    text(s, MX + 0.75, y, 4.7, 0.35, h1, size=17, bold=True)
    text(s, MX + 0.75, y + 0.38, 4.7, 0.8, h2, size=13, color=MUTED)
picture(s, "media/04_fix_half_built_walls/one_wall_before_after.png", 6.4, TOP, 6.33, 2.0, "One half-built wall before and after the fix: the old code left only a hidden bottom face")
caption(s, 6.4, TOP + 2.1, 6.33, "Fix: one wall that is 48% built. Old code kept only its hidden bottom face. New code cuts the wall to the height built.")
stat(s, 6.4, 4.5, 3.05, "4.8 → 2.1", "average progress error (points) after this one fix", color=BRICK, h=1.4, big_size=30)
stat(s, 9.68, 4.5, 3.05, "3-8 points", "too low at every stage before the fix: half-built walls were missed", color=STEEL, h=1.4, big_size=30)

# =============================================================================
# 11. visibility
# =============================================================================
s = new_slide("Not built, or just not seen?", "An element with no points is not always an element that is not built", "NEW", notes=(
    "The most important idea we added. If we find no points for an element, there are two explanations: it is not built, or "
    "the cameras could not see it. At mid-term we treated both as not built. Now, for every element we did not find, we "
    "raycast from the actual camera positions, using the estimated alignment and only the elements we did detect, and ask: "
    "if this element were built, would we have collected enough points? If yes it is not started. If not, it is unsure and "
    "left out of the progress number. In the picture the pink see-through elements are not started, and the dark grey ones "
    "are unsure. We also added a rule: a detection with fewer than 10% of the points we would expect is thrown out, because "
    "noisy points near wall junctions were making unbuilt walls look built. On the hardest test the old rule had 658 built "
    "elements missed; the visibility check has 327, and the average error went from 3.2 to 2.0 points."))
picture(s, "media/06_visibility_check/visibility_07.png", MX, TOP, 7.4, 3.3, "Visibility check at stage 7: not-started elements in pink, unsure elements in dark grey, next to the truth")
caption(s, MX, TOP + 3.4, 7.4, "Stage 7. Pink and see-through: not started. Dark grey: unsure, left out of the count. Truth on the right.")
box(s, 8.35, TOP, 4.38, 3.3)
text(s, 8.6, TOP + 0.15, 3.9, 0.4, "The question we ask", size=17, bold=True)
text(s, 8.6, TOP + 0.62, 3.9, 2.6,
     [[("Element not found. ", {"bold": True}), ("If it were built, would the cameras have collected enough points?", {})],
      [("Yes: ", {"bold": True, "color": GREEN}), ("not started.", {})],
      [("No: ", {"bold": True, "color": AMBER}), ("unsure, left out of the progress number.", {})],
      [("It uses the estimated alignment and only the elements we found, so it never peeks at the answer.", {"color": MUTED, "size": 13})]],
     size=14, space_after=10)
stat(s, MX, 5.45, 3.6, "3.2 → 2.0", "average error (points), old final-stage rule vs visibility check", color=BRICK, h=1.35, big_size=30)
stat(s, MX + 3.85, 5.45, 3.6, "658 → 327", "built elements missed, old rule vs visibility check", color=STEEL, h=1.35, big_size=30)
box(s, 8.35, 5.45, 4.38, 1.35, fill=SOFT_STEEL)
text(s, 8.6, 5.57, 3.9, 1.15, [[("10% rule: ", {"bold": True}), ("a detection with under 10% of the expected points is thrown out as noise.", {})]],
     size=14, anchor=MSO_ANCHOR.MIDDLE)

# =============================================================================
# 12. error bars
# =============================================================================
s = new_slide("A progress number with a 90% range", "The raw estimate runs low as the building fills up. A learned correction fixes most of it", "NEW", notes=(
    "Instead of one number we now report a progress estimate with a 90% range. The chart shows the error against the "
    "truth for each of the 16 stages on the hardest test. The orange line is the raw estimate, which drifts down to minus "
    "5 points late in the job, because built elements hide in places the cameras do not see. The blue line is the "
    "corrected estimate and the dashed lines are the 90% range. How: we group elements by their evidence, such as their "
    "status, whether they are below or at the construction front, their type, and how many points we found compared with "
    "expected. On one run we learn each group's typical bias. Then we apply it to a different run. Here we calibrated on "
    "the clean realistic run and tested on the busy site. Mean error went from 2.0 to 0.7 and the truth was inside the "
    "range at 15 of 16 stages. For volume-weighted progress the correction helps less, and the range is wider."))
cd = CategoryChartData()
cd.categories = [f"{i}" for i in range(16)]
cd.add_series("Raw estimate", [round(float(v), 2) for v in t13.signed_error_count])
cd.add_series("Corrected", [round(float(a - b), 2) for a, b in zip(t13.progress_cal_count, t13.progress_gt_count)])
cd.add_series("90% range, low", [round(float(a - b), 2) for a, b in zip(t13.progress_lo_count, t13.progress_gt_count)])
cd.add_series("90% range, high", [round(float(a - b), 2) for a, b in zip(t13.progress_hi_count, t13.progress_gt_count)])
text(s, MX, TOP - 0.05, 7.9, 0.3, "Error against the true progress (points), by construction stage", size=14, bold=True)
gf = s.shapes.add_chart(XL_CHART_TYPE.LINE_MARKERS, Inches(MX), Inches(TOP + 0.3), Inches(7.9), Inches(4.85), cd)
alt_chart(gf, "Line chart of the progress error for 16 stages: raw estimate, corrected estimate and 90% range")
ch = gf.chart
style_chart(ch)
ch.has_legend = True
ch.legend.position = XL_LEGEND_POSITION.BOTTOM
ch.legend.include_in_layout = False
ch.legend.font.size = Pt(12)
grid(ch.value_axis)
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale = -6, 4
ch.value_axis.tick_labels.font.size = Pt(12)
ch.value_axis.has_title = True
ch.value_axis.axis_title.text_frame.text = "points"
ch.value_axis.axis_title.text_frame.paragraphs[0].runs[0].font.size = Pt(12)
ch.value_axis.axis_title.text_frame.paragraphs[0].runs[0].font.bold = False
ch.category_axis.tick_labels.font.size = Pt(12)
ch.category_axis.tick_label_position = XL_TICK_LABEL_POSITION.LOW
ch.category_axis.has_title = True
ch.category_axis.axis_title.text_frame.text = "construction stage"
ch.category_axis.axis_title.text_frame.paragraphs[0].runs[0].font.size = Pt(12)
ch.category_axis.axis_title.text_frame.paragraphs[0].runs[0].font.bold = False
for ser, col, w, dash in zip(ch.plots[0].series, (BRICK, STEEL, MUTED, MUTED), (2.5, 2.5, 1.25, 1.25),
                             (None, None, MSO_LINE.DASH, MSO_LINE.DASH)):
    ser.format.line.color.rgb = rgb(col)
    ser.format.line.width = Pt(w)
    ser.smooth = False
    if dash:
        ser.format.line.dash_style = dash
        ser.marker.style = None
        ser.marker.format.fill.background()
        ser.marker.format.line.fill.background()
    else:
        ser.marker.format.fill.solid()
        ser.marker.format.fill.fore_color.rgb = rgb(col)
        ser.marker.format.line.color.rgb = rgb(col)
        ser.marker.size = 6
mean_raw = float(t13.error_count.mean())
mean_cal = float((t13.progress_cal_count - t13.progress_gt_count).abs().mean())
inside = int(t13.gt_in_range_count.sum())
stat(s, 8.85, TOP, 3.88, f"{mean_raw:.1f} → {mean_cal:.1f}", "average error (points), raw vs corrected", color=BRICK, h=1.3)
stat(s, 8.85, TOP + 1.5, 3.88, f"{inside} of 16", "stages with the truth inside the 90% range (about +/-1.6 points)", color=GREEN, h=1.3)
box(s, 8.85, TOP + 3.0, 3.88, 2.2, fill=SOFT_STEEL)
text(s, 9.1, TOP + 3.12, 3.4, 2.0,
     [[("How: ", {"bold": True}), ("group elements by their evidence, learn each group's usual bias on one run, apply it to a different run.", {})],
      [("Learned on the clean realistic run, tested on the busy site.", {"color": MUTED, "size": 13})]],
     size=14, space_after=8)

# =============================================================================
# 13. colour
# =============================================================================
s = new_slide("Colour: tried it, kept geometry", "Camera colour was tested for alignment and clutter, and geometry still decided", "EXPERIMENT", notes=(
    "Experiment one: colour. We gave every scan point a simulated camera colour, with materials from the design, "
    "sun, shadows, dirt, brick patterns, and some elements built from another material, and a material catalogue that "
    "is deliberately different from the one used to render. We used colour to drop clutter points and to rank candidate "
    "alignments. In the registration test all 12 cases were right with and without colour, and colour never actually decided: "
    "the second-best pose was always clearly worse on geometry. In the full pipeline plan_color gave 2.1 points against "
    "2.0 without colour. The reason: once lighting varies, nine of the materials look the same grey, and brick piles look "
    "like brick walls. So geometry stays the default. A natural next step is a model that recognises materials from texture "
    "patches, which we put on the list of future work."))
picture(s, "media/10_materials_color/scan_camera_colour.png", MX, TOP, 6.8, 2.02, "A scan with element-type colour compared with simulated camera colour")
picture(s, "media/10_materials_color/colour_filter.png", MX, TOP + 2.17, 6.8, 2.55, "Points the colour filter drops because their colour fits no material in the design")
caption(s, MX, TOP + 4.82, 6.8, "Top: scan colours, before and after. Bottom: points dropped by the colour filter (red).")
res = [("What we did", "Gave points a camera colour. Used it to drop clutter and to rank candidate alignments."),
       ("What happened", "All 12 test cases right, but colour never decided the answer. Full run: 2.1 points against 2.0 without it."),
       ("Why", "Under changing light, nine of the materials look the same grey. Brick piles look like brick walls."),
       ("Decision", "Geometry stays the default. Material recognition from texture is on the future list.")]
for i, (h1, h2) in enumerate(res):
    y = TOP + i * 1.3
    text(s, 7.75, y, 4.98, 0.35, h1, size=16, bold=True, color=AMBER if i == 3 else INK)
    text(s, 7.75, y + 0.36, 4.98, 0.9, h2, size=13, color=MUTED)

# =============================================================================
# 14. DART
# =============================================================================
s = new_slide("AI clutter removal with DART", "An image model labels clutter in camera views so we can remove it from the cloud", "EXPERIMENT", notes=(
    "Experiment two: AI clutter removal, done on one stage so far. We added machinery and tools on the built floors, such "
    "as a generator, mixer, scissor lift, ladder and barrels. A segmentation model called DART, built on SAM3, looks at "
    "simulated camera images and labels what is clutter. In the picture, the columns are: the camera image, DART's labels "
    "with clutter in red, the view after removal, and the merged cloud. With the right prompts it removed 41% of the true "
    "clutter points and kept 99.6% of the structure. A first attempt with prompts for structure kept only 68% and the "
    "estimate collapsed. The progress estimate did not change, about 48% against 85% true, because the cloud made only "
    "from camera images covers much less of the building than the scanner cloud. It needs a GPU we do not have on this "
    "machine, so it ran elsewhere and stays at one stage for now."))
picture(s, "results/dart/11_03_derde_verdieping_structure_pose_sheet1.png", MX, 1.6, 6.6, 5.3, "DART test: camera image, DART labels with clutter in red, the view after removal, and the merged cloud")
text(s, 7.5, TOP, 5.2, 0.9,
     "DART (built on SAM3) labels each camera image. Points on things it calls clutter are removed.", size=15)
stat(s, 7.5, TOP + 1.05, 2.5, "41%", "of the clutter points removed", color=STEEL, h=1.3)
stat(s, 10.2, TOP + 1.05, 2.53, "99.6%", "of the structure kept", color=GREEN, h=1.3)
box(s, 7.5, TOP + 2.55, 5.23, 2.25, fill=SOFT_BRICK)
text(s, 7.75, TOP + 2.7, 4.75, 2.0,
     [[("Progress did not change", {"bold": True, "size": 16})],
      "About 48% with or without removal, against 85% true. The cloud made only from camera images covers much less of the building.",
      [("One stage so far. It needs a GPU, so it ran on another machine.", {"color": MUTED, "size": 13})]],
     size=14, space_after=7)

# =============================================================================
# 15. COLMAP photos -> cloud
# =============================================================================
s = new_slide("Point clouds from drone photos", "COLMAP builds a 3D cloud from 60 simulated drone photos, like a real survey", "NEW", notes=(
    "Experiment three: photogrammetry with COLMAP. Until now our clouds came straight from raycasting. A real drone "
    "survey gives photos, and COLMAP turns photos into a cloud. We rendered 60 drone photos of the final stage, three "
    "rings of 20, with material colours, brick pattern, sun and shadows. One lesson: COLMAP matches points between photos, "
    "so the surfaces need fine detail that stays fixed on the wall; flat colours gave it nothing to match. COLMAP "
    "registered all 60 photos and produced about 487,000 points in 28 minutes on the GPU. Its own scale was nine times off "
    "metres, which is realistic, so we move it to the survey frame by fitting the solved camera positions to noisy GPS tags. "
    "Measured against the true surface, its median error is 3.7 cm. In the right picture, green points are close to the "
    "design. The facades are good, windows are holes, roof edges have stray points, and the ground is red only because it "
    "is not part of the design. The right column is a perfect cloud from the same photos."))
picture(s, "media/11_colmap_photogrammetry/drone_photos.png", MX, TOP, 3.9, 3.5, "Four of the 60 simulated drone photos used as input to COLMAP")
picture(s, "media/11_colmap_photogrammetry/colmap_cloud_vs_ideal.png", 4.65, TOP, 8.08, 4.86, "The COLMAP cloud in camera colour and by distance to the design, next to a perfect cloud from the same photos")
caption(s, MX, TOP + 3.6, 3.9, "Input: 60 photos, 3 altitude rings.")
stat(s, MX, 5.65, 1.85, "487k", "points", color=STEEL, h=1.2, big_size=28)
stat(s, MX + 2.05, 5.65, 1.85, "3.7 cm", "median error", color=GREEN, h=1.2, big_size=28)
caption(s, 4.65, TOP + 4.95, 8.08, "Green: close to the design. Red: far (the ground is not in the design). Right: a perfect cloud from the same photos.")

# =============================================================================
# 16. COLMAP results
# =============================================================================
s = new_slide("COLMAP: fine mid-build, weak at the end", "Through our pipeline, photo clouds work mid-construction but lose the finished building", "NEW", notes=(
    "What happens when we put the COLMAP cloud through the same pipeline. Chart: estimated progress for two stages, the "
    "first-floor interior and the final stage, with the true progress in grey. Green is our usual raycast cloud with indoor "
    "scans, blue is a perfect cloud from the same 60 photos, and orange is COLMAP. At the mid stage COLMAP is five points "
    "low. At the finished building it gives 65% against 100% true, with 263 built elements missed. The blue bar tells us "
    "why: a perfect cloud from the same photos gives 94%, so the drone viewpoints are good enough; the loss comes from "
    "the reconstruction, with holes at windows, stray points at roof edges and lost thin parts. The scale and position were "
    "fixed with GPS tags and registration worked on both stages. Next steps are more and closer photos, oblique views and "
    "cleaning the cloud. We do not yet know why the finished building does so much worse than the mid stage."))
st_ids = ["07_01_eerste_verdieping_interior", "15_04_dak_envelope"]
cd = CategoryChartData()
cd.categories = ["Stage 07: first-floor interior", "Stage 15: finished roof"]
cd.add_series("True", [round(float(sfm.loc[k, "progress_gt_count"]), 1) for k in st_ids])
cd.add_series("Raycast + indoor scans", [round(float(t13i.loc[k, "progress_est_count"]), 1) for k in st_ids])
cd.add_series("Perfect photo cloud", [round(float(sfm_ideal.loc[k, "progress_est_count"]), 1) for k in st_ids])
cd.add_series("COLMAP cloud", [round(float(sfm.loc[k, "progress_est_count"]), 1) for k in st_ids])
text(s, MX, TOP - 0.05, 7.4, 0.3, "Estimated progress (%) through the same pipeline", size=14, bold=True)
gf = s.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(MX), Inches(TOP + 0.3), Inches(7.4), Inches(4.85), cd)
alt_chart(gf, "Column chart of estimated progress at two stages for raycast, perfect photo cloud and COLMAP cloud, with true progress")
ch = gf.chart
style_chart(ch)
ch.has_legend = True
ch.legend.position = XL_LEGEND_POSITION.BOTTOM
ch.legend.include_in_layout = False
ch.legend.font.size = Pt(12)
grid(ch.value_axis)
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale = 0, 110
ch.value_axis.major_unit = 20
ch.value_axis.tick_labels.font.size = Pt(12)
ch.category_axis.tick_labels.font.size = Pt(12)
ch.plots[0].gap_width = 60
ch.plots[0].overlap = -5
pl = ch.plots[0]
pl.has_data_labels = True
pl.data_labels.number_format = "0"
pl.data_labels.number_format_is_linked = False
pl.data_labels.position = XL_LABEL_POSITION.OUTSIDE_END
pl.data_labels.font.size = Pt(12)
for ser, col in zip(pl.series, ("B8C2CC", GREEN, STEEL, BRICK)):
    ser.format.fill.solid()
    ser.format.fill.fore_color.rgb = rgb(col)
e07 = abs(float(sfm.loc[st_ids[0], "progress_est_count"]) - float(sfm.loc[st_ids[0], "progress_gt_count"]))
e15 = abs(float(sfm.loc[st_ids[1], "progress_est_count"]) - float(sfm.loc[st_ids[1], "progress_gt_count"]))
stat(s, 8.35, TOP, 2.1, f"{e07:.0f}", "points off, mid-build", color=GREEN, h=1.3)
stat(s, 10.63, TOP, 2.1, f"{e15:.0f}", "points off, finished building", color=BRICK, h=1.3)
box(s, 8.35, TOP + 1.5, 4.38, 3.2)
text(s, 8.6, TOP + 1.65, 3.9, 3.0,
     [[("The loss is in the reconstruction", {"bold": True, "size": 16})],
      "A perfect cloud from the same photos gives 94%, so the drone viewpoints are good enough.",
      "COLMAP leaves holes at windows, adds stray points at roof edges and loses thin parts.",
      [("Next: more and closer photos, oblique views, cleaning the cloud.", {"color": MUTED, "size": 13})]],
     size=14, space_after=8)

# =============================================================================
# 16b. COLMAP indoors
# =============================================================================
mi = json.load(open("results/sfm/07_01_eerste_verdieping_interior_int_metrics.json"))
sfm_int = pd.read_csv(E + "main_table_sfm_int.csv").iloc[0]
s = new_slide("COLMAP indoors: accurate, but hard to link", "The same test inside the building: 72 photos of the ground floor", "EXPERIMENT", notes=(
    "We also tried the interior, because everything so far used exterior drone photos. We rendered 72 photos on the ground floor: "
    "18 standing positions, four views each. COLMAP split them into two separate models and the larger one used only 26 photos, because "
    "rooms separated by walls share almost no views. Where the cloud exists it is very accurate, a median error of 4.6 millimetres. "
    "But it covers only a few rooms, and that broke registration: the floor-plan search chose a look-alike place 11.75 metres away, and "
    "our own checks did not warn, because the look-alike ratio was 0.85, just under the 0.9 threshold. So the progress estimate from it is "
    "meaningless. Two lessons: indoor photo capture needs a continuous walk so the photos stay connected, and registration of small partial "
    "clouds needs a stricter safety check. A perfect cloud from the same photos works, 88% against 97% true, but sees only the ground floor."))
picture(s, "results/report/figs/interior_photos.png", MX, TOP - 0.05, 5.9, 4.5, "Four of the 72 simulated interior photos of the ground floor")
caption(s, MX, TOP + 4.5, 5.9, "Simulated ground-floor photos: 18 standing positions, 4 views each.")
stat(s, 6.9, TOP, 1.85, f"{mi['n_images']} of {mi['n_images_total']}", "photos linked into one model", color=BRICK, h=1.5, big_size=26)
stat(s, 8.9, TOP, 1.85, f"{1000 * mi['median_dist_true_fit_m']:.1f} mm", "median error where covered", color=GREEN, h=1.5, big_size=26)
stat(s, 10.9, TOP, 1.83, f"{sfm_int.registration_err_m:.2f} m", "registration error, with no warning", color=BRICK, h=1.5, big_size=26)
box(s, 6.9, TOP + 1.75, 5.83, 2.75)
text(s, 7.15, TOP + 1.9, 5.35, 2.5,
     [[("Why it fails", {"bold": True, "size": 16})],
      "Rooms separated by walls share almost no views, so the photos of one room do not connect to the next.",
      "A cloud of a few look-alike rooms fits several places. Alignment picked a wrong one and our look-alike check stayed just under its threshold.",
      [("Next: a continuous walk through the doorways, and a stricter check for small clouds.", {"color": MUTED, "size": 13})]],
     size=14, space_after=8)

# =============================================================================
# 17. what each change did
# =============================================================================
s = new_slide("What each change did", "Average progress error over the 16 stages, in points. Lower is better", notes=(
    "The summary of results. Left chart: how the pipeline improved on the easy world. The first pipeline had 4.8 points of "
    "error. The half-built wall fix brought it to 2.1, the visibility check to 1.5, and GPS-sized offsets with the new "
    "registration to 0.8, even though that test is much harder. Right chart: on the hardest world (realistic sensor, busy "
    "site, late and misplaced elements) we remove one piece at a time and show two measures: the count error, where every "
    "element counts equally, and the volume error, where big elements count more. Without the 10 percent noise filter the "
    "count error looks fine, 1.6, but the volume error triples, 6.4 against 2.2. With the old rule for unseen elements both "
    "errors are worse. The 10 percent filter rejects a detection that got fewer than 10 percent of the points we expected for that "
    "element; it stops stray points near an unbuilt wall from making it look built. Bottom boxes: the old ICP alignment is 40 "
    "points off with no warning. The error-bar correction is a learned fix for the usual under-count of hidden built elements; "
    "it is added on top of the pipeline and brings the average error from 2.0 to 0.7 points, and gives a 90 percent range. "
    "As the world gets harder from the simple sensor to the busy site the same pipeline goes from 0.8 to 2.0."))
cd = CategoryChartData()
hist = [("First pipeline", "First pipeline"), ("+ half-built wall fix", "+ half-built wall fix"),
        ("+ visibility check", "+ visibility check"), ("+ GPS offsets, new alignment", "+ GPS-sized offsets, new registration")]
cd.categories = [a for a, _ in hist]
cd.add_series("Average error (points)", [round(lad(b), 3) for _, b in hist])
text(s, MX, TOP - 0.05, 5.9, 0.3, "How the pipeline improved (easy world)", size=14, bold=True)
gf = s.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(MX), Inches(TOP + 0.3), Inches(5.9), Inches(3.3), cd)
alt_chart(gf, "Column chart: average error falls from 4.8 to 0.8 points as fixes were added")
ch = gf.chart
style_chart(ch)
ch.has_legend = False
grid(ch.value_axis)
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale, ch.value_axis.major_unit = 0, 6, 2
ch.value_axis.tick_labels.font.size = Pt(12)
ch.category_axis.tick_labels.font.size = Pt(11)
pl = ch.plots[0]
pl.gap_width = 50
pl.has_data_labels = True
pl.data_labels.number_format = "0.0"
pl.data_labels.number_format_is_linked = False
pl.data_labels.position = XL_LABEL_POSITION.OUTSIDE_END
pl.data_labels.font.size = Pt(12)
pl.series[0].format.fill.solid()
pl.series[0].format.fill.fore_color.rgb = rgb(STEEL)

cd = CategoryChartData()
hard = [("Our full pipeline", "Full pipeline (all changes)"),
        ("Without the 10% noise filter", "... without the 10%-of-expected-points rule"),
        ("Old rule for unseen elements", "... old 'detected at final stage' rule instead of visibility check")]
cd.categories = [a for a, _ in hard]
cd.add_series("Count error", [round(lad(b), 3) for _, b in hard])
cd.add_series("Volume error", [round(float(ladder.loc[b, "mean_error_volume"]), 3) for _, b in hard])
text(s, 6.85, TOP - 0.05, 5.88, 0.3, "Hardest world: what happens if we remove one piece", size=14, bold=True)
gf = s.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(6.85), Inches(TOP + 0.3), Inches(5.88), Inches(3.3), cd)
alt_chart(gf, "Column chart: count error and volume error on the hardest world for the full pipeline and with one piece removed")
ch = gf.chart
style_chart(ch)
ch.has_legend = True
ch.legend.position = XL_LEGEND_POSITION.BOTTOM
ch.legend.include_in_layout = False
ch.legend.font.size = Pt(12)
grid(ch.value_axis)
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale, ch.value_axis.major_unit = 0, 8, 2
ch.value_axis.tick_labels.font.size = Pt(12)
ch.category_axis.tick_labels.font.size = Pt(11)
pl = ch.plots[0]
pl.gap_width, pl.overlap = 60, -5
pl.has_data_labels = True
pl.data_labels.number_format = "0.0"
pl.data_labels.number_format_is_linked = False
pl.data_labels.position = XL_LABEL_POSITION.OUTSIDE_END
pl.data_labels.font.size = Pt(12)
for ser, col in zip(pl.series, (GREEN, STEEL)):
    ser.format.fill.solid()
    ser.format.fill.fore_color.rgb = rgb(col)
old_icp = lad("... old ICP registration")
no_rule = ladder.loc["... without the 10%-of-expected-points rule"]
full = ladder.loc["Full pipeline (all changes)"]
cal_err = float((t13.progress_cal_count - t13.progress_gt_count).abs().mean())
expl = [("Old alignment (ICP)", f"{old_icp:.0f} points of error", BRICK,
         "The scan ends about 3 m off the design and nothing warns. This is why we built a new alignment."),
        ("The 10% noise filter", f"volume error {float(full.mean_error_volume):.1f} vs {float(no_rule.mean_error_volume):.1f} without it", STEEL,
         f"Rejects a detection that got under 10% of the points expected for that element. Without it, false 'built' calls go {int(full.n_false_built)} to {int(no_rule.n_false_built)}."),
        ("Error-bar correction", f"{float(full.mean_error):.1f} down to {cal_err:.1f} points", GREEN,
         "A learned fix for the usual under-count of hidden built elements. It also gives a 90% range. Not shown in the chart: it is added on top.")]
for i, (h1, big, col, body) in enumerate(expl):
    x = MX + i * 4.09
    box(s, x, 5.35, 3.95, 1.58)
    text(s, x + 0.2, 5.42, 3.55, 0.28, h1, size=13, bold=True)
    text(s, x + 0.2, 5.7, 3.55, 0.3, big, size=14, bold=True, color=col)
    text(s, x + 0.2, 6.05, 3.55, 0.85, body, size=12, color=MUTED)

# =============================================================================
# 18. limits
# =============================================================================
s = new_slide("Where it still falls short", "The honest list of what the results do not show yet", notes=(
    "Three honest limits. First, clutter still fools the matching: 28 of the 67 false built calls on the busy site came "
    "from scaffold decks, workers and material piles that sit where unbuilt elements would be. Second, not everything is "
    "seen: about a quarter of the elements are never seen by our camera plans even when built, and COLMAP loses windows "
    "and thin parts, with the finished building 35 points off. Third, all of this is simulation: one building, and no "
    "real survey has been tested yet. Our thresholds were tuned on these runs, so a frozen test split and a real dataset "
    "are the most important next checks."))
lim = [("28 of 67", "Clutter fools matching", "False 'built' calls on the busy site came from scaffold decks, workers and piles standing where unbuilt elements would be.", BRICK),
       ("25%", "Not everything is seen", "About a quarter of elements are never seen by the camera plans, even when built. COLMAP also loses windows and thin parts.", AMBER),
       ("1 building", "Simulation only", "No real survey has been tested yet, and the thresholds were tuned on these runs. A frozen test split is still needed.", STEEL)]
for i, (big, h1, h2, col) in enumerate(lim):
    x = MX + i * 4.12
    box(s, x, TOP, 3.9, 3.9)
    text(s, x + 0.3, TOP + 0.3, 3.3, 0.9, big, size=40, bold=True, color=col, anchor=MSO_ANCHOR.MIDDLE)
    text(s, x + 0.3, TOP + 1.4, 3.3, 0.45, h1, size=19, bold=True)
    text(s, x + 0.3, TOP + 1.95, 3.3, 1.9, h2, size=15, color=MUTED)
box(s, MX, TOP + 4.15, 12.13, 0.85, fill=SOFT_STEEL)
text(s, MX + 0.3, TOP + 4.15, 11.5, 0.85,
     [[("Most important next check: ", {"bold": True}), ("freeze a test split, then run the pipeline unchanged on one real survey.", {})]],
     size=16, anchor=MSO_ANCHOR.MIDDLE)

# =============================================================================
# 19. next
# =============================================================================
s = new_slide("What comes next", "Six directions, in the order we would take them", notes=(
    "Where this goes next. One: a real data check. Run the pipeline unchanged on a real survey, so we learn which simulated "
    "effects mattered. Two: drone-only flights with oblique and closer views, since a real drone cannot scan indoors. "
    "Three: better COLMAP clouds, with more photos, cleaning the cloud and a different dense step. Four: recognise materials "
    "from texture with a model, and compare against the material the design says. Five: use colour registration to merge "
    "two scans of the same building, where colour should help more than against the design. Six: work activities such as "
    "formwork, rebar and concrete, not only built or not built."))
nxt = [("Real data check", "Run the pipeline unchanged on a real survey. Learn which simulated effects mattered."),
       ("Drone-only flights", "Oblique and closer views. A real drone cannot scan indoors."),
       ("Better COLMAP clouds", "More photos, a continuous indoor walk, cleaning the cloud."),
       ("Material recognition", "A model reads material from a texture patch and compares it with the design."),
       ("Merge two scans", "Align two runs of the same building. Colour should help more here."),
       ("Work activities", "Formwork, rebar, concrete: more than built or not built.")]
for i, (h1, h2) in enumerate(nxt):
    x = MX + (i % 3) * 4.12
    y = TOP + (i // 3) * 2.6
    box(s, x, y, 3.9, 2.4)
    numbered(s, x + 0.25, y + 0.25, i + 1, color=NAVY)
    text(s, x + 0.25, y + 0.9, 3.4, 0.4, h1, size=18, bold=True)
    text(s, x + 0.25, y + 1.35, 3.4, 1.0, h2, size=14, color=MUTED)

# =============================================================================
# 20. closing
# =============================================================================
s = prs.slides.add_slide(prs.slide_layouts[6])
s.background.fill.solid()
s.background.fill.fore_color.rgb = rgb(NAVY)
text(s, 0.8, 0.9, 11.5, 0.4, "SUMMARY", size=14, bold=True, color=AMBER)
text(s, 0.8, 1.4, 11.7, 1.4, "A realistic test, and a method that holds up", size=40, bold=True, color=WHITE, name="title")
cards = [("4.8 → 0.8", "average error (points) on the easy test, after fixing wall clipping, adding the visibility check and the new alignment"),
         (f"{mean_raw:.1f}", "average error on a much harder, realistic site with scaffolding, workers and late elements"),
         (f"{mean_cal:.1f}", "with the error-bar correction, and the truth inside the 90% range at 15 of 16 stages")]
for i, (big, lab) in enumerate(cards):
    x = 0.8 + i * 4.0
    box(s, x, 3.2, 3.75, 2.5, fill="22303F")
    text(s, x + 0.3, 3.4, 3.2, 0.9, big, size=44, bold=True, color=AMBER if i != 1 else "F0B9A5", anchor=MSO_ANCHOR.MIDDLE)
    text(s, x + 0.3, 4.4, 3.2, 1.25, lab, size=14, color="CFE0EE")
text(s, 0.8, 6.2, 8, 0.5, "Thank you. Questions?", size=26, bold=True, color=WHITE)
s.notes_slide.notes_text_frame.text = (
    "To summarise. On the easy test, the average progress error went from 4.8 to 0.8 points. On a much harder, realistic "
    "site the same pipeline has about 2 points of error, and with the error-bar correction about 0.7, with the truth inside "
    "the range at 15 of 16 stages. The old ICP would have been 40 points off with no warning. We also tested colour, AI "
    "clutter removal and COLMAP photogrammetry, and we know the next steps. Thank you, we are happy to take questions.")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
prs.save(OUT)
print(f"wrote {OUT} ({len(prs.slides)} slides)")
