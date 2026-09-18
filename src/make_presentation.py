"""Build the mid-review presentation from project results.

Reads results/main_table.csv, data/bim/bim_complete.csv, results/p4_eval_summary.csv,
and embeds the already-generated figures (results/p1_centroids.png,
results/figures/p4_segmentation_quality_vs_stage.png, results/figures/p7_progress_error.png).

Screenshot slides (viewer UI) are left as labelled placeholders -- swap in real
images by dropping files into results/presentation/screenshots/ with the names
printed by this script and re-running.

Run: .venv/Scripts/python.exe src/make_presentation.py
Output: results/presentation/BIM_Progress_Monitoring_MidReview.pptx
"""
import os

import pandas as pd
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn

OUT_DIR = "results/presentation"
SCREENSHOT_DIR = os.path.join(OUT_DIR, "screenshots")
OUT_PPTX = os.path.join(OUT_DIR, "BIM_Progress_Monitoring_MidReview.pptx")

# -- theme -------------------------------------------------------------
BG = RGBColor(0x14, 0x16, 0x1a)
PANEL = RGBColor(0x1e, 0x21, 0x27)
TEXT = RGBColor(0xe8, 0xe6, 0xe1)
TEXT_DIM = RGBColor(0x9a, 0xa0, 0xa8)
ACCENT = RGBColor(0xef, 0x83, 0x54)   # confirmed-built orange, matches viewer
ACCENT2 = RGBColor(0x5e, 0xc8, 0xf2)  # wireframe blue, matches viewer
WALL_C = RGBColor(0x4c, 0x78, 0xa8)
SLAB_C = RGBColor(0xf5, 0x85, 0x18)
BEAM_C = RGBColor(0x54, 0xa2, 0x4b)
COL_C = RGBColor(0xe4, 0x57, 0x56)

FONT_TITLE = "IBM Plex Sans"
FONT_BODY = "IBM Plex Sans"
FONT_MONO = "IBM Plex Mono"

SLIDE_W = Inches(13.333)
SLIDE_H = Inches(7.5)

prs = Presentation()
prs.slide_width = SLIDE_W
prs.slide_height = SLIDE_H
BLANK = prs.slide_layouts[6]


def add_slide():
    slide = prs.slides.add_slide(BLANK)
    bg = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, SLIDE_W, SLIDE_H)
    bg.fill.solid()
    bg.fill.fore_color.rgb = BG
    bg.line.fill.background()
    bg.shadow.inherit = False
    return slide


def textbox(slide, left, top, width, height, text, size=18, color=TEXT, bold=False,
            font=FONT_BODY, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, line_spacing=1.0,
            italic=False):
    tb = slide.shapes.add_textbox(left, top, width, height)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    lines = text.split("\n") if isinstance(text, str) else text
    for i, line in enumerate(lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.line_spacing = line_spacing
        r = p.add_run()
        r.text = line
        r.font.size = Pt(size)
        r.font.color.rgb = color
        r.font.bold = bold
        r.font.italic = italic
        r.font.name = font
    return tb


def bullets(slide, left, top, width, height, items, size=15, color=TEXT, font=FONT_BODY,
            space_after=10, marker_color=ACCENT):
    tb = slide.shapes.add_textbox(left, top, width, height)
    tf = tb.text_frame
    tf.word_wrap = True
    for i, item in enumerate(items):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.space_after = Pt(space_after)
        p.line_spacing = 1.12
        indent = item[0] if isinstance(item, tuple) else 0
        txt = item[1] if isinstance(item, tuple) else item
        prefix = "     " * indent + ("-  " if indent else "▪  ")
        r = p.add_run()
        r.text = prefix
        r.font.size = Pt(size)
        r.font.color.rgb = marker_color if indent == 0 else TEXT_DIM
        r.font.name = font
        r2 = p.add_run()
        r2.text = txt
        r2.font.size = Pt(size)
        r2.font.color.rgb = color if indent == 0 else TEXT_DIM
        r2.font.name = font
    return tb


def kicker_title(slide, kicker, title, num=None):
    textbox(slide, Inches(0.6), Inches(0.35), Inches(8), Inches(0.35), kicker.upper(),
            size=13, color=ACCENT, bold=True, font=FONT_MONO)
    textbox(slide, Inches(0.6), Inches(0.68), Inches(11.5), Inches(0.9), title,
            size=30, color=TEXT, bold=True)
    if num is not None:
        textbox(slide, Inches(12.4), Inches(6.95), Inches(0.6), Inches(0.4), num,
                size=12, color=TEXT_DIM, font=FONT_MONO, align=PP_ALIGN.RIGHT)
    line = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.6), Inches(1.55),
                                   Inches(12.13), Pt(1.5))
    line.fill.solid()
    line.fill.fore_color.rgb = RGBColor(0x33, 0x36, 0x3c)
    line.line.fill.background()
    line.shadow.inherit = False
    return slide


def panel(slide, left, top, width, height, fill=PANEL, line_color=None):
    box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
    try:
        box.adjustments[0] = 0.04
    except Exception:
        pass
    box.fill.solid()
    box.fill.fore_color.rgb = fill
    if line_color:
        box.line.color.rgb = line_color
        box.line.width = Pt(1)
    else:
        box.line.fill.background()
    box.shadow.inherit = False
    return box


def stat_card(slide, left, top, width, height, value, label, value_color=ACCENT):
    panel(slide, left, top, width, height)
    textbox(slide, left, top + Inches(0.12), width, Inches(0.55), value,
            size=26, color=value_color, bold=True, font=FONT_MONO, align=PP_ALIGN.CENTER)
    textbox(slide, left + Inches(0.1), top + height - Inches(0.4), width - Inches(0.2), Inches(0.35),
            label, size=11, color=TEXT_DIM, align=PP_ALIGN.CENTER)


def picture_fit(slide, path, left, top, max_w, max_h):
    from PIL import Image
    with Image.open(path) as im:
        iw, ih = im.size
    ar = iw / ih
    box_ar = max_w / max_h
    if ar > box_ar:
        w = max_w
        h = w / ar
    else:
        h = max_h
        w = h * ar
    x = left + (max_w - w) / 2
    y = top + (max_h - h) / 2
    slide.shapes.add_picture(path, x, y, width=int(w), height=int(h))


def screenshot_placeholder(slide, left, top, width, height, label):
    box = panel(slide, left, top, width, height, fill=RGBColor(0x22, 0x25, 0x2b),
                line_color=RGBColor(0x3a, 0x3d, 0x44))
    textbox(slide, left, top + height / 2 - Inches(0.35), width, Inches(0.35),
            "[ SCREENSHOT PENDING ]", size=13, color=TEXT_DIM, align=PP_ALIGN.CENTER,
            font=FONT_MONO)
    textbox(slide, left + Inches(0.2), top + height / 2 + Inches(0.05), width - Inches(0.4), Inches(0.5),
            label, size=11, color=TEXT_DIM, align=PP_ALIGN.CENTER, italic=True)


def picture_or_placeholder(slide, name, left, top, width, height, label):
    path = os.path.join(SCREENSHOT_DIR, name)
    if os.path.exists(path):
        picture_fit(slide, path, left, top, width, height)
    else:
        screenshot_placeholder(slide, left, top, width, height, label)


def footer(slide, page):
    textbox(slide, Inches(0.6), Inches(7.08), Inches(6), Inches(0.3),
            "BIM-Based Construction Progress Monitoring — Mid Review", size=9,
            color=TEXT_DIM, font=FONT_MONO)
    textbox(slide, Inches(12.0), Inches(7.08), Inches(0.8), Inches(0.3), str(page),
            size=9, color=TEXT_DIM, font=FONT_MONO, align=PP_ALIGN.RIGHT)


# =======================================================================
# Slide 1 -- Title
# =======================================================================
s = add_slide()
accent_bar = slide_bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(3.55), Inches(0.12), Inches(1.6))
accent_bar.fill.solid(); accent_bar.fill.fore_color.rgb = ACCENT; accent_bar.line.fill.background()
accent_bar.shadow.inherit = False

textbox(s, Inches(0.9), Inches(2.9), Inches(11.5), Inches(0.4), "MID-TERM PROJECT REVIEW",
        size=15, color=ACCENT, bold=True, font=FONT_MONO)
textbox(s, Inches(0.9), Inches(3.35), Inches(11.5), Inches(1.6),
        "BIM-Based Construction Progress\nMonitoring via Drone Simulation",
        size=38, color=TEXT, bold=True, line_spacing=1.05)
textbox(s, Inches(0.9), Inches(5.05), Inches(11), Inches(0.5),
        "Synthetic drone point clouds · BIM-registration matching · automated per-element progress scoring",
        size=15, color=TEXT_DIM)
textbox(s, Inches(0.9), Inches(6.7), Inches(6), Inches(0.4), "Sagar", size=12, color=TEXT_DIM, font=FONT_MONO)
textbox(s, Inches(10.5), Inches(6.7), Inches(2.2), Inches(0.4), "September 2026", size=12,
        color=TEXT_DIM, font=FONT_MONO, align=PP_ALIGN.RIGHT)

# =======================================================================
# Slide 2 -- Motivation
# =======================================================================
s = add_slide()
kicker_title(s, "Motivation", "A BIM tells you what SHOULD be built — not what IS", num="01")
bullets(s, Inches(0.6), Inches(1.9), Inches(6.0), Inches(4.5), [
    "A design BIM encodes intended geometry: every wall, slab, beam and column, exactly as planned.",
    "It says nothing about what has actually been constructed on site.",
    "Manual progress inspection (walkthroughs, photos, spreadsheets) does not scale to large or fast-moving sites.",
    "Research question: can drone-collected point cloud data automatically determine which BIM elements are built, and from that estimate overall project progress?",
], size=16, space_after=16)

panel(s, Inches(7.1), Inches(1.9), Inches(5.6), Inches(4.6))
textbox(s, Inches(7.4), Inches(2.1), Inches(5.0), Inches(0.4), "WHY SIMULATION FIRST", size=13,
        color=ACCENT, bold=True, font=FONT_MONO)
bullets(s, Inches(7.4), Inches(2.6), Inches(5.0), Inches(3.7), [
    "On a real site, the true completion state is never known exactly — you can't separate detection error from matching error from scoring error.",
    "In simulation, the completion state is controlled exactly by construction.",
    "This makes it possible to produce error-vs-completion curves that are impossible to produce on real data — that controllability is the core contribution.",
], size=14, space_after=14)
footer(s, 2)

# =======================================================================
# Slide 3 -- Pipeline overview
# =======================================================================
s = add_slide()
kicker_title(s, "Pipeline", "End-to-end pipeline, phase by phase", num="02")

stages = [
    ("P1", "Parse BIM", "IFC → ground-truth\nelement table"),
    ("P2", "Ablate", "Realistic partial\nbuild states"),
    ("P3", "Simulate", "Raycast drone\npoint clouds"),
    ("P4", "Match", "Register + assign\npoints to elements"),
    ("P7", "Score", "Per-element status\n→ progress %"),
]
n = len(stages)
gap = Inches(0.35)
total_w = Inches(12.13)
box_w = Emu(int((total_w - gap * (n - 1)) / n))
y = Inches(2.3)
box_h = Inches(1.9)
x = Inches(0.6)
xs = []
for i, (tag, name, desc) in enumerate(stages):
    xs.append(x)
    panel(s, x, y, box_w, box_h, fill=PANEL, line_color=RGBColor(0x33,0x36,0x3c))
    textbox(s, x, y + Inches(0.15), box_w, Inches(0.3), tag, size=13, color=ACCENT, bold=True,
            font=FONT_MONO, align=PP_ALIGN.CENTER)
    textbox(s, x + Inches(0.1), y + Inches(0.5), box_w - Inches(0.2), Inches(0.4), name, size=16,
            color=TEXT, bold=True, align=PP_ALIGN.CENTER)
    textbox(s, x + Inches(0.1), y + Inches(0.95), box_w - Inches(0.2), Inches(0.8), desc, size=11,
            color=TEXT_DIM, align=PP_ALIGN.CENTER, line_spacing=1.1)
    x = Emu(int(x + box_w + gap))

for i in range(n - 1):
    arrow_x = Emu(int(xs[i] + box_w))
    arr = s.shapes.add_shape(MSO_SHAPE.CHEVRON, arrow_x, y + box_h/2 - Inches(0.12), gap, Inches(0.24))
    arr.fill.solid(); arr.fill.fore_color.rgb = ACCENT2; arr.line.fill.background()
    arr.shadow.inherit = False

panel(s, Inches(0.6), Inches(4.65), Inches(12.13), Inches(2.0))
textbox(s, Inches(0.9), Inches(4.85), Inches(11.5), Inches(0.4), "SCOPE DECISION vs. THE ORIGINAL SPEC",
        size=13, color=ACCENT, bold=True, font=FONT_MONO)
bullets(s, Inches(0.9), Inches(5.3), Inches(11.4), Inches(1.3), [
    "Original spec routed through Blender-rendered RGB/depth/index images, then a learned/oracle 2D detector, then 3D fusion and Hungarian matching.",
    "v1 goes straight from BIM to a raycast-simulated point cloud (Open3D), then matches points directly against the known design BIM via ICP registration — a leaner, classical scan-vs-BIM approach (Bosché & Haas 2008/2009) that skips the image pipeline entirely.",
], size=14, space_after=10)
footer(s, 3)

# =======================================================================
# Slide 4 -- Dataset
# =======================================================================
s = add_slide()
kicker_title(s, "Dataset", "Schependomlaan — design BIM as ground truth", num="03")

textbox(s, Inches(0.6), Inches(1.85), Inches(5.8), Inches(0.4), "IN-SCOPE STRUCTURAL ELEMENTS",
        size=13, color=ACCENT, bold=True, font=FONT_MONO)

counts = [("Wall", 879, WALL_C), ("Slab", 279, SLAB_C), ("Beam", 174, BEAM_C), ("Column", 23, COL_C)]
cx = Inches(0.6)
cw = Inches(1.38)
for label, count, color in counts:
    stat_card(s, cx, Inches(2.3), cw, Inches(1.3), str(count), label, value_color=color)
    cx = Emu(int(cx + cw + Inches(0.12)))
stat_card(s, cx, Inches(2.3), Inches(1.38), Inches(1.3), "1,355", "Total", value_color=TEXT)

bullets(s, Inches(0.6), Inches(3.95), Inches(5.8), Inches(2.6), [
    "Dutch residential project, TU Eindhoven / RAAMAC — IFC2x3 design model from Archicad.",
    "Low-rise residential typology: walls and slabs dominate; columns are sparse (23 total).",
    (1, "Doors, windows, MEP — out of scope, per spec."),
    "Sanity check: the Synchro-exported weekly as-planned IFCs reassign GlobalIds — 0% overlap with the design IFC's identities.",
    (1, "→ real multi-week ground truth unusable for element identity; fully synthetic staged ablation built instead."),
], size=14, space_after=10)

picture_fit(s, "results/presentation/screenshots/design_bim.png", Inches(6.7), Inches(1.85),
            Inches(6.0), Inches(4.9))
footer(s, 4)

# =======================================================================
# Slide 5 -- Phase 1: Parse
# =======================================================================
s = add_slide()
kicker_title(s, "Phase 1", "Parsing the BIM into a canonical ground-truth table", num="04")
bullets(s, Inches(0.6), Inches(1.9), Inches(6.0), Inches(4.6), [
    "Per in-scope element, extracted directly from IFC geometry (world coordinates):",
    (1, "global_id, ifc_type, storey (via spatial containment, not the mm-scaled Elevation attribute)"),
    (1, "centroid, axis-aligned bounding box"),
    (1, "Z-aligned oriented bounding box — PCA in the XY plane, since an AABB alone is a poor descriptor for a wall at 45°"),
    (1, "mesh volume, elevation_bottom (drives build-order sequencing later)"),
    "IfcWallStandardCase is already a subtype returned by IfcWall's by_type() — verified 100% overlap, counted once.",
    "Output: data/bim/bim_complete.csv — 1,355 rows, zero NaNs.",
], size=15, space_after=13)

picture_fit(s, "results/presentation/screenshots/design_bim.png", Inches(7.0), Inches(1.9),
            Inches(5.7), Inches(4.6))
footer(s, 5)

# =======================================================================
# Slide 6 -- Phase 2: Ablation
# =======================================================================
s = add_slide()
kicker_title(s, "Phase 2", "Generating realistic incomplete build states", num="05")

textbox(s, Inches(0.6), Inches(1.85), Inches(6.0), Inches(0.35), "MILESTONE MODEL (per storey, bottom-up)",
        size=13, color=ACCENT, bold=True, font=FONT_MONO)
mstages = [("1. structure", "Columns + beams + slab poured", COL_C),
           ("2. envelope", "Exterior walls erected", WALL_C),
           ("3. interior", "Interior partition walls built", SLAB_C)]
y = Inches(2.3)
for name, desc, color in mstages:
    panel(s, Inches(0.6), y, Inches(5.9), Inches(0.85), fill=PANEL)
    dot = s.shapes.add_shape(MSO_SHAPE.OVAL, Inches(0.8), y + Inches(0.28), Inches(0.28), Inches(0.28))
    dot.fill.solid(); dot.fill.fore_color.rgb = color; dot.line.fill.background(); dot.shadow.inherit = False
    textbox(s, Inches(1.25), y + Inches(0.1), Inches(2.0), Inches(0.35), name, size=15, color=TEXT, bold=True)
    textbox(s, Inches(1.25), y + Inches(0.42), Inches(4.9), Inches(0.35), desc, size=12, color=TEXT_DIM)
    y = Emu(int(y + Inches(0.85) + Inches(0.15)))

bullets(s, Inches(6.9), Inches(1.9), Inches(5.8), Inches(4.6), [
    "A storey without a milestone (e.g. foundation has no interior walls) simply skips it — stage count falls out of the data, isn't a fixed target.",
    "Within a milestone: elements ordered by (x, y) for a contiguous work front, then 8% of adjacent pairs swapped (seeded) so the boundary isn't razor-sharp.",
    "Partial completion: 50% chance a newly-appearing wall/column is height-clipped to 25–85% of final height — \"poured but not finished\", not a clean binary.",
    "Cross-checked against the real Planning/XML schedule: confirms bottom-up per-storey sequencing.",
    "Result: 16 real construction stages, foundation → roof envelope, each with a manifest.json ground truth.",
], size=14.5, space_after=13)
footer(s, 6)

# =======================================================================
# Slide 7 -- Phase 3: Simulation
# =======================================================================
s = add_slide()
kicker_title(s, "Phase 3", "Simulating drone point clouds per stage", num="06")

bullets(s, Inches(0.6), Inches(1.9), Inches(6.0), Inches(4.7), [
    "Open3D raycasting instead of Blender for v1 — a deliberate scoping decision, documented in code.",
    "Camera scheme: exterior drone orbit + per-floor interior walkthrough, Fibonacci-sphere view sampling (avoids grid-aliasing rings a uniform grid would leave).",
    "Only first-hit rays kept — not uniform surface sampling — so the cloud looks like something an actual sensor could produce.",
    "Realism degradations applied per stage:",
    (1, "sensor noise (Gaussian, fixed σ)"),
    (1, "15% random point dropout"),
    (1, "per-element pose jitter (as-built deviates slightly from as-designed)"),
    (1, "whole-scene rigid registration-offset jitter (mimics imperfect pre-alignment)"),
    "Fixed ground plane + trees shared across all stages, as realistic clutter.",
], size=14, space_after=10)

panel(s, Inches(7.0), Inches(1.9), Inches(5.7), Inches(4.7))
textbox(s, Inches(7.3), Inches(2.1), Inches(5.1), Inches(0.35), "16 STAGES SIMULATED", size=13,
        color=ACCENT, bold=True, font=FONT_MONO)
stagelist = ["00 Foundation — structure", "01 Foundation — envelope", "02–04 Ground floor",
             "05–07 1st floor", "08–10 2nd floor", "11–13 3rd floor", "14–15 Roof — structure/envelope"]
bullets(s, Inches(7.3), Inches(2.55), Inches(5.1), Inches(3.8), stagelist, size=14, space_after=11,
        marker_color=ACCENT2)
footer(s, 7)

# =======================================================================
# Slide 8 -- Phase 4a: classical segmentation (exploratory)
# =======================================================================
s = add_slide()
kicker_title(s, "Phase 4 — first attempt", "Classical geometric segmentation (RANSAC + DBSCAN)", num="07")

bullets(s, Inches(0.6), Inches(1.9), Inches(6.0), Inches(3.0), [
    "Purely geometric, blind discovery — no BIM knowledge used at this stage:",
    (1, "iterative RANSAC plane extraction, DBSCAN within each plane to split spatially separate coplanar instances"),
    (1, "normal angle classifies wall vs. slab; residual non-planar points get their own DBSCAN pass, classified by PCA aspect ratio into column / beam / clutter"),
    "This is the classical replacement for the spec's image-based detection + 3D localisation phases (P4+P5 combined).",
], size=14.5, space_after=12)

panel(s, Inches(0.6), Inches(4.6), Inches(6.0), Inches(2.1))
textbox(s, Inches(0.9), Inches(4.8), Inches(5.4), Inches(0.35), "EVAL FINDING", size=13, color=ACCENT,
        bold=True, font=FONT_MONO)
bullets(s, Inches(0.9), Inches(5.2), Inches(5.4), Inches(1.4), [
    "High clutter leakage and element split/merge rates, especially at early and late stages — repetitive, coplanar structures (rows of identical walls) confuse blind clustering.",
    "Motivated the revised, BIM-aware matching approach (next).",
], size=13.5, space_after=8)

picture_fit(s, "results/figures/p4_segmentation_quality_vs_stage.png", Inches(6.9), Inches(1.9),
            Inches(5.8), Inches(4.8))
footer(s, 8)

# =======================================================================
# Slide 9 -- Phase 4b: BIM-matching (registration-based)
# =======================================================================
s = add_slide()
kicker_title(s, "Phase 4 — revised approach", "Direct BIM matching via registration", num="08")

textbox(s, Inches(0.6), Inches(1.85), Inches(11.9), Inches(0.55),
        "Key insight: the complete design BIM is always known before construction starts — the task is\n"
        "“which points belong to which known element”, not blind object discovery.",
        size=15, color=TEXT, italic=True, line_spacing=1.15)

steps = [
    ("1", "Dense BIM sampling", "Every in-scope element's surface sampled once, area-proportional, tagged with its GlobalId — one KD-tree over the whole design, reused unchanged across all 16 stages."),
    ("2", "ICP registration", "Per stage, ICP against the BIM sample cloud undoes the synthetic whole-scene misalignment — coarse-filtered first to keep only points already near some BIM surface, so ground/tree clutter can't drag the alignment off."),
    ("3", "Margin-gated assignment", "Nearest-neighbour query per point with a TIGHT distance threshold (matched to sensor noise, not registration jitter — that's already corrected). Points beyond threshold are unassigned: this alone rejects ground/tree/noise, no separate heuristics needed."),
    ("4", "Per-element aggregation", "Assigned points grouped per GlobalId → point count, centroid, AABB, and height-coverage fraction vs. the element's true height — the completion signal used for scoring."),
]
y = Inches(2.55)
for num, title, desc in steps:
    panel(s, Inches(0.6), y, Inches(12.13), Inches(1.02), fill=PANEL)
    circ = s.shapes.add_shape(MSO_SHAPE.OVAL, Inches(0.82), y + Inches(0.24), Inches(0.5), Inches(0.5))
    circ.fill.solid(); circ.fill.fore_color.rgb = ACCENT; circ.line.fill.background(); circ.shadow.inherit = False
    tf = circ.text_frame; tf.word_wrap = False
    p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
    r = p.add_run(); r.text = num; r.font.bold = True; r.font.size = Pt(16); r.font.color.rgb = BG
    textbox(s, Inches(1.55), y + Inches(0.08), Inches(2.6), Inches(0.4), title, size=15, color=TEXT, bold=True)
    textbox(s, Inches(4.25), y + Inches(0.08), Inches(8.3), Inches(0.86), desc, size=12, color=TEXT_DIM,
            line_spacing=1.1)
    y = Emu(int(y + Inches(1.02) + Inches(0.1)))
footer(s, 9)

# =======================================================================
# Slide 10 -- Phase 7: scoring
# =======================================================================
s = add_slide()
kicker_title(s, "Phase 7", "Per-element status and progress scoring", num="09")

textbox(s, Inches(0.6), Inches(1.85), Inches(6.0), Inches(0.35), "STATUS RULES (height-coverage based)",
        size=13, color=ACCENT, bold=True, font=FONT_MONO)
rows = [
    ("completed", "height_coverage ≥ 0.5", "Cᵢ = 1.0", ACCENT),
    ("partial", "0.15 ≤ height_coverage < 0.5", "Cᵢ = height_coverage", SLAB_C),
    ("not_started", "not detected, but observable", "Cᵢ = 0.0", TEXT_DIM),
    ("uncertain", "never observable, any stage", "excluded", RGBColor(0x55,0x58,0x5e)),
]
y = Inches(2.3)
for status, cond, ci, color in rows:
    panel(s, Inches(0.6), y, Inches(5.9), Inches(0.62), fill=PANEL)
    textbox(s, Inches(0.8), y + Inches(0.1), Inches(1.6), Inches(0.4), status, size=13, color=color, bold=True,
            font=FONT_MONO)
    textbox(s, Inches(2.4), y + Inches(0.1), Inches(2.6), Inches(0.4), cond, size=11.5, color=TEXT_DIM)
    textbox(s, Inches(5.0), y + Inches(0.1), Inches(1.4), Inches(0.4), ci, size=11.5, color=TEXT,
            font=FONT_MONO, align=PP_ALIGN.RIGHT)
    y = Emu(int(y + Inches(0.62) + Inches(0.1)))

bullets(s, Inches(0.6), Inches(5.15), Inches(6.0), Inches(1.6), [
    "“Uncertain” exclusion is the critical rule: an element the drone never observed must not be scored as “not built”.",
    "Measured: 136/1355 elements (10%) are structurally hard-to-observe at every stage — excluded from scoring, not scored as absent.",
    "Progress = Σ(wᵢ·Cᵢ) / Σ(wᵢ) × 100, reported with count weights and volume weights.",
], size=13.5, space_after=8)

picture_fit(s, "results/figures/p7_progress_error.png", Inches(6.9), Inches(1.9), Inches(5.8), Inches(4.85))
footer(s, 10)

# =======================================================================
# Slide 11 -- Results
# =======================================================================
s = add_slide()
kicker_title(s, "Results", "Estimated vs. ground-truth progress, across 16 stages", num="10")

stat_card(s, Inches(0.6), Inches(1.85), Inches(2.7), Inches(1.25), "16", "Stages evaluated")
stat_card(s, Inches(3.45), Inches(1.85), Inches(2.7), Inches(1.25), "4.7 pp", "Mean abs. progress error",
          value_color=SLAB_C)
stat_card(s, Inches(6.3), Inches(1.85), Inches(2.7), Inches(1.25), "7.5 pp", "Max error (mid-build)",
          value_color=COL_C)
stat_card(s, Inches(9.15), Inches(1.85), Inches(3.58), Inches(1.25), "1.98 pp", "Error at final stage (roof)",
          value_color=ACCENT)

df = pd.read_csv("results/main_table.csv")
sample_idx = [0, 3, 7, 11, 15]
headers = ["Stage", "Est. %", "GT %", "Error (pp)"]
tbl_left, tbl_top = Inches(0.6), Inches(3.5)
tbl_w, tbl_h = Inches(12.13), Inches(3.2)
gtable = s.shapes.add_table(len(sample_idx) + 1, 4, tbl_left, tbl_top, tbl_w, tbl_h).table
gtable.columns[0].width = Inches(6.4)
gtable.columns[1].width = Inches(1.9)
gtable.columns[2].width = Inches(1.9)
gtable.columns[3].width = Inches(1.93)
for c, h in enumerate(headers):
    cell = gtable.cell(0, c)
    cell.text = h
    cell.fill.solid(); cell.fill.fore_color.rgb = RGBColor(0x2a, 0x2d, 0x33)
    for p in cell.text_frame.paragraphs:
        p.alignment = PP_ALIGN.LEFT if c == 0 else PP_ALIGN.CENTER
        for r in p.runs:
            r.font.bold = True; r.font.size = Pt(13); r.font.color.rgb = ACCENT; r.font.name = FONT_MONO
for ri, idx in enumerate(sample_idx, start=1):
    row = df.iloc[idx]
    vals = [row["stage"], f"{row['progress_est_count']:.1f}", f"{row['progress_gt_count']:.1f}",
            f"{row['error_count']:.2f}"]
    for c, v in enumerate(vals):
        cell = gtable.cell(ri, c)
        cell.text = str(v)
        cell.fill.solid(); cell.fill.fore_color.rgb = PANEL
        for p in cell.text_frame.paragraphs:
            p.alignment = PP_ALIGN.LEFT if c == 0 else PP_ALIGN.CENTER
            for r in p.runs:
                r.font.size = Pt(12.5); r.font.color.rgb = TEXT
                r.font.name = FONT_MONO if c > 0 else FONT_BODY
footer(s, 11)

# =======================================================================
# Slide 12 -- Visualizations: overlay viewer
# =======================================================================
s = add_slide()
kicker_title(s, "Visualization", "Digital-twin progress overlay viewer", num="11")
bullets(s, Inches(0.6), Inches(1.85), Inches(11.9), Inches(0.9), [
    "Custom interactive 3D viewer (Three.js): the complete BIM shown as a ghost wireframe, with each stage's confirmed-built points solidified into a voxel cloud, in the same coordinate frame — so intended vs. actual align pixel-for-pixel.",
], size=14.5, space_after=6)
picture_or_placeholder(s, "overlay_uniform.png", Inches(0.6), Inches(2.85), Inches(5.85), Inches(4.1),
                        "Overlay viewer — uniform “confirmed built” colour, wireframe ghost visible")
picture_or_placeholder(s, "overlay_instance.png", Inches(6.65), Inches(2.85), Inches(5.85), Inches(4.1),
                        "Overlay viewer — “by element” toggle: each BIM instance its own colour")
footer(s, 12)

# =======================================================================
# Slide 13 -- Visualizations: segmentation viewer
# =======================================================================
s = add_slide()
kicker_title(s, "Visualization", "Segmentation-after-registration viewer", num="12")
bullets(s, Inches(0.6), Inches(1.85), Inches(11.9), Inches(0.9), [
    "A second standalone viewer: every raw scan point coloured by which BIM element class it matched to (wall / slab / beam / column), post-ICP — a direct visual read of segmentation quality, not a separate demo run.",
    (1, "also supports a per-element “by element” colour toggle, same mechanism shown on the previous slide."),
], size=14.5, space_after=6)
picture_or_placeholder(s, "segview_class.png", Inches(2.4), Inches(2.7), Inches(8.5), Inches(4.35),
                        "Segmentation viewer — class-coloured (wall/slab/beam/column + unmatched)")
footer(s, 13)

# =======================================================================
# Slide 14 -- Limitations
# =======================================================================
s = add_slide()
kicker_title(s, "Honest limitations", "What this prototype does not yet show", num="13")
bullets(s, Inches(0.6), Inches(1.9), Inches(11.9), Inches(4.7), [
    "Sim-to-real appearance gap is entirely bypassed — v1 replaced image-based detection with idealized point-to-BIM geometric matching. No camera-image realism, no clutter/scaffolding/workers modelled visually.",
    "Ablation realism is a crude proxy: binary presence + height-clipping cannot represent every real half-built state (partial pours, rebar-only, etc.).",
    "Single building typology — Schependomlaan is low-rise Dutch residential. Generalisation to other typologies is untested.",
    "Class imbalance: only 23 columns and 174 beams vs. 879 walls — per-class metrics for the rare classes are noisy by nature of the dataset, not the method.",
    "The classical segmentation baseline (Phase 4a) shows blind point-cloud segmentation is genuinely hard on repetitive coplanar structures — this motivates the registration-based approach, but means fully unsupervised detection hasn't been validated at scale.",
    "Comparison against the dataset's real weekly as-planned IFCs was ruled out early (0% GlobalId overlap) — all results here are on synthetic ground truth, not yet cross-checked against real drone data.",
], size=15, space_after=13)
footer(s, 14)

# =======================================================================
# Slide 15 -- Next steps
# =======================================================================
s = add_slide()
kicker_title(s, "Next steps", "Where this goes next", num="14")
bullets(s, Inches(0.6), Inches(1.9), Inches(11.9), Inches(4.7), [
    "Phase 9 reality check: register Schependomlaan's real week-28 drone point cloud against the as-planned IFC, run the same scoring pipeline, compare against the dataset's own (non-ground-truth) comparison spreadsheet.",
    "Second building typology, held out entirely, to support a genuine generalisation claim.",
    "Compare camera trajectories (grid / perimeter / waypoint / adaptive) — currently a single fixed exterior-orbit + interior-walkthrough scheme.",
    "Refine the “ever observable” heuristic that drives the uncertain-status exclusion, beyond the current 100%-stage proxy.",
    "Revisit the image-based detection track (YOLOv8-seg on rendered RGB) as an explicit oracle-vs-learned comparison, to quantify the sim-to-real gap this prototype currently sidesteps.",
], size=15.5, space_after=15)
footer(s, 15)

# =======================================================================
# Slide 16 -- Thank you
# =======================================================================
s = add_slide()
textbox(s, Inches(0.9), Inches(3.1), Inches(11.5), Inches(1.0), "Thank you", size=44, color=TEXT, bold=True)
textbox(s, Inches(0.9), Inches(4.0), Inches(11.5), Inches(0.5), "Questions & discussion", size=18,
        color=TEXT_DIM)
accent_bar2 = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(2.8), Inches(0.12), Inches(1.2))
accent_bar2.fill.solid(); accent_bar2.fill.fore_color.rgb = ACCENT; accent_bar2.line.fill.background()
accent_bar2.shadow.inherit = False

os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(SCREENSHOT_DIR, exist_ok=True)
prs.save(OUT_PPTX)

needed = ["overlay_uniform.png", "overlay_instance.png", "segview_class.png"]
missing = [n for n in needed if not os.path.exists(os.path.join(SCREENSHOT_DIR, n))]
print(f"Saved {OUT_PPTX} ({os.path.getsize(OUT_PPTX)/1e6:.2f} MB), {len(prs.slides._sldIdLst)} slides")
if missing:
    print("\nMissing screenshots (placeholders shown in deck) -- drop these files into")
    print(f"  {SCREENSHOT_DIR}/")
    print("and re-run this script to fill them in:")
    for m in missing:
        print(f"  - {m}")
