"""
PCB Toner Transfer Sheet Generator
====================================
Requirements:  pip install pymupdf pillow reportlab

Usage:         python toner_transfer.py
Output:        toner_transfer_output.pdf  (in the same folder as this script)
"""

import fitz  # PyMuPDF
from PIL import Image, ImageOps
from reportlab.lib.pagesizes import A4, landscape
from reportlab.pdfgen import canvas as rl_canvas

# ─────────────────────────────────────────────
# CHANGE THESE
# ─────────────────────────────────────────────

INPUT_PDF = "Wristband_PCB-F_Cu.pdf"  # path to your KiCad exported PDF

MIRROR = True  # True  → mirrors horizontally (use for F_Cu / front copper)
# False → no mirror            (use for B_Cu / back copper)

GAP_MM = 2  # gap between copies in mm

# --- Copper sheet / print region ---
# Option A: specific copper sheet size (comment out Option B below)
COPPER_W_MM = 200  # width  of your copper sheet in mm  (20 cm)
COPPER_H_MM = 150  # height of your copper sheet in mm  (15 cm)
USE_FULL_A4 = True  # keep False to use the copper dimensions above

# Option B: fill the entire A4 page
# set USE_FULL_A4 = True and the COPPER_W/H values above are ignored
# USE_FULL_A4 = True

MARGIN_MM = 1  # margin inside the copper/A4 region in mm
# 0 = start right at the corner
# increase if your printer can't print to the edge

DPI = 600  # render resolution — 600 is fine for most traces
# use 1200 for extremely fine traces (slower, larger file)

OUTPUT_PDF = "toner_transfer_output.pdf"

# ─────────────────────────────────────────────
# SCRIPT (no need to edit below this line)
# ─────────────────────────────────────────────


def mm_to_pt(mm):
    return mm * 72 / 25.4


# --- Open source PDF and find PCB bounding box ---
doc = fitz.open(INPUT_PDF)
src_page = doc[0]

paths = src_page.get_drawings()
if not paths:
    raise RuntimeError(
        "No vector drawings found in PDF. Make sure it's a KiCad copper layer export."
    )

x0 = min(p["rect"].x0 for p in paths)
y0 = min(p["rect"].y0 for p in paths)
x1 = max(p["rect"].x1 for p in paths)
y1 = max(p["rect"].y1 for p in paths)
pad = 1  # 1 pt padding so edges aren't clipped

pcb_w_pt = x1 - x0 + 2 * pad
pcb_h_pt = y1 - y0 + 2 * pad

print(f"PCB size: {pcb_w_pt * 25.4 / 72:.2f} x {pcb_h_pt * 25.4 / 72:.2f} mm")

# --- Render to high-res image and crop to PCB ---
zoom = DPI / 72
mat = fitz.Matrix(zoom, zoom)
pix = src_page.get_pixmap(matrix=mat)
pix.save("/tmp/_pcb_full_page.png")

img = Image.open("/tmp/_pcb_full_page.png")
crop_box = (
    int((x0 - pad) * zoom),
    int((y0 - pad) * zoom),
    int((x1 + pad) * zoom),
    int((y1 + pad) * zoom),
)
img_crop = img.crop(crop_box)

if MIRROR:
    img_crop = ImageOps.mirror(img_crop)

img_crop.save("/tmp/_pcb_processed.png")

# --- Determine print region ---
page_w, page_h = landscape(A4)  # 841.89 x 595.28 pts

if USE_FULL_A4:
    region_w_pt = page_w - 2 * mm_to_pt(MARGIN_MM)
    region_h_pt = page_h - 2 * mm_to_pt(MARGIN_MM)
    origin_x = mm_to_pt(MARGIN_MM)
    origin_y = mm_to_pt(MARGIN_MM)
else:
    region_w_pt = mm_to_pt(COPPER_W_MM) - 2 * mm_to_pt(MARGIN_MM)
    region_h_pt = mm_to_pt(COPPER_H_MM) - 2 * mm_to_pt(MARGIN_MM)
    origin_x = mm_to_pt(MARGIN_MM)  # bottom-left corner of page
    origin_y = mm_to_pt(MARGIN_MM)

gap_pt = mm_to_pt(GAP_MM)

cols = int((region_w_pt + gap_pt) / (pcb_w_pt + gap_pt))
rows = int((region_h_pt + gap_pt) / (pcb_h_pt + gap_pt))

print(f"Tiling: {cols} cols x {rows} rows = {cols * rows} copies")

# --- Build output PDF ---
c = rl_canvas.Canvas(OUTPUT_PDF, pagesize=(page_w, page_h))

# Draw a faint guide rectangle showing the copper/print region boundary
c.setStrokeColorRGB(0.7, 0.7, 0.7)
c.setLineWidth(0.5)
c.rect(origin_x, origin_y, region_w_pt, region_h_pt)

for row in range(rows):
    for col in range(cols):
        dest_x = origin_x + col * (pcb_w_pt + gap_pt)
        dest_y = origin_y + row * (pcb_h_pt + gap_pt)
        c.drawImage(
            "/tmp/_pcb_processed.png", dest_x, dest_y, width=pcb_w_pt, height=pcb_h_pt
        )

c.save()
print(f"Saved: {OUTPUT_PDF}")
print("IMPORTANT: print at 100% / Actual Size — do not scale to fit page.")
