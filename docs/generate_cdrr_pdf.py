#!/usr/bin/env python3
"""
Generate docs/cdrr_perception_navigation.pdf from the corresponding Markdown file.
Run from the docs/ directory:  python generate_cdrr_pdf.py
"""
import os
import re
import sys
from pathlib import Path
from typing import Optional

from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, KeepTogether, Preformatted, PageBreak,
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase.pdfmetrics import registerFontFamily

# ── Page geometry ─────────────────────────────────────────────────────────────
PW, PH = A4          # 595 × 842 pt
ML = MR = 2.0 * cm
MT = 2.5 * cm        # content starts below header bar
MB = 1.5 * cm        # content ends above footer bar

# ── Colour palette ────────────────────────────────────────────────────────────
NAVY  = colors.HexColor('#1a2e4a')
BLUE  = colors.HexColor('#2c5f8a')
LBLUE = colors.HexColor('#dce8f5')
LGREY = colors.HexColor('#f0f0f0')
MGREY = colors.HexColor('#bbbbbb')
WHITE = colors.white
BLACK = colors.black


# ── Font registration ─────────────────────────────────────────────────────────
def _setup_fonts():
    WIN = r'C:\Windows\Fonts'

    def reg(name: str, fname: str) -> bool:
        path = os.path.join(WIN, fname)
        if os.path.exists(path):
            try:
                pdfmetrics.registerFont(TTFont(name, path))
                return True
            except Exception:
                pass
        return False

    # Body: Segoe UI has good Unicode coverage (arrows, bullets, etc.)
    b_n = reg('Body',        'segoeui.ttf')
    b_b = reg('Body-Bold',   'segoeuib.ttf')
    b_i = reg('Body-Italic', 'segoeuii.ttf')
    b_bi= reg('Body-BI',     'segoeuiz.ttf')

    if b_n and b_b:
        kw = dict(normal='Body', bold='Body-Bold')
        if b_i:  kw['italic'] = 'Body-Italic'
        if b_bi: kw['boldItalic'] = 'Body-BI'
        registerFontFamily('Body', **kw)
        BF = 'Body'
    else:
        BF = 'Helvetica'          # built-in Type 1 fallback

    # Mono: Consolas includes box-drawing characters
    m_n = reg('Mono',      'consola.ttf')
    m_b = reg('Mono-Bold', 'consolab.ttf')
    if m_n:
        if m_b: registerFontFamily('Mono', normal='Mono', bold='Mono-Bold')
        MF = 'Mono'
    else:
        MF = 'Courier'

    return BF, MF


BF, MF = _setup_fonts()
BF_BOLD = f'{BF}-Bold' if BF != 'Helvetica' else 'Helvetica-Bold'


# ── Styles ────────────────────────────────────────────────────────────────────
def _ps(name: str, **kw) -> ParagraphStyle:
    base = dict(fontName=BF, fontSize=9, leading=13,
                textColor=BLACK, spaceBefore=0, spaceAfter=0)
    base.update(kw)
    return ParagraphStyle(name, **base)


S = {
    'h1':      _ps('h1', fontName=BF_BOLD, fontSize=18, textColor=NAVY,
                   leading=22, spaceBefore=14, spaceAfter=8),
    'h2_cell': _ps('h2c', fontName=BF_BOLD, fontSize=12,
                   textColor=WHITE, leading=16),
    'h3':      _ps('h3', fontName=BF_BOLD, fontSize=10.5, textColor=NAVY,
                   leading=14, spaceBefore=10, spaceAfter=3),
    'h4':      _ps('h4', fontName=BF_BOLD, fontSize=9.5, textColor=BLUE,
                   leading=13, spaceBefore=8, spaceAfter=2),
    'body':    _ps('body', leading=13, spaceAfter=5),
    'bullet':  _ps('bullet', leftIndent=14, leading=13, spaceAfter=2),
    'pre':     ParagraphStyle('pre', fontName=MF, fontSize=6.8, leading=9,
                              spaceAfter=0, spaceBefore=0),
    'th':      _ps('th', fontName=BF_BOLD, fontSize=8,
                   textColor=WHITE, leading=11),
    'td':      _ps('td', fontName=BF, fontSize=8,
                   textColor=BLACK, leading=11),
}


# ── Inline markup ─────────────────────────────────────────────────────────────
# Fallback substitutions for Helvetica (no Unicode arrows)
_UMAP = {
    '\u2192': '->',  '\u2190': '<-',  '\u2193': 'v',   '\u2191': '^',
    '\u21d2': '=>',  '\u25ba': '>',   '\u25c4': '<',
    '\u25bc': 'v',   '\u25b2': '^',   '\u2022': '*',
    '\u00a0': ' ',
}


def inline(raw: str) -> str:
    """Convert markdown inline markup to ReportLab XML markup."""
    # If using a built-in font, replace Unicode chars it can't render
    if BF == 'Helvetica':
        for ch, rep in _UMAP.items():
            raw = raw.replace(ch, rep)
    # XML-escape so the parser doesn't choke on bare < > &
    s = raw.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
    # **bold**
    s = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', s)
    # *italic* (not part of **)
    s = re.sub(r'(?<!\*)\*([^*\n]+?)\*(?!\*)', r'<i>\1</i>', s)
    # `inline code`
    s = re.sub(r'`([^`\n]+?)`',
               lambda m: f'<font name="{MF}" size="7.5">{m.group(1)}</font>',
               s)
    return s


# ── Code / preformatted block ─────────────────────────────────────────────────
# Box-drawing substitution map (used when Consolas unavailable)
_BOX = str.maketrans({
    '\u250c': '+', '\u2510': '+', '\u2514': '+', '\u2518': '+',
    '\u251c': '+', '\u2524': '+', '\u252c': '+', '\u2534': '+',
    '\u253c': '+',
    '\u2500': '-', '\u2502': '|',
    '\u2550': '=', '\u2551': '|',
    '\u2554': '+', '\u2557': '+', '\u255a': '+', '\u255d': '+',
    '\u25bc': 'v', '\u25b2': '^', '\u25ba': '>', '\u25c4': '<',
    '\u2192': '->', '\u2190': '<-', '\u2193': 'v', '\u2191': '^',
    '\u2022': '*',
})


def _code_block(code_lines: list) -> Table:
    """Render a fenced code block inside a light-grey box."""
    if MF == 'Courier':
        code_lines = [l.translate(_BOX) for l in code_lines]
    # XML-escape so Preformatted's XML parser handles them literally
    escaped = [
        l.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
        for l in code_lines
    ]
    pre = Preformatted('\n'.join(escaped), S['pre'])
    avail = PW - ML - MR
    tbl = Table([[pre]], colWidths=[avail])
    tbl.setStyle(TableStyle([
        ('BACKGROUND',    (0, 0), (-1, -1), LGREY),
        ('TOPPADDING',    (0, 0), (-1, -1), 7),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 7),
        ('LEFTPADDING',   (0, 0), (-1, -1), 8),
        ('RIGHTPADDING',  (0, 0), (-1, -1), 8),
        ('BOX',           (0, 0), (-1, -1), 0.5, MGREY),
    ]))
    return tbl


# ── Markdown table → ReportLab Table ─────────────────────────────────────────
def _md_table(lines: list) -> Optional[Table]:
    rows = []
    for line in lines:
        # skip separator rows like |---|---|
        if re.match(r'^\s*\|[\s\-:|]+\|\s*$', line):
            continue
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        rows.append(cells)
    if not rows:
        return None

    n_cols = max(len(r) for r in rows)
    avail  = PW - ML - MR
    col_w  = avail / n_cols

    data = []
    for ri, row in enumerate(rows):
        while len(row) < n_cols:
            row.append('')
        sty = S['th'] if ri == 0 else S['td']
        data.append([Paragraph(inline(c), sty) for c in row])

    tbl = Table(data, colWidths=[col_w] * n_cols,
                repeatRows=1, splitByRow=True)

    alternating = [
        ('BACKGROUND', (0, i), (-1, i), LBLUE if i % 2 == 0 else WHITE)
        for i in range(1, len(data))
    ]
    tbl.setStyle(TableStyle([
        ('BACKGROUND',    (0, 0), (-1,  0), NAVY),
        ('FONTSIZE',      (0, 0), (-1, -1), 8),
        ('GRID',          (0, 0), (-1, -1), 0.4, MGREY),
        ('LINEBELOW',     (0, 0), (-1,  0), 1.5, NAVY),
        ('TOPPADDING',    (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING',   (0, 0), (-1, -1), 6),
        ('RIGHTPADDING',  (0, 0), (-1, -1), 6),
        ('VALIGN',        (0, 0), (-1, -1), 'TOP'),
    ] + alternating))
    return tbl


# ── H2 full-width navy banner ─────────────────────────────────────────────────
def _h2_banner(text: str) -> Table:
    tbl = Table([[Paragraph(inline(text), S['h2_cell'])]],
                colWidths=[PW - ML - MR])
    tbl.setStyle(TableStyle([
        ('BACKGROUND',    (0, 0), (-1, -1), NAVY),
        ('TOPPADDING',    (0, 0), (-1, -1), 7),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 7),
        ('LEFTPADDING',   (0, 0), (-1, -1), 8),
        ('RIGHTPADDING',  (0, 0), (-1, -1), 8),
    ]))
    return tbl


# ── Markdown → ReportLab story ────────────────────────────────────────────────
def _is_nontrivial(story: list) -> bool:
    return any(not isinstance(x, Spacer) for x in story)


def parse_md(md_text: str) -> list:
    lines = md_text.splitlines()
    story = []
    i, n  = 0, len(lines)

    while i < n:
        raw     = lines[i]
        stripped = raw.strip()

        # ── blank line ────────────────────────────────────────────────────────
        if not stripped:
            story.append(Spacer(1, 4))
            i += 1
            continue

        # ── horizontal rule ───────────────────────────────────────────────────
        if re.match(r'^-{3,}$', stripped):
            story.append(HRFlowable(width='100%', thickness=1, color=BLUE,
                                    spaceBefore=8, spaceAfter=8))
            i += 1
            continue

        # ── fenced code block ─────────────────────────────────────────────────
        if stripped.startswith('```'):
            i += 1
            code = []
            while i < n and not lines[i].strip().startswith('```'):
                code.append(lines[i])
                i += 1
            i += 1  # closing fence
            if code:
                story.append(Spacer(1, 3))
                story.append(KeepTogether([_code_block(code), Spacer(1, 5)]))
            continue

        # ── headings ──────────────────────────────────────────────────────────
        m = re.match(r'^(#{1,4})\s+(.+)$', stripped)
        if m:
            lvl, txt = len(m.group(1)), m.group(2)
            if lvl == 1:
                story.append(Paragraph(inline(txt), S['h1']))
            elif lvl == 2:
                if _is_nontrivial(story):
                    story.append(PageBreak())
                story.append(_h2_banner(txt))
                story.append(Spacer(1, 6))
            elif lvl == 3:
                story.append(Paragraph(inline(txt), S['h3']))
            else:
                story.append(Paragraph(inline(txt), S['h4']))
            i += 1
            continue

        # ── markdown table ────────────────────────────────────────────────────
        if stripped.startswith('|'):
            tbl_lines = []
            while i < n and lines[i].strip().startswith('|'):
                tbl_lines.append(lines[i])
                i += 1
            tbl = _md_table(tbl_lines)
            if tbl:
                story.append(KeepTogether([tbl, Spacer(1, 6)]))
            continue

        # ── unordered list ────────────────────────────────────────────────────
        if re.match(r'^[-*+]\s', stripped):
            items = []
            while i < n and re.match(r'^[-*+]\s', lines[i].strip()):
                text = re.sub(r'^[-*+]\s+', '', lines[i].strip())
                items.append(
                    Paragraph('\u2022\u00a0' + inline(text), S['bullet']))
                i += 1
            story.append(KeepTogether(items + [Spacer(1, 4)]))
            continue

        # ── ordered list ──────────────────────────────────────────────────────
        if re.match(r'^\d+\.\s', stripped):
            items = []
            while i < n and re.match(r'^\d+\.\s', lines[i].strip()):
                m2 = re.match(r'^(\d+)\.\s+(.+)$', lines[i].strip())
                if m2:
                    items.append(
                        Paragraph(
                            f'{m2.group(1)}.\u00a0{inline(m2.group(2))}',
                            S['bullet']))
                i += 1
            story.append(KeepTogether(items + [Spacer(1, 4)]))
            continue

        # ── regular paragraph ─────────────────────────────────────────────────
        para_lines = []
        while i < n:
            l = lines[i].strip()
            if (not l
                    or l.startswith('#')
                    or l.startswith('|')
                    or l.startswith('```')
                    or re.match(r'^[-*+]\s', l)
                    or re.match(r'^\d+\.\s', l)
                    or re.match(r'^-{3,}$', l)):
                break
            para_lines.append(l)
            i += 1
        if para_lines:
            story.append(Paragraph(inline(' '.join(para_lines)), S['body']))
            story.append(Spacer(1, 4))

    return story


# ── Page header / footer ──────────────────────────────────────────────────────
def _decorate(canvas, doc):
    canvas.saveState()

    # header bar
    canvas.setFillColor(NAVY)
    canvas.rect(0, PH - 1.4 * cm, PW, 1.4 * cm, fill=1, stroke=0)
    canvas.setFillColor(WHITE)
    canvas.setFont('Helvetica-Bold', 8)
    canvas.drawString(ML, PH - 0.88 * cm,
                      'AutoNexa \u2014 Autonomous Parking System')
    canvas.setFont('Helvetica', 7.5)
    canvas.drawRightString(PW - MR, PH - 0.88 * cm,
                           'Critical Design Review Report  |  Sections 3 & 4')

    # thin accent line under header
    canvas.setStrokeColor(BLUE)
    canvas.setLineWidth(0.5)
    canvas.line(ML, PH - 1.42 * cm, PW - MR, PH - 1.42 * cm)

    # footer bar
    canvas.setFillColor(NAVY)
    canvas.rect(0, 0, PW, 1.1 * cm, fill=1, stroke=0)
    canvas.setFillColor(WHITE)
    canvas.setFont('Helvetica', 7)
    canvas.drawString(ML, 0.38 * cm, 'AutoNexa Project')
    canvas.drawCentredString(PW / 2, 0.38 * cm,
                             'Perception & Navigation Subsystems CDRR')
    canvas.drawRightString(PW - MR, 0.38 * cm, f'Page {doc.page}')

    canvas.restoreState()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    docs_dir = Path(__file__).resolve().parent
    md_path  = docs_dir / 'cdrr_perception_navigation.md'
    pdf_path = docs_dir / 'cdrr_perception_navigation.pdf'

    if not md_path.exists():
        sys.exit(f'ERROR: {md_path} not found')

    print(f'Reading  {md_path}')
    with open(md_path, encoding='utf-8') as f:
        md_text = f.read()

    print(f'Body font : {BF}')
    print(f'Mono font : {MF}')
    print('Parsing markdown ...')
    story = parse_md(md_text)

    print('Building PDF ...')
    doc = SimpleDocTemplate(
        str(pdf_path),
        pagesize=A4,
        leftMargin=ML, rightMargin=MR,
        topMargin=MT, bottomMargin=MB,
        title='AutoNexa CDRR \u2014 Perception & Navigation Subsystems',
        author='AutoNexa Team',
        subject='Critical Design Review Report',
        creator='AutoNexa PDF Generator',
    )
    doc.build(story,
              onFirstPage=_decorate,
              onLaterPages=_decorate)

    print(f'Written  \u2192 {pdf_path}')


if __name__ == '__main__':
    main()
