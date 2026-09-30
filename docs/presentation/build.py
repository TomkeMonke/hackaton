"""Sklada prezentacje z src/ w jeden plik index.html (otwiera sie w przegladarce, bez serwera).

Zrodlo: src/deck.json (kolejnosc slajdow, fonty) + src/slides/<id>.html (jeden <section> na slajd,
format Slides z claude.ai). Tagi x-shape / x-icon zamieniane na zwykly HTML/SVG.

Uzycie:
    python docs/presentation/build.py
"""
from __future__ import annotations

import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "src")

# Proste ikony (styl lucide), kolor z currentColor.
ICONS = {
    "Activity": '<polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/>',
    "Search": '<circle cx="11" cy="11" r="8"/><line x1="21" y1="21" x2="16.65" y2="16.65"/>',
    "Users": ('<path d="M17 21v-2a4 4 0 0 0-4-4H5a4 4 0 0 0-4 4v2"/><circle cx="9" cy="7" r="4"/>'
              '<path d="M23 21v-2a4 4 0 0 0-3-3.87"/><path d="M16 3.13a4 4 0 0 1 0 7.75"/>'),
}


def _style(attrs: str) -> str:
    m = re.search(r'style="([^"]*)"', attrs)
    return m.group(1) if m else ""


def _prop(style: str, name: str, default: str = "") -> str:
    m = re.search(r"(?:^|;)\s*" + name + r"\s*:\s*([^;]+)", style)
    return m.group(1).strip() if m else default


def _shape(m: re.Match) -> str:
    attrs = m.group(1)
    kind = re.search(r'kind="([^"]+)"', attrs).group(1)
    style = _style(attrs)
    if kind == "ellipse":
        return f'<div style="{style};border-radius:50%"></div>'
    if kind == "arrow-right":
        color = _prop(style, "background", "#000")
        rest = ";".join(p for p in style.split(";") if not p.strip().startswith("background"))
        return (f'<svg viewBox="0 0 100 50" preserveAspectRatio="none" style="{rest};flex:none">'
                f'<polygon points="0,15 60,15 60,0 100,25 60,50 60,35 0,35" fill="{color}"/></svg>')
    raise ValueError(f"x-shape kind={kind} nieobslugiwany")


def _icon(m: re.Match) -> str:
    attrs = m.group(1)
    name = re.search(r'name="([^"]+)"', attrs).group(1)
    return (f'<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" '
            f'stroke-linecap="round" stroke-linejoin="round" style="{_style(attrs)};flex:none">'
            f'{ICONS[name]}</svg>')


def convert(html: str) -> str:
    html = re.sub(r"<x-shape([^>]*)></x-shape>", _shape, html)
    html = re.sub(r"<x-icon([^>]*)></x-icon>", _icon, html)
    return html


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
{fonts}
<style>
html, body {{ margin: 0; height: 100%; background: #0b120e; overflow: hidden; }}
#stage {{ position: absolute; left: 0; top: 0; width: 1920px; height: 1080px; transform-origin: 0 0; }}
section {{ position: absolute; left: 0; top: 0; width: 1920px; height: 1080px; box-sizing: border-box; overflow: hidden; }}
section.off {{ display: none !important; }}
h1, h2, h3, p, ul, hr {{ margin: 0; }}
h1 {{ font-size: 96px; font-weight: 600; line-height: 1.1; }}
h2 {{ font-size: 64px; font-weight: 600; line-height: 1.15; }}
h3 {{ font-size: 44px; font-weight: 600; line-height: 1.2; }}
p {{ line-height: 1.4; }}
ul {{ padding-left: 1.2em; }}
hr {{ border: none; }}
div {{ box-sizing: border-box; }}
aside {{ display: none; }}
#count {{ position: fixed; right: 12px; bottom: 8px; font: 14px sans-serif; color: #8fa398; }}
</style>
</head>
<body>
<div id="stage">
{slides}
</div>
<div id="count"></div>
<script>
var s = document.querySelectorAll('#stage > section'), i = 0;
function show(n) {{
  i = Math.max(0, Math.min(s.length - 1, n));
  for (var k = 0; k < s.length; k++) s[k].classList.toggle('off', k !== i);
  document.getElementById('count').textContent = (i + 1) + ' / ' + s.length + '  (arrows / click)';
  location.hash = i + 1;
}}
function fit() {{
  var k = Math.min(innerWidth / 1920, innerHeight / 1080), st = document.getElementById('stage');
  st.style.transform = 'translate(' + (innerWidth - 1920 * k) / 2 + 'px,' + (innerHeight - 1080 * k) / 2 + 'px) scale(' + k + ')';
}}
addEventListener('resize', fit);
addEventListener('keydown', function (e) {{
  if (['ArrowRight', 'PageDown', ' '].indexOf(e.key) >= 0) show(i + 1);
  if (['ArrowLeft', 'PageUp'].indexOf(e.key) >= 0) show(i - 1);
  if (e.key === 'Home') show(0);
  if (e.key === 'End') show(s.length - 1);
}});
addEventListener('click', function (e) {{ show(e.clientX < innerWidth / 3 ? i - 1 : i + 1); }});
fit(); show((parseInt(location.hash.slice(1), 10) || 1) - 1);
</script>
</body>
</html>
"""


def main() -> None:
    with open(os.path.join(SRC, "deck.json"), encoding="utf-8") as f:
        deck = json.load(f)
    slides = []
    for sid in deck["order"]:
        with open(os.path.join(SRC, "slides", sid + ".html"), encoding="utf-8") as f:
            slides.append(convert(f.read().strip()))
    fonts = "\n".join(f'<link rel="stylesheet" href="{face["href"]}">'
                      for face in deck["faces"].values() if "href" in face)
    out = PAGE.format(title=deck["title"], fonts=fonts, slides="\n".join(slides))
    with open(os.path.join(HERE, "index.html"), "w", encoding="utf-8", newline="\n") as f:
        f.write(out)
    print(f"index.html: {len(slides)} slajdow")


if __name__ == "__main__":
    main()
