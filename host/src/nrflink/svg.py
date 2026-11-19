"""Minimal SVG line-chart writer (no plotting library needed)."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from xml.sax.saxutils import escape

# Colour-blind-safe categorical palette (Okabe-Ito subset).
PALETTE = ("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9")
MARKERS = ("circle", "square", "triangle", "diamond", "circle", "square")


@dataclass(frozen=True)
class Series:
    label: str
    xs: Sequence[float]
    ys: Sequence[float]


def _ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    if hi <= lo:
        return [lo]
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s * mag for s in (1, 2, 2.5, 5, 10) if s * mag >= raw), default=raw)
    start = math.ceil(lo / step - 1e-9) * step
    out = []
    v = start
    while v <= hi + 1e-9:
        out.append(round(v, 10))
        v += step
    return out


def _fmt(v: float) -> str:
    return f"{v:g}"


def _marker(kind: str, x: float, y: float, color: str) -> str:
    if kind == "square":
        return f'<rect x="{x - 3.5:.1f}" y="{y - 3.5:.1f}" width="7" height="7" fill="{color}"/>'
    if kind == "triangle":
        pts = f"{x:.1f},{y - 4.5:.1f} {x - 4.5:.1f},{y + 3.5:.1f} {x + 4.5:.1f},{y + 3.5:.1f}"
        return f'<polygon points="{pts}" fill="{color}"/>'
    if kind == "diamond":
        pts = f"{x:.1f},{y - 5:.1f} {x + 5:.1f},{y:.1f} {x:.1f},{y + 5:.1f} {x - 5:.1f},{y:.1f}"
        return f'<polygon points="{pts}" fill="{color}"/>'
    return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{color}"/>'


def line_chart(
    series: Sequence[Series],
    *,
    title: str,
    subtitle: str = "",
    x_label: str,
    y_label: str,
    x_range: tuple[float, float] | None = None,
    y_range: tuple[float, float] | None = None,
    width: int = 760,
    height: int = 440,
    x_format: str = "{:g}",
    y_format: str = "{:g}",
) -> str:
    """Render ``series`` as a standalone SVG document (white background)."""
    left, right, top, bottom = 70, 24, 64 if subtitle else 46, 96
    pw = width - left - right
    ph = height - top - bottom
    all_x = [x for s in series for x in s.xs]
    all_y = [y for s in series for y in s.ys if math.isfinite(y)]
    x0, x1 = x_range or (min(all_x), max(all_x))
    y0, y1 = y_range or (min(all_y), max(all_y))

    def sx(x: float) -> float:
        return left + (x - x0) / (x1 - x0) * pw

    def sy(y: float) -> float:
        return top + ph - (y - y0) / (y1 - y0) * ph

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="Helvetica, Arial, sans-serif" '
        f'font-size="12" role="img" aria-label="{escape(title)}">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<text x="{left}" y="22" font-size="15" font-weight="bold" fill="#222">'
        f"{escape(title)}</text>",
    ]
    if subtitle:
        parts.append(f'<text x="{left}" y="41" fill="#555">{escape(subtitle)}</text>')
    for yt in _ticks(y0, y1):
        y = sy(yt)
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{left + pw}" y2="{y:.1f}" stroke="#e3e3e3"/>'
        )
        parts.append(
            f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end" fill="#444">'
            f"{escape(y_format.format(yt))}</text>"
        )
    for xt in _ticks(x0, x1, 6):
        x = sx(xt)
        parts.append(
            f'<line x1="{x:.1f}" y1="{top + ph}" x2="{x:.1f}" y2="{top + ph + 5}" stroke="#444"/>'
        )
        parts.append(
            f'<text x="{x:.1f}" y="{top + ph + 19}" text-anchor="middle" fill="#444">'
            f"{escape(x_format.format(xt))}</text>"
        )
    parts.append(
        f'<rect x="{left}" y="{top}" width="{pw}" height="{ph}" fill="none" stroke="#888"/>'
    )
    parts.append(
        f'<text x="{left + pw / 2:.1f}" y="{top + ph + 40}" text-anchor="middle" fill="#222">'
        f"{escape(x_label)}</text>"
    )
    parts.append(
        f'<text transform="translate(18 {top + ph / 2:.1f}) rotate(-90)" text-anchor="middle" '
        f'fill="#222">{escape(y_label)}</text>'
    )
    for i, s in enumerate(series):
        color = PALETTE[i % len(PALETTE)]
        pts = [(sx(x), sy(y)) for x, y in zip(s.xs, s.ys, strict=True) if math.isfinite(y)]
        if len(pts) > 1:
            path = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
            parts.append(
                f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="2"/>'
            )
        parts.extend(_marker(MARKERS[i % len(MARKERS)], x, y, color) for x, y in pts)
    # Legend below the plot, one row per two series.
    for i, s in enumerate(series):
        color = PALETTE[i % len(PALETTE)]
        lx = left + (i % 2) * (pw / 2)
        ly = top + ph + 62 + (i // 2) * 18
        parts.append(
            f'<line x1="{lx:.1f}" y1="{ly - 4}" x2="{lx + 22:.1f}" y2="{ly - 4}" '
            f'stroke="{color}" stroke-width="2"/>'
        )
        parts.append(_marker(MARKERS[i % len(MARKERS)], lx + 11, ly - 4, color))
        parts.append(f'<text x="{lx + 30:.1f}" y="{ly}" fill="#222">{escape(s.label)}</text>')
    parts.append("</svg>")
    return "\n".join(parts) + "\n"
