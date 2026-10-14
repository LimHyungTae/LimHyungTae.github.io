"""Draw original conceptual diagrams for the Transformer architecture series."""

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/transformer-matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
FONT = Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf")
font_manager.fontManager.addfont(str(FONT))
plt.rcParams.update({"font.family": "Times New Roman", "font.size": 12})
BLUE, GREEN, GRAY, INK = "#356A8A", "#39816B", "#F0F2F3", "#252525"


def box(ax, x, y, w, h, label, fill=GRAY, color=INK, size=12):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=fill, edgecolor=INK, linewidth=.8))
    ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", color=color, fontsize=size)


def arrow(ax, start, end, color=INK):
    ax.annotate("", xy=end, xytext=start, arrowprops={"arrowstyle": "->", "color": color, "lw": 1})


def gqa():
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.4))
    for ax, count, title in zip(axes, [8, 2, 1], ["(a) MHA: 8 Q heads, 8 KV heads", "(b) GQA: 8 Q heads, 2 KV heads", "(c) MQA: 8 Q heads, 1 KV head"]):
        ax.set_xlim(-1.15, 8)
        ax.set_ylim(-.2, 2.3)
        ax.axis("off")
        ax.set_title(title, loc="left", fontsize=14, pad=6)
        ax.text(-.9, .35, "Query", ha="center", va="center")
        ax.text(-.9, 1.8, "Key /\nValue", ha="center", va="center")
        for h in range(8):
            x = h + .15
            box(ax, x, 0, .65, .7, f"Q{h + 1}", BLUE, "white")
            group = h // (8 // count)
            center = (group + .5) * (8 / count) - .025
            arrow(ax, (x + .325, .75), (center, 1.43), GREEN)
        for g in range(count):
            center = (g + .5) * (8 / count) - .025
            width = .75 if count == 8 else 1.25
            box(ax, center - width / 2, 1.45, width, .7, f"KV{g + 1}", GREEN, "white")
    fig.subplots_adjust(left=.07, right=.98, top=.94, bottom=.09, hspace=.65)
    fig.text(.5, .025, "Connections indicate KV sharing, not attention weights. Token positions are omitted.", ha="center")
    fig.savefig(ROOT / "img/gqa-head-sharing.png", dpi=240, facecolor="white")
    plt.close(fig)


def gates():
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.8))
    titles = ["(a) SwiGLU: input-dependent gating inside the FFN", "(b) LayerScale: learned channel scale on a residual branch", "(c) Gated Attention: input-dependent gating after attention"]
    for ax, title in zip(axes, titles):
        ax.set_xlim(0, 12)
        ax.set_ylim(0, 3)
        ax.axis("off")
        ax.set_title(title, loc="left", fontsize=14, pad=4)

    ax = axes[0]
    box(ax, .1, 1.15, .7, .6, "X")
    box(ax, 2, 2, 2.5, .65, "Linear + SiLU", GREEN, "white")
    box(ax, 2, .25, 2.5, .65, "Linear", BLUE, "white")
    for y in [2.325, .575]:
        arrow(ax, (.85, 1.45), (1.95, y))
        arrow(ax, (4.55, y), (6.15, 1.45))
    box(ax, 6.2, 1.1, 1.8, .7, "Multiply")
    box(ax, 9, 1.1, 1.8, .7, "Linear")
    arrow(ax, (8.05, 1.45), (8.95, 1.45))
    arrow(ax, (10.85, 1.45), (11.6, 1.45))
    ax.text(11.8, 1.45, "Y", ha="center", va="center")

    ax = axes[1]
    box(ax, .1, 1.15, .7, .6, "X")
    box(ax, 2, .45, 2.5, .7, "Branch F(X)", BLUE, "white")
    box(ax, 6.2, .45, 1.8, .7, "Multiply")
    box(ax, 5.85, 2, 2.5, .65, "Learned scale", GREEN, "white")
    box(ax, 9, .45, 1.8, .7, "Add X")
    arrow(ax, (.85, 1.45), (1.95, .8))
    arrow(ax, (4.55, .8), (6.15, .8))
    arrow(ax, (7.1, 1.95), (7.1, 1.2))
    arrow(ax, (8.05, .8), (8.95, .8))
    ax.plot([.45, .45, 9.9], [1.8, 2.85, 2.85], color=INK, lw=1)
    arrow(ax, (9.9, 2.85), (9.9, 1.2))
    arrow(ax, (10.85, .8), (11.6, .8))
    ax.text(11.8, .8, "Y", ha="center", va="center")

    ax = axes[2]
    box(ax, .1, 1.15, .7, .6, "X")
    box(ax, 2, 2, 2.5, .65, "Linear + sigmoid", GREEN, "white")
    box(ax, 1.65, .2, 3.2, .8, "Q, K, V projections\n+ attention", BLUE, "white")
    for y in [2.325, .6]:
        arrow(ax, (.85, 1.45), (1.6 if y == .6 else 1.95, y))
        arrow(ax, (4.9 if y == .6 else 4.55, y), (6.15, 1.45))
    box(ax, 6.2, 1.1, 1.8, .7, "Multiply")
    box(ax, 8.7, 1.1, 2.5, .7, "Output projection")
    arrow(ax, (8.05, 1.45), (8.65, 1.45))
    arrow(ax, (11.25, 1.45), (11.6, 1.45))
    ax.text(11.8, 1.45, "Y", ha="center", va="center")
    fig.subplots_adjust(left=.035, right=.98, top=.94, bottom=.08, hspace=.5)
    fig.text(.5, .02, "Conceptual operations only; normalization and the surrounding Transformer block are omitted.", ha="center")
    fig.savefig(ROOT / "img/transformer-gating-comparison.png", dpi=240, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    gqa()
    gates()
