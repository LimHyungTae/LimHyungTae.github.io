"""Generate conceptual blog figures with Times New Roman typography."""

from pathlib import Path
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/transformer-matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FONT = Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf")
font_manager.fontManager.addfont(str(FONT))
plt.rcParams.update({"font.family": "Times New Roman", "font.size": 12})
BLUE, GREEN, GRAY = "#356A8A", "#39816B", "#F0F2F3"


def attention():
    n = 12
    i, j = np.indices((n, n))
    masks = [np.ones((n, n)), i // 4 == j // 4, abs(i - j) <= 2]
    titles = ["(a) Full attention", "(b) Fixed-window attention", "(c) Sliding-window attention"]
    notes = ["All token pairs", "Non-overlapping groups of 4", "Radius 2 around each query"]
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 4.3))
    for ax, mask, title, note in zip(axes, masks, titles, notes):
        ax.imshow(mask, cmap=ListedColormap([GRAY, BLUE]), vmin=0, vmax=1)
        ax.set_title(title, pad=13, fontsize=14)
        ax.set_xticks([0, 3, 7, 11], [1, 4, 8, 12])
        ax.set_yticks([0, 3, 7, 11], [1, 4, 8, 12])
        ax.set_xlabel("Key position")
        ax.set_ylabel("Query position")
        ax.set_xticks(np.arange(-0.5, n, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, n, 1), minor=True)
        ax.grid(which="minor", color="white", linewidth=.6)
        ax.tick_params(which="minor", length=0)
        ax.text(.5, -.24, note, transform=ax.transAxes, ha="center", fontsize=12)
    fig.subplots_adjust(left=.06, right=.985, top=.86, bottom=.25, wspace=.32)
    fig.text(.5, .025, "Colored cells: allowed attention connections. Bidirectional examples; no causal mask.", ha="center", fontsize=12)
    fig.savefig(ROOT / "img/transformer-attention-windows.png", dpi=240, facecolor="white")
    plt.close(fig)


def registers():
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5))
    for ax, with_regs in zip(axes, [False, True]):
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.axis("off")
        ax.set_title("(b) With registers" if with_regs else "(a) Without registers", fontsize=15, pad=12)
        tokens = ["CLS", "P1", "P2", "P3"] + (["R1", "R2"] if with_regs else [])
        xs = np.linspace(1.1, 8.9, len(tokens))
        for x, label in zip(xs, tokens):
            color = GREEN if label.startswith("R") else BLUE
            for y in [7.9, 3.6]:
                ax.add_patch(Rectangle((x-.47, y), .94, .72, facecolor=color, edgecolor=color))
                ax.text(x, y+.36, label, ha="center", va="center", color="white", fontsize=12)
            ax.annotate("", xy=(x, 6.75), xytext=(x, 7.85), arrowprops={"arrowstyle": "->", "color": color})
            ax.annotate("", xy=(x, 4.38), xytext=(x, 5.25), arrowprops={"arrowstyle": "->", "color": color})
        ax.add_patch(Rectangle((.3, 5.25), 9.4, 1.5, facecolor=GRAY, edgecolor="#555555", linewidth=1))
        ax.text(5, 6.2, "Transformer blocks", ha="center", fontsize=15)
        ax.text(5, 5.65, "Self-attention across all input tokens", ha="center", fontsize=12)
        ax.text(5, 9.15, "Patch tokens + learnable registers" if with_regs else "Patch tokens + learnable CLS", ha="center")
        if with_regs:
            ax.text(2.65, 2.65, "CLS / patches: retained", ha="center", color=BLUE)
            ax.text(8, 2.65, "Registers:\ndiscarded", ha="center", color=GREEN)
            note = "Explicit workspace for internal computation"
        else:
            ax.text(5, 2.65, "CLS / patches: retained", ha="center", color=BLUE)
            note = "Some background patches may serve as workspace"
        ax.text(5, 1.35, note, ha="center", fontsize=12)
    fig.subplots_adjust(left=.025, right=.975, top=.88, bottom=.1, wspace=.15)
    fig.text(.5, .035, "Conceptual architecture. P: patch token; R: register token. Two registers shown for clarity.", ha="center", fontsize=12)
    fig.savefig(ROOT / "img/vit-register-token-workspace.png", dpi=240, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    attention()
    registers()
