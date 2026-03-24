import argparse
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# --- Data ---
n_values = [2, 3, 4]
ram_bytes = [56812, 68204, 80748]
ram_total_kb = 128
ram_pct = [b / (ram_total_kb * 1024) * 100 for b in ram_bytes]
ram_kb = [round(b / 1024) for b in ram_bytes]

# --- Plot ---
fig, ax = plt.subplots(figsize=(6, 5))

bars = ax.bar(
    [str(n) for n in n_values],
    ram_pct,
    color=["#4C72B0", "#55A868", "#C44E52"],
    width=0.5,
    zorder=3,
)

# Annotate each bar with kB value
for bar, kb in zip(bars, ram_kb):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 0.8,
        f"{kb} kB",
        ha="center",
        va="bottom",
        fontsize=11,
        fontweight="bold",
    )

ax.set_xlabel("n", fontsize=13)
ax.set_ylabel("RAM Usage (%)", fontsize=13)
ax.set_title("RAM Usage on nRF52833 (128 kB total)", fontsize=13)
ax.set_ylim(0, 80)
ax.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.0f%%"))
ax.grid(axis="y", linestyle="--", alpha=0.6, zorder=0)
ax.spines[["top", "right"]].set_visible(False)

plt.tight_layout()

# --- Save or show ---
parser = argparse.ArgumentParser()
parser.add_argument("--save", action="store_true", help="Save plot as SVG instead of showing it")
args = parser.parse_args()

if args.save:
    out = "ram_chart.svg"
    fig.savefig(out, format="svg")
    print(f"Saved to {out}")
else:
    plt.show()