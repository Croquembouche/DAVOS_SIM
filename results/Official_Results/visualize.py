import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

def plot_min_avg_max_p95_p99(stats, y_label="Value", title=None, yscale="linear"):
    """
    stats: list of dicts like:
        {"label": "Series A", "min": 10, "avg": 20, "p95": 50, "p99": 80, "max": 120}
    y_label: axis label (e.g., "Latency (ms)")
    title: optional title
    yscale: "linear" or "log"
    """
    # basic validation
    for s in stats:
        mn, avg, p95, p99, mx = s["min"], s["avg"], s["p95"], s["p99"], s["max"]
        assert mn <= avg <= p95 <= p99 <= mx, f"Monotonicity violated for {s.get('label','(unlabeled)')}: {s}"

    fig, ax = plt.subplots(figsize=(8, 5))

    xs = range(1, len(stats) + 1)
    tick_half = 0.15  # width for min/max caps
    p95_half = 0.30   # width for p95 tick
    p99_half = 0.25   # width for p99 tick

    for i, s in enumerate(stats, start=1):
        label = s.get("label", f"S{i}")
        mn, avg, p95, p99, mx = s["min"], s["avg"], s["p95"], s["p99"], s["max"]

        # main min–max line
        ax.vlines(i, mn, mx)

        # end caps at min and max
        ax.hlines([mn, mx], i - tick_half, i + tick_half)

        # p95 tick (solid)
        ax.hlines(p95, i - p95_half, i + p95_half)

        # p99 tick (dashed)
        ax.hlines(p99, i - p99_half, i + p99_half, linestyles="dashed")

        # mean point
        ax.scatter([i], [avg], marker="o", zorder=3)


    labels = [s.get("label", f"S{i+1}") for i, s in enumerate(stats)]
    ax.set_xticks(list(xs))
    ax.set_xticklabels(labels, fontsize=20)  # <-- control x-tick label size here
    ax.set_ylabel(y_label, fontsize=20)
    if title:
        ax.set_title(title)
    ax.set_yscale(yscale)
    ax.grid(True, axis="y", linestyle=":", linewidth=0.5)

    # legend
    handles = [
        Line2D([0, 1], [0, 0], linestyle="-", label="Min–Max"),
        Line2D([0, 1], [0, 0], linestyle="-", label="p95 tick"),
        Line2D([0, 1], [0, 0], linestyle="--", label="p99 tick"),
        Line2D([0], [0], marker="o", linestyle="None", label="Average"),
    ]
    ax.legend(
        handles=handles,
        loc="upper left",
        frameon=False,
        fontsize=16,        # <--- bigger text
        markerscale=1.6,    # <--- scales marker size in legend
        handlelength=3.0,   # <--- length of line samples
        handletextpad=0.6,  # spacing between handle and text
        borderpad=0.6,      # padding inside legend box (if frameon=True)
        labelspacing=0.7    # vertical spacing between entries
    )
    plt.tight_layout()
    return fig, ax

# --- Example: fill in your values below ---
stats = [
    {"label": "SIM (w/ SP)", "min": 0.1260, "avg": 0.4566, "p95": 0.7296, "p99": 0.8776, "max": 1.1149},
    {"label": "SIM", "min": 0.1418, "avg": 0.7093, "p95": 1.4956, "p99": 2.2280, "max": 12.2053},
    {"label": "FAST (ZC)", "min": 1.7576,  "avg": 7.4015, "p95": 14.1246, "p99": 15.0144, "max": 334.3383},
    {"label": "Zenoh", "min": 2.6048,  "avg": 10.5044, "p95": 17.9120, "p99": 21.0511, "max": 32.5036},
]
plot_min_avg_max_p95_p99(stats, y_label="Latency (ms)", yscale="log")
plt.show()
