"""Reproducible manual semantic polygons clipped to navigable free space."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np
import yaml


def pixel_to_map(u, v, meta, height):
    r = meta["resolution"]
    ox, oy, t = meta["origin"]
    qx, qy = (u + 0.5) * r, (height - v - 0.5) * r
    return (
        ox + math.cos(t) * qx - math.sin(t) * qy,
        oy + math.sin(t) * qx + math.cos(t) * qy,
    )


def map_to_pixel(x, y, meta, height):
    ox, oy, t = meta["origin"]
    dx, dy = x - ox, y - oy
    r = meta["resolution"]
    return math.floor(
        (math.cos(t) * dx + math.sin(t) * dy) / r
    ), height - 1 - math.floor((-math.sin(t) * dx + math.cos(t) * dy) / r)


def load_map(path):
    path = Path(path)
    meta = yaml.safe_load(path.read_text())
    im = cv2.imread(str((path.parent / meta["image"]).resolve()), cv2.IMREAD_GRAYSCALE)
    if im is None:
        raise ValueError("Cannot read map image")
    probability = (
        im.astype(float) / 255 if meta.get("negate", 0) else 1 - im.astype(float) / 255
    )
    grid = np.full(im.shape, -1, dtype=np.int8)
    grid[probability < meta["free_thresh"]] = 0
    grid[probability > meta["occupied_thresh"]] = 100
    return meta, im, grid


def generate(root):
    root = Path(root)
    meta, im, grid = load_map(root / "config/map_navigation.yaml")
    h, w = im.shape
    cfg = yaml.safe_load((root / "config/region_seeds.yaml").read_text())
    n, components, stats, _ = cv2.connectedComponentsWithStats(
        (grid == 0).astype("uint8"), 8
    )
    free = components == (1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    distance = (
        cv2.distanceTransform(free.astype("uint8"), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        * meta["resolution"]
    )
    vv, uu = np.indices(im.shape)
    ox, oy, t = meta["origin"]
    r = meta["resolution"]
    qx = (uu + 0.5) * r
    qy = (h - vv - 0.5) * r
    xx = ox + np.cos(t) * qx - np.sin(t) * qy
    yy = oy + np.sin(t) * qx + np.cos(t) * qy
    labels = np.zeros(im.shape, dtype="uint8")
    regions = []
    for i, region in enumerate(cfg["regions"], 1):
        # Explicit half-open rectangles share no cells at the semantic boundary.
        poly = np.array(region["polygon"])
        low = poly.min(axis=0)
        high = poly.max(axis=0)
        mask = free & (xx >= low[0]) & (xx < high[0]) & (yy >= low[1]) & (yy < high[1])
        if np.any(labels[mask]):
            raise ValueError("Overlapping regions")
        labels[mask] = i
        valid = mask & (distance >= cfg["minimum_clearance_m"])
        if not valid.any():
            raise ValueError("No goal with required clearance")
        ax, ay = region.pop("anchor")
        score = (xx - ax) ** 2 + (yy - ay) ** 2
        score[~valid] = np.inf
        v, u = np.unravel_index(np.argmin(score), score.shape)
        x, y = pixel_to_map(int(u), int(v), meta, h)
        regions.append(
            {
                **region,
                "label": i,
                "goal": {"x": x, "y": y, "yaw": 0.0},
                "goal_pixel": [int(u), int(v)],
                "clearance_m": float(distance[v, u]),
                "free_cells": int(mask.sum()),
            }
        )
    np.save(root / "maps/region_labels.npy", labels, allow_pickle=False)
    output = {
        "frame_id": "map",
        "method": "manual polygons clipped to largest free component",
        "map_yaml": "map_navigation.yaml",
        "labels": "../maps/region_labels.npy",
        "robot_radius_m": cfg["robot_radius_m"],
        "minimum_clearance_m": cfg["minimum_clearance_m"],
        "goal_tolerance_m": 0.30,
        "yaw_tolerance_rad": 0.30,
        "regions": regions,
    }
    (root / "config/regions.yaml").write_text(
        yaml.safe_dump(output, allow_unicode=True, sort_keys=False)
    )
    # Watershed comparison only: automatic local maxima produce many small basins.
    peaks = (
        (distance == cv2.dilate(distance, np.ones((21, 21), np.uint8)))
        & (distance > 0.55)
    ).astype("uint8")
    count, seeds = cv2.connectedComponents(peaks)
    markers = seeds.astype("int32")
    markers[~free] = count + 1
    ws = cv2.watershed(cv2.cvtColor(im, cv2.COLOR_GRAY2BGR), markers)
    comparison = {
        "watershed_seed_components": count - 1,
        "manual_regions": len(regions),
        "unlabelled_free_cells": int(((grid == 0) & (labels == 0)).sum()),
        "free_cells": int((grid == 0).sum()),
        "unknown_cells": int((grid == -1).sum()),
        "occupied_cells": int((grid == 100).sum()),
        "labels_sha256": hashlib.sha256(
            (root / "maps/region_labels.npy").read_bytes()
        ).hexdigest(),
        "decision": "Manual semantic regions retained; automatic maxima oversegment connected maze corridors.",
    }
    (root / "maps/segmentation_metrics.json").write_text(
        json.dumps(comparison, indent=2)
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    fig, ax = plt.subplots(figsize=(7, 6))
    extent = [ox, ox + w * r, oy, oy + h * r]
    ax.imshow(im, cmap="gray", vmin=0, vmax=255, extent=extent)
    overlay = np.ma.masked_where(labels == 0, labels)
    ax.imshow(
        overlay,
        cmap=ListedColormap(["#5da5da", "#faa43a", "#60bd68"]),
        vmin=1,
        vmax=3,
        alpha=0.55,
        extent=extent,
        interpolation="nearest",
    )
    for reg in regions:
        x, y = reg["goal"]["x"], reg["goal"]["y"]
        ax.plot(x, y, "k*", ms=12)
        ax.text(x + 0.35, y + 0.35, reg["id"], weight="bold", fontsize=14)
    ax.set(
        xlabel="x (m)", ylabel="y (m)", title="A: Tay Bac | B: Tay Nam | C: Phia Dong"
    )
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(root / "maps/regions.png", dpi=180)
    plt.close(fig)
    assert np.all(grid[labels > 0] == 0)
    for theta in [0, 0.7, -1.2]:
        test = {**meta, "origin": [-10, -10, theta]}
        for u, v in [(0, 0), (400, 400), (200, 200), (75, 125)]:
            assert map_to_pixel(*pixel_to_map(u, v, test, h), test, h) == (u, v)
    print(
        json.dumps(
            {"regions": regions, "metrics": comparison}, ensure_ascii=False, indent=2
        )
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path)
    args = parser.parse_args()
    root = args.root
    if root is None:
        source = Path(__file__).resolve().parents[1]
        if (source / "config/map_navigation.yaml").is_file():
            root = source
        else:
            from ament_index_python.packages import get_package_share_directory

            root = Path(get_package_share_directory("bai07_turtlebot4"))
    generate(root)


if __name__ == "__main__":
    main()
