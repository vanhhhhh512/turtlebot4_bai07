#!/usr/bin/env python3
"""Keep the three demo windows in a recording-friendly 2/3 + 1/3 layout."""
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


def xdotool_command():
    installed = shutil.which("xdotool")
    if installed:
        return installed, os.environ.copy()
    local = os.environ.get("BAI07_XDOTOOL")
    if local and Path(local).is_file():
        return local, os.environ.copy()
    raise RuntimeError("xdotool is required: sudo apt install xdotool")


def main():
    tool, env = xdotool_command()

    def run(*args):
        return subprocess.run(
            [tool, *map(str, args)],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

    size = run("getdisplaygeometry")
    if size.returncode:
        raise RuntimeError("DISPLAY is not an accessible X11 desktop")
    width, height = map(int, size.stdout.split())
    left = round(width * 0.66)
    upper = height // 2
    layouts = (
        (r".* - RViz$", (0, 0, left, height)),
        (r"^Gazebo$", (left, 0, width - left, upper)),
        (r"Bai07 - Giong noi va Nav2", (left, upper, width - left, height - upper)),
    )
    moved = {}
    print(
        f"Demo layout: RViz {left}x{height}, Gazebo/YOLO "
        f'{width-left}x{upper}; DISPLAY={os.environ.get("DISPLAY")}',
        flush=True,
    )
    while True:
        for title, (x, y, w, h) in layouts:
            result = run("search", "--onlyvisible", "--name", title)
            if result.returncode:
                continue
            for window in result.stdout.splitlines():
                if not window.isdigit() or moved.get(window, 0) >= 4:
                    continue
                run("windowsize", window, w, h)
                run("windowmove", window, x, y)
                if moved.get(window, 0) == 0:
                    run("windowactivate", window)
                moved[window] = moved.get(window, 0) + 1
        time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Window arrangement unavailable: {exc}", file=sys.stderr)
        sys.exit(1)
