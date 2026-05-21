#!/usr/bin/env python3
"""Manual verification for the capture_and_analyze vision tool.

Static image (any CUDA box):
  python scripts/test_vision.py --target "water bottle" --image photo.jpg

Live camera (on the rover, camera driver running):
  python scripts/test_vision.py --target "water bottle"

Requires a CUDA GPU and the dependencies in requirements.txt.
"""

import argparse
import sys
from pathlib import Path

# Make `perception` importable when run as `python scripts/test_vision.py`.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.moondream_client import MoondreamClient  # noqa: E402
from perception.vision_tool import capture_and_analyze, ros_capture_fn  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, help="object to look for")
    parser.add_argument(
        "--image", default=None,
        help="path to an image file; omit to capture live from the camera",
    )
    args = parser.parse_args()

    print("Loading Moondream2 (first run downloads the model)...")
    moondream = MoondreamClient()

    if args.image is not None:
        from PIL import Image
        image = Image.open(args.image).convert("RGB")
        capture_fn = lambda: image  # noqa: E731
    else:
        capture_fn = ros_capture_fn

    observation = capture_and_analyze(
        args.target, capture_fn=capture_fn, moondream=moondream,
    )
    print(observation)
    return 0


if __name__ == "__main__":
    sys.exit(main())
