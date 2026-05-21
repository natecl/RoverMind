#!/usr/bin/env python3
"""Manual verification for the capture_and_analyze vision tool (static image).

Usage:
  python scripts/test_vision.py --target "water bottle" --image photo.jpg

Requires a CUDA GPU and the dependencies in requirements.txt.
"""

import argparse
import sys
from pathlib import Path

# Make `perception` importable when run as `python scripts/test_vision.py`.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PIL import Image  # noqa: E402

from perception.moondream_client import MoondreamClient  # noqa: E402
from perception.vision_tool import capture_and_analyze  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, help="object to look for")
    parser.add_argument("--image", required=True, help="path to an image file")
    args = parser.parse_args()

    print("Loading Moondream2 (first run downloads the model)...")
    moondream = MoondreamClient()

    image = Image.open(args.image).convert("RGB")
    observation = capture_and_analyze(
        args.target, capture_fn=lambda: image, moondream=moondream,
    )
    print(observation)
    return 0


if __name__ == "__main__":
    sys.exit(main())
