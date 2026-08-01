#!/usr/bin/env python3
"""Download an Ultralytics checkpoint and export a fixed-size ONNX model."""

import argparse
from pathlib import Path
import shutil

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='yolov8n.pt')
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('src/perception/models/yolov8n.onnx'),
    )
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--opset', type=int, default=12)
    parser.add_argument(
        '--simplify',
        action='store_true',
        help='Run onnxslim; disabled by default for a reproducible raw export.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.model)
    exported = Path(
        model.export(
            format='onnx',
            imgsz=args.imgsz,
            opset=args.opset,
            dynamic=False,
            simplify=args.simplify,
            nms=False,
        )
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != args.output.resolve():
        shutil.copy2(exported, args.output)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
