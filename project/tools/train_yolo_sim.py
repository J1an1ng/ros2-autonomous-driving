#!/usr/bin/env python3
"""Fine-tune YOLO on Unity images and export the best checkpoint to ONNX."""

import argparse
from pathlib import Path
import shutil
import tempfile

from ultralytics import YOLO
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--model', default='yolov8n.pt')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument(
        '--project', type=Path, default=Path('runs/team20_yolo')
    )
    parser.add_argument('--name', default='unity_finetune')
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('src/perception/models/yolov8n.onnx'),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    data_file = args.data.resolve()
    with data_file.open(encoding='utf-8') as stream:
        dataset = yaml.safe_load(stream)
    dataset_root = Path(dataset.get('path', '.'))
    if not dataset_root.is_absolute():
        dataset_root = (data_file.parent / dataset_root).resolve()
    dataset['path'] = str(dataset_root)

    trainer = YOLO(args.model)
    with tempfile.TemporaryDirectory(prefix='team20-yolo-data-') as temporary:
        resolved_data = Path(temporary) / 'dataset.yaml'
        with resolved_data.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(dataset, stream, sort_keys=False)
        results = trainer.train(
            data=str(resolved_data),
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            project=str(args.project),
            name=args.name,
            pretrained=True,
        )
    best = Path(results.save_dir) / 'weights' / 'best.pt'
    exported = Path(
        YOLO(best).export(
            format='onnx',
            imgsz=args.imgsz,
            opset=12,
            dynamic=False,
            simplify=False,
            nms=False,
        )
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != args.output.resolve():
        shutil.copy2(exported, args.output)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
