# YOLO model

Place the fixed-size ONNX detector here as:

```text
yolov8n.onnx
```

Generate the initial COCO model from the project root:

```bash
python3 -m venv .venv-yolo-training
source .venv-yolo-training/bin/activate
python -m pip install -r requirements-yolo-training.txt
python tools/export_yolo_onnx.py \
  --model yolov8n.pt \
  --output src/perception/models/yolov8n.onnx
```

The training environment is not required at runtime. The ROS node loads ONNX
through the vendored Microsoft ONNX Runtime CPU library. OpenCV is used only
for preprocessing, non-maximum suppression, debug drawing, and HSV traffic
light classification.

For a custom two-class model (`vehicle`, `traffic_light`), copy its exported
ONNX file here and set these parameters in `config/yolo_rgbd.yaml`:

```yaml
class_count: 2
vehicle_class_ids: [0]
traffic_light_class_ids: [1]
```

Ultralytics model code and weights are AGPL-3.0 by default. Record the exact
model, version, dataset, and license in the project documentation.
