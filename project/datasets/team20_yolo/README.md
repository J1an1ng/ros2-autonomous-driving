# Team 20 YOLO dataset

Put labelled Unity RGB images and YOLO-format text labels here:

```text
images/train/
images/val/
labels/train/
labels/val/
```

Each label row is:

```text
class_id center_x center_y width height
```

Coordinates are normalized to `0..1`. Class `0` is `vehicle`; class `1` is
`traffic_light`. Do not commit raw/unlabelled captures or large generated
training runs unless they are intentionally part of the submission.
