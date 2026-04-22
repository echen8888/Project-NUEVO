# Vision Model Data

The default ROS vision node model is an exported Ultralytics YOLO26n NCNN
model:

```text
ros2_ws/src/vision/data/yolo26n_ncnn_imgsz_640/
```

It was exported with image size 640 and is tracked in Git because the NCNN
runtime files are small enough for this project.

Default detection classes:

- `traffic light`
- `stop sign`
- `person`

To try another Ultralytics model, place the exported model folder under this
directory and launch the node with:

```bash
ros2 run vision vision_node --ros-args \
  -p model_path:=/ros2_ws/src/vision/data/<model_folder> \
  -p model_imgsz:=640
```

For all COCO classes, pass an empty class filter:

```bash
ros2 run vision vision_node --ros-args -p class_filter:=""
```

Ignored local model files remain useful for experiments:

- `*.weights`
- `*.pt`
- `*.onnx`
- `*.engine`
