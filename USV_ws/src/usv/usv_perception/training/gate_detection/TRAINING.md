# Gate OBB dataset and training contract

This directory prepares an offline vision baseline. It is not a navigation or
actuation component.

## Labels

- `0 gate`: one oriented quadrilateral around each complete white gate body.
- `1 opening`: one oriented quadrilateral around each relevant dark opening.

Each YOLO-OBB row has nine values:

```text
class_id x1 y1 x2 y2 x3 y3 x4 y4
```

Coordinates are normalized to `[0, 1]` and corners must follow the object
perimeter. Do not label reflections. Use an empty `.txt` file to declare an
intentional negative image. A missing label file is an error.

`auto_annotate.py` writes only to `labels_candidates/` plus diagnostics and
review overlays. Those files are proposals, not ground truth. Review and
correct every image before copying approved files into `labels/`; the validator
deliberately ignores the candidate directory.

## Processed layout

```text
processed/
  images/
  labels/
  manifest.csv
  manifest_split.csv
  splits/{train,val,test}.txt
  data.yaml
```

The normalized manifest uses `image_path` and `source_media`. The extractor's
`output_file`, `source_file` and `accepted` columns are consumed directly and
normalized; rejected extractor rows are ignored. `label_path` is optional when
labels mirror the `images/` tree. Source media, never individual adjacent
frames, is the unit of train/validation/test separation.

## Validate and create leak-free splits

```bash
python3 validate_dataset.py /ABS/PATH/processed \
  --assign-splits --write-config --report /ABS/PATH/processed/report.json
```

The command refuses missing images or labels, orphaned labels, invalid OBB
coordinates, empty splits and any source video that leaks across splits.
The extractor's original `manifest.csv` is read-only; normalized assignments
are written to `manifest_split.csv`.

For later re-validation without changing assignments:

```bash
python3 validate_dataset.py /ABS/PATH/processed \
  --manifest manifest_split.csv --write-config
```

## Training preflight

```bash
python3 train_obb.py --data /ABS/PATH/processed/data.yaml --dry-run
```

Real training requires a separately managed and pinned PyTorch/CUDA plus
Ultralytics environment. The wrapper never installs dependencies implicitly
and refuses accidental CPU training unless `--allow-cpu` is supplied.

```bash
python3 train_obb.py \
  --data /ABS/PATH/processed/data.yaml \
  --model yolo11n-obb.pt --epochs 100 --imgsz 960 --batch 8
```

Do not accept a model only from random-frame mAP. Keep the test source videos
unseen and also measure three-gate recall, consecutive misses, false positives,
and temporal jitter of gate center and angle.
