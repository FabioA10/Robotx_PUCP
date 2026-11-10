# Gate dataset frame extraction

This offline tool samples the original RobotX gate media without modifying it.
It writes normalized JPEG frames plus CSV and JSON manifests that retain the
source file, media timestamp, display rotation, image dimensions, blur score,
perceptual hash and any rejection reason.

Preview the deterministic plan without creating files:

```bash
python3 extract_frames.py \
  ../../../../../../DATASET_COMPUERTA/'DATASET TASK3' \
  ../../../../../../DATASET_COMPUERTA/processed/gate_frames_v1 \
  --interval-s 2.5 \
  --dry-run
```

Run extraction only after reviewing the plan:

```bash
python3 extract_frames.py \
  ../../../../../../DATASET_COMPUERTA/'DATASET TASK3' \
  ../../../../../../DATASET_COMPUERTA/processed/gate_frames_v1 \
  --interval-s 2.5 \
  --min-blur-variance 0 \
  --max-dhash-distance 2
```

The output directory must be new or empty and must be outside the source
directory. Frame decoding uses ffmpeg's display rotation, so portrait clips are
not silently treated as landscape. Rejected candidates are kept in the
manifests but not copied to `images/`.

