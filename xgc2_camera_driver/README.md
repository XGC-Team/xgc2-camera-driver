# XGC2 ROS 1 Camera Driver

The driver preserves a camera's native compressed stream instead of routing
4K video through a raw-image decode/re-encode loop.

The shipped LRCP imx415 profile requests native MJPEG 3840x2160 at 30 fps with
the 110° lens. A lower measured delivery rate is runtime evidence about the
physical camera path, not a configured fallback. RViz,
Lichtblick, calibration snapshots, and scientific bags must retain those source
dimensions; 1080p or 720p is not a successful fallback.

## Topic contract

For a node launched below namespace `/usb_cam`:

| Topic | Type | Use |
| --- | --- | --- |
| `/usb_cam/image_raw/compressed` | `sensor_msgs/CompressedImage` | Native MJPEG bytes |
| `/usb_cam/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics stamped with the encoded frame |
| `/usb_cam/frame_timing` | `xgc_camera_msgs/FrameTiming` | Native source, dequeue, mapped ROS, epoch, drop, and keyframe timing |
| `/usb_cam/stream_info` | `xgc_camera_msgs/StreamInfo` | Latched codec, geometry, clock-domain, and transport contract |
| `/usb_cam/image_raw` | `sensor_msgs/Image` | Optional decoded image for local algorithms |

`raw_publish_mode=on_demand` is the default. MJPEG is decoded only while
`/image_raw` has a subscriber. `always` publishes every frame and `never`
guarantees that the capture process never pays raw decode cost.

The shipped camera configuration uses `pixel_format: mjpeg`, so its native
source-quality topic is `/usb_cam/image_raw/compressed`. Native camera H.264
capture is not supported. Recording, calibration and source-quality exports
consume the native topic.

## Timestamp policy

V4L2 monotonic timestamps are mapped to system realtime using a measured,
per-frame `CLOCK_MONOTONIC`/`CLOCK_REALTIME` clock pair. The native timestamp,
mapping offset, uncertainty, dequeue time, and SOE/EOF reference are all
published in `FrameTiming`.

One mapping sample is retained with the source frame and reused by both the
native encoded message and any on-demand decoded image. Decoder latency
therefore cannot move the raw preview onto a different timestamp. A clock
domain or SOE/EOF reference change starts a new stream epoch. Values outside
the representable ROS 1 time range are rejected instead of wrapping.

The `reject` policy remains available and deliberately publishes no frame whose
source clock cannot be identified; the driver never substitutes
`ros::Time::now`. The shipped station profile has been independently verified
and therefore selects `assume_monotonic`.

When a calibration URL is supplied, its `image_width` and
`image_height` must be exactly 3840x2160. A 1080p or 720p intrinsic file is
rejected at startup instead of being relabeled as 4K CameraInfo.

## Offline derivatives and delay alignment

The bag is the immutable scientific record. For a recorded
`foxglove_msgs/CompressedVideo` H.264 stream (for example the Gazebo world
camera), create a small, playable derivative without decoding or re-encoding:

```bash
rosrun xgc2_camera_driver xgc_camera_bag_export \
  /data/session/xgc*.bag \
  --video-topic /xgc/camera/world/video_h264 \
  --timing-topic /xgc/camera/world/frame_timing \
  --stream-info-topic /xgc/camera/world/stream_info \
  --output /data/session/world-camera.mkv
```

The command defaults target the Gazebo world camera. With the shipped MJPEG
default, keep and replay `/usb_cam/image_raw/compressed` directly rather than
manufacturing an H.264 derivative for interface symmetry.

The exporter also writes an exact per-frame timing CSV and a provenance
manifest with source/derivative SHA-256 hashes. Timing messages are indexed
across every split bag before video export, so a `video`/`frame_timing` pair
that straddles a split boundary is retained. Duplicate source timestamps are
matched by encoded byte count and keyframe state; ambiguous matches without a
unique signature are left blank and counted in the manifest rather than
assigned to the wrong sequence. MKV/MP4 uses the declared nominal frame rate;
variable timing remains in the bag/CSV and must be used for analysis.
The source bags are opened read-only and checked for concurrent modification.
Sidecars are published before the video, which acts as the bundle commit
marker; failed exports leave no committed video.

Record a measured stable 120 ms physical-camera delay as a separate,
versioned artifact (the bags are never rewritten):

```bash
rosrun xgc2_camera_driver xgc_camera_alignment \
  /data/session/xgc*.bag --image-delay-ms 120 \
  --timing-topic /usb_cam/frame_timing \
  --output /data/session/alignment.yaml
```

Two manual `IMAGE_TIME_NS:SCENE_TIME_NS` anchors produce an affine clock model;
three or more produce a piecewise-linear model for drift. Affine scale is
stored as an exact integer ratio around an origin, avoiding precision loss at
Unix-epoch nanosecond magnitudes. A positive `--image-delay-ms 120` means that
an image arriving at time `I` depicts scene time `I - 120 ms`.

### Consume the alignment in replay

An alignment sidecar does not implicitly alter rosbag or Lichtblick. Explicitly
materialize only the low-bandwidth scene/TF topics on the image timeline:

```bash
rosrun xgc2_camera_driver xgc_camera_apply_alignment \
  /data/session/xgc*.bag \
  --alignment /data/session/alignment.yaml \
  --topic-map /tf=/xgc/aligned/tf \
  --topic-map /xgc/tf=/xgc/aligned/xgc_tf \
  --topic-map /xgc/scene=/xgc/aligned/scene \
  --topic-map /xgc/formation_scene=/xgc/aligned/formation_scene \
  --output /data/session/aligned-scene.bag
```

Open the immutable source bag and the small `aligned-scene.bag` together, then
select `/xgc/aligned/tf` and `/xgc/aligned/scene` for augmented replay. The
tool applies the inverse model: a scene sample at `S` is written at image time
`I` for which `S = model(I)`. It shifts bag record times and nested ROS `time`
fields (including `Header.stamp`) while preserving zero/unset stamps.

For bridge-based Lichtblick replay, one rosbag player can merge both timelines:

```bash
rosbag play /data/session/xgc*.bag \
  /data/session/aligned-scene.bag
```

Keep the original scene topics available for audit, but configure the
augmentation panels to consume the distinct `/xgc/aligned/*` topics. Static TF
is timeless for this purpose and may continue to use the original
`/tf_static`. Gazebo model states are not part of the default augmentation
mapping; if an offline algorithm needs them, explicitly add for example
`--topic-map /gazebo/model_states=/xgc/aligned/gazebo_model_states`.

Every input topic must have an explicit, distinct output name. Common
image/video topics are blocked to avoid accidentally duplicating a 4K record;
copying one requires the additional `--allow-large-topics` opt-in. Piecewise
models reject samples outside their anchor interval unless
`--allow-extrapolation` is explicitly supplied. `/clock` is never copied; the
source replay remains its sole owner. The derived bag has its own provenance
manifest and never rewrites either the source bags or `alignment.yaml`.
Before writing, the tool hashes the complete naturally ordered split-bag set
and requires its `(size, SHA-256)` multiset to exactly match the provenance in
`alignment.yaml`. Recorded paths may change after archival, but a sidecar from
another experiment, a missing split, or an extra split fails closed.

## Known limits

- The viewing container is constant-frame-rate and does not encode arbitrary
  ROS timestamp gaps; use its timing CSV or the source bag for measurements.
- H.264 export starts at a decodable IDR and requires SPS/PPS before it.
- Native MJPEG passthrough preserves the camera bytes but is usually much
  larger than H.264; this exporter intentionally handles H.264 only.
- Alignment generation/consumption reads the JSON-form YAML 1.2 emitted by
  `xgc_camera_alignment`; arbitrary YAML syntax is intentionally unsupported.
- Generic alignment shifts ROS fields declared with slot type `time`. Semantic
  timestamps encoded as integers or strings cannot be discovered and need a
  topic-specific adapter.
