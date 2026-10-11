# XGC2 ROS 1 Camera Driver

The driver preserves a camera's native compressed stream instead of routing
4K video through a raw-image decode/re-encode loop.

The shipped LRCP imx415 profile requests native MJPEG 3840x2160 at 30 fps with
the 110° lens. A lower measured delivery rate is runtime evidence about the
physical camera path, not a configured fallback. RViz,
Lichtblick, calibration snapshots, and scientific bags must retain those source
dimensions; 1080p or 720p is not a successful fallback.

## Media outputs

The node serves the two consumers that watch the camera, and works only while
somebody watches. They are built on Ubuntu 20.04 and newer, where FFmpeg, the
XRPC `http.v1` server and `foxglove_msgs` are available; a Melodic build is the
ROS-only camera and rejects `control_socket`.

| Consumer | Input | Output | Runs |
| --- | --- | --- | --- |
| media-edge (browser WebRTC) | XRPC lease on the camera-source service | H.264/RTP to `rtp_host:rtp_port` (loopback) | between `start` and `stop` |
| Lichtblick ImageMode | subscribers of `video_h264` | `foxglove_msgs/CompressedVideo`, at most 1920x1080 | while the topic has subscribers |

Every captured JPEG is decoded once; the picture is shared by both encoders. The
encoder is `h264_encoder`: `h264_nvenc` prefers the NVIDIA hardware encoder and
falls back to `libx264` when it cannot be opened (the reason is in the log, in
`status.native.encoder_fallback` and in the describe facts); `libx264` never
tries the hardware. `h264_bitrate`, `h264_preset` (x264), `h264_gop_seconds`,
`h264_rate_control` (`bitrate`, `quality` or `auto`: quality for a still) and
`h264_quality` (CRF/CQ) tune the RTP encode; `preview_*` tune the preview. Media
needs an MJPEG camera (or a still, or the synthetic source) with even
dimensions. media-edge needs `net.core.rmem_max >= 8 MiB` for the RTP socket.

A still scene is encoded as one IDR; the node decodes that IDR and feeds the
reconstruction back as input, so every following picture (one per `1 /
still_fps` seconds) is a run of skip macroblocks. A new viewer costs one IDR
(`request-keyframe` is answered at once, not at the next picture) and the idle
stream is a few bytes per second. `still_gop_seconds` bounds how long a viewer
that lost a packet waits for the next IDR.

### camera-source service

`control_socket` is the absolute path of the service socket inside an owned
0700 directory; the supervisor passes it on the command line. An empty value
serves no media-edge. The service implements the media-edge
[source-control contract](https://github.com/XGC-Team/xgc2-media-edge/blob/main/contracts/source-control-v1.md)
for the single source `source_id` (default: the camera name).

| Route | Behaviour |
| --- | --- |
| `GET /v1/describe` | service discovery (unfenced): `service`, `api_version`, `instance_id`, `ready`, `facts`, `service_ref`, `sources` |
| `GET /v1/describe?wait_ready_ms=N` | holds the call, without polling, until the source is open and has captured a picture or `N` ms (at most 30000) pass |
| `GET .../describe` | descriptor: H264, payload 96, 90 kHz, destination, geometry, `fps` (for a still, `still_fps`), `frameId`, capabilities, force-IDR |
| `GET .../status` | `idle`, `starting`, `active`, `stopping` or `faulted`, desired/applied active, revisions, native counters |
| `POST .../start`, `.../stop` | applied receipts: `start` completes when the first access unit has been sent; both are idempotent; `start` of a source that is not ready fails |
| `POST .../request-keyframe` | the next picture is a real IDR with SPS and PPS |
| `POST .../capture` | `multipart/mixed`: metadata, the camera JPEG and optionally the exact RGB8 of the same frame; `requireFresh` waits for a picture after admission |
| `GET\|PATCH .../config` | `rtp_host`, `rtp_port`, `bitrate`; revisioned, ephemeral, applied only while idle |

Ready means the source is open and has captured a picture (a still: the image
is loaded), not that the process is alive. When the camera is unplugged the
service stays up with its socket and instance identity and reports
`ready:false` with the reason until the device returns. Only a real calibration
travels with a capture (`calibrationState: available`); an uncalibrated camera
reports `unavailable` and the node never invents intrinsics.

## ROS image pipe

`xgc2_ros_image_pipe` is a small roscpp tool for robots whose camera is already
owned by a ROS driver (for example the Scout's D435 colour stream) but whose
H.264 encoder lives in a media-edge pipeline: it subscribes a `sensor_msgs/Image`
topic and writes each image to standard output as a raw frame, and exits when
the reader closes the pipe.

```bash
rosrun xgc2_camera_driver xgc2_ros_image_pipe \
  --topic /d435/image_raw --format bgr8 --width 640 --height 480 --max-fps 30
```

`--format` is `bgr8` (default), `rgb8` or `mono8`; `bgr8`, `rgb8`, `bgra8`,
`rgba8`, `mono8` and the 16-bit depth encodings are converted (depth is scaled
so that `--depth-max` millimetres is white), row padding is removed. With
`--width/--height` an image of another size is fatal (exit 1); `--max-fps` drops
surplus frames. Standard output carries frames only: logs go to standard error,
and anything else that writes to standard output is moved there. A slow reader
throttles the tool, which keeps just the newest image (queue of one). It needs
only roscpp and `sensor_msgs`, builds as C++14 and is part of the Melodic
package.

## Topic contract

For a node launched below namespace `/usb_cam`:

| Topic | Type | Use |
| --- | --- | --- |
| `/usb_cam/image_raw/compressed` | `sensor_msgs/CompressedImage` | Native MJPEG bytes |
| `/usb_cam/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics stamped with the encoded frame |
| `/usb_cam/frame_timing` | `xgc_camera_msgs/FrameTiming` | Native source, dequeue, mapped ROS, epoch, drop, and keyframe timing |
| `/usb_cam/stream_info` | `xgc_camera_msgs/StreamInfo` | Latched codec, geometry, clock-domain, and transport contract |
| `/usb_cam/image_raw` | `sensor_msgs/Image` | Optional decoded image for local algorithms |
| `/usb_cam/video_h264` | `foxglove_msgs/CompressedVideo` | 1080p H.264 preview for Lichtblick, encoded only while subscribed (media outputs) |

`raw_publish_mode=on_demand` is the default. MJPEG is decoded only while
`/image_raw` has a subscriber. `always` publishes every frame and `never`
guarantees that the capture process never pays raw decode cost.

For `source_kind: still`, `image_raw/compressed`, `camera_info` and `stream_info`
are published once and latched, the static TF is on `/tf_static`, and neither
`frame_timing` nor `image_raw` exists.

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
