#!/usr/bin/env python3
"""Static native camera regression gate; no ROS master or device is needed."""

import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET


def check(root):
    package = root / "xgc2_camera_driver"
    definitions = sorted((package / "process-definitions").glob("*.json"))
    assert len(definitions) == 2, "station cameras have one still and one V4L2 definition"
    for path in definitions:
        document = json.loads(path.read_text())
        (definition,) = document["definitions"]
        parameters = definition["parameters"]
        for name in ("controlSocket", "rosMasterUri"):
            assert name in parameters["required"], (path, name, "must come from the registry")
            assert "default" not in parameters["properties"][name], (path, name)
        command = definition["command"]
        assert command["executable"].endswith("/xgc2_camera_driver_node"), path
        assert command["args"].count("_control_socket:=${controlSocket}") == 1, path
        assert command["env"]["ROS_MASTER_URI"] == "${rosMasterUri}", path
        assert definition["readiness"]["kind"] == "describe", path
        assert definition["services"][0]["endpointParameter"] == "controlSocket", path
        if "pixelFormat" in parameters["properties"]:
            assert parameters["properties"]["pixelFormat"]["enum"] == ["mjpeg"], path

    launch = ET.parse(package / "launch/camera.launch").getroot()
    nodes = list(launch.iter("node"))
    assert len(nodes) == 1 and nodes[0].get("type") == "xgc2_camera_driver_node"
    node = (package / "src/camera_driver_node.cpp").read_text()
    assert "image_transport" not in node, "transport plugins must not duplicate the native JPEG stream"
    assert 'nh_.advertise<sensor_msgs::Image>("image_raw", 1)' in node
    assert re.search(r'advertise<sensor_msgs::CameraInfo>\("camera_info", 4, still\)', node)
    assert re.search(r'"image_raw/compressed", still \? 1U : encoded_queue_capacity_, still\)', node)
    assert node.count("publishStill();") == 1, "still images are published once"
    still_run = node.split("int runStill()", 1)[1].split("void publishStill()", 1)[0]
    assert still_run.index("publishStill();") < still_run.index("updater_.update();")
    assert "createTimer" not in node and "createWallTimer" not in node

    outputs = (package / "src/media_outputs.cpp").read_text()
    assert "pipeline_->setPreviewWanted(count > 0)" in outputs
    assert "if (preview_subscribers_.load() <= 0)" in outputs
    assert "if (pipeline_->wanted())" in outputs
    assert 'ros::param::get("/run_id"' in outputs and "current != run_id" in outputs
    assert "ros::shutdown();" in outputs, "a replaced ROS master invalidates the camera binding"
    pipeline = (package / "src/media_pipeline.cpp").read_text()
    selection = pipeline.split("bool MediaPipeline::selectWorkLocked", 1)[1].split("std::int64_t MediaPipeline::rtpTicks", 1)[0]
    assert "const bool rtp = state_ == State::Starting || state_ == State::Active" in selection
    assert "const bool preview = config_.preview_enabled && preview_wanted_" in selection
    assert "work->want_rtp = rtp" in selection and "work->want_preview = preview" in selection
    state = (package / "src/source_state.cpp").read_text()
    assert "return open_ && have_frame_;" in state

    service = (package / "src/source_control_service.cpp").read_text()
    assert 'HttpIdentity{instance_id_, {"/v1/describe"}}' in service
    assert "request.target.find('?')" in service, "discovery is matched by path without a fence"
    assert "kMaximumWaitMilliseconds = 30000" in service
    cap = re.search(r"limits.request_timeout = std::chrono::seconds\((\d+)\)", service)
    assert cap and int(cap.group(1)) > 30, "the server must admit the full readiness wait"
    serve = service.split("void SourceControlService::serve()", 1)[1].split("void SourceControlService::waitForDeadlines", 1)[0]
    assert "processWaiters();" not in serve, "readiness must not be polled"
    assert "waiters_changed_.wait_until" in service and "state_->setChangeHandler" in service
    assert '"ros_master_uri"' in service and '"ros_master_run_id"' in service
    assert 'if (operation == "capture")' in service
    pipe = (package / "src/ros_image_pipe.cpp").read_text()
    assert "subscribe" in pipe and "v4l2" not in pipe and "/dev/video" not in pipe


if __name__ == "__main__":
    check(Path(__file__).resolve().parents[2])
    print("Native camera static checks passed")
