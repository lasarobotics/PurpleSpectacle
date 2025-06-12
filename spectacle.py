#!/usr/bin/env python

import json
import math
import logging
import threading

import depthai
import spectacularAI

import ntcore
from wpiutil import wpistruct
from wpimath.geometry import Pose3d, Rotation3d, Quaternion

oak_d_lite_imu_to_camera_left = [
    [
        0.9993864566531208,
        0.002608506818609768,
        0.03492715205248295,
        0.004358885459078838
    ],
    [
        0.0033711974751103025,
        -0.9997567647621044,
        -0.02179555780417905,
        0.0002560060614699508
    ],
    [
        0.034861802677196824,
        0.021899931611508897,
        -0.9991521644421877,
        0.0018364413451974568
    ],
    [
        0.0,
        0.0,
        0.0,
        1.0
    ]
]

# Config variables
config = {
    "AutoExposure": True,
    "DotProjectorIntensity": 0.9,
    "IRFloodlightIntensity": 0.0,
    "AprilTagMapPath": "",
    "MappingMode": False
}

# Orientation offset
ORIENTATION_OFFSET = Rotation3d(0.0, math.pi / 2, math.pi)

def spectacular_session(
    stop_event: threading.Event,
    status_publisher: ntcore.BooleanPublisher, pose_publisher: ntcore.StructPublisher):
    """Spectacular AI session

    Does VIO tracking
    Args:
        stop_event (threading.Event): Threading event for Spectacular AI session to trigger stop
        status_publisher (ntcore.BooleanPublisher): NT4 status publisher
        pose_publisher (ntcore.StructPublisher): NT4 Pose3d struct publisher
    """

    # Init VIO session
    pipeline = depthai.Pipeline()
    pipeline_config = spectacularAI.depthai.Configuration()
    logging.debug("Spectacle config: " + str(config))

    # Set config options
    if config["AutoExposure"]: pipeline_config.useVioAutoExposure = True
    if config["AprilTagMapPath"]: pipeline_config.aprilTagPath = config["AprilTagMapPath"]
    if config["MappingMode"]: pipeline_config.mapSavePath = "slam_map._"

    # Open device and pipeline
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, pipeline_config)
    device = depthai.Device(pipeline)

    # Read device name
    calib = device.readCalibration()
    eeprom = calib.getEepromData()
    pname = eeprom.productName
    logging.info(pname)

    # Check if device is OAK-D Lite,
    # Spectacular AI does not support by default so we need to do some extra work.
    if pname == "OAK-D-LITE":
        vio_pipeline.imuToCameraLeft = oak_d_lite_imu_to_camera_left
        logging.info("Using OAK-D Lite IMU matrix")

    # Check if OAK-D Pro series
    if "OAK-D-PRO" in pname:
        # Enable dot projector
        device.setIrLaserDotProjectorIntensity(config["DotProjectorIntensity"])
        device.setIrFloodLightIntensity(config["IRFloodlightIntensity"])

    logging.info("VIO session initialized")

    # VIO session processing loop
    with vio_pipeline.startSession(device) as vio_session:
        while not stop_event.is_set():
            if not vio_session.hasOutput(): continue

            # Get session output
            out = vio_session.getOutput()
            data = json.loads(out.asJson())

            # Check if tracking
            status = data.get("status", "TRACKING")

            # Get pose
            # Axes edited to match WPILib convention
            quaternion = Quaternion(data["orientation"]["w"], data["orientation"]["z"], data["orientation"]["x"], data["orientation"]["y"])
            rotation = Rotation3d(quaternion).rotateBy(ORIENTATION_OFFSET)
            rotation = Rotation3d(-rotation.X(), -rotation.Y(), -rotation.Z())
            pose = Pose3d(data["position"]["y"], data["position"]["x"], data["position"]["z"], rotation)

            # Publish pose
            status_publisher.set(status == "TRACKING")
            pose_publisher.set(pose)

            logging.debug(status + " " + str(pose))


    # Close device
    device.close()
    logging.info("VIO session stopped")
