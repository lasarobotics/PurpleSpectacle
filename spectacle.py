#!/usr/bin/env python

import os
import sys
import time
import json
import math
import signal
import logging
import argparse
import threading

import depthai
import spectacularAI
import cv2

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

epilog = """
Modes:
test: Run a NT4 server for debugging
sim: Connect to simulation NT4 server
robot: Connect to robot NT4 server
"""

# Config variables
spectacle_config = {
"AutoExposure": False,
"DotProjectorIntensity": 0.9,
"IRFloodlightIntensity": 0.0,
"AprilTagMapPath": "",
"MappingMode": False
}

global spectacular_thread
global nt_listener_handles

global status_publisher
global pose_publisher
global stop_event


def signal_handler(sig, frame):
    logging.info("Exiting...")
    nt_instance = ntcore.NetworkTableInstance.getDefault()
    for listener_handle in nt_listener_handles:
        nt_instance.removeListener(listener_handle)
    stop_event.set()
    spectacular_thread.join()
    time.sleep(1)
    sys.exit(0)

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
    config = spectacularAI.depthai.Configuration()
    logging.debug("Spectacle config: " + str(spectacle_config))

    # Set config options
    if spectacle_config["AutoExposure"]: config.useVioAutoExposure = True
    if spectacle_config["AprilTagMapPath"]: config.aprilTagPath = spectacle_config["AprilTagMapPath"]
    if spectacle_config["MappingMode"]: config.mapSavePath = "slam_map._"

    # Open device and pipeline
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)
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
        device.setIrLaserDotProjectorIntensity(spectacle_config["DotProjectorIntensity"])
        device.setIrFloodLightIntensity(spectacle_config["IRFloodlightIntensity"])

    logging.info("VIO session initialized")

    # VIO session processing loop
    with vio_pipeline.startSession(device) as vio_session:
        while not stop_event.is_set():
            if not vio_session.hasOutput(): continue

            # Get session output
            out = vio_session.getOutput()
            data = json.loads(out.asJson())

            # Check if tracking
            status = data.get('status', "TRACKING")

            # Get pose, #messed with rotation axises a bit to try and make wpilib see it like it is.
            quaternion = Quaternion(data["orientation"]['w'], data["orientation"]['y'], data["orientation"]['x'], data["orientation"]['z'])
            pose = Pose3d(data["position"]['x'], data["position"]['y'], data["position"]['z'], Rotation3d(quaternion))

            # Publish pose
            status_publisher.set(status == "TRACKING")
            pose_publisher.set(pose)

            logging.debug(status + " " + str(pose))


    # Close device
    device.close()
    logging.info("VIO session stopped")

def start_spectacular():
    """Start Spectacular AI session
    """

    stop_event.clear()
    spectacular_thread = threading.Thread(target=spectacular_session, args=(stop_event, status_publisher, pose_publisher))
    spectacular_thread.start()

def on_config_change(event: ntcore.Event):
    """NT4 config change callback

    Stops Spectacular AI session, updates config, and restarts session

    Args:
        event (ntcore.Event): NT4 event
    """

    # Stop Spectacular
    stop_event.set()
    spectacular_thread.join()
    time.sleep(1)

    # Get data
    data_type = event.data.topic.getType()
    topic_name = event.data.topic.getName().split("/")[-1]
    match data_type:
        case ntcore.NetworkTableType.kBoolean:
            spectacle_config[topic_name] = event.data.value.getBoolean()
        case ntcore.NetworkTableType.kBooleanArray:
            spectacle_config[topic_name] = event.data.value.getBooleanArray()
        case ntcore.NetworkTableType.kDouble:
            spectacle_config[topic_name] = event.data.value.getDouble()
        case ntcore.NetworkTableType.kDoubleArray:
            spectacle_config[topic_name] = event.data.value.getDoubleArray()
        case ntcore.NetworkTableType.kInteger:
            spectacle_config[topic_name] = event.data.value.getInteger()
        case ntcore.NetworkTableType.kIntegerArray:
            spectacle_config[topic_name] = event.data.value.getIntegerArray()
        case ntcore.NetworkTableType.kRaw:
            spectacle_config[topic_name] = event.data.value.getRaw()
        case ntcore.NetworkTableType.kString:
            spectacle_config[topic_name] = event.data.value.getString()
        case ntcore.NetworkTableType.kStringArray:
            spectacle_config[topic_name] = event.data.value.getStringArray()
        case _:
            logging.exception("Unsupported config data type!")

    # Restart Spectacular pipeline
    start_spectacular()

# Main function
if __name__ ==  "__main__":
    # Bind SIGINT handler
    signal.signal(signal.SIGINT, signal_handler)

    # Init argparse
    parser = argparse.ArgumentParser(
        prog="Spectacle",
        description="Publish pose data from Spectacular AI",
        epilog=epilog,
        formatter_class=argparse.RawTextHelpFormatter
    )

    # Add arguments
    parser.add_argument("--mode", required=True, choices=["test", "sim", "robot"], help="select script mode")
    parser.add_argument("--tag-map", help="path to AprilTag map JSON file")
    parser.add_argument("--map", action="store_true")

    # Parse arguments
    args = parser.parse_args()

    # Configure logging
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # Init NT4
    nt_instance = ntcore.NetworkTableInstance.getDefault()
    spectacle_nt = nt_instance.getTable("Spectacle")

    # Start NT4 server/client
    match args.mode:
        case 'test':
            nt_instance.startServer()
            logging.info("Test mode, starting NT4 server...")
        case 'sim':
            nt_instance.setServer("localhost")
            logging.info("Simulation mode, connecting to localhost NT4 server...")
        case _:
            logging.info("Robot mode, connecting to robot NT4 server...")

    # Set tag map path
    if args.tag_map:
        spectacle_config["AprilTagMapPath"] = args.tag_map
        logging.info("Using AprilTag map at " + args.tag_map)
    else:
        logging.info("No AprilTag map provided, not using AprilTags!")

    # Enable mapping mode
    if args.map:
        logging.info("Mapping environment...")
        spectacle_config["MappingMode"] = True

    # Start NT4 clients
    nt_instance.startClient4("Spectacle")
    nt_instance.startDSClient()

    # Create NT4 output publishers
    status_publisher = spectacle_nt.getBooleanTopic("Status").publish(ntcore.PubSubOptions(keepDuplicates=True, sendAll=True))
    pose_publisher = spectacle_nt.getStructTopic("Pose", Pose3d).publish(ntcore.PubSubOptions(keepDuplicates=True, sendAll=True))

    # Create NT4 config entries
    topics = list(spectacle_config.keys())
    auto_exposure_entry = spectacle_nt.getBooleanTopic(topics[0]).getEntry(spectacle_config[topics[0]])
    auto_exposure_entry.setDefault(spectacle_config[topics[0]])
    dot_projector_intensity_entry = spectacle_nt.getDoubleTopic(topics[1]).getEntry(spectacle_config[topics[1]])
    dot_projector_intensity_entry.setDefault(spectacle_config[topics[1]])
    ir_floodlight_intensity_entry = spectacle_nt.getDoubleTopic(topics[2]).getEntry(spectacle_config[topics[2]])
    ir_floodlight_intensity_entry.setDefault(spectacle_config[topics[2]])
    apriltag_map_path_entry = spectacle_nt.getStringTopic(topics[3]).getEntry(spectacle_config[topics[3]])
    apriltag_map_path_entry.setDefault(spectacle_config[topics[3]])
    mapping_mode_entry = spectacle_nt.getBooleanTopic(topics[4]).getEntry(spectacle_config[topics[4]])
    mapping_mode_entry.setDefault(spectacle_config[topics[4]])

    # Bind listener callback to subscribers
    nt_listener_handles = []
    nt_listener_handles.append(nt_instance.addListener(auto_exposure_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(dot_projector_intensity_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(ir_floodlight_intensity_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(apriltag_map_path_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(mapping_mode_entry, ntcore.EventFlags.kValueAll, on_config_change))

    # Start Spectacular AI for first time
    stop_event = threading.Event()
    spectacular_thread = threading.Thread(target=spectacular_session, args=(stop_event, status_publisher, pose_publisher))
    spectacular_thread.start()

    # Stay running
    while True:
        time.sleep(0.2)
