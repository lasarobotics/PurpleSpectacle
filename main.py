#!/usr/bin/env python

import os
import sys
import time
import json
import signal
import logging
import argparse
import threading

import ntcore
from wpiutil import wpistruct
from wpimath.geometry import Pose3d, Rotation3d, Quaternion

import spectacle

global spectacular_thread
global nt_listener_handles

global status_publisher
global pose_publisher
global stop_event

epilog = """
Modes:
test: Run a NT4 server for debugging
sim: Connect to simulation NT4 server
robot: Connect to robot NT4 server
"""

def signal_handler(sig, frame):
    logging.info("Exiting...")
    nt_instance = ntcore.NetworkTableInstance.getDefault()
    for listener_handle in nt_listener_handles:
        nt_instance.removeListener(listener_handle)
    stop_event.set()
    spectacular_thread.join()
    time.sleep(1)
    sys.exit(0)

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
            spectacle.config[topic_name] = event.data.value.getBoolean()
        case ntcore.NetworkTableType.kBooleanArray:
            spectacle.config[topic_name] = event.data.value.getBooleanArray()
        case ntcore.NetworkTableType.kDouble:
            spectacle.config[topic_name] = event.data.value.getDouble()
        case ntcore.NetworkTableType.kDoubleArray:
            spectacle.config[topic_name] = event.data.value.getDoubleArray()
        case ntcore.NetworkTableType.kInteger:
            spectacle.config[topic_name] = event.data.value.getInteger()
        case ntcore.NetworkTableType.kIntegerArray:
            spectacle.config[topic_name] = event.data.value.getIntegerArray()
        case ntcore.NetworkTableType.kRaw:
            spectacle.config[topic_name] = event.data.value.getRaw()
        case ntcore.NetworkTableType.kString:
            spectacle.config[topic_name] = event.data.value.getString()
        case ntcore.NetworkTableType.kStringArray:
            spectacle.config[topic_name] = event.data.value.getStringArray()
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
    parser.add_argument("--verbose", action="store_true")

    # Parse arguments
    args = parser.parse_args()

    # Configure logging
    loglevel = logging.INFO
    if args.verbose: loglevel = logging.DEBUG
    logging.basicConfig(
        level=loglevel,
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
        spectacle.config["AprilTagMapPath"] = args.tag_map
        logging.info("Using AprilTag map at " + args.tag_map)
    else:
        logging.info("No AprilTag map provided, not using AprilTags!")

    # Enable mapping mode
    if args.map:
        logging.info("Mapping environment...")
        spectacle.config["MappingMode"] = True

    # Start NT4 clients
    nt_instance.startClient4("PurpleSpectacle")
    nt_instance.startDSClient()

    # Create NT4 output publishers
    status_publisher = spectacle_nt.getBooleanTopic("Status").publish(ntcore.PubSubOptions(keepDuplicates=True, sendAll=True))
    pose_publisher = spectacle_nt.getStructTopic("Pose", Pose3d).publish(ntcore.PubSubOptions(keepDuplicates=True, sendAll=True))

    # Create NT4 config entries
    topics = list(spectacle.config.keys())
    auto_exposure_entry = spectacle_nt.getBooleanTopic(topics[0]).getEntry(spectacle.config[topics[0]])
    auto_exposure_entry.setDefault(spectacle.config[topics[0]])
    dot_projector_intensity_entry = spectacle_nt.getDoubleTopic(topics[1]).getEntry(spectacle.config[topics[1]])
    dot_projector_intensity_entry.setDefault(spectacle.config[topics[1]])
    ir_floodlight_intensity_entry = spectacle_nt.getDoubleTopic(topics[2]).getEntry(spectacle.config[topics[2]])
    ir_floodlight_intensity_entry.setDefault(spectacle.config[topics[2]])
    apriltag_map_path_entry = spectacle_nt.getStringTopic(topics[3]).getEntry(spectacle.config[topics[3]])
    apriltag_map_path_entry.setDefault(spectacle.config[topics[3]])
    mapping_mode_entry = spectacle_nt.getBooleanTopic(topics[4]).getEntry(spectacle.config[topics[4]])
    mapping_mode_entry.setDefault(spectacle.config[topics[4]])

    # Bind listener callback to subscribers
    nt_listener_handles = []
    nt_listener_handles.append(nt_instance.addListener(auto_exposure_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(dot_projector_intensity_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(ir_floodlight_intensity_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(apriltag_map_path_entry, ntcore.EventFlags.kValueAll, on_config_change))
    nt_listener_handles.append(nt_instance.addListener(mapping_mode_entry, ntcore.EventFlags.kValueAll, on_config_change))

    # Start Spectacular AI for first time
    stop_event = threading.Event()
    spectacular_thread = threading.Thread(target=spectacle.spectacular_session, args=(stop_event, status_publisher, pose_publisher))
    spectacular_thread.start()

    # Stay running
    while True:
        time.sleep(0.2)
