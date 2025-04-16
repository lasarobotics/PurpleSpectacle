#!/usr/bin/env python

import sys
import time
import json
import signal
import argparse

import depthai
import spectacularAI

import ntcore
from wpiutil import wpistruct
from wpimath.geometry import Pose3d, Rotation3d, Quaternion

def signal_handler(sig, frame):
  print("\nExiting...")
  sys.exit(0)

pipeline = depthai.Pipeline()
config = spectacularAI.depthai.Configuration()

epilog = """
Modes:
test: Run a NT4 server for debugging
sim: Connect to simulation NT4 server
robot: Connect to robot NT4 server
"""

# Main function
if __name__ == "__main__":
  # Bind SIGINT handler
  signal.signal(signal.SIGINT, signal_handler)

  # Init argparse
  parser = argparse.ArgumentParser(
                    prog="Spectacle",
                    description="Publish pose data from Spectacular AI",
                    epilog=epilog,
                    formatter_class=argparse.RawTextHelpFormatter)

  # Add arguments
  parser.add_argument("--mode", required=True, choices=['test', 'sim', 'robot'], help="select script mode")
  parser.add_argument("--tag-map", help="path to AprilTag map JSON file")

  # Parse arguments
  args = parser.parse_args()

  # Init NT4
  inst = ntcore.NetworkTableInstance.getDefault()
  table = inst.getTable("Spectacle")

  # Start NT4 server/client
  match args.mode:
    case 'test':
      inst.startServer()
      print("Test mode, starting NT4 server...")
    case 'sim':
      inst.setServer("localhost")
      print("Simulation mode, connecting to localhost NT4 server...")
    case _:
      print("Robot mode, connecting to robot NT4 server...")

  # Start NT4 clients
  inst.startClient4("Spectacle")
  inst.startDSClient()

  # Create NT4 publishers
  statusPub = table.getBooleanTopic("Status").publish()
  posePub = table.getStructTopic("Pose", Pose3d).publish()

  # Set tag map path
  if args.tag_map:
    config.aprilTagPath = args.tag_map
    print("Using tag map at " + args.tag_map)
  else:
    print("No tag map provided, not using AprilTags!")

  # Init VIO session
  # config.useVioAutoExposure = True
  vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)
  device = depthai.Device(pipeline)

  # Read device name
  calib = device.readCalibration()
  eeprom = calib.getEepromData()
  pname = eeprom.productName
  print(pname)

  # Check if device is OAK-D Lite,
  # Spectacular AI does not support by default so we need to do some extra work.
  if pname == "OAK-D-LITE":
    vio_pipeline.imuToCameraLeft = [
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
    print("Using OAK-D Lite IMU matrix")

  # Check if OAK-D Pro series
  if "OAK-D-PRO" in pname:
    # Enable dot projector
    device.setIrLaserDotProjectorIntensity(0.9)

  vio_session = vio_pipeline.startSession(device)
  print("VIO session initialized.")

  print("Starting VIO...")

  # Processing loop
  while True:
    # Get session output
    out = vio_session.waitForOutput()
    data = json.loads(out.asJson())

    # Check if tracking
    status = data.get('status', "TRACKING")

    # Get pose
    quaternion = Quaternion(data["orientation"]['w'], data["orientation"]['x'], data["orientation"]['y'], data["orientation"]['z'])
    pose = Pose3d(data["position"]['x'], data["position"]['y'], data["position"]['z'], Rotation3d(quaternion))

    # Publish pose
    statusPub.set(status == "TRACKING")
    posePub.set(pose)

    print(status + " " + str(pose))
