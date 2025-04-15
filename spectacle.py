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

# Main function
if __name__ == "__main__":
  # Bind SIGINT handler
  signal.signal(signal.SIGINT, signal_handler)

  # Init argparse
  parser = argparse.ArgumentParser(
                    prog='Spectacle',
                    description='Publish pose data from Spectacular AI',
                    epilog='')

  # Add arguments
  parser.add_argument('--mode', required=True, choices=['test', 'sim', 'robot'])
  parser.add_argument('--tag-map')

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
      inst.startClient4("Spectacle")
      inst.startDSClient()
      print("Simulation mode, connecting to localhost NT4 server...")
    case _:
      inst.startClient4("Spectacle")
      inst.startDSClient()
      print("Robot mode, connecting to robot NT4 server...")

  # Create NT4 publishers
  statusPub = table.getBooleanTopic("status").publish()
  posePub = table.getStructTopic("pose", Pose3d).publish()


  # Set tag map path
  if args.tag_map:
    config.aprilTagPath = args.tag_map
    print("Using tag map at " + args.tag_map)
  else:
    print("No tag map provided, not using AprilTags!")

   # Init VIO session
  vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)
  device = depthai.Device(pipeline)
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
    quat = Quaternion(data["orientation"]['w'], data["orientation"]['x'], data["orientation"]['y'], data["orientation"]['z'])
    pose = Pose3d(data["position"]['x'], data["position"]['y'], data["position"]['z'], Rotation3d(quat))

    # Publish pose
    statusPub.set(status == "TRACKING")
    posePub.set(pose)

    print(status + " " + str(pose))