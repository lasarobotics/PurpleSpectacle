#!/usr/bin/env python

import sys
import time
import json
import signal
import argparse

from numpyencoder import NumpyEncoder

import ntcore
from wpimath.geometry import Pose3d, Transform3d
from robotpy_apriltag import AprilTag, AprilTagField, AprilTagFieldLayout

def signal_handler(sig, frame):
  print("\nExiting...")
  sys.exit(0)

epilog = """
"""

# Main function
if __name__ == "__main__":
  # Bind SIGINT handler
  signal.signal(signal.SIGINT, signal_handler)

  # Init argparse
  parser = argparse.ArgumentParser(
                    prog="WPILib to Spectacle",
                    description="Transform WPILib AprilTag map JSON to Spetacle format JSON",
                    epilog=epilog,
                    formatter_class=argparse.RawTextHelpFormatter)

  # Add arguments
  parser.add_argument("--tag-map", help="path to AprilTag map JSON file")

  # Parse arguments
  args = parser.parse_args()

  #field_layout = AprilTagFieldLayout(args.tag_map)
  field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

  # Reformat tag objects
  spectacle = []
  for wpilib_tag in field_layout.getTags():
    spectacle_tag = {}
    spectacle_tag["id"] = wpilib_tag.ID
    spectacle_tag["size"] = 0.1651
    spectacle_tag["family"] = "tag36h11"
    spectacle_tag["tagToWorld"] = wpilib_tag.pose.toMatrix()
    print(str(spectacle_tag["id"]) + " " + str(wpilib_tag.pose.rotation))
    spectacle.append(spectacle_tag)

  # Dump JSON
  spectacle_json = json.dumps(spectacle, indent=2, cls=NumpyEncoder)
  #print(spectacle_json)

  output_file = open("spectacle_tags.json", "w")
  output_file.write(spectacle_json)