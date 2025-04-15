#!/usr/bin/env python

import depthai
import spectacularAI
import time
import json

import ntcore
from wpiutil import wpistruct
from wpimath.geometry import Pose3d, Rotation3d, Quaternion

pipeline = depthai.Pipeline()

config = spectacularAI.depthai.Configuration()
#config.aprilTagPath = "/home/vignesh/Documents/source/Spectacle/tags.json"

vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)
device = depthai.Device(pipeline)
vio_session = vio_pipeline.startSession(device)

out = {}

if __name__ == "__main__":
  inst = ntcore.NetworkTableInstance.getDefault()
  table = inst.getTable("Spectacle")

  inst.startClient4("Spectacle")
  inst.setServer("localhost")
  inst.startDSClient()

  statusPub = table.getBooleanTopic("status").publish()
  posePub = table.getStructTopic("pose", Pose3d).publish()

  while True:
    out = vio_session.waitForOutput()
    data = json.loads(out.asJson())

    status = data.get('status', "TRACKING")

    quat = Quaternion(data["orientation"]['w'], data["orientation"]['x'], data["orientation"]['y'], data["orientation"]['z'])
    pose = Pose3d(data["position"]['x'], data["position"]['y'], data["position"]['z'], Rotation3d(quat))

    statusPub.set(status == "TRACKING")
    posePub.set(pose)

    print(status + " " + str(pose))