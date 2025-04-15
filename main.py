#!/usr/bin/env python

import tkinter as tk

from math import pi

import PIL
from mpl_toolkits.mplot3d.art3d import math
from mpl_toolkits.mplot3d.axes3d import textwrap
import spectacularAI
import depthai
import threading
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg)
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageTk


#UI
window = tk.Tk()
dataframe = tk.Frame(window)
vframe = tk.Frame(window)
# iframe = tk.Frame(window)

dataframe.pack(side=tk.LEFT)
vframe.pack()

tlabel = tk.Label(dataframe, text="awaiting tracking data", font=('Arial', 24))
tlabel.pack()

tlabel = tk.Label(dataframe, text="  Translation: ", font=('Arial', 24))
txlabel = tk.Label(dataframe, text="x: 0", font=('Arial', 24))
tylabel = tk.Label(dataframe, text="y: 0", font=('Arial', 24))
tzlabel = tk.Label(dataframe, text="z: 0", font=('Arial', 24))

tlabel.pack()
txlabel.pack()
tylabel.pack()
tzlabel.pack()

rlabel = tk.Label(dataframe, text="Rotation:", font=('Arial', 24))
rxlabel = tk.Label(dataframe, text="x: 0", font=('Arial', 24))
rylabel = tk.Label(dataframe, text="y: 0", font=('Arial', 24))
rzlabel = tk.Label(dataframe, text="z: 0", font=('Arial', 24))

rlabel.pack()
rxlabel.pack()
rylabel.pack()
rzlabel.pack()

glabel = tk.Label(dataframe, text="Gyro:", font=('Arial', 24))
gxlabel = tk.Label(dataframe, text="x: 0 Rad/s", font=('Arial', 24))
gylabel = tk.Label(dataframe, text="y: 0 Rad/s", font=('Arial', 24))
gzlabel = tk.Label(dataframe, text="z: 0 Rad/s", font=('Arial', 24))

glabel.pack()
gxlabel.pack()
gylabel.pack()
gzlabel.pack()


v1label = tk.Label(vframe)
v1label.pack()

#<graph>
fig = plt.figure()
ax = Axes3D(fig)
fig.add_axes(ax)

data = { c: [] for c in 'xyz' }

ax_bounds = (-4, 4) # meters
ax.set(xlim=ax_bounds, ylim=ax_bounds, zlim=ax_bounds)
ax.view_init(azim=-140) # initial plot orientation
vio_plot = ax.plot(
    xs=[], ys=[], zs=[],
    linestyle="-",
    marker=""
)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_zlabel("z (m)")

title = ax.set_title("VIO trajectory")
canvas = FigureCanvasTkAgg(fig, master=vframe)  # A tk.DrawingArea.
canvas.draw()
canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)


def update_data(vio_out):
        # supports two slightly different JSONL formats
        if 'pose' in vio_out: vio_out = vio_out['pose']
        # SDK < 0.12 does not expose the TRACKING status
        is_tracking = vio_out.get('status', 'TRACKING') == 'TRACKING'
        for c in 'xyz':
            val = vio_out['position'][c]
            if not is_tracking: val = np.nan
            data[c].append(val)
        return True

def update_graph(frames):
        x, y, z = [np.array(data[c]) for c in 'xyz']
        vio_plot[0].set_data(x, y)
        vio_plot[0].set_3d_properties(z)
        return (vio_plot[0],)

from matplotlib.animation import FuncAnimation
anim = FuncAnimation(fig, update_graph, interval=15, blit=True)
#</graph>

#camera

config = spectacularAI.depthai.Configuration()
config.aprilTagPath = "./tagtest.json"
#
pipeline = depthai.Pipeline()

vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)
#| 0 -1 0 0.368 |
#| -1 0 0 0.2 |
#| 0 0 -1 0.0541 |
#| 0 0 0 1 |

device = depthai.Device(pipeline)
calib = device.readCalibration()
eeprom = calib.getEepromData()

pname = eeprom.productName
print(pname)

#check for oak d lite, spectacular does not support by default so we need to do some extra work.
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
    print("Using Oak D lite imu matrix")

features = []
image = None

# def updateImage():
#     image
#     v1label.config(image=todo)
#

def onImageFactor(name):
    def onImage(img):
        if img.getWidth() <= 0 or img.getHeight() <= 0:
            # When SLAM is enabled, monocular frames are only used at 1/6th of normal frame rate,
            # rest of the frames are [0,0] in size and must be filtered
            return

        new = Image.fromarray(img.getCvFrame())

        if img != ImageTk.PhotoImage:
            global image
            image = ImageTk.PhotoImage(new)
            v1label.config(image=image)

        if (image.width() != new.width):
            print("imconfig change")
            v1label.config(image=image)

        image.paste(new)

    return onImage

vio_pipeline.hooks.monoPrimary = onImageFactor("Primary")

print(vio_pipeline.imuToCameraLeft)

# def onFeatures(featurebuf):
#     if type(featurebuf) is not np.ndarray: return
#     featurebuf[:] = 0
#     features = featurebuf
#
    # for feature in features.trackedFeatures:
    #     cv2.circle(features, (int(feature.position.x), int(feature.position.y)), 2, , -1, cv2.LINE_AA, 0)
    # cv2.imshow("Features", featureBuffer)
    # if cv2.waitKey(1) == ord("q"):
    #     exit(0)
    #

def onImuData(imuData):
    for imuPacket in imuData.packets:
        acceleroValues = imuPacket.acceleroMeter
        gyroValues = imuPacket.gyroscope
        acceleroTs = acceleroValues.getTimestampDevice().total_seconds() * 1000
        gyroTs = gyroValues.getTimestampDevice().total_seconds() * 1000
        imuF = "{:.06f}"
        tsF  = "{:.03f}"
        # print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
        # print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
        # print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
        # print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")
        #
        gxlabel.config(text=f"x: {imuF.format(gyroValues.x)} rad/s")
        gylabel.config(text=f"y: {imuF.format(gyroValues.y)} rad/s")
        gzlabel.config(text=f"z: {imuF.format(gyroValues.z)} rad/s")

vio_pipeline.hooks.imu = onImuData

vio_session = vio_pipeline.startSession(device)

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

def camloop():
    import json
    while True:
        print(vio_session.hasOutput())

        out = vio_session.waitForOutput()
        data = json.loads(out.asJson())

        # tlabel.config(text=f"Tracking: {data.get('status', "TRACKING")}")
        tlabel.config(text=f"Tracking: {data.get('status')}")

        txlabel.config(text=f"x: {round(data["position"]['x'], 3)}")
        tylabel.config(text=f"y: {round(data["position"]['y'], 3)}")
        tzlabel.config(text=f"z: {round(data["position"]['z'], 3)}")

        angles = euler_from_quaternion(data["orientation"]['x'], data["orientation"]['y'], data["orientation"]['z'], data["orientation"]['w'])

        rxlabel.config(text=f"x: {round(angles[0] * (180/pi))}")
        rylabel.config(text=f"y: {round(angles[1] * (180/pi))}")
        rzlabel.config(text=f"z: {round(angles[2] * (180/pi))}")

        update_data(data)

        print(data["orientation"])





thread = threading.Thread(target=camloop)
# threads.append(thread)
thread.start()

window.mainloop()
thread.join()

