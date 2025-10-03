import py_rtab

if __name__ == '__main__':
    camera = py_rtab.CameraRealSense2("")
    if camera.init():
        odom = py_rtab.Odometry.create()
        rtabmap = py_rtab.Rtabmap()
        rtabmap.init()

        data = camera.takeImage()
        while data.isValid():
            info = py_rtab.OdometryInfo()
            pose = odom.process(data, info)

            if rtabmap.process(data, pose):
                if rtabmap.getLoopClosureId() > 0:
                    print("Loop closure detected!\n")

            data = camera.takeImage()
        print("Processed all frames\n")
    else:
        print("Failed to initialize camera!\n")
