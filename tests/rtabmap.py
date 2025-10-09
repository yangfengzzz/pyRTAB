#  Copyright (c) 2025 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

import py_rtab
import numpy as np

if __name__ == '__main__':
    camera = py_rtab.CameraRealSense2("146222253630")
    if camera.init():
        odom = py_rtab.Odometry.create()
        rtabmap = py_rtab.Rtabmap()
        rtabmap.init()

        data = camera.takeImage()
        while data.isValid():
            info = py_rtab.OdometryInfo()
            pose = odom.process(data, info)

            if rtabmap.process(data, pose, np.eye(6), [], {}):
                stats = rtabmap.getStatistics()
                correction = stats.mapCorrection()
                pose = correction * pose
                if rtabmap.getLoopClosureId() > 0:
                    print("Loop closure detected!\n")
            print(pose.toEigen4f())

            data = camera.takeImage()
        print("Processed all frames\n")
    else:
        print("Failed to initialize camera!\n")
