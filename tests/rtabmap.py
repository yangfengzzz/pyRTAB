import py_rtab

if __name__ == '__main__':
    odom = py_rtab.Odometry.create()
    rtabmap = py_rtab.Rtabmap()
    rtabmap.init()

    info = py_rtab.OdometryInfo()
