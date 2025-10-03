//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>
#include <rtabmap/core/SensorCapture.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindSensorCapture(nb::module_ &m) {
    nb::class_<SensorCapture>(m, "SensorCapture")
            .def("takeData", &SensorCapture::takeData, "info"_a)

            .def("init", &SensorCapture::init, "calibrationFolder"_a = ".", "cameraName"_a = "")
            .def("getSerial", &SensorCapture::getSerial)
            .def("odomProvided", &SensorCapture::odomProvided)
            .def("getPose", &SensorCapture::getPose, "stamp"_a, "pose"_a, "covariance"_a, "maxWaitTime"_a = 0.06)

            .def("getFrameRate", &SensorCapture::getFrameRate)
            .def("getLocalTransform", &SensorCapture::getLocalTransform)

            .def("setFrameRate", &SensorCapture::setFrameRate, "frameRate"_a)
            .def("setLocalTransform", &SensorCapture::setLocalTransform, "localTransform"_a)

            .def("resetTimer", &SensorCapture::resetTimer);

    nb::class_<SensorCaptureInfo>(m, "SensorCaptureInfo")
            .def(nb::init<>())
            .def_rw("cameraName", &SensorCaptureInfo::cameraName)
            .def_rw("id", &SensorCaptureInfo::id)
            .def_rw("stamp", &SensorCaptureInfo::stamp)
            .def_rw("timeCapture", &SensorCaptureInfo::timeCapture)
            .def_rw("timeDeskewing", &SensorCaptureInfo::timeDeskewing)
            .def_rw("timeDisparity", &SensorCaptureInfo::timeDisparity)
            .def_rw("timeMirroring", &SensorCaptureInfo::timeMirroring)
            .def_rw("timeStereoExposureCompensation", &SensorCaptureInfo::timeStereoExposureCompensation)
            .def_rw("timeImageDecimation", &SensorCaptureInfo::timeImageDecimation)
            .def_rw("timeHistogramEqualization", &SensorCaptureInfo::timeHistogramEqualization)
            .def_rw("timeScanFromDepth", &SensorCaptureInfo::timeScanFromDepth)
            .def_rw("timeUndistortDepth", &SensorCaptureInfo::timeUndistortDepth)
            .def_rw("timeBilateralFiltering", &SensorCaptureInfo::timeBilateralFiltering)
            .def_rw("timeTotal", &SensorCaptureInfo::timeTotal)
            .def_rw("odomPose", &SensorCaptureInfo::odomPose)
            .def_rw("odomCovariance", &SensorCaptureInfo::odomCovariance)
            .def_rw("odomVelocity", &SensorCaptureInfo::odomVelocity);
}