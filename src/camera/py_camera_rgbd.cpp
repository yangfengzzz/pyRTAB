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
#include <rtabmap/core/CameraRGBD.h>
#include "../cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindCameraRGBD(nb::module_& m) {
    nb::class_<CameraRGBDImages, CameraImages>(m, "CameraRGBDImages")
            .def(nb::init<const std::string&, const std::string&, float, float>(), "pathRGBImages"_a,
                 "pathDepthImages"_a, "depthScaleFactor"_a = 1.0f, "imageRate"_a = 0.0f)
            .def("init", &CameraRGBDImages::init, "calibrationFolder"_a, "cameraName"_a = 0)
            .def("setStartIndex", &CameraRGBDImages::setStartIndex, "index"_a)
            .def("setMaxFrames", &CameraRGBDImages::setMaxFrames, "value"_a);

    nb::class_<CameraRealSense2, Camera>(m, "CameraRealSense2")
            .def(nb::init<const std::string&, float>(), "deviceId"_a = "", "imageRate"_a = 0)
            .def("init", &CameraRealSense2::init, "calibrationFolder"_a = ".", "cameraName"_a = "")
            .def("isCalibrated", &CameraRealSense2::isCalibrated)
            .def("getSerial", &CameraRealSense2::getSerial)
            .def("odomProvided", &CameraRealSense2::odomProvided)
            .def(
                    "getPose",
                    [](CameraRealSense2* self, double stamp, double maxWaitTime) {
                        Transform pose;
                        cv::Mat covariance;
                        auto result = self->getPose(stamp, pose, covariance, maxWaitTime);
                        return std::make_tuple(result, pose, covariance);
                    },
                    "stamp"_a, "maxWaitTime"_a = 0.06)
            .def("setEmitterEnabled", &CameraRealSense2::setEmitterEnabled, "enabled"_a)
            .def("setIRFormat", &CameraRealSense2::setIRFormat, "enabled"_a, "useDepthInsteadOfRightImage"_a)
            .def("setResolution", &CameraRealSense2::setResolution, "width"_a, "height"_a, "fps"_a = 30)
            .def("setDepthResolution", &CameraRealSense2::setDepthResolution, "width"_a, "height"_a, "fps"_a = 30)
            .def("setGlobalTimeSync", &CameraRealSense2::setGlobalTimeSync, "enabled"_a)
            .def("setDualMode", &CameraRealSense2::setDualMode, "enabled"_a, "extrinsics"_a)
            .def("setJsonConfig", &CameraRealSense2::setJsonConfig, "json"_a)
            .def("setImagesRectified", &CameraRealSense2::setImagesRectified, "enabled"_a)
            .def("setOdomProvided", &CameraRealSense2::setOdomProvided, "enabled"_a, "imageStreamsDisabled"_a = false,
                 "onlyLeftStream"_a = false);
}