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

void bindCameraImages(nb::module_& m) {
    nb::class_<CameraImages, Camera>(m, "CameraImages")
            .def(nb::init<>())
            .def(nb::init<const std::string&, float>(), "path"_a, "imageRate"_a = 0)
            .def("init", &CameraImages::init, "calibrationFolder"_a = ".", "cameraName"_a = "")
            .def("isCalibrated", &CameraImages::isCalibrated)
            .def("getSerial", &CameraImages::getSerial)
            .def("odomProvided", &CameraImages::odomProvided)
            .def("getPath", &CameraImages::getPath)
            .def("imagesCount", &CameraImages::imagesCount)
            .def("filenames", &CameraImages::filenames)
            .def("isImagesRectified", &CameraImages::isImagesRectified)
            .def("getBayerMode", &CameraImages::getBayerMode)
            .def("cameraModel", &CameraImages::cameraModel)
            .def("setPath", &CameraImages::setPath, "dir"_a)
            .def("setStartIndex", &CameraImages::setStartIndex, "index"_a)
            .def("setMaxFrames", &CameraImages::setMaxFrames, "value"_a)
            .def("setDirRefreshed", &CameraImages::setDirRefreshed, "enabled"_a)
            .def("setImagesRectified", &CameraImages::setImagesRectified, "enabled"_a)
            .def("setBayerMode", &CameraImages::setBayerMode, "mode"_a)
            .def("setTimestamps", &CameraImages::setTimestamps, "fileNamesAreStamps"_a, "filePath"_a = "",
                 "syncImageRateWithStamps"_a = true)
            .def("setConfigForEachFrame", &CameraImages::setConfigForEachFrame, "value"_a)
            .def("setScanPath", &CameraImages::setScanPath, "dir"_a, "maxScanPts"_a, "localTransform"_a)
            .def("setDepthFromScan", &CameraImages::setDepthFromScan, "enabled"_a, "fillHoles"_a = 1,
                 "fillHolesFromBorder"_a = false)
            .def("setOdometryPath", &CameraImages::setOdometryPath, "filePath"_a, "format"_a = 0)
            .def("setGroundTruthPath", &CameraImages::setGroundTruthPath, "filePath"_a, "format"_a = 0)
            .def("setMaxPoseTimeDiff", &CameraImages::setMaxPoseTimeDiff, "diff"_a)
            .def("getMaxPoseTimeDiff", &CameraImages::getMaxPoseTimeDiff)
            .def("setDepth", &CameraImages::setDepth, "isDepth"_a, "depthScaleFactor"_a = 1.0f);
}