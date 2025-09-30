//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/LaserScan.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindLaserScan(nb::module_ &m) {
    nb::class_<LaserScan>(m, "LaserScan")
            .def_static("isScan2d", &LaserScan::isScan2d)
            .def_static("isScanHasNormals", &LaserScan::isScanHasNormals)
            .def_static("isScanHasRGB", &LaserScan::isScanHasRGB)
            .def_static("isScanHasIntensity", &LaserScan::isScanHasIntensity)
            .def_static("isScanHasTime", &LaserScan::isScanHasTime)
            .def_static("backwardCompatibility",
                        [](const cv::Mat &oldScanFormat, int maxPoints = 0, int maxRange = 0) {
                            return LaserScan::backwardCompatibility(oldScanFormat, maxPoints, maxRange);
                        })
            .def_static("backwardCompatibility",
                        [](const cv::Mat &oldScanFormat, float minRange, float maxRange, float angleMin, float angleMax,
                           float angleInc) {
                            return LaserScan::backwardCompatibility(oldScanFormat, minRange, maxRange, angleMin,
                                                                    angleMax, angleInc);
                        })

            .def(nb::init<>())
            .def(nb::init<const LaserScan &, int, float>())
            .def(nb::init<const cv::Mat &, int, float, LaserScan::Format>())
            .def(nb::init<const LaserScan &, float, float, float, float, float>())
            .def(nb::init<const cv::Mat &, LaserScan::Format, float, float, float, float, float>())
            .def("clone", &LaserScan::clone)
            .def("data", &LaserScan::data)
            .def("format", &LaserScan::format)
            .def("formatName", [](LaserScan *scan) { return scan->formatName(); })
            .def("channels", [](LaserScan *scan) { return scan->channels(); })
            .def("maxPoints", &LaserScan::maxPoints)
            .def("rangeMin", &LaserScan::rangeMin)
            .def("rangeMax", &LaserScan::rangeMax)
            .def("angleMin", &LaserScan::angleMin)
            .def("angleMax", &LaserScan::angleMax)
            .def("angleIncrement", &LaserScan::angleIncrement)
            .def("setLocalTransform", &LaserScan::setLocalTransform)
            .def("localTransform", &LaserScan::localTransform)
            .def("empty", &LaserScan::empty)
            .def("isEmpty", &LaserScan::isEmpty)
            .def("size", &LaserScan::size)
            .def("dataType", &LaserScan::dataType)
            .def("is2d", &LaserScan::is2d)
            .def("hasNormals", &LaserScan::hasNormals)
            .def("hasRGB", &LaserScan::hasRGB)
            .def("hasIntensity", &LaserScan::hasIntensity)
            .def("hasTime", &LaserScan::hasTime)
            .def("isCompressed", &LaserScan::isCompressed)
            .def("isOrganized", &LaserScan::isOrganized)
            .def("clone", &LaserScan::clone)
            .def("densify", &LaserScan::densify)
            .def("getIntensityOffset", &LaserScan::getIntensityOffset)
            .def("getRGBOffset", &LaserScan::getRGBOffset)
            .def("getNormalsOffset", &LaserScan::getNormalsOffset)
            .def("getTimeOffset", &LaserScan::getTimeOffset)
            .def("field", &LaserScan::field)
            .def("clear", &LaserScan::clear);

    nb::enum_<LaserScan::Format>(m, "LaserScanFormat")
            .value("kUnknown", LaserScan::Format::kUnknown)
            .value("kXY", LaserScan::Format::kXY)
            .value("kXYI", LaserScan::Format::kXYI)
            .value("kXYNormal", LaserScan::Format::kXYNormal)
            .value("kXYINormal", LaserScan::Format::kXYINormal)
            .value("kXYZ", LaserScan::Format::kXYZ)
            .value("kXYZI", LaserScan::Format::kXYZI)
            .value("kXYZRGB", LaserScan::Format::kXYZRGB)
            .value("kXYZNormal", LaserScan::Format::kXYZNormal)
            .value("kXYZINormal", LaserScan::Format::kXYZINormal)
            .value("kXYZRGBNormal", LaserScan::Format::kXYZRGBNormal)
            .value("kXYZIT", LaserScan::Format::kXYZIT);
}