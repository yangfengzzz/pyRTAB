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
            .def_static("isScan2d", &LaserScan::isScan2d, "format"_a)
            .def_static("isScanHasNormals", &LaserScan::isScanHasNormals, "format"_a)
            .def_static("isScanHasRGB", &LaserScan::isScanHasRGB, "format"_a)
            .def_static("isScanHasIntensity", &LaserScan::isScanHasIntensity, "format"_a)
            .def_static("isScanHasTime", &LaserScan::isScanHasTime, "format"_a)
            .def_static(
                    "backwardCompatibility",
                    [](const cv::Mat &oldScanFormat, int maxPoints = 0, int maxRange = 0) {
                        return LaserScan::backwardCompatibility(oldScanFormat, maxPoints, maxRange);
                    },
                    "oldScanFormat"_a, "maxPoints"_a = 0, "maxRange"_a = 0)
            .def_static(
                    "backwardCompatibility",
                    [](const cv::Mat &oldScanFormat, float minRange, float maxRange, float angleMin, float angleMax,
                       float angleInc) {
                        return LaserScan::backwardCompatibility(oldScanFormat, minRange, maxRange, angleMin, angleMax,
                                                                angleInc);
                    },
                    "oldScanFormat"_a, "minRange"_a, "maxRange"_a, "angleMin"_a, "angleMax"_a, "angleInc"_a)

            .def(nb::init<>())
            .def(nb::init<const LaserScan &, int, float>(), "data"_a, "maxPoints"_a, "maxRange"_a)
            .def(nb::init<const cv::Mat &, int, float, LaserScan::Format>(), "data"_a, "maxPoints"_a, "maxRange"_a,
                 "format"_a)
            .def(nb::init<const LaserScan &, float, float, float, float, float>(), "data"_a, "minRange"_a, "maxRange"_a,
                 "angleMin"_a, "angleMax"_a, "angleIncrement"_a)
            .def(nb::init<const cv::Mat &, LaserScan::Format, float, float, float, float, float>(), "data"_a,
                 "format"_a, "minRange"_a, "maxRange"_a, "angleMin"_a, "angleMax"_a, "angleIncrement"_a)
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
            .def("setLocalTransform", &LaserScan::setLocalTransform, "t"_a)
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
            .def("field", &LaserScan::field, "pointIndex"_a, "channelOffset"_a)
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