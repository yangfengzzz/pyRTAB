//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/eigen/dense.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/IMUFilter.h>
#include "../cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindCamera(nb::module_& m) {
    nb::class_<Camera, SensorCapture>(m, "Camera")
            .def("takeImage", &Camera::takeImage, "info"_a)
            .def("getImageRate", &Camera::getImageRate)
            .def("setImageRate", &Camera::setImageRate, "imageRate"_a)
            .def("setInterIMUPublishing", &Camera::setInterIMUPublishing, "enabled"_a, "filter"_a)
            .def("isInterIMUPublishing", &Camera::isInterIMUPublishing)
            .def("initFromFile", &Camera::initFromFile, "calibrationPath"_a)
            .def("isCalibrated", &Camera::isCalibrated);

    nb::class_<IMUFilter>(m, "IMUFilter")
            .def("parseParameters", &IMUFilter::parseParameters, "parameters"_a)
            .def("update", &IMUFilter::update, "gx"_a, "gy"_a, "gz"_a, "ax"_a, "ay"_a, "az"_a, "stamp"_a)
            .def("type", &IMUFilter::type)
            .def("getOrientation",
                 [](IMUFilter* self) {
                     double qx, qy, qz, qw;
                     self->getOrientation(qx, qy, qz, qw);
                     return std::make_tuple(qx, qy, qz, qw);
                 })
            .def("reset", &IMUFilter::reset, "qx"_a, "qy"_a, "qz"_a, "qw"_a);

    nb::enum_<IMUFilter::Type>(m, "IMUFilterType")
            .value("kMadgwick", IMUFilter::Type::kMadgwick)
            .value("kComplementaryFilter", IMUFilter::Type::kComplementaryFilter);
}