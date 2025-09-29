//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindOdometry(nb::module_ &m) {
    nb::class_<Odometry>(m, "Odometry")
            .def("process", nb::overload_cast<SensorData &, OdometryInfo *>(&Odometry::process))
            .def("process", nb::overload_cast<SensorData &, const Transform &, OdometryInfo *>(&Odometry::process))
            .def("reset", &Odometry::reset)
            .def("getType", &Odometry::getType)
            .def("canProcessRawImages", &Odometry::canProcessRawImages)
            .def("canProcessAsyncIMU", &Odometry::canProcessAsyncIMU);
}