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
}