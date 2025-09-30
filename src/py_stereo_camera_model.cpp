//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/vector.h>
#include <rtabmap/core/StereoCameraModel.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindStereoCameraModel(nb::module_ &m) {
    nb::class_<StereoCameraModel>(m, "StereoCameraModel")
            .def(nb::init<>())
            .def(nb::init<const std::string &, const cv::Size &, const cv::Mat &, const cv::Mat &, const cv::Mat &,
                          const cv::Mat &, const cv::Size &, const cv::Mat &, const cv::Mat &, const cv::Mat &,
                          const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Mat &>(),
                 "name"_a, "imageSize1"_a, "K1"_a, "D1"_a, "R1"_a, "P1"_a, "imageSize2"_a, "K2"_a, "D2"_a, "R2"_a,
                 "P2"_a, "R"_a, "T"_a, "E"_a, "F"_a)
            .def(nb::init<const std::string &, const CameraModel &, const CameraModel &>(), "name"_a,
                 "leftCameraModel"_a, "rightCameraModel"_a)
            .def(nb::init<const std::string &, const CameraModel &, const CameraModel &, const Transform &>(), "name"_a,
                 "leftCameraModel"_a, "rightCameraModel"_a, "extrinsics"_a)
            .def(nb::init<double, double, double, double, double>(), "fx"_a, "fy"_a, "cx"_a, "cy"_a, "baseline"_a)
            .def(nb::init<const std::string &, double, double, double, double, double>(), "name"_a, "fx"_a, "fy"_a,
                 "cx"_a, "cy"_a, "baseline"_a)
            .def("isValidForProjection", &StereoCameraModel::isValidForProjection)
            .def("isValidForRectification", &StereoCameraModel::isValidForRectification)
            .def("initRectificationMap", &StereoCameraModel::initRectificationMap)
            .def("isRectificationMapInitialized", &StereoCameraModel::isRectificationMapInitialized)
            .def("setName", &StereoCameraModel::setName)
            .def("name", &StereoCameraModel::name)
            .def("setImageSize", &StereoCameraModel::setImageSize)
            .def("load", &StereoCameraModel::load)
            .def("save", &StereoCameraModel::save)
            .def("saveStereoTransform", &StereoCameraModel::saveStereoTransform)
            .def("serialize", &StereoCameraModel::serialize)
            .def("deserialize", nb::overload_cast<const std::vector<unsigned char> &>(&StereoCameraModel::deserialize))
            .def("baseline", &StereoCameraModel::baseline)
            .def("computeDepth", &StereoCameraModel::computeDepth)
            .def("computeDisparity", nb::overload_cast<float>(&StereoCameraModel::computeDisparity, nb::const_))
            .def("computeDisparity",
                 nb::overload_cast<unsigned short>(&StereoCameraModel::computeDisparity, nb::const_))
            .def("R", &StereoCameraModel::R)
            .def("T", &StereoCameraModel::T)
            .def("E", &StereoCameraModel::E)
            .def("F", &StereoCameraModel::F)
            .def("scale", &StereoCameraModel::scale)
            .def("roi", &StereoCameraModel::roi)
            .def("setLocalTransform", &StereoCameraModel::setLocalTransform)
            .def("localTransform", &StereoCameraModel::localTransform)
            .def("stereoTransform", &StereoCameraModel::stereoTransform)
            .def("left", &StereoCameraModel::left)
            .def("right", &StereoCameraModel::right)
            .def("getLeftSuffix", &StereoCameraModel::getLeftSuffix)
            .def("getRightSuffix", &StereoCameraModel::getRightSuffix);
}