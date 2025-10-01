//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <rtabmap/core/CameraModel.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindCameraModel(nb::module_ &m) {
    nb::class_<CameraModel>(m, "CameraModel")
            .def_static("opticalRotation", &CameraModel::opticalRotation)
            .def(nb::init<>())
            .def(nb::init<const std::string &, const cv::Size &, const cv::Mat &, const cv::Mat &, const cv::Mat &,
                          const cv::Mat &, const Transform &>(),
                 "name"_a, "imageSize"_a, "K"_a, "D"_a, "R"_a, "P"_a, "localTransform"_a)
            .def(nb::init<double, double, double, double, const Transform &, double, const cv::Size &>(), "fx"_a,
                 "fy"_a, "cx"_a, "cy"_a, "localTransform"_a, "Tx"_a, "imageSize"_a)
            .def(nb::init<const std::string &, double, double, double, double, const Transform &, double,
                          const cv::Size &>(),
                 "name"_a, "fx"_a, "fy"_a, "cx"_a, "cy"_a, "localTransform"_a, "Tx"_a, "imageSize"_a)
            .def("initRectificationMap", &CameraModel::initRectificationMap)
            .def("isRectificationMapInitialized", &CameraModel::isRectificationMapInitialized)
            .def("isValidForProjection", &CameraModel::isValidForProjection)
            .def("isValidForReprojection", &CameraModel::isValidForReprojection)
            .def("isValidForRectification", &CameraModel::isValidForRectification)
            .def("setName", &CameraModel::setName, "name"_a)
            .def("name", &CameraModel::name)
            .def("fx", &CameraModel::fx)
            .def("fy", &CameraModel::fy)
            .def("cx", &CameraModel::cx)
            .def("cy", &CameraModel::cy)
            .def("Tx", &CameraModel::Tx)
            .def("setLocalTransform", &CameraModel::setLocalTransform, "transform"_a)
            .def("localTransform", &CameraModel::localTransform)
            .def("setImageSize", &CameraModel::setImageSize, "size"_a)
            .def("imageSize", &CameraModel::imageSize)
            .def("imageWidth", &CameraModel::imageWidth)
            .def("imageHeight", &CameraModel::imageHeight)
            .def("fovX", &CameraModel::fovX)
            .def("fovY", &CameraModel::fovY)
            .def("horizontalFOV", &CameraModel::horizontalFOV)
            .def("verticalFOV", &CameraModel::verticalFOV)
            .def("isFisheye", &CameraModel::isFisheye)
            .def("load", nb::overload_cast<const std::string &>(&CameraModel::load), "filePath"_a)
            .def("load", nb::overload_cast<const std::string &, const std::string &>(&CameraModel::load), "directory"_a,
                 "cameraName"_a)
            .def("save", &CameraModel::save, "directory"_a)
            .def("serialize", &CameraModel::serialize)
            .def("deserialize", nb::overload_cast<const std::vector<unsigned char> &>(&CameraModel::deserialize),
                 "data"_a)
            .def("scaled", &CameraModel::scaled, "scale"_a)
            .def("roi", &CameraModel::roi, "roi"_a)
            .def("project", &CameraModel::project, "u"_a, "v"_a, "depth"_a, "x"_a, "y"_a, "z"_a)
            // .def("reproject", nb::overload_cast<float, float, float, float &, float &>(&CameraModel::reproject))
            // .def("reproject", nb::overload_cast<float, float, float, int &, int &>(&CameraModel::reproject))
            .def("inFrame", &CameraModel::inFrame, "u"_a, "v"_a);
}