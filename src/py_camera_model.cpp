//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <rtabmap/core/CameraModel.h>

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindCameraModel(nb::module_ &m) {
    nb::class_<CameraModel>(m, "CameraModel")
            .def_static("opticalRotation", &CameraModel::opticalRotation)

            .def(nb::init<double, double, double, double, const Transform &, double, const cv::Size &>())
            .def(nb::init<const std::string &, double, double, double, double, const Transform &, double,
                          const cv::Size &>())
            .def("initRectificationMap", &CameraModel::initRectificationMap)
            .def("isRectificationMapInitialized", &CameraModel::isRectificationMapInitialized)
            .def("isValidForProjection", &CameraModel::isValidForProjection)
            .def("isValidForReprojection", &CameraModel::isValidForReprojection)
            .def("isValidForRectification", &CameraModel::isValidForRectification)
            .def("setName", &CameraModel::setName)
            .def("name", &CameraModel::name)
            .def("fx", &CameraModel::fx)
            .def("fy", &CameraModel::fy)
            .def("cx", &CameraModel::cx)
            .def("cy", &CameraModel::cy)
            .def("Tx", &CameraModel::Tx)
            .def("setLocalTransform", &CameraModel::setLocalTransform)
            .def("localTransform", &CameraModel::localTransform)
            .def("setImageSize", &CameraModel::setImageSize)
            .def("imageSize", &CameraModel::imageSize)
            .def("imageWidth", &CameraModel::imageWidth)
            .def("imageHeight", &CameraModel::imageHeight)
            .def("fovX", &CameraModel::fovX)
            .def("fovY", &CameraModel::fovY)
            .def("horizontalFOV", &CameraModel::horizontalFOV)
            .def("verticalFOV", &CameraModel::verticalFOV)
            .def("isFisheye", &CameraModel::isFisheye)
            .def("load", nb::overload_cast<const std::string &>(&CameraModel::load))
            .def("load", nb::overload_cast<const std::string &, const std::string &>(&CameraModel::load))
            .def("save", &CameraModel::save)
            .def("serialize", &CameraModel::serialize)
            .def("deserialize", nb::overload_cast<const std::vector<unsigned char> &>(&CameraModel::deserialize))
            .def("scaled", &CameraModel::scaled)
            .def("roi", &CameraModel::roi)
            .def("project", &CameraModel::project)
            // .def("reproject", nb::overload_cast<float, float, float, float &, float &>(&CameraModel::reproject))
            // .def("reproject", nb::overload_cast<float, float, float, int &, int &>(&CameraModel::reproject))
            .def("inFrame", &CameraModel::inFrame);
}