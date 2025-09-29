//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/Transform.h>

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindTransform(nb::module_ &m) {
    nb::class_<Transform>(m, "Transform")
            .def(nb::init<float, float, float, float, float, float, float, float, float, float, float, float>())
            .def(nb::init<float, float, float, float, float, float>())
            .def(nb::init<float, float, float, float, float, float, float>())
            .def(nb::init<float, float, float>())
            .def("clone", &Transform::clone)
            .def("r11", &Transform::r11)
            .def("r12", &Transform::r12)
            .def("r13", &Transform::r13)
            .def("r21", &Transform::r21)
            .def("r22", &Transform::r22)
            .def("r23", &Transform::r23)
            .def("r31", &Transform::r31)
            .def("r32", &Transform::r32)
            .def("r33", &Transform::r33)
            .def("o14", &Transform::o14)
            .def("o24", &Transform::o24)
            .def("o34", &Transform::o34)
            .def("isNull", &Transform::isNull)
            .def("isIdentity", &Transform::isIdentity)
            .def("size", &Transform::size)
            .def("theta", &Transform::theta)
            .def("isInvertible", &Transform::isInvertible)
            .def("inverse", &Transform::inverse)
            .def("rotation", &Transform::rotation)
            .def("translation", &Transform::translation)
            .def("to3DoF", &Transform::to3DoF)
            .def("to4DoF", &Transform::to4DoF)
            .def("is3DoF", &Transform::is3DoF)
            .def("is4DoF", &Transform::is4DoF)
            .def("rotationMatrix", &Transform::rotationMatrix)
            .def("translationMatrix", &Transform::translationMatrix)
            .def("getTranslationAndEulerAngles", &Transform::getTranslationAndEulerAngles)
            .def("getEulerAngles", &Transform::getEulerAngles)
            .def("getTranslation", &Transform::getTranslation)
            .def("getAngle", &Transform::getAngle)
            .def("getNorm", &Transform::getNorm)
            .def("getNormSquared", &Transform::getNormSquared)
            .def("getDistance", &Transform::getDistance)
            .def("getDistanceSquared", &Transform::getDistanceSquared)
            .def("interpolate", &Transform::interpolate)
            .def("normalizeRotation", &Transform::normalizeRotation)
            .def("prettyPrint", &Transform::prettyPrint)
            .def("toEigen4f", &Transform::toEigen4f)
            .def("toEigen4d", &Transform::toEigen4d)
            .def("toEigen3f", &Transform::toEigen3f)
            .def("toEigen3d", &Transform::toEigen3d)
            .def("getQuaternionf", &Transform::getQuaternionf)
            .def("getQuaterniond", &Transform::getQuaterniond)
            .def_static("getIdentity", &Transform::getIdentity)
            .def_static("fromEigen4f", &Transform::fromEigen4f)
            .def_static("fromEigen4d", &Transform::fromEigen4d)
            .def_static("opengl_T_rtabmap", &Transform::opengl_T_rtabmap)
            .def_static("rtabmap_T_opengl", &Transform::rtabmap_T_opengl)
            .def_static("fromString", &Transform::fromString)
            .def_static("canParseString", &Transform::canParseString)
            .def_static("getTransform", &Transform::getTransform);
}