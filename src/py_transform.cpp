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
#include <rtabmap/core/Transform.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindTransform(nb::module_ &m) {
    nb::class_<Transform>(m, "Transform")
            .def(nb::init<float, float, float, float, float, float, float, float, float, float, float, float>(),
                 "r11"_a, "r12"_a, "r13"_a, "o14"_a, "r21"_a, "r22"_a, "r23"_a, "o24"_a, "r31"_a, "r32"_a, "r33"_a,
                 "o34"_a)
            .def(nb::init<const cv::Mat &>(), "transformationMatrix"_a)
            .def(nb::init<float, float, float, float, float, float>(), "x"_a, "y"_a, "z"_a, "roll"_a, "pitch"_a,
                 "yaw"_a)
            .def(nb::init<float, float, float, float, float, float, float>(), "x"_a, "y"_a, "z"_a, "qx"_a, "qy"_a,
                 "qz"_a, "qw"_a)
            .def(nb::init<float, float, float>(), "x"_a, "y"_a, "theta"_a)

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

            .def("setNull", &Transform::setNull)
            .def("setIdentity", &Transform::setIdentity)

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

            .def("getTranslationAndEulerAngles",
                 [](Transform *self) {
                     float x, y, z, roll, pitch, ya;
                     self->getTranslationAndEulerAngles(x, y, z, roll, pitch, ya);
                     return std::make_tuple(x, y, z, roll, pitch, ya);
                 })
            .def("getEulerAngles",
                 [](Transform *self) {
                     float roll, pitch, yaw;
                     self->getEulerAngles(roll, pitch, yaw);
                     return std::make_tuple(roll, pitch, yaw);
                 })
            .def("getTranslation",
                 [](Transform *self) {
                     float x, y, z;
                     self->getTranslation(x, y, z);
                     return std::make_tuple(x, y, z);
                 })
            .def("getAngle", &Transform::getAngle, "t"_a)
            .def("getNorm", &Transform::getNorm)
            .def("getNormSquared", &Transform::getNormSquared)
            .def("getDistance", &Transform::getDistance, "t"_a)
            .def("getDistanceSquared", &Transform::getDistanceSquared, "t"_a)
            .def("interpolate", &Transform::interpolate, "t"_a, "other"_a)
            .def("normalizeRotation", &Transform::normalizeRotation)
            .def("prettyPrint", &Transform::prettyPrint)
            .def("__str__", &Transform::prettyPrint)

            .def("toEigen4f", &Transform::toEigen4f)
            .def("toEigen4d", &Transform::toEigen4d)

            // .def("toEigen3f", &Transform::toEigen3f)
            // .def("toEigen3d", &Transform::toEigen3d)
            // .def("getQuaternionf", &Transform::getQuaternionf)
            // .def("getQuaterniond", &Transform::getQuaterniond)
            .def_static("getIdentity", &Transform::getIdentity)
            .def_static("fromEigen4f", &Transform::fromEigen4f, "matrix"_a)
            .def_static("fromEigen4d", &Transform::fromEigen4d, "matrix"_a)
            .def_static("opengl_T_rtabmap", &Transform::opengl_T_rtabmap)
            .def_static("rtabmap_T_opengl", &Transform::rtabmap_T_opengl)
            .def_static("fromString", &Transform::fromString, "string"_a)
            .def_static("canParseString", &Transform::canParseString, "string"_a)
            .def_static("getTransform", &Transform::getTransform, "tfBuffer"_a, "stamp"_a);
}