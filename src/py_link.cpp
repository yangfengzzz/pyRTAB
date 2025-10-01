//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/Link.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindLink(nb::module_ &m) {
    nb::class_<Link>(m, "Link")
            .def(nb::init<int, int, Link::Type, const Transform &>(), "from"_a, "to"_a, "type"_a, "transform"_a)
            .def("isValid", &Link::isValid)
            .def("from", &Link::from)
            .def("to", &Link::to)
            .def("transform", &Link::transform)
            .def("type", &Link::type)
            // .def("typeName", &Link::typeName)
            .def("infMatrix", &Link::infMatrix)
            .def("rotVariance", &Link::rotVariance, "minimum"_a = true)
            .def("transVariance", &Link::transVariance, "minimum"_a = true)
            .def("setFrom", &Link::setFrom, "from"_a)
            .def("setTo", &Link::setTo, "to"_a)
            .def("setTransform", &Link::setTransform, "transform"_a)
            .def("setType", &Link::setType, "type"_a)
            .def("setInfMatrix", &Link::setInfMatrix, "infMatrix"_a)
            .def("userDataRaw", &Link::userDataRaw)
            .def("userDataCompressed", &Link::userDataCompressed)
            .def("uncompressUserData", &Link::uncompressUserData)
            .def("uncompressUserDataConst", &Link::uncompressUserDataConst)
            .def("merge", &Link::merge, "link"_a, "outputType"_a)
            .def("inverse", &Link::inverse);

    nb::enum_<Link::Type>(m, "LinkType")
            .value("kNeighbor", Link::Type::kNeighbor)
            .value("kGlobalClosure", Link::Type::kGlobalClosure)
            .value("kLocalSpaceClosure", Link::Type::kLocalSpaceClosure)
            .value("kLocalTimeClosure", Link::Type::kLocalTimeClosure)
            .value("kUserClosure", Link::Type::kUserClosure)
            .value("kVirtualClosure", Link::Type::kVirtualClosure)
            .value("kNeighborMerged", Link::Type::kNeighborMerged)
            .value("kPosePrior", Link::Type::kPosePrior)
            .value("kLandmark", Link::Type::kLandmark)
            .value("kGravity", Link::Type::kGravity)
            .value("kEnd", Link::Type::kEnd)
            .value("kSelfRefLink", Link::Type::kSelfRefLink)
            .value("kAllWithLandmarks", Link::Type::kAllWithLandmarks)
            .value("kAllWithoutLandmarks", Link::Type::kAllWithoutLandmarks)
            .value("kUndef", Link::Type::kUndef);
}