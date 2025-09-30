//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/vector.h>
#include <rtabmap/core/Signature.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindSignature(nb::module_ &m) {
    nb::class_<Signature>(m, "Signature")
            .def(nb::init<>())
            .def(nb::init<int>(), "id"_a)
            .def("compareTo", &Signature::compareTo)
            .def("isBadSignature", &Signature::isBadSignature)
            .def("id", &Signature::id)
            .def("id", &Signature::id)
            .def("mapId", &Signature::mapId)
            .def("setWeight", &Signature::setWeight)
            .def("getWeight", &Signature::getWeight)
            .def("setLabel", &Signature::setLabel)
            .def("getLabel", &Signature::getLabel)
            .def("getStamp", &Signature::getStamp)
            .def("addLinks", nb::overload_cast<const std::list<Link> &>(&Signature::addLinks))
            .def("addLinks", nb::overload_cast<const std::map<int, Link> &>(&Signature::addLinks))
            .def("addLink", &Signature::addLink)
            .def("hasLink", &Signature::hasLink)
            .def("changeLinkIds", &Signature::changeLinkIds)
            .def("removeLinks", &Signature::removeLinks)
            .def("removeLink", &Signature::removeLink)
            .def("removeVirtualLinks", &Signature::removeVirtualLinks)
            .def("addLandmark", &Signature::addLandmark)
            .def("getLandmarks", &Signature::getLandmarks)
            .def("removeLandmarks", &Signature::removeLandmarks)
            .def("removeLandmark", &Signature::removeLandmark)
            .def("setSaved", &Signature::setSaved)
            .def("setModified", &Signature::setModified)
            // .def("getLinks", &Signature::getLinks)
            .def("isSaved", &Signature::isSaved)
            .def("isModified", &Signature::isModified)
            .def("isLinksModified", &Signature::isLinksModified)
            .def("changeWordsRef", &Signature::changeWordsRef)
            // .def("setWords", &Signature::setWords)
            .def("isEnabled", &Signature::isEnabled)
            .def("setEnabled", &Signature::setEnabled)
            // .def("getWords", &Signature::getWords)
            .def("getWordsKpts", &Signature::getWordsKpts)
            .def("getInvalidWordsCount", &Signature::getInvalidWordsCount)
            .def("getWordsChanged", &Signature::getWordsChanged)
            .def("getWordsDescriptors", &Signature::getWordsDescriptors)
            .def("setWordsDescriptors", &Signature::setWordsDescriptors)
            .def("setPose", &Signature::setPose)
            .def("setGroundTruthPose", &Signature::setGroundTruthPose)
            .def("setVelocity", &Signature::setVelocity)
            .def("getWords3", &Signature::getWords3)
            .def("getPose", &Signature::getPose)
            .def("getPoseCovariance", &Signature::getPoseCovariance)
            .def("getGroundTruthPose", &Signature::getGroundTruthPose)
            .def("getVelocity", &Signature::getVelocity)
            .def("sensorData", nb::overload_cast<>(&Signature::sensorData, nb::const_))
            .def("getMemoryUsed", &Signature::getMemoryUsed);
}